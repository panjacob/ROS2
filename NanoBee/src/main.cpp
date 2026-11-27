// NanoBee: minimal robotic-arm firmware for the MKS TinyBee.
// Commands over serial (115200) or HTTP GET /cmd?c=<line>; all units in steps.
// Reply: "ok" or "err <reason>". Tunables: config.yaml (see settings.cpp), editable from the web page.
//   M p0 p1 p2 p3 p4 [speed [accel]]  queue synchronized absolute move (stops at the end)
//   R d0 d1 d2 d3 d4 [speed [accel]]  queue move relative to last queued target
//   T p0 p1 p2 p3 p4 dt_ms            queue timed segment: reach p in dt_ms at constant speed;
//                                     consecutive T segments blend (no stop) if motion.blend
//   J v0 v1 v2 v3 v4 [accel]          jog velocities (steps/s); repeat within jog_timeout_ms
//   S                                 decelerate to stop, clear queue, cancel homing
//   !                                 instant halt (real-time on serial, no newline needed)
//   E 0|1                             disable / enable drivers (enabled at boot; disable drops the home reference);
//                                     joints with joints.jointN.idle_disable also switch off by themselves
//                                     after motion.idle_timeout_ms at rest and wake on the next move
//   Z [p0 p1 p2 p3 p4]                set current position (default 0); joints count as referenced
//   H [j]                             home joint j, or all joints with joints.jointN.homing != none
//   V speed accel                     default speed/accel (until reboot)
//   G us                              servo pulse on GPIO2 (500..2500, 0 = off)
//   VER                               "NanoBee <protocol> joints=<n>"
//   ?                                 <State P:p0,..,p4 Q:queued F:free E:enabled L:endstops H:referenced O:powered>
// Asynchronous messages (not replies) start with "# ", e.g. "# homed 0", "# err home 0: ...".

#include <Arduino.h>
#include "config.h"
#include "motion.h"
#include "settings.h"
#include "web.h"

static const char* VERSION = "NanoBee 2 joints=5";

struct Move {
    int32_t  target[NUM_JOINTS];
    float    speed;
    float    accel;
    uint32_t ticks;  // > 0: timed segment (T), 0: trapezoidal move (M/R)
};

static Move    queue[MOVE_QUEUE_LEN];
static int     qHead = 0, qCount = 0;
static int32_t plannedPos[NUM_JOINTS];  // end position of the last queued move
static bool    referenced[NUM_JOINTS];  // position known (homed or set with Z): soft limits apply

static uint32_t lastJogMs = 0;
static String   events;  // async messages not yet fetched by the web page

static void notify(const String& msg) {
    Serial.println("# " + msg);
    if (events.length() < 1024) events += "# " + msg + "\n";
}

// ---------------- Endstops / limits ----------------

static bool endstopHit(int j) {
    if (ENDSTOP_PIN[j] < 0) return false;
    return digitalRead(ENDSTOP_PIN[j]) == (cfg.endstopActiveLow ? LOW : HIGH);
}

static int32_t degToSteps(int j, float deg) { return (int32_t)lroundf(deg * cfg.stepsPerDeg[j]); }

static bool limitsOn(int j) { return cfg.softLimits && referenced[j]; }

static bool withinLimits(const int32_t target[NUM_JOINTS]) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!limitsOn(j)) continue;
        if (target[j] < degToSteps(j, cfg.minDeg[j]) || target[j] > degToSteps(j, cfg.maxDeg[j])) return false;
    }
    return true;
}

static void updateLimits() {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        Motion::setLimit(j, limitsOn(j), degToSteps(j, cfg.minDeg[j]), degToSteps(j, cfg.maxDeg[j]));
    }
}

// ---------------- Homing ----------------
// edge:   approach the switch in homing_dir, back off, touch again slowly; that point = home_offset_deg.
// center: touch the switch from both sides of the travel; the middle = home_offset_deg.

enum HomePhase { H_NONE, H_SEEK, H_BACKOFF, H_CLEAR, H_LOCATE, H_LEAVE, H_RETRACT };
static HomePhase homePhase = H_NONE;
static int       homeJoint = -1;
static uint8_t   homeQueue = 0;  // joints still to home after the current one (bit mask)
static int8_t    touchDir  = -1;  // direction of the current approach to the switch
static int       touchNo   = 0;
static int32_t   firstTouch = 0;
static int32_t   homeStart  = 0;  // position where the current phase began (travel limit)
static int32_t   seekGuard  = 0;  // ignore the switch for this many steps after leaving it

static void syncPlanned() { Motion::getPositions(plannedPos); }

static void clearQueue() {
    qHead = qCount = 0;
}

static int32_t jointPos(int j) {
    int32_t p[NUM_JOINTS];
    Motion::getPositions(p);
    return p[j];
}

static void moveJoint(int j, int32_t to, float speed) {
    int32_t t[NUM_JOINTS];
    Motion::getPositions(t);
    t[j] = to;
    Motion::moveTo(t, speed, cfg.homingAccel);
}

static void homeFail(const char* why) {
    Motion::halt();
    homePhase = H_NONE;
    homeQueue = 0;
    syncPlanned();
    notify("err home " + String(homeJoint) + ": " + why);
}

static void beginPhase(HomePhase p) {
    homePhase = p;
    homeStart = jointPos(homeJoint);
}

static void jogHoming(int dir, float speed) {
    Motion::jogJoint(homeJoint, dir * speed, cfg.homingAccel);
}

static float homeSpeed(int j) { return cfg.jointHomingSpeed[j] > 0 ? cfg.jointHomingSpeed[j] : cfg.homingSpeed; }

static int32_t homeMaxTravel(int j) {
    return cfg.jointHomingMaxTravel[j] > 0 ? cfg.jointHomingMaxTravel[j] : cfg.homingMaxTravel;
}

static void startTouch(int dir) {
    int j    = homeJoint;
    touchDir = dir;
    if (endstopHit(j)) {
        moveJoint(j, jointPos(j) - dir * cfg.homingBackoff, homeSpeed(j));
        beginPhase(H_BACKOFF);
    } else {
        jogHoming(dir, homeSpeed(j));
        beginPhase(H_SEEK);
    }
}

static void startHomingJoint(int j) {
    homeJoint     = j;
    referenced[j] = false;
    updateLimits();
    touchNo   = 0;
    seekGuard = 0;
    startTouch(cfg.homingDir[j]);
}

static void finishTouch(int32_t at) {
    int     j      = homeJoint;
    int32_t offset = degToSteps(j, cfg.homeOffsetDeg[j]);
    if (cfg.homing[j] == HOME_CENTER && touchNo == 0) {
        // Leave the switch the other way and travel round to hit it from its other side.
        firstTouch = at;
        touchNo    = 1;
        touchDir   = -touchDir;
        jogHoming(touchDir, homeSpeed(j));
        beginPhase(H_LEAVE);
        return;
    }
    if (cfg.homing[j] == HOME_CENTER) {
        int32_t span = at > firstTouch ? at - firstTouch : firstTouch - at;
        if (span <= 2 * cfg.homingBackoff) return homeFail("travel between switch hits too short");
        int32_t center = (int32_t)(((int64_t)at + firstTouch) / 2);
        Motion::setPosition(j, at - center + offset);
    } else {
        Motion::setPosition(j, offset);
    }
    moveJoint(j, jointPos(j) - touchDir * cfg.homingBackoff, homeSpeed(j));
    beginPhase(H_RETRACT);
}

static void homingTask() {
    if (homePhase == H_NONE) return;
    int     j      = homeJoint;
    int32_t pos    = jointPos(j);
    int32_t travel = pos - homeStart;
    if (travel < 0) travel = -travel;
    bool hit     = endstopHit(j);
    bool stopped = Motion::jointIdle(j);
    bool tooFar  = travel > homeMaxTravel(j);
    float slow   = homeSpeed(j) / 5;

    switch (homePhase) {
        case H_SEEK:
            if (hit && travel >= seekGuard) {
                Motion::haltJoint(j);
                moveJoint(j, pos - touchDir * cfg.homingBackoff, homeSpeed(j));
                beginPhase(H_BACKOFF);
            } else if (tooFar) {
                homeFail("endstop not found");
            } else if (stopped) {
                homeFail("aborted");
            }
            break;
        case H_BACKOFF:
            if (stopped) {
                jogHoming(hit ? -touchDir : touchDir, slow);
                beginPhase(hit ? H_CLEAR : H_LOCATE);
            }
            break;
        case H_CLEAR:  // still on the switch after backing off: creep off it
            if (!hit) {
                Motion::haltJoint(j);
                jogHoming(touchDir, slow);
                beginPhase(H_LOCATE);
            } else if (tooFar) {
                homeFail("endstop stays triggered");
            } else if (stopped) {
                homeFail("aborted");
            }
            break;
        case H_LOCATE:
            if (hit) {
                Motion::haltJoint(j);
                finishTouch(pos);
            } else if (tooFar) {
                homeFail("endstop lost during slow approach");
            } else if (stopped) {
                homeFail("aborted");
            }
            break;
        case H_LEAVE:  // center homing: off the switch, then keep going (H_SEEK) to its other side
            if (!hit) {
                seekGuard = cfg.homingBackoff;
                beginPhase(H_SEEK);
            } else if (tooFar) {
                homeFail("endstop stays triggered");
            } else if (stopped) {
                homeFail("aborted");
            }
            break;
        case H_RETRACT:
            if (stopped) {
                referenced[j] = true;
                updateLimits();
                homePhase = H_NONE;
                syncPlanned();
                notify("homed " + String(j));
                for (int n = 0; n < NUM_JOINTS; ++n) {
                    if (homeQueue & (1u << n)) {
                        homeQueue &= ~(1u << n);
                        startHomingJoint(n);
                        break;
                    }
                }
            }
            break;
        default:
            break;
    }
}

// ---------------- Servo (gripper) ----------------

static void servoWrite(int us) {
    if (us <= 0) {
        ledcWrite(PIN_SERVO, 0);
        return;
    }
    us = constrain(us, 500, 2500);
    ledcWrite(PIN_SERVO, (uint32_t)us * 65535 / 20000);
}

// ---------------- Status ----------------

String statusLine() {
    int32_t p[NUM_JOINTS];
    Motion::getPositions(p);
    const char* state = homePhase != H_NONE  ? "Home"
                        : Motion::anyJogging() ? "Jog"
                        : (!Motion::idle() || qCount) ? "Run"
                                                       : "Idle";
    String s = String("<") + state + " P:";
    for (int j = 0; j < NUM_JOINTS; ++j) s += (j ? "," : "") + String(p[j]);
    s += " Q:" + String(qCount) + " F:" + String(MOVE_QUEUE_LEN - qCount);
    s += String(" E:") + (Motion::enabled() ? "1" : "0") + " L:";
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (ENDSTOP_PIN[j] >= 0) s += endstopHit(j) ? '1' : '0';
    }
    s += " H:";
    for (int j = 0; j < NUM_JOINTS; ++j) s += referenced[j] ? '1' : '0';
    s += " O:";
    for (int j = 0; j < NUM_JOINTS; ++j) s += Motion::powered(j) ? '1' : '0';
    return s + ">";
}

bool machineIdle() { return homePhase == H_NONE && !qCount && Motion::idle(); }

// Switches off joints with joints.jointN.idle_disable once the machine has rested for motion.idle_timeout_ms.
// Positions and home references are kept; the next move/jog/home re-powers every joint.
static void idlePowerTask() {
    static uint32_t busyMs = 0;
    uint32_t now = millis();
    if (!machineIdle()) {
        busyMs = now;
        return;
    }
    if (!cfg.idleTimeoutMs || !Motion::enabled()) return;
    uint32_t rest = now - busyMs;
    uint32_t sinceEnable = now - Motion::enabledAtMs();  // a short move may finish between two loops
    if (sinceEnable < rest) rest = sinceEnable;
    if (rest < cfg.idleTimeoutMs) return;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (cfg.idleDisable[j] && Motion::powered(j)) Motion::powerOff(j);
    }
}

String takeEvents() {
    String e = events;
    events   = "";
    return e;
}

static void haltAll() {
    clearQueue();
    homePhase = H_NONE;
    homeQueue = 0;
    Motion::halt();
    syncPlanned();
}

// ---------------- Command parser ----------------

// Parses up to `max` numbers; returns how many were found, or -1 on garbage.
static int parseNums(const char* s, float* out, int max) {
    int n = 0;
    while (*s) {
        while (*s == ' ' || *s == ',' || *s == '\t') ++s;
        if (!*s) break;
        if (n == max) return -1;
        char* end;
        out[n] = strtof(s, &end);
        if (end == s) return -1;
        ++n;
        s = end;
    }
    return n;
}

static const char* queueMove(const int32_t target[NUM_JOINTS], float speed, float accel, uint32_t ticks) {
    if (!withinLimits(target)) return "limit";
    Move& m = queue[(qHead + qCount) % MOVE_QUEUE_LEN];
    memcpy(m.target, target, sizeof(m.target));
    memcpy(plannedPos, target, sizeof(plannedPos));
    m.speed = speed;
    m.accel = accel;
    m.ticks = ticks;
    ++qCount;
    return "";
}

static const char* handleLine(const char* line) {
    while (*line == ' ') ++line;
    char  cmd = toupper(*line);
    float a[NUM_JOINTS + 2];
    int   n = parseNums(line + (cmd ? 1 : 0), a, NUM_JOINTS + 2);
    if (n < 0) return "bad number";
    bool busyHoming = homePhase != H_NONE;
    int32_t t[NUM_JOINTS];

    switch (cmd) {
        case '!':
            haltAll();
            return "";

        case 'M':  // M p0..p4 [speed [accel]]   absolute
        case 'R': {  // R d0..d4 [speed [accel]] relative to last queued target
            if (n < NUM_JOINTS) return "need 5 joint values";
            if (busyHoming) return "homing";
            if (Motion::anyJogging()) return "jogging";
            if (qCount == MOVE_QUEUE_LEN) return "full";
            if (!qCount && Motion::idle()) syncPlanned();
            for (int j = 0; j < NUM_JOINTS; ++j) t[j] = (int32_t)lroundf(a[j]) + (cmd == 'R' ? plannedPos[j] : 0);
            float speed = n > NUM_JOINTS && a[NUM_JOINTS] > 0 ? a[NUM_JOINTS] : cfg.defaultSpeed;
            float accel = n > NUM_JOINTS + 1 && a[NUM_JOINTS + 1] > 0 ? a[NUM_JOINTS + 1] : cfg.defaultAccel;
            return queueMove(t, speed, accel, 0);
        }

        case 'T': {  // T p0..p4 dt_ms  timed segment
            if (n != NUM_JOINTS + 1) return "usage: T p0 p1 p2 p3 p4 dt_ms";
            float dt = a[NUM_JOINTS];
            if (!(dt > 0 && dt <= 60000)) return "dt_ms must be in (0, 60000]";
            if (busyHoming) return "homing";
            if (Motion::anyJogging()) return "jogging";
            if (qCount == MOVE_QUEUE_LEN) return "full";
            if (!qCount && Motion::idle()) syncPlanned();
            float dmax = 0;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                t[j]    = (int32_t)lroundf(a[j]);
                float d = fabsf((float)t[j] - (float)plannedPos[j]);
                if (d * 1000 / dt > cfg.maxSpeed[j] * 1.02f + 1) return "too fast";
                if (d > dmax) dmax = d;
            }
            if (cfg.blend) {
                uint32_t ticks = (uint32_t)lroundf(dt * STEP_TICK_HZ / 1000);
                return queueMove(t, 0, 0, ticks ? ticks : 1);
            }
            // blend off: plain stop-to-stop move, timing only approximate
            return queueMove(t, dmax > 0 ? dmax * 1000 / dt : cfg.defaultSpeed, cfg.defaultAccel, 0);
        }

        case 'J': {  // J v0..v4 [accel]  velocities in steps/s
            if (n < NUM_JOINTS) return "need 5 joint values";
            if (busyHoming) return "homing";
            if (qCount || (!Motion::idle() && !Motion::anyJogging())) return "moving";
            Motion::jog(a, n > NUM_JOINTS && a[NUM_JOINTS] > 0 ? a[NUM_JOINTS] : cfg.defaultAccel);
            lastJogMs = millis();
            return "";
        }

        case 'S':  // smooth stop, drop queue, cancel homing
            clearQueue();
            homePhase = H_NONE;
            homeQueue = 0;
            Motion::stop();
            return "";

        case 'E':  // E 0|1
            if (n != 1) return "usage: E 0|1";
            if (a[0] == 0) {
                clearQueue();
                homePhase = H_NONE;
                homeQueue = 0;
                Motion::halt();
                for (auto& r : referenced) r = false;  // joints can be moved by hand now
                updateLimits();
            }
            Motion::setEnabled(a[0] != 0);
            syncPlanned();
            return "";

        case 'Z':  // Z [p0..p4]  set current position (default all zero)
            if (!Motion::idle() || qCount || busyHoming) return "not idle";
            if (n != 0 && n != NUM_JOINTS) return "need 0 or 5 values";
            for (int j = 0; j < NUM_JOINTS; ++j) {
                Motion::setPosition(j, n ? (int32_t)lroundf(a[j]) : 0);
                referenced[j] = true;
            }
            updateLimits();
            syncPlanned();
            return "";

        case 'H': {  // H [j]
            if (!cfg.homingEnabled) return "homing disabled (homing.enabled)";
            if (n > 1) return "usage: H [joint]";
            if (!Motion::idle() || qCount || busyHoming) return "not idle";
            uint8_t todo = 0;
            if (n == 1) {
                int j = (int)a[0];
                if (j < 0 || j >= NUM_JOINTS || a[0] != j) return "bad joint";
                if (cfg.homing[j] == HOME_NONE) return "joint has no homing (joints.jointN.homing)";
                todo = 1u << j;
            } else {
                for (int j = 0; j < NUM_JOINTS; ++j) {
                    if (cfg.homing[j] != HOME_NONE) todo |= 1u << j;
                }
                if (!todo) return "no joint has homing (joints.jointN.homing)";
            }
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (todo & (1u << j)) {
                    homeQueue = todo & ~(1u << j);
                    startHomingJoint(j);
                    break;
                }
            }
            return "";
        }

        case 'V':  // V speed accel  defaults for M/R/J
            if (n != 2 || a[0] <= 0 || a[1] <= 0) return "usage: V speed accel";
            cfg.defaultSpeed = a[0];
            cfg.defaultAccel = a[1];
            return "";

        case 'G':  // G us  servo pulse 500..2500, 0 = off
            if (n != 1) return "usage: G us";
            servoWrite((int)a[0]);
            return "";
    }
    return "unknown command";
}

// Runs one command line; returns the reply (empty for a blank line).
String runCommand(const String& line) {
    String l = line;
    l.trim();
    if (l.isEmpty()) return "";
    if (l == "?") return statusLine();
    if (l.equalsIgnoreCase("VER")) return VERSION;
    const char* r = handleLine(l.c_str());
    return *r ? String("err ") + r : String("ok");
}

// Serial-only extras: "CONFIG" + yaml lines + "END" uploads config, "CONFIG?" dumps the stored file,
// "SETTINGS?" dumps the values in use (both end with "END"), "REBOOT".
static bool   cfgReceiving = false;
static String cfgYaml;

static void printBlock(const String& y) {
    Serial.print(y);
    if (!y.endsWith("\n")) Serial.println();
    Serial.println("END");
}

static void handleSerialLine(const char* line) {
    if (cfgReceiving) {
        if (strcmp(line, "END") != 0) {
            cfgYaml += line;
            cfgYaml += '\n';
            return;
        }
        cfgReceiving = false;
        String err;
        if (!machineIdle()) {
            Serial.println("err machine busy, stop first");
        } else if (SettingsStore::save(cfgYaml, err)) {
            updateLimits();
            Serial.println("ok");
        } else {
            Serial.println("err " + err);
        }
        cfgYaml = "";
        return;
    }
    if (!strcmp(line, "CONFIG")) {
        cfgReceiving = true;
        cfgYaml      = "";
        return;
    }
    if (!strcmp(line, "CONFIG?")) return printBlock(SettingsStore::currentYaml());
    if (!strcmp(line, "SETTINGS?")) return printBlock(SettingsStore::effectiveYaml());
    if (!strcmp(line, "REBOOT")) {
        runCommand("E 0");
        Serial.println("ok");
        Serial.flush();
        delay(100);
        ESP.restart();
    }
    String r = runCommand(line);
    if (r.length()) Serial.println(r);
}

static void readSerial() {
    static char buf[256];
    static int  len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '!' && !cfgReceiving) {  // real-time emergency halt, works mid-line
            haltAll();
            Serial.println("# halted");
            continue;
        }
        if (c == '\r' || c == '\n') {
            buf[len] = 0;
            len      = 0;
            handleSerialLine(buf);
        } else if (len < (int)sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
}

// ---------------- Main ----------------

void setup() {
    Motion::begin();  // first: puts the shift register into a safe state
    Motion::setEnabled(true);  // hold the arm from boot on: a reset must not drop it
    Serial.setRxBufferSize(2048);  // whole config.yaml can arrive in one burst
    Serial.begin(SERIAL_BAUD);
    SettingsStore::begin();
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (ENDSTOP_PIN[j] >= 0) pinMode(ENDSTOP_PIN[j], INPUT_PULLUP);
    }
    ledcAttach(PIN_SERVO, 50, 16);
    servoWrite(0);
    syncPlanned();
    updateLimits();
    Web::begin();
    Serial.println("# NanoBee ready");
}

void loop() {
    readSerial();
    Web::loop();
    homingTask();

    // Feed the motion core: timed segments go to its segment buffer while it runs segments,
    // M/R moves start once everything before them has finished.
    while (homePhase == H_NONE && qCount) {
        Move& m = queue[qHead];
        if (m.ticks) {
            if (!Motion::pushSegment(m.target, m.ticks)) break;
        } else {
            if (!Motion::idle()) break;
            Motion::moveTo(m.target, m.speed, m.accel);
        }
        qHead = (qHead + 1) % MOVE_QUEUE_LEN;
        --qCount;
    }

    static uint32_t lastLimitMs = 0;  // limits follow config changes from the web page
    if (millis() - lastLimitMs >= 20) {
        lastLimitMs = millis();
        updateLimits();
    }

    if (Motion::takeUnderrun()) notify("warn segment stream ran dry while moving, decelerating");

    if (cfg.jogTimeoutMs && homePhase == H_NONE && Motion::anyJogging() && millis() - lastJogMs > cfg.jogTimeoutMs) {
        Motion::stop();
    }

    idlePowerTask();
}
