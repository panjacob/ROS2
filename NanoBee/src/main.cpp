// NanoBee: minimal robotic-arm firmware for the MKS TinyBee.
// Commands over serial (115200) or HTTP GET /cmd?c=<line>; all units in steps.
// Reply: "ok" or "err <reason>". Tunables: config.yaml (see settings.cpp), editable from the web page.
//   M p0 p1 p2 p3 p4 [speed [accel]]  queue synchronized absolute move
//   R d0 d1 d2 d3 d4 [speed [accel]]  queue move relative to last queued target
//   J v0 v1 v2 v3 v4 [accel]          jog velocities (steps/s); repeat within jog_timeout_ms
//   S                                 decelerate to stop, clear queue
//   !                                 instant halt (real-time on serial, no newline needed)
//   E 0|1                             disable / enable drivers
//   Z [p0 p1 p2 p3 p4]                set current position (default 0)
//   H j                               home joint j (0..2) on its endstop
//   V speed accel                     default speed/accel (until reboot)
//   G us                              servo pulse on GPIO2 (500..2500, 0 = off)
//   ?                                 <State P:p0,..,p4 Q:queued E:enabled L:endstops>

#include <Arduino.h>
#include "config.h"
#include "motion.h"
#include "settings.h"
#include "web.h"

struct Move {
    int32_t target[NUM_JOINTS];
    float   speed;
    float   accel;
};

static Move    queue[MOVE_QUEUE_LEN];
static int     qHead = 0, qCount = 0;
static int32_t plannedPos[NUM_JOINTS];  // end position of the last queued move

static uint32_t lastJogMs = 0;
static String   events;  // async messages not yet fetched by the web page

static void notify(const String& msg) {
    Serial.println(msg);
    if (events.length() < 1024) events += msg + "\n";
}

// ---------------- Endstops ----------------

static bool endstopHit(int j) {
    if (ENDSTOP_PIN[j] < 0) return false;
    return digitalRead(ENDSTOP_PIN[j]) == (cfg.endstopActiveLow ? LOW : HIGH);
}

// ---------------- Homing ----------------

enum HomePhase { H_NONE, H_SEEK, H_BACKOFF, H_LOCATE, H_RETRACT };
static HomePhase homePhase = H_NONE;
static int       homeJoint = -1;
static int32_t   homeStart = 0;

static void syncPlanned() { Motion::getPositions(plannedPos); }

static void clearQueue() {
    qHead = qCount = 0;
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
    syncPlanned();
    notify("err home " + String(homeJoint) + ": " + why);
}

static void homingTask() {
    if (homePhase == H_NONE) return;
    int     j   = homeJoint;
    int32_t pos[NUM_JOINTS];
    Motion::getPositions(pos);
    int32_t travel = pos[j] - homeStart;
    if (travel < 0) travel = -travel;

    switch (homePhase) {
        case H_SEEK:
        case H_LOCATE:
            if (endstopHit(j)) {
                Motion::haltJoint(j);
                if (homePhase == H_SEEK) {
                    moveJoint(j, pos[j] - cfg.homingDir[j] * cfg.homingBackoff, cfg.homingSpeed);
                    homePhase = H_BACKOFF;
                } else {
                    Motion::setPosition(j, 0);
                    moveJoint(j, -cfg.homingDir[j] * cfg.homingBackoff, cfg.homingSpeed);
                    homePhase = H_RETRACT;
                }
            } else if (travel > cfg.homingMaxTravel) {
                homeFail("endstop not found");
            } else if (Motion::jointIdle(j)) {
                homeFail("aborted");
            }
            break;
        case H_BACKOFF:
            if (Motion::jointIdle(j)) {
                if (endstopHit(j)) {
                    homeFail("endstop still triggered after backoff");
                    break;
                }
                homeStart = pos[j];
                Motion::jogJoint(j, cfg.homingDir[j] * cfg.homingSpeed / 5, cfg.homingAccel);
                homePhase = H_LOCATE;
            }
            break;
        case H_RETRACT:
            if (Motion::jointIdle(j)) {
                homePhase = H_NONE;
                syncPlanned();
                notify("homed " + String(j));
            }
            break;
        default:
            break;
    }
}

static bool startHoming(int j) {
    if (j < 0 || j >= NUM_JOINTS || ENDSTOP_PIN[j] < 0) return false;
    homeJoint = j;
    int32_t pos[NUM_JOINTS];
    Motion::getPositions(pos);
    homeStart = pos[j];
    if (endstopHit(j)) {
        moveJoint(j, pos[j] - cfg.homingDir[j] * cfg.homingBackoff, cfg.homingSpeed);
        homePhase = H_BACKOFF;
    } else {
        Motion::jogJoint(j, cfg.homingDir[j] * cfg.homingSpeed, cfg.homingAccel);
        homePhase = H_SEEK;
    }
    return true;
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
    s += " Q:" + String(qCount) + " E:" + (Motion::enabled() ? "1" : "0") + " L:";
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (ENDSTOP_PIN[j] >= 0) s += endstopHit(j) ? '1' : '0';
    }
    return s + ">";
}

bool machineIdle() { return homePhase == H_NONE && !qCount && Motion::idle(); }

String takeEvents() {
    String e = events;
    events   = "";
    return e;
}

static void haltAll() {
    clearQueue();
    homePhase = H_NONE;
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

static const char* handleLine(const char* line) {
    while (*line == ' ') ++line;
    char  cmd = toupper(*line);
    float a[NUM_JOINTS + 2];
    int   n = parseNums(line + (cmd ? 1 : 0), a, NUM_JOINTS + 2);
    if (n < 0) return "bad number";
    bool busyHoming = homePhase != H_NONE;

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
            Move& m = queue[(qHead + qCount) % MOVE_QUEUE_LEN];
            for (int j = 0; j < NUM_JOINTS; ++j) {
                m.target[j] = (int32_t)lroundf(a[j]) + (cmd == 'R' ? plannedPos[j] : 0);
                plannedPos[j] = m.target[j];
            }
            m.speed = n > NUM_JOINTS && a[NUM_JOINTS] > 0 ? a[NUM_JOINTS] : cfg.defaultSpeed;
            m.accel = n > NUM_JOINTS + 1 && a[NUM_JOINTS + 1] > 0 ? a[NUM_JOINTS + 1] : cfg.defaultAccel;
            ++qCount;
            return "";
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
            Motion::stop();
            return "";

        case 'E':  // E 0|1
            if (n != 1) return "usage: E 0|1";
            if (a[0] == 0) {
                clearQueue();
                homePhase = H_NONE;
                Motion::halt();
            }
            Motion::setEnabled(a[0] != 0);
            syncPlanned();
            return "";

        case 'Z':  // Z [p0..p4]  set current position (default all zero)
            if (!Motion::idle() || qCount || busyHoming) return "not idle";
            if (n != 0 && n != NUM_JOINTS) return "need 0 or 5 values";
            for (int j = 0; j < NUM_JOINTS; ++j) Motion::setPosition(j, n ? (int32_t)lroundf(a[j]) : 0);
            syncPlanned();
            return "";

        case 'H':  // H j
            if (n != 1) return "usage: H joint";
            if (!Motion::idle() || qCount || busyHoming) return "not idle";
            return startHoming((int)a[0]) ? "" : "joint has no endstop";

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
    const char* r = handleLine(l.c_str());
    return *r ? String("err ") + r : String("ok");
}

// Serial-only extras: "CONFIG" + yaml lines + "END" uploads config, "CONFIG?" dumps it (ends with "END"), "REBOOT".
static bool   cfgReceiving = false;
static String cfgYaml;

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
    if (!strcmp(line, "CONFIG?")) {
        String y = SettingsStore::currentYaml();
        Serial.print(y);
        if (!y.endsWith("\n")) Serial.println();
        Serial.println("END");
        return;
    }
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
            Serial.println("halted");
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
    Serial.setRxBufferSize(2048);  // whole config.yaml can arrive in one burst
    Serial.begin(SERIAL_BAUD);
    SettingsStore::begin();
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (ENDSTOP_PIN[j] >= 0) pinMode(ENDSTOP_PIN[j], INPUT_PULLUP);
    }
    ledcAttach(PIN_SERVO, 50, 16);
    servoWrite(0);
    syncPlanned();
    Web::begin();
    Serial.println("NanoBee ready");
}

void loop() {
    readSerial();
    Web::loop();
    homingTask();

    if (homePhase == H_NONE && qCount && Motion::idle()) {
        Move& m = queue[qHead];
        Motion::moveTo(m.target, m.speed, m.accel);
        qHead = (qHead + 1) % MOVE_QUEUE_LEN;
        --qCount;
    }

    if (cfg.jogTimeoutMs && homePhase == H_NONE && Motion::anyJogging() && millis() - lastJogMs > cfg.jogTimeoutMs) {
        Motion::stop();
    }
}
