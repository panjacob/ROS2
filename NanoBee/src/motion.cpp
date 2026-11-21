// Step generation: one timer ISR computes per-joint trapezoidal ramps (integer only)
// and writes STEP/DIR/EN to the TinyBee's 74HC595 chain by bit-banging the I2S pins.
// Timed segments (T command) run all joints at constant speed with Bresenham stepping and
// hand over to the next queued segment in the same tick, so a stream of segments never stops.

#include "motion.h"
#include "settings.h"
#include <Arduino.h>
#include <soc/gpio_reg.h>

namespace Motion {

enum Mode : uint8_t { IDLE, POS, VEL, SEG };

// Velocities are Q32.32 steps per tick: 1 << 32 == one step every tick.
static constexpr int64_t ONE_STEP = 1LL << 32;

struct Joint {
    Mode     mode;
    int32_t  pos;
    int32_t  target;  // POS mode
    int64_t  vgoal;   // VEL mode, signed
    int64_t  vmax;
    int64_t  vmin;
    int64_t  da;      // velocity change per tick
    int64_t  vel;     // signed current velocity
    uint32_t frac;    // step phase accumulator
    int32_t  ramp;    // steps spent accelerating = steps needed to stop
    bool     dirPos;
};

struct Segment {
    int32_t  delta[NUM_JOINTS];
    int64_t  vel[NUM_JOINTS];  // Q32.32, used to decelerate if the stream ends while moving
    uint32_t ticks;
};

static constexpr int SEG_LEN = 16;

static Joint          joints[NUM_JOINTS];
static portMUX_TYPE   mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t enBits  = 0;
static uint32_t       lastOut    = 0xFFFFFFFF;
static bool           motorsOn   = false;
static hw_timer_t*    timer      = nullptr;

static Segment  segBuf[SEG_LEN];
static int      segHead = 0, segCount = 0;
static bool     segRun  = false;
static Segment  segCur;
static uint32_t segT = 0;
static uint32_t segErr[NUM_JOINTS];
static int32_t  segEnd[NUM_JOINTS];    // position at the end of the last queued segment
static int64_t  stopDa[NUM_JOINTS];    // deceleration used when a segment stream stops
static int64_t  stopVmin[NUM_JOINTS];
static bool     underrun = false;

static bool    limOn[NUM_JOINTS];
static int32_t limLo[NUM_JOINTS], limHi[NUM_JOINTS];

static constexpr uint32_t BCK  = 1u << PIN_SR_BCK;
static constexpr uint32_t WS   = 1u << PIN_SR_WS;
static constexpr uint32_t DATA = 1u << PIN_SR_DATA;

static inline int64_t iabs64(int64_t v) { return v < 0 ? -v : v; }

// Same bit order as FluidNC's i2s_out shift-out: bit 31 first, latch on WS rising edge.
static void IRAM_ATTR shiftOut(uint32_t v) {
    REG_WRITE(GPIO_OUT_W1TC_REG, WS);
    for (int i = 31; i >= 0; --i) {
        REG_WRITE((v >> i) & 1 ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, DATA);
        REG_WRITE(GPIO_OUT_W1TS_REG, BCK);
        REG_WRITE(GPIO_OUT_W1TC_REG, BCK);
    }
    REG_WRITE(GPIO_OUT_W1TS_REG, WS);
}

// Must hold mux.
static void IRAM_ATTR startNextSegment() {
    segCur = segBuf[segHead];
    segHead = (segHead + 1) % SEG_LEN;
    --segCount;
    segT   = 0;
    segRun = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        segErr[j]      = segCur.ticks / 2;
        joints[j].mode = SEG;
    }
}

// Must hold mux. Leaves segment mode; joints still moving decelerate at their max_accel.
static void IRAM_ATTR endSegments(bool flagUnderrun) {
    segRun = false;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        Joint&  J = joints[j];
        int64_t v = segCur.vel[j];
        J.ramp    = 0;
        if (iabs64(v) >= stopVmin[j]) {
            if (flagUnderrun) underrun = true;
            J.vel   = v;
            J.vgoal = 0;
            J.da    = stopDa[j];
            J.vmin  = stopVmin[j];
            J.mode  = VEL;
        } else {
            J.vel  = 0;
            J.mode = IDLE;
        }
    }
}

static void IRAM_ATTR onTick() {
    uint32_t steps = 0;
    uint32_t out   = enBits;

    portENTER_CRITICAL_ISR(&mux);
    if (!segRun && segCount) startNextSegment();

    if (segRun) {
        for (int j = 0; j < NUM_JOINTS; ++j) {
            Joint&  J = joints[j];
            int32_t d = segCur.delta[j];
            if (d) {
                J.dirPos = d > 0;
                segErr[j] += d > 0 ? d : -d;
                if (segErr[j] >= segCur.ticks) {
                    segErr[j] -= segCur.ticks;
                    J.pos += J.dirPos ? 1 : -1;
                    steps |= 1u << STEP_BIT[j];
                }
            }
        }
        if (++segT >= segCur.ticks) {
            if (segCount) {
                startNextSegment();
            } else {
                endSegments(true);
            }
        }
    } else {
        for (int j = 0; j < NUM_JOINTS; ++j) {
            Joint& J = joints[j];
            if (J.mode != POS && J.mode != VEL) continue;
            int64_t v   = J.vel;
            int64_t av0 = iabs64(v);
            int64_t goal;
            int32_t d = 0;

            if (J.mode == POS) {
                d = J.target - J.pos;
                if (d == 0) {
                    J.vel  = 0;
                    J.ramp = 0;
                    J.mode = IDLE;
                    continue;
                }
                bool toward = (d > 0 && v >= 0) || (d < 0 && v <= 0);
                int32_t ad  = d < 0 ? -d : d;
                goal        = (!toward || ad <= J.ramp) ? 0 : (d > 0 ? J.vmax : -J.vmax);
            } else {
                goal = J.vgoal;
                // Soft limits: start braking when the remaining room equals the stopping distance.
                if (limOn[j]) {
                    if (goal > 0 && limHi[j] - J.pos <= J.ramp + 1) goal = 0;
                    if (goal < 0 && J.pos - limLo[j] <= J.ramp + 1) goal = 0;
                }
            }

            if (v < goal) {
                v = (v + J.da < goal) ? v + J.da : goal;
            } else if (v > goal) {
                v = (v - J.da > goal) ? v - J.da : goal;
            }

            // Never crawl below the start/stop speed while there is somewhere to go.
            if (J.mode == POS) {
                if (d > 0 && v >= 0 && v < J.vmin) v = J.vmin;
                if (d < 0 && v <= 0 && v > -J.vmin) v = -J.vmin;
            } else if (goal > 0 && v >= 0 && v < J.vmin) {
                v = J.vmin;
            } else if (goal < 0 && v <= 0 && v > -J.vmin) {
                v = -J.vmin;
            } else if (goal == 0 && iabs64(v) < J.vmin) {
                v = 0;
            }

            if (v > 0) J.dirPos = true;
            if (v < 0) J.dirPos = false;

            int64_t  av = iabs64(v);
            uint64_t s  = (uint64_t)J.frac + (uint64_t)av;
            J.frac      = (uint32_t)s;
            if ((s >> 32) && v != 0) {
                J.pos += J.dirPos ? 1 : -1;
                steps |= 1u << STEP_BIT[j];
                if (av > av0) {
                    J.ramp++;
                } else if (av < av0 && J.ramp > 0) {
                    J.ramp--;
                }
            }
            J.vel = v;
            if (J.mode == VEL && v == 0 && goal == 0) {
                J.ramp = 0;
                J.mode = IDLE;
            }
        }
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (joints[j].dirPos != cfg.dirInvert[j]) out |= 1u << DIR_BIT[j];
    }
    portEXIT_CRITICAL_ISR(&mux);

    // Direction/enable changes go out first, giving DIR setup time before the STEP edge.
    if (out != lastOut) {
        shiftOut(out);
        lastOut = out;
    }
    if (steps) {
        shiftOut(out | steps);  // pulse width = one shift (~several us)
        shiftOut(out);
    }
}

static int64_t toQ(float stepsPerSec) {
    float q = stepsPerSec / STEP_TICK_HZ * (float)ONE_STEP;
    if (q >= (float)ONE_STEP) return ONE_STEP - 1;
    if (q < 1) return 1;
    return (int64_t)q;
}

static int64_t toQaccel(float stepsPerSec2) {
    float q = stepsPerSec2 / ((float)STEP_TICK_HZ * STEP_TICK_HZ) * (float)ONE_STEP;
    return q < 1 ? 1 : (int64_t)q;
}

void begin() {
    pinMode(PIN_SR_BCK, OUTPUT);
    pinMode(PIN_SR_WS, OUTPUT);
    pinMode(PIN_SR_DATA, OUTPUT);
    setEnabled(false);
    shiftOut(enBits);  // also forces heaters/fans/beeper off
    lastOut = enBits;

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTick);
    timerAlarm(timer, 1000000 / STEP_TICK_HZ, true, 0);
}

void setEnabled(bool on) {
    uint32_t bits = 0;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (on != EN_ACTIVE_LOW) bits |= 1u << EN_BIT[j];
    }
    motorsOn = on;
    enBits   = bits;
}

bool enabled() { return motorsOn; }

void moveTo(const int32_t target[NUM_JOINTS], float speed, float accel) {
    int32_t cur[NUM_JOINTS];
    getPositions(cur);

    float dist[NUM_JOINTS];
    float dmax = 0;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        dist[j] = fabsf((float)target[j] - (float)cur[j]);
        if (dist[j] > dmax) dmax = dist[j];
    }
    if (dmax == 0) return;

    // Scale down so no joint exceeds its own limits; all joints keep the same timing.
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (dist[j] == 0) continue;
        float f = dist[j] / dmax;
        if (speed * f > cfg.maxSpeed[j]) speed = cfg.maxSpeed[j] / f;
        if (accel * f > cfg.maxAccel[j]) accel = cfg.maxAccel[j] / f;
    }
    if (speed > STEP_TICK_HZ) speed = STEP_TICK_HZ;
    float vmin = cfg.minSpeed < speed ? cfg.minSpeed : speed;

    setEnabled(true);
    portENTER_CRITICAL(&mux);
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (dist[j] == 0) continue;
        float   f = dist[j] / dmax;
        Joint&  J = joints[j];
        J.target  = target[j];
        J.vmax    = toQ(speed * f);
        J.vmin    = toQ(vmin * f);
        J.da      = toQaccel(accel * f);
        J.mode    = POS;
    }
    portEXIT_CRITICAL(&mux);
}

bool pushSegment(const int32_t target[NUM_JOINTS], uint32_t ticks) {
    if (ticks == 0) ticks = 1;
    int64_t da[NUM_JOINTS], vmin[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; ++j) {
        da[j]   = toQaccel(cfg.maxAccel[j]);
        vmin[j] = toQ(cfg.minSpeed);
    }

    portENTER_CRITICAL(&mux);
    bool busy = segCount == SEG_LEN;
    for (auto& J : joints) busy |= J.mode == POS || J.mode == VEL;
    if (busy) {
        portEXIT_CRITICAL(&mux);
        return false;
    }
    if (!segRun && !segCount) {
        for (int j = 0; j < NUM_JOINTS; ++j) segEnd[j] = joints[j].pos;
    }
    Segment& s = segBuf[(segHead + segCount) % SEG_LEN];
    for (int j = 0; j < NUM_JOINTS; ++j) {
        s.delta[j]   = target[j] - segEnd[j];
        uint32_t ad = s.delta[j] < 0 ? -s.delta[j] : s.delta[j];
        if (ad > ticks) ticks = ad;  // at most one step per tick
    }
    s.ticks = ticks;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        s.vel[j]    = ((int64_t)s.delta[j] << 32) / (int64_t)ticks;
        segEnd[j]   = target[j];
        stopDa[j]   = da[j];
        stopVmin[j] = vmin[j];
    }
    ++segCount;
    portEXIT_CRITICAL(&mux);
    setEnabled(true);
    return true;
}

int segmentFree() {
    portENTER_CRITICAL(&mux);
    int n = SEG_LEN - segCount;
    portEXIT_CRITICAL(&mux);
    return n;
}

bool takeUnderrun() {
    portENTER_CRITICAL(&mux);
    bool u   = underrun;
    underrun = false;
    portEXIT_CRITICAL(&mux);
    return u;
}

void setLimit(int j, bool on, int32_t lo, int32_t hi) {
    if (j < 0 || j >= NUM_JOINTS) return;
    portENTER_CRITICAL(&mux);
    limOn[j] = on;
    limLo[j] = lo;
    limHi[j] = hi;
    portEXIT_CRITICAL(&mux);
}

void jogJoint(int j, float vel, float accel) {
    if (j < 0 || j >= NUM_JOINTS) return;
    float lim = cfg.maxSpeed[j] < STEP_TICK_HZ ? cfg.maxSpeed[j] : STEP_TICK_HZ;
    if (vel > lim) vel = lim;
    if (vel < -lim) vel = -lim;
    if (accel > cfg.maxAccel[j]) accel = cfg.maxAccel[j];
    if (vel != 0) setEnabled(true);

    int64_t q = vel == 0 ? 0 : toQ(fabsf(vel));
    portENTER_CRITICAL(&mux);
    Joint& J = joints[j];
    if ((vel == 0 && J.mode == IDLE) || J.mode == SEG) {
        portEXIT_CRITICAL(&mux);
        return;
    }
    J.vgoal = vel < 0 ? -q : q;
    J.vmin  = toQ(cfg.minSpeed);
    J.da    = toQaccel(accel);
    J.mode  = VEL;
    portEXIT_CRITICAL(&mux);
}

void jog(const float vel[NUM_JOINTS], float accel) {
    for (int j = 0; j < NUM_JOINTS; ++j) jogJoint(j, vel[j], accel);
}

void stop() {
    portENTER_CRITICAL(&mux);
    segCount = 0;
    if (segRun) endSegments(false);
    for (auto& J : joints) {
        if (J.mode != IDLE) {
            J.vgoal = 0;
            J.mode  = VEL;
        }
    }
    portEXIT_CRITICAL(&mux);
}

void haltJoint(int j) {
    portENTER_CRITICAL(&mux);
    Joint& J = joints[j];
    J.mode   = IDLE;
    J.vel    = 0;
    J.ramp   = 0;
    portEXIT_CRITICAL(&mux);
}

void halt() {
    portENTER_CRITICAL(&mux);
    segCount = 0;
    segRun   = false;
    portEXIT_CRITICAL(&mux);
    for (int j = 0; j < NUM_JOINTS; ++j) haltJoint(j);
}

bool jointIdle(int j) {
    portENTER_CRITICAL(&mux);
    bool r = joints[j].mode == IDLE;
    portEXIT_CRITICAL(&mux);
    return r;
}

bool idle() {
    portENTER_CRITICAL(&mux);
    bool r = !segRun && !segCount;
    for (auto& J : joints) r &= J.mode == IDLE;
    portEXIT_CRITICAL(&mux);
    return r;
}

bool anyJogging() {
    bool r = false;
    portENTER_CRITICAL(&mux);
    for (auto& J : joints) r |= J.mode == VEL;
    portEXIT_CRITICAL(&mux);
    return r;
}

void getPositions(int32_t out[NUM_JOINTS]) {
    portENTER_CRITICAL(&mux);
    for (int j = 0; j < NUM_JOINTS; ++j) out[j] = joints[j].pos;
    portEXIT_CRITICAL(&mux);
}

void setPosition(int j, int32_t pos) {
    portENTER_CRITICAL(&mux);
    joints[j].pos    = pos;
    joints[j].target = pos;
    portEXIT_CRITICAL(&mux);
}

}  // namespace Motion
