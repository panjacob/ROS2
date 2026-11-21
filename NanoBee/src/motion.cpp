// Step generation: one timer ISR computes per-joint trapezoidal ramps (integer only)
// and writes STEP/DIR/EN to the TinyBee's 74HC595 chain by bit-banging the I2S pins.

#include "motion.h"
#include "settings.h"
#include <Arduino.h>
#include <soc/gpio_reg.h>

namespace Motion {

enum Mode : uint8_t { IDLE, POS, VEL };

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

static Joint          joints[NUM_JOINTS];
static portMUX_TYPE   mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t enBits  = 0;
static uint32_t       lastOut    = 0xFFFFFFFF;
static bool           motorsOn   = false;
static hw_timer_t*    timer      = nullptr;

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

static void IRAM_ATTR onTick() {
    uint32_t steps = 0;
    uint32_t out   = enBits;

    portENTER_CRITICAL_ISR(&mux);
    for (int j = 0; j < NUM_JOINTS; ++j) {
        Joint& J = joints[j];
        if (J.mode != IDLE) {
            int64_t v  = J.vel;
            int64_t av0 = iabs64(v);
            int64_t goal;
            int32_t d = 0;

            if (J.mode == POS) {
                d = J.target - J.pos;
                if (d == 0) {
                    J.vel  = 0;
                    J.ramp = 0;
                    J.mode = IDLE;
                    goto dirbit;
                }
                bool toward = (d > 0 && v >= 0) || (d < 0 && v <= 0);
                int32_t ad  = d < 0 ? -d : d;
                goal        = (!toward || ad <= J.ramp) ? 0 : (d > 0 ? J.vmax : -J.vmax);
            } else {
                goal = J.vgoal;
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
    dirbit:
        if (J.dirPos != cfg.dirInvert[j]) out |= 1u << DIR_BIT[j];
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
    if (vel == 0 && J.mode == IDLE) {
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
    for (int j = 0; j < NUM_JOINTS; ++j) haltJoint(j);
}

bool jointIdle(int j) {
    portENTER_CRITICAL(&mux);
    bool r = joints[j].mode == IDLE;
    portEXIT_CRITICAL(&mux);
    return r;
}

bool idle() {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!jointIdle(j)) return false;
    }
    return true;
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
