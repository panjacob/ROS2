#include <cassert>
#include <cmath>
#include "../src/motion.cpp"

Settings cfg;

static void tick(unsigned count) {
    while (count--) Motion::advance();
}

int main() {
    cfg.minSpeed = 100;
    cfg.maxSpeed[0] = 1000;
    cfg.maxAccel[0] = 10000;

    // At 1% of 1000 steps/s, the 100 steps/s start speed must not take over.
    Motion::jogJoint(0, 10, 10000);
    tick(STEP_TICK_HZ * 2);
    assert(Motion::joints[0].pos >= 19 && Motion::joints[0].pos <= 20);
    assert(Motion::joints[0].vel == Motion::toQ(10));

    // Updating a running jog, reversing, and stopping must also honor the requested speed.
    Motion::jogJoint(0, 200, 10000);
    tick(STEP_TICK_HZ);
    assert(Motion::joints[0].vel == Motion::toQ(200));
    Motion::jogJoint(0, 10, 10000);
    tick(STEP_TICK_HZ);
    assert(Motion::joints[0].vel == Motion::toQ(10));
    Motion::jogJoint(0, -10, 10000);
    tick(STEP_TICK_HZ);
    assert(Motion::joints[0].vel == -Motion::toQ(10));
    Motion::jogJoint(0, 0, 10000);
    tick(STEP_TICK_HZ);
    assert(Motion::jointIdle(0));

    Motion::setPosition(0, 0);
    Motion::setLimit(0, true, -5, 5);
    Motion::jogJoint(0, 10, 10000);
    tick(STEP_TICK_HZ * 2);
    assert(Motion::joints[0].pos <= 5);
    assert(Motion::jointIdle(0));
}
