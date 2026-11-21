#pragma once
#include <stdint.h>
#include "config.h"

namespace Motion {

void begin();

void setEnabled(bool on);
bool enabled();

// Synchronized absolute move in steps; speed/accel apply to the joint with the longest travel.
void moveTo(const int32_t target[NUM_JOINTS], float speed, float accel);

// Per-joint velocity in steps/s (signed).
void jog(const float vel[NUM_JOINTS], float accel);
void jogJoint(int j, float vel, float accel);

void stop();             // decelerate all joints to zero
void halt();             // stop all joints immediately
void haltJoint(int j);

bool idle();
bool jointIdle(int j);
bool anyJogging();

void getPositions(int32_t out[NUM_JOINTS]);
void setPosition(int j, int32_t pos);

}  // namespace Motion
