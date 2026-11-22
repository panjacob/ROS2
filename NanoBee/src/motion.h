#pragma once
#include <stdint.h>
#include "config.h"

namespace Motion {

void begin();

void setEnabled(bool on);  // arm/disarm all drivers; enabling re-powers joints switched off by powerOff
bool enabled();            // armed (E 1), even if some joints are powered off while idle
uint32_t enabledAtMs();    // millis() of the last setEnabled(true)

// Switches off the driver of a joint while the whole machine is at rest (keeps position).
// Any later move/jog re-powers all joints. Returns false if anything is moving or queued.
bool powerOff(int j);
bool powered(int j);

// Synchronized absolute move in steps; speed/accel apply to the joint with the longest travel.
void moveTo(const int32_t target[NUM_JOINTS], float speed, float accel);

// Timed segment: every joint moves at constant speed to `target` in `ticks` ISR ticks.
// Segments queued back to back run without stopping in between (blending). If the buffer runs
// empty while moving, joints decelerate at max_accel. Returns false when the buffer is full or a
// move/jog is running.
bool pushSegment(const int32_t target[NUM_JOINTS], uint32_t ticks);
int  segmentFree();
bool takeUnderrun();  // true once after the segment stream ended while still moving

// Per-joint velocity in steps/s (signed).
void jog(const float vel[NUM_JOINTS], float accel);
void jogJoint(int j, float vel, float accel);

// Jogs decelerate to a stop before leaving [lo, hi] (steps) while `on`.
void setLimit(int j, bool on, int32_t lo, int32_t hi);

void stop();             // decelerate all joints to zero, drop queued segments
void halt();             // stop all joints immediately, drop queued segments
void haltJoint(int j);

bool idle();
bool jointIdle(int j);
bool anyJogging();

void getPositions(int32_t out[NUM_JOINTS]);
void setPosition(int j, int32_t pos);

}  // namespace Motion