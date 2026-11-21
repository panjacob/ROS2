#pragma once
#include <stdint.h>

// ---- Board: MKS TinyBee (ESP32 + 74HC595 chain on the I2S pins) ----
// Wiring is fixed at compile time; tunables live in settings.cpp / config.yaml.

constexpr int NUM_JOINTS = 5;  // driver slots X, Y, Z, E0, E1

constexpr int PIN_SR_BCK  = 25;  // shift clock
constexpr int PIN_SR_WS   = 26;  // latch
constexpr int PIN_SR_DATA = 27;

// Shift-register bit = Marlin pin number - 128 (see pins_MKS_TINYBEE.h)
constexpr uint8_t EN_BIT[NUM_JOINTS]   = { 0, 3, 6, 9, 12 };
constexpr uint8_t STEP_BIT[NUM_JOINTS] = { 1, 4, 7, 10, 13 };
constexpr uint8_t DIR_BIT[NUM_JOINTS]  = { 2, 5, 8, 11, 14 };
constexpr bool    EN_ACTIVE_LOW        = true;

// Endstop inputs (X, Y, Z connectors); -1 = joint has no endstop
constexpr int ENDSTOP_PIN[NUM_JOINTS] = { 33, 32, 22, -1, -1 };

constexpr int PIN_SERVO = 2;  // servo header (gripper)

constexpr uint32_t STEP_TICK_HZ = 20000;  // ISR rate = max step rate per joint

constexpr uint32_t SERIAL_BAUD    = 115200;
constexpr int      MOVE_QUEUE_LEN = 32;
