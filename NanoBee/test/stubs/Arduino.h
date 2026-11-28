#pragma once
#include <cmath>
#include <cstdint>
#include <string>

using String = std::string;
using portMUX_TYPE = int;
struct hw_timer_t {};
#define IRAM_ATTR
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(...)
#define portEXIT_CRITICAL(...)
#define portENTER_CRITICAL_ISR(...)
#define portEXIT_CRITICAL_ISR(...)
#define OUTPUT 1
inline void pinMode(int, int) {}
inline uint32_t millis() { return 0; }
inline hw_timer_t* timerBegin(int) { return nullptr; }
inline void timerAttachInterrupt(hw_timer_t*, void (*)()) {}
inline void timerAlarm(hw_timer_t*, int, bool, int) {}
