#pragma once
#include <Arduino.h>

// Implemented in main.cpp
String runCommand(const String& line);
String statusLine();
String takeEvents();
bool   machineIdle();

namespace Web {
void begin();
void loop();
}  // namespace Web
