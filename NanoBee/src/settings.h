#pragma once
#include <Arduino.h>
#include "config.h"

// Runtime-tunable settings, loaded from /config.yaml on LittleFS.
struct Settings {
    // wifi
    bool   wifiSta  = false;  // false = own access point
    String ssid     = "NanoBee";
    String password = "nanobee123";  // AP needs >= 8 chars, empty = open
    String hostname = "nanobee";

    // motion (steps, steps/s, steps/s^2)
    float    defaultSpeed = 1000;
    float    defaultAccel = 2000;
    float    minSpeed     = 100;  // start/stop speed
    uint32_t jogTimeoutMs = 500;  // 0 = off

    // joints
    float  stepsPerDeg[NUM_JOINTS] = { 8.8889, 8.8889, 8.8889, 8.8889, 8.8889 };  // not used by firmware, for clients
    float  maxSpeed[NUM_JOINTS]  = { 5000, 5000, 5000, 5000, 5000 };
    float  maxAccel[NUM_JOINTS]  = { 10000, 10000, 10000, 10000, 10000 };
    bool   dirInvert[NUM_JOINTS] = { false, false, false, false, false };
    int8_t homingDir[NUM_JOINTS] = { -1, -1, -1, -1, -1 };

    // homing
    float   homingSpeed     = 500;
    float   homingAccel     = 2000;
    int32_t homingBackoff   = 200;
    int32_t homingMaxTravel = 200000;

    bool endstopActiveLow = true;  // matches stock MKS Marlin config
};

extern Settings cfg;

namespace SettingsStore {

void begin();  // mount LittleFS and load /config.yaml if present

// Parses YAML into `out` (starting from current values). On failure returns false and sets err.
bool   parse(const String& yaml, Settings& out, String& err);
String toYaml(const Settings& s);

String currentYaml();  // file contents, or generated from cfg if there is no file
bool   save(const String& yaml, String& err);  // validate, store and apply

}  // namespace SettingsStore
