#include "settings.h"
#include <LittleFS.h>

Settings cfg;

namespace SettingsStore {

static const char* PATH = "/config.yaml";

static String unquote(String v) {
    if (v.length() >= 2 && (v[0] == '"' || v[0] == '\'') && v[v.length() - 1] == v[0]) {
        return v.substring(1, v.length() - 1);
    }
    return v;
}

static bool toFloat(const String& v, float& out, float lo, float hi) {
    char* end;
    float f = strtof(v.c_str(), &end);
    if (end == v.c_str() || *end || f < lo || f > hi) return false;
    out = f;
    return true;
}

static bool toInt(const String& v, int32_t& out, int32_t lo, int32_t hi) {
    float f;
    if (!toFloat(v, f, lo, hi) || f != (int32_t)f) return false;
    out = (int32_t)f;
    return true;
}

static bool toBool(String v, bool& out) {
    v.toLowerCase();
    if (v == "true" || v == "yes" || v == "on" || v == "1") return out = true, true;
    if (v == "false" || v == "no" || v == "off" || v == "0") return out = false, true;
    return false;
}

// "[a, b, c]" -> items; requires exactly NUM_JOINTS entries.
static bool splitList(const String& v, String items[NUM_JOINTS]) {
    if (!v.startsWith("[") || !v.endsWith("]")) return false;
    String body = v.substring(1, v.length() - 1);
    int    n = 0, start = 0;
    for (int i = 0; i <= (int)body.length(); ++i) {
        if (i == (int)body.length() || body[i] == ',') {
            if (n == NUM_JOINTS) return false;
            items[n] = body.substring(start, i);
            items[n].trim();
            ++n;
            start = i + 1;
        }
    }
    return n == NUM_JOINTS;
}

static bool floatList(const String& v, float out[NUM_JOINTS], float lo, float hi) {
    String it[NUM_JOINTS];
    float  tmp[NUM_JOINTS];
    if (!splitList(v, it)) return false;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!toFloat(it[j], tmp[j], lo, hi)) return false;
    }
    memcpy(out, tmp, sizeof(tmp));
    return true;
}

static bool applyKey(const String& k, const String& raw, Settings& s) {
    String v = unquote(raw);
    String it[NUM_JOINTS];
    float  hz = STEP_TICK_HZ;

    if (k == "wifi.mode") {
        if (v == "ap") return s.wifiSta = false, true;
        if (v == "sta") return s.wifiSta = true, true;
        return false;
    }
    if (k == "wifi.ssid") return v.length() > 0 && v.length() <= 32 && (s.ssid = v, true);
    if (k == "wifi.password") return (v.length() == 0 || (v.length() >= 8 && v.length() <= 63)) && (s.password = v, true);
    if (k == "wifi.hostname") return v.length() > 0 && v.length() <= 32 && (s.hostname = v, true);

    if (k == "motion.default_speed") return toFloat(v, s.defaultSpeed, 1, hz);
    if (k == "motion.default_accel") return toFloat(v, s.defaultAccel, 1, 1e7);
    if (k == "motion.min_speed") return toFloat(v, s.minSpeed, 1, hz);
    if (k == "motion.jog_timeout_ms") {
        int32_t t;
        return toInt(v, t, 0, 60000) && (s.jogTimeoutMs = t, true);
    }
    if (k == "motion.blend") return toBool(v, s.blend);

    if (k == "joints.steps_per_deg") return floatList(v, s.stepsPerDeg, 0.001, 1e6);
    if (k == "joints.max_speed") return floatList(v, s.maxSpeed, 1, hz);
    if (k == "joints.max_accel") return floatList(v, s.maxAccel, 1, 1e7);
    if (k == "joints.dir_invert") {
        bool tmp[NUM_JOINTS];
        if (!splitList(v, it)) return false;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            if (!toBool(it[j], tmp[j])) return false;
        }
        memcpy(s.dirInvert, tmp, sizeof(tmp));
        return true;
    }
    if (k == "joints.homing_dir") {
        int8_t tmp[NUM_JOINTS];
        if (!splitList(v, it)) return false;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            int32_t d;
            if (!toInt(it[j], d, -1, 1) || d == 0) return false;
            tmp[j] = d;
        }
        memcpy(s.homingDir, tmp, sizeof(tmp));
        return true;
    }
    if (k == "joints.homing") {  // none | edge | center per joint
        HomingMode tmp[NUM_JOINTS];
        if (!splitList(v, it)) return false;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            String m = unquote(it[j]);
            if (m == "none") tmp[j] = HOME_NONE;
            else if (m == "edge") tmp[j] = HOME_EDGE;
            else if (m == "center") tmp[j] = HOME_CENTER;
            else return false;
            if (tmp[j] != HOME_NONE && ENDSTOP_PIN[j] < 0) return false;  // no endstop input on that slot
        }
        memcpy(s.homing, tmp, sizeof(tmp));
        return true;
    }
    if (k == "joints.home_offset_deg") return floatList(v, s.homeOffsetDeg, -100000, 100000);
    if (k == "joints.soft_limits") return toBool(v, s.softLimits);
    if (k == "joints.min_deg") return floatList(v, s.minDeg, -100000, 100000);
    if (k == "joints.max_deg") return floatList(v, s.maxDeg, -100000, 100000);

    if (k == "homing.enabled") return toBool(v, s.homingEnabled);
    if (k == "homing.speed") return toFloat(v, s.homingSpeed, 1, hz);
    if (k == "homing.accel") return toFloat(v, s.homingAccel, 1, 1e7);
    if (k == "homing.backoff") return toInt(v, s.homingBackoff, 0, 1000000);
    if (k == "homing.max_travel") return toInt(v, s.homingMaxTravel, 1, 100000000);

    if (k == "endstops.active_low") return toBool(v, s.endstopActiveLow);

    return false;
}

bool parse(const String& yaml, Settings& out, String& err) {
    Settings s = out;
    String   section;
    int      lineNo = 0, pos = 0;

    while (pos <= (int)yaml.length()) {
        int nl = yaml.indexOf('\n', pos);
        if (nl < 0) nl = yaml.length();
        String line = yaml.substring(pos, nl);
        pos         = nl + 1;
        ++lineNo;

        bool inQuote = false;
        for (int i = 0; i < (int)line.length(); ++i) {
            char c = line[i];
            if (c == '"' || c == '\'') inQuote = !inQuote;
            if (c == '#' && !inQuote) {
                line = line.substring(0, i);
                break;
            }
        }
        line.replace("\r", "");
        if (line.indexOf('\t') >= 0) return err = "line " + String(lineNo) + ": tabs not allowed", false;

        int indent = 0;
        while (indent < (int)line.length() && line[indent] == ' ') ++indent;
        line.trim();
        if (line.isEmpty()) continue;

        int colon = line.indexOf(':');
        if (colon <= 0) return err = "line " + String(lineNo) + ": expected 'key: value'", false;
        String key = line.substring(0, colon);
        String val = line.substring(colon + 1);
        key.trim();
        val.trim();

        if (indent == 0) {
            section = val.isEmpty() ? key : "";
            if (val.isEmpty()) continue;
        } else if (section.isEmpty()) {
            return err = "line " + String(lineNo) + ": indented key outside a section", false;
        }
        String full = section.isEmpty() ? key : section + "." + key;
        if (!applyKey(full, val, s)) {
            return err = "line " + String(lineNo) + ": bad or unknown setting '" + full + "'", false;
        }
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (s.minDeg[j] >= s.maxDeg[j]) return err = "joint " + String(j) + ": min_deg must be below max_deg", false;
    }
    out = s;
    return true;
}

static String list(const float* v, int decimals = 0) {
    String r = "[";
    for (int j = 0; j < NUM_JOINTS; ++j) r += (j ? ", " : "") + String(v[j], decimals);
    return r + "]";
}

String toYaml(const Settings& s) {
    String y;
    y += "# NanoBee settings. Units: steps, steps/s, steps/s^2. Lists are per joint J0..J4.\n";
    y += "wifi:\n";
    y += "  mode: " + String(s.wifiSta ? "sta" : "ap") + "   # ap = own network, sta = join existing\n";
    y += "  ssid: \"" + s.ssid + "\"\n";
    y += "  password: \"" + s.password + "\"\n";
    y += "  hostname: " + s.hostname + "\n";
    y += "motion:\n";
    y += "  default_speed: " + String(s.defaultSpeed, 0) + "\n";
    y += "  default_accel: " + String(s.defaultAccel, 0) + "\n";
    y += "  min_speed: " + String(s.minSpeed, 0) + "\n";
    y += "  jog_timeout_ms: " + String(s.jogTimeoutMs) + "\n";
    y += "  blend: " + String(s.blend ? "true" : "false") + "\n";
    y += "joints:\n";
    y += "  steps_per_deg: " + list(s.stepsPerDeg, 4) + "\n";
    y += "  max_speed: " + list(s.maxSpeed) + "\n";
    y += "  max_accel: " + list(s.maxAccel) + "\n";
    y += "  dir_invert: [";
    for (int j = 0; j < NUM_JOINTS; ++j) y += String(j ? ", " : "") + (s.dirInvert[j] ? "true" : "false");
    y += "]\n  homing_dir: [";
    for (int j = 0; j < NUM_JOINTS; ++j) y += String(j ? ", " : "") + String(s.homingDir[j]);
    y += "]\n  homing: [";
    static const char* MODES[] = { "none", "edge", "center" };
    for (int j = 0; j < NUM_JOINTS; ++j) y += String(j ? ", " : "") + MODES[s.homing[j]];
    y += "]\n";
    y += "  home_offset_deg: " + list(s.homeOffsetDeg, 3) + "\n";
    y += "  soft_limits: " + String(s.softLimits ? "true" : "false") + "\n";
    y += "  min_deg: " + list(s.minDeg, 3) + "\n";
    y += "  max_deg: " + list(s.maxDeg, 3) + "\n";
    y += "homing:\n";
    y += "  enabled: " + String(s.homingEnabled ? "true" : "false") + "\n";
    y += "  speed: " + String(s.homingSpeed, 0) + "\n";
    y += "  accel: " + String(s.homingAccel, 0) + "\n";
    y += "  backoff: " + String(s.homingBackoff) + "\n";
    y += "  max_travel: " + String(s.homingMaxTravel) + "\n";
    y += "endstops:\n";
    y += "  active_low: " + String(s.endstopActiveLow ? "true" : "false") + "\n";
    return y;
}

void begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("# err LittleFS mount failed, using defaults");
        return;
    }
    File f = LittleFS.open(PATH, "r");
    if (!f) return;
    String yaml = f.readString();
    f.close();
    String err;
    if (!parse(yaml, cfg, err)) Serial.println("# err config.yaml " + err + ", using defaults");
}

String effectiveYaml() { return toYaml(cfg); }

String currentYaml() {
    File f = LittleFS.open(PATH, "r");
    if (!f) return toYaml(cfg);
    String y = f.readString();
    f.close();
    return y;
}

bool save(const String& yaml, String& err) {
    Settings s;  // parse onto defaults so removed keys fall back to default values
    if (!parse(yaml, s, err)) return false;
    File f = LittleFS.open(PATH, "w");
    if (!f) return err = "cannot write file", false;
    f.print(yaml);
    f.close();
    cfg = s;
    return true;
}

}  // namespace SettingsStore
