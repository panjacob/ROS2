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

// "[a, b, c]" -> items; requires exactly NUM_JOINTS entries. Only for the old list-per-setting form.
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

// One setting of joint j (firmware index, config block "joint<j+1>").
static bool applyJointKey(int j, const String& k, const String& v, Settings& s) {
    float hz = STEP_TICK_HZ;
    if (k == "steps_per_deg") return toFloat(v, s.stepsPerDeg[j], 0.001, 1e6);
    if (k == "max_speed") return toFloat(v, s.maxSpeed[j], 1, hz);
    if (k == "max_accel") return toFloat(v, s.maxAccel[j], 1, 1e7);
    if (k == "dir_invert") return toBool(v, s.dirInvert[j]);
    if (k == "idle_disable") return toBool(v, s.idleDisable[j]);
    if (k == "homing_dir") {
        int32_t d;
        if (!toInt(v, d, -1, 1) || d == 0) return false;
        s.homingDir[j] = d;
        return true;
    }
    if (k == "homing") {  // none | edge | center
        HomingMode m;
        if (v == "none") m = HOME_NONE;
        else if (v == "edge") m = HOME_EDGE;
        else if (v == "center") m = HOME_CENTER;
        else return false;
        if (m != HOME_NONE && ENDSTOP_PIN[j] < 0) return false;  // no endstop input on that slot
        s.homing[j] = m;
        return true;
    }
    if (k == "home_offset_deg") return toFloat(v, s.homeOffsetDeg[j], -100000, 100000);
    if (k == "homing_speed") return toFloat(v, s.jointHomingSpeed[j], 0, hz);
    if (k == "homing_max_travel") return toInt(v, s.jointHomingMaxTravel[j], 0, 100000000);
    if (k == "min_deg") return toFloat(v, s.minDeg[j], -100000, 100000);
    if (k == "max_deg") return toFloat(v, s.maxDeg[j], -100000, 100000);
    return false;
}

static bool applyKey(const String& k, const String& raw, Settings& s) {
    String v = unquote(raw);
    float  hz = STEP_TICK_HZ;

    // joints.joint<N>.<key>, N = 1..NUM_JOINTS
    static const String JOINT_PREFIX = "joints.joint";
    if (k.startsWith(JOINT_PREFIX)) {
        int dot = k.indexOf('.', JOINT_PREFIX.length());
        if (dot < 0) return false;
        String num = k.substring(JOINT_PREFIX.length(), dot);
        int    n   = num.toInt();
        if (num != String(n) || n < 1 || n > NUM_JOINTS) return false;
        return applyJointKey(n - 1, k.substring(dot + 1), v, s);
    }
    // Old form, one list per setting: joints.<key>: [j0, j1, ...]
    if (k.startsWith("joints.") && v.startsWith("[")) {
        String   it[NUM_JOINTS];
        Settings tmp = s;
        if (!splitList(v, it)) return false;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            if (!applyJointKey(j, k.substring(7), unquote(it[j]), tmp)) return false;
        }
        s = tmp;
        return true;
    }

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
    if (k == "motion.idle_timeout_ms") {
        int32_t t;
        return toInt(v, t, 0, 3600000) && (s.idleTimeoutMs = t, true);
    }

    if (k == "joints.soft_limits") return toBool(v, s.softLimits);

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
    static constexpr int MAX_DEPTH = 3;  // e.g. joints > joint1 > key
    String   path[MAX_DEPTH];
    int      pathIndent[MAX_DEPTH];
    int      depth  = 0;
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

        while (depth && pathIndent[depth - 1] >= indent) --depth;
        if (indent > 0 && depth == 0) {
            return err = "line " + String(lineNo) + ": indented key outside a section", false;
        }
        String full;
        for (int i = 0; i < depth; ++i) full += path[i] + ".";
        full += key;
        if (val.isEmpty()) {  // opens a section
            if (depth == MAX_DEPTH) return err = "line " + String(lineNo) + ": nested too deep", false;
            path[depth]       = key;
            pathIndent[depth] = indent;
            ++depth;
            continue;
        }
        if (!applyKey(full, val, s)) {
            return err = "line " + String(lineNo) + ": bad or unknown setting '" + full + "'", false;
        }
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (s.minDeg[j] >= s.maxDeg[j]) return err = "joint" + String(j + 1) + ": min_deg must be below max_deg", false;
    }
    out = s;
    return true;
}

String toYaml(const Settings& s) {
    static const char* MODES[] = { "none", "edge", "center" };
    String y;
    y += "# NanoBee settings. Units: steps, steps/s, steps/s^2. joint1..joint5 = driver slots X, Y, Z, E0, E1.\n";
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
    y += "  idle_timeout_ms: " + String(s.idleTimeoutMs) + "\n";
    y += "joints:\n";
    y += "  soft_limits: " + String(s.softLimits ? "true" : "false") + "\n";
    for (int j = 0; j < NUM_JOINTS; ++j) {
        y += "  joint" + String(j + 1) + ":\n";
        y += "    steps_per_deg: " + String(s.stepsPerDeg[j], 4) + "\n";
        y += "    max_speed: " + String(s.maxSpeed[j], 0) + "\n";
        y += "    max_accel: " + String(s.maxAccel[j], 0) + "\n";
        y += "    dir_invert: " + String(s.dirInvert[j] ? "true" : "false") + "\n";
        y += "    idle_disable: " + String(s.idleDisable[j] ? "true" : "false") + "\n";
        y += "    homing: " + String(MODES[s.homing[j]]) + "\n";
        y += "    homing_dir: " + String(s.homingDir[j]) + "\n";
        y += "    home_offset_deg: " + String(s.homeOffsetDeg[j], 3) + "\n";
        y += "    homing_speed: " + String(s.jointHomingSpeed[j], 0) + "\n";
        y += "    homing_max_travel: " + String(s.jointHomingMaxTravel[j]) + "\n";
        y += "    min_deg: " + String(s.minDeg[j], 3) + "\n";
        y += "    max_deg: " + String(s.maxDeg[j], 3) + "\n";
    }
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
