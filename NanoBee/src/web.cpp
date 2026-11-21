#include "web.h"
#include "settings.h"
#include "webpage.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

namespace Web {

static WebServer server(80);

static const char* FALLBACK_AP_SSID = "NanoBee-setup";
static const char* FALLBACK_AP_PASS = "nanobee123";

static void startAp(const char* ssid, const char* pass) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, (pass && strlen(pass) >= 8) ? pass : nullptr);
    Serial.printf("# WiFi AP \"%s\"  http://%s/\n", ssid, WiFi.softAPIP().toString().c_str());
}

static void startWifi() {
    WiFi.setHostname(cfg.hostname.c_str());
    if (cfg.wifiSta) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(100);
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("# WiFi connected  http://%s/  http://%s.local/\n", WiFi.localIP().toString().c_str(),
                          cfg.hostname.c_str());
        } else {
            Serial.println("# WiFi connect failed, starting fallback AP");
            startAp(FALLBACK_AP_SSID, FALLBACK_AP_PASS);
        }
    } else {
        startAp(cfg.ssid.c_str(), cfg.password.c_str());
    }
    WiFi.setSleep(false);  // lower latency for jogging
    if (MDNS.begin(cfg.hostname.c_str())) MDNS.addService("http", "tcp", 80);
}

static void sendText(int code, const String& body) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(code, "text/plain", body);
}

void begin() {
    startWifi();

    server.on("/", HTTP_GET, [] { server.send_P(200, "text/html", WEBPAGE); });

    server.on("/cmd", HTTP_GET, [] { sendText(200, runCommand(server.arg("c"))); });

    server.on("/status", HTTP_GET, [] { sendText(200, statusLine() + "\n" + takeEvents()); });

    server.on("/config", HTTP_GET, [] { sendText(200, SettingsStore::currentYaml()); });

    server.on("/settings", HTTP_GET, [] { sendText(200, SettingsStore::effectiveYaml()); });

    server.on("/config", HTTP_POST, [] {
        if (!machineIdle()) return sendText(409, "err machine busy, stop first");
        String err;
        if (!SettingsStore::save(server.arg("plain"), err)) return sendText(400, "err " + err);
        sendText(200, "ok (wifi changes apply after reboot)");
    });

    server.on("/reboot", HTTP_POST, [] {
        runCommand("E 0");
        sendText(200, "ok rebooting");
        delay(200);
        ESP.restart();
    });

    server.onNotFound([] { sendText(404, "not found"); });
    server.begin();
}

void loop() { server.handleClient(); }

}  // namespace Web
