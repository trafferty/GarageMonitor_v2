#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include "credentials.h"

#define version_str "v0.10-2026-04-12(non-blocking WiFi/MQTT)"

// GPIO assignments
const int in_GarageDoor    = D5;
const int out_GarageLight  = D1;
const int out_KitchenLight = D3;
const int out_PiezoAlarm   = D2;
const int out_HB_LED       = D0;

// MQTT topic
static const char* TOPIC_GARAGE = "crowpanel/garage";

// Door state
typedef enum { state_unknown, state_open, state_closed } Door_States_t;
Door_States_t doorState = state_unknown;
unsigned long doorEvent_time_milli;
unsigned long GarageLightOnPeriod_ms = 1000UL * 60 * 10;
bool garageLightOn = false;

// Periodic status publish
#define STATUS_PUBLISH_INTERVAL_MS 300000UL  // 5 minutes
unsigned long lastStatusPublish = 0;

// Heartbeat
unsigned long prevMillis;
unsigned long HB_period_ms = 1000;

// Network
static const IPAddress staticIP (IP1, IP2, IP3, GARAGE_MONITOR_v2_IP_LAST_FIELD);
static const IPAddress gatewayIP(GW1, GW2, GW3, GW4);
static const IPAddress subnetIP (SN1, SN2, SN3, SN4);
static const IPAddress dnsIP    (DNS1, DNS2, DNS3, DNS4);

BearSSL::WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(wifiClientSecure);
AsyncWebServer server(80);

// Non-blocking reconnect cooldowns
#define WIFI_RECONNECT_INTERVAL_MS  30000UL
#define MQTT_RECONNECT_INTERVAL_MS  10000UL
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;

// Forward declarations
void doBeep(int piezoOutput, int duration_ms, int numTimes);
bool connectWiFi(unsigned long timeout_ms = 10000);
bool connectMQTT();
String buildTimeDateStr();
String createStatusHTML();
void publishDoorEvent(const char* event);
void publishDoorStatus(const char* status);

// Attempt WiFi connection with a timeout. Returns true if connected.
// Never blocks indefinitely — the door state machine must keep running.
bool connectWiFi(unsigned long timeout_ms) {
    Serial.printf("Connecting to WiFi with static IP (%s)\n", GARAGE_MONITOR_v2_HOSTNAME);
    WiFi.config(staticIP, gatewayIP, subnetIP, dnsIP);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start >= timeout_ms) {
            Serial.println("\nWiFi connect timed out, continuing without WiFi");
            return false;
        }
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
}

// Single MQTT connection attempt. Returns true if connected.
// Caller is responsible for retry scheduling so the main loop never blocks.
bool connectMQTT() {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("garage-monitor", MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("connected");
        return true;
    }
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
    return false;
}

String buildTimeDateStr() {
    time_t now = time(nullptr);
    struct tm *ptm = localtime(&now);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", ptm);
    return String(buf);
}

String createStatusHTML() {
    const char* doorStateStr =
        doorState == state_open   ? "OPEN" :
        doorState == state_closed ? "CLOSED" : "UNKNOWN";
    const char* doorColor =
        doorState == state_open   ? "#c0392b" :
        doorState == state_closed ? "#27ae60" : "#888888";

    String ptr = "<!DOCTYPE html><html>\n";
    ptr += "<style>\n";
    ptr += "body{font-family:sans-serif;margin:20px;}\n";
    ptr += "table,th,td{font-size:14px;border:1px solid;border-collapse:collapse;padding:5px;}\n";
    ptr += ".door{font-size:48px;font-weight:bold;color:" + String(doorColor) + ";}\n";
    ptr += "</style>\n";
    ptr += "<body>\n";
    ptr += "<h1>Garage Monitor v2</h1>\n";
    ptr += "<p style=\"font-size:16px;\">Version: " + String(version_str) + "</p>\n";
    ptr += "<p class=\"door\">Door: " + String(doorStateStr) + "</p>\n";
    ptr += "<p style=\"font-size:16px;\">Last event: " + buildTimeDateStr() + "</p>\n";
    ptr += "<h2>ESP Debug Info</h2>\n";
    ptr += "<table><tbody>\n";
    ptr += "<tr><th>Param</th><th>Value</th></tr>\n";
    ptr += "<tr><td>Timestamp</td><td>" + buildTimeDateStr() + "</td></tr>\n";
    ptr += "<tr><td>IP Address</td><td>" + WiFi.localIP().toString() + "</td></tr>\n";
    ptr += "<tr><td>RSSI (dBm)</td><td>" + String(WiFi.RSSI()) + "</td></tr>\n";
    ptr += "<tr><td>FreeHeap</td><td>" + String(ESP.getFreeHeap()) + "</td></tr>\n";
    ptr += "<tr><td>MaxFreeBlockSize</td><td>" + String(ESP.getMaxFreeBlockSize()) + "</td></tr>\n";
    ptr += "<tr><td>HeapFragmentation</td><td>" + String(ESP.getHeapFragmentation()) + "%</td></tr>\n";
    ptr += "<tr><td>Uptime (s)</td><td>" + String(millis() / 1000) + "</td></tr>\n";
    ptr += "<tr><td>Door state var</td><td>" + String(doorState) + "</td></tr>\n";
    ptr += "</tbody></table>\n";
    ptr += "</body></html>\n";
    return ptr;
}

void publishDoorEvent(const char* event) {
    if (!mqttClient.connected()) {
        Serial.println("MQTT not connected, skipping publish");
        return;
    }
    char payload[96];
    String ts = buildTimeDateStr();
    snprintf(payload, sizeof(payload), "{\"event\":\"%s\",\"ts\":\"%s\"}", event, ts.c_str());
    Serial.print("Publish ");
    Serial.print(TOPIC_GARAGE);
    Serial.print(": ");
    Serial.println(payload);
    mqttClient.publish(TOPIC_GARAGE, payload, true);  // retained
}

void publishDoorStatus(const char* status) {
    if (!mqttClient.connected()) {
        Serial.println("MQTT not connected, skipping publish");
        return;
    }
    char payload[96];
    String ts = buildTimeDateStr();
    snprintf(payload, sizeof(payload), "{\"status\":\"%s\",\"ts\":\"%s\"}", status, ts.c_str());
    Serial.print("Publish ");
    Serial.print(TOPIC_GARAGE);
    Serial.print(": ");
    Serial.println(payload);
    mqttClient.publish(TOPIC_GARAGE, payload, true);  // retained
}

void setup() {
    Serial.begin(115200);

    pinMode(in_GarageDoor,    INPUT);
    pinMode(out_GarageLight,  OUTPUT);
    pinMode(out_KitchenLight, OUTPUT);
    pinMode(out_PiezoAlarm,   OUTPUT);
    pinMode(out_HB_LED,       OUTPUT);

    digitalWrite(out_GarageLight,  LOW);
    digitalWrite(out_KitchenLight, LOW);
    digitalWrite(out_PiezoAlarm,   LOW);

    // Blink HB LED to indicate startup
    for (int i = 0; i < 6; ++i) {
        digitalWrite(out_HB_LED, !digitalRead(out_HB_LED));
        delay(300);
    }

    doBeep(out_PiezoAlarm, 200, 1);

    Serial.print("\nGarageMonitor v2. Version: ");
    Serial.println(version_str);

    bool wifiOk = connectWiFi(15000);

    if (wifiOk) {
        // NTP setup — POSIX tz string handles CST/CDT DST transitions automatically
        configTime("CST6CDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
        Serial.print("Waiting for NTP sync");
        unsigned long ntpStart = millis();
        time_t now = time(nullptr);
        while (now < 100000 && millis() - ntpStart < 15000) {
            delay(500);
            Serial.print(".");
            now = time(nullptr);
        }
        if (now < 100000) {
            Serial.println("\nNTP sync timed out, continuing without time sync");
        } else {
            Serial.println();
            Serial.print("NTP time: ");
            Serial.println(buildTimeDateStr());
        }

        // MQTT setup
        wifiClientSecure.setInsecure();
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        mqttClient.setBufferSize(256);
        connectMQTT();
    } else {
        Serial.println("Skipping NTP and MQTT setup (no WiFi)");
    }

    // Web server
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        Serial.println(" - Handling request for /status...");
        request->send(200, "text/html", createStatusHTML());
    });
    server.begin();
    Serial.println("Web server started on port 80");

    Serial.println("Setup complete. Starting loop...");
    // Seed initial door state without triggering beeps/lights/publish
    doorState = (digitalRead(in_GarageDoor) == LOW) ? state_open : state_closed;
    doorEvent_time_milli = millis();
    Serial.print("Initial door state: ");
    Serial.println(doorState == state_open ? "open" : "closed");
    publishDoorStatus(doorState == state_open ? "open" : "closed");

    prevMillis = millis();
    lastStatusPublish = millis();
}

void loop() {
    // Heartbeat LED
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= HB_period_ms) {
        digitalWrite(out_HB_LED, !digitalRead(out_HB_LED));
        prevMillis = currentMillis;
    }

    // WiFi watchdog — non-blocking: only attempt reconnect every 30s
    if (WiFi.status() != WL_CONNECTED) {
        if (currentMillis - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL_MS) {
            lastWifiReconnectAttempt = currentMillis;
            Serial.println("WiFi lost, attempting reconnect...");
            connectWiFi(10000);
        }
    }

    // MQTT keepalive + reconnect — only try if WiFi is up, at most every 10s
    if (WiFi.status() == WL_CONNECTED) {
        if (!mqttClient.connected()) {
            if (currentMillis - lastMqttReconnectAttempt >= MQTT_RECONNECT_INTERVAL_MS) {
                lastMqttReconnectAttempt = currentMillis;
                connectMQTT();
            }
        } else {
            mqttClient.loop();
        }
    }

    // Periodic status publish (every 5 minutes regardless of state change)
    if (millis() - lastStatusPublish >= STATUS_PUBLISH_INTERVAL_MS) {
        lastStatusPublish = millis();
        if (doorState != state_unknown) {
            publishDoorStatus(doorState == state_open ? "open" : "closed");
        }
    }

    // Door state machine
    bool garageDoor_current = digitalRead(in_GarageDoor);

    if (garageDoor_current == LOW) {  // DOOR_OPEN
        if (doorState != state_open) {
            doorState = state_open;
            doorEvent_time_milli = millis();
            Serial.println("Garage door is now open...");
            digitalWrite(out_KitchenLight, HIGH);
            digitalWrite(out_GarageLight,  HIGH);
            garageLightOn = true;
            doBeep(out_PiezoAlarm, 200, 2);
            publishDoorEvent("open");
        }
    } else {
        if (doorState == state_open) {
            doorState = state_closed;
            doorEvent_time_milli = millis();
            Serial.println("Garage door is now closed...");
            digitalWrite(out_GarageLight, HIGH);
            garageLightOn = true;
            digitalWrite(out_KitchenLight, LOW);
            doBeep(out_PiezoAlarm, 200, 3);
            publishDoorEvent("closed");
        }
    }

    // Garage light auto-off after 10 minutes
    if (garageLightOn && (millis() - doorEvent_time_milli >= GarageLightOnPeriod_ms)) {
        Serial.println("Shutting off garage light...");
        digitalWrite(out_GarageLight, LOW);
        garageLightOn = false;
    }

    delay(50);
}

void doBeep(int piezoOutput, int duration_ms, int numTimes) {
    for (int i = 0; i < numTimes; ++i) {
        digitalWrite(piezoOutput, HIGH);
        delay(duration_ms);
        digitalWrite(piezoOutput, LOW);
        delay(duration_ms);
    }
}
