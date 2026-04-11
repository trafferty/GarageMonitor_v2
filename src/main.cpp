#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include "credentials.h"

#define version_str "v0.7-2026-04-11(add WiFi, MQTT, NTP)"

// GPIO assignments
const int in_GarageDoor    = D5;
const int out_GarageLight  = D1;
const int out_KitchenLight = D3;
const int out_PiezoAlarm   = D2;
const int out_HB_LED       = D0;

// MQTT topic
static const char* TOPIC_GARAGE = "crowpanel/garage";

// Timing
#define NTP_INTERVAL_MS 3600000UL  // 1 hour

// Door state
typedef enum { state_unknown, state_open, state_closed } Door_States_t;
Door_States_t doorState = state_unknown;
unsigned long doorEvent_time_milli;
unsigned long GarageLightOnPeriod_ms = 1000UL * 60 * 10;
bool garageLightOn = false;

// Heartbeat
unsigned long prevMillis;
unsigned long HB_period_ms = 1000;

// Network
BearSSL::WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(wifiClientSecure);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
unsigned long lastNTPUpdate = 0;
int reconnectAttempts = 0;

// Forward declarations
void doBeep(int piezoOutput, int duration_ms, int numTimes);
void connectWiFi();
void connectMQTT();
String buildTimeDateStr();
void publishDoorEvent(const char* event);

void connectWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected, IP: ");
    Serial.println(WiFi.localIP());
}

void connectMQTT() {
    unsigned long backoff = 1000;
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("garage-monitor", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("connected");
            reconnectAttempts = 0;
        } else {
            reconnectAttempts++;
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.print(", retrying in ");
            Serial.print(backoff / 1000);
            Serial.println("s");
            if (reconnectAttempts >= 10) {
                Serial.println("Too many failures, restarting...");
                ESP.restart();
            }
            delay(backoff);
            if (backoff < 30000) backoff *= 2;
        }
    }
}

String buildTimeDateStr() {
    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime(&epochTime);
    int yr  = ptm->tm_year + 1900;
    int mon = ptm->tm_mon  + 1;
    int day = ptm->tm_mday;
    String ts = String(yr) + "-";
    if (mon < 10) ts += "0";
    ts += String(mon) + "-";
    if (day < 10) ts += "0";
    ts += String(day) + "T";
    ts += timeClient.getFormattedTime();
    return ts;
}

void publishDoorEvent(const char* event) {
    char payload[96];
    String ts = buildTimeDateStr();
    snprintf(payload, sizeof(payload), "{\"event\":\"%s\",\"ts\":\"%s\"}", event, ts.c_str());
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

    connectWiFi();

    // NTP setup
    timeClient.begin();
    timeClient.setTimeOffset(-6 * 3600);  // CST (UTC-6)
    delay(1000);
    timeClient.update();
    Serial.print("NTP time: ");
    Serial.println(timeClient.getFormattedTime());
    lastNTPUpdate = millis();

    // MQTT setup
    wifiClientSecure.setInsecure();
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setBufferSize(256);
    connectMQTT();

    Serial.println("Setup complete. Starting loop...");
    prevMillis = millis();
}

void loop() {
    // Heartbeat LED
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= HB_period_ms) {
        digitalWrite(out_HB_LED, !digitalRead(out_HB_LED));
        prevMillis = currentMillis;
    }

    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        connectWiFi();
    }

    // MQTT keepalive + reconnect
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();

    // NTP hourly refresh
    if (millis() - lastNTPUpdate >= NTP_INTERVAL_MS) {
        lastNTPUpdate = millis();
        timeClient.update();
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
