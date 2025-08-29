#include <Arduino.h>

#define version_str "v0.6-2025-02-22(remove WiFi and dependency on NTP)"


/*
**  global variables...
*/

enum DoorState_t {
    DOOR_OPEN   = LOW,
    DOOR_CLOSED = HIGH
};

// GPIO assignments
const int in_GarageDoor = D5;
const int out_GarageLight = D1;
const int out_KitchenLight = D3;
const int out_PiezoAlarm = D2;
const int out_HB_LED = D0;

unsigned long prevMillis;
unsigned long HB_period_ms = 1000;

bool garageDoorState_prev = false;

typedef enum {
    state_unknown, state_open, state_closed
} Door_States_t;

Door_States_t doorState = state_unknown;
long unsigned doorEvent_time_milli;

long unsigned GarageLightOnPeriod_ms = 1000 * 60 * 10;
bool garageLightOn = false;

// forward declarations...
void doBeep(int piezoOutput, int duration_ms, int numTimes);

void setup()
{
    Serial.begin(115200);

    // Prepare the pins to fire
    pinMode(in_GarageDoor, INPUT);
    pinMode(out_GarageLight, OUTPUT);
    pinMode(out_KitchenLight, OUTPUT);
    pinMode(out_PiezoAlarm, OUTPUT);
    pinMode(out_HB_LED, OUTPUT);

    digitalWrite(out_GarageLight, LOW);
    digitalWrite(out_KitchenLight, LOW);
    digitalWrite(out_PiezoAlarm, LOW);

    // blink the heartbeat LED a few times to indicate we're starting 
    for (int i = 0; i < 6; ++i)
    {
        digitalWrite(out_HB_LED, !digitalRead(out_HB_LED));
        delay(300);
    }

    doBeep(out_PiezoAlarm, 200, 1);

    delay(3000);
    Serial.print("\nGarageMonitor v2. Version: ");
    Serial.println(version_str);
    Serial.println("Setup complete.  Starting loop...");

    // get timestamp for heartbeat LED
    prevMillis = millis();
}

void loop()
{
    // First let's toggle the HB LED if HB_period_ms has elapsed
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= HB_period_ms)
    {
        digitalWrite(out_HB_LED, !digitalRead(out_HB_LED)); // if so, change the state of the LED.
        prevMillis = currentMillis;
    }

    // Now check if garage door is open, then toggle state if necessary

    bool garageDoor_current = digitalRead(in_GarageDoor);

    if (garageDoor_current == DOOR_OPEN)
    {
        if (doorState != state_open)
        {
            doorState = state_open;
            doorEvent_time_milli = millis();

            String str  = "Garage door is now open...";
            Serial.println(str);

            digitalWrite(out_KitchenLight, HIGH );
            digitalWrite(out_GarageLight, HIGH );
            garageLightOn = true;

            doBeep(out_PiezoAlarm, 200, 2);
        }
    }
    else
    {
        if (doorState == state_open)
        {
            doorState = state_closed;
            doorEvent_time_milli = millis();

            String str  = "Garage door is now closed...";
            Serial.println(str);

            digitalWrite(out_GarageLight, HIGH);
            garageLightOn = true;

            digitalWrite(out_KitchenLight, LOW);

            doBeep(out_PiezoAlarm, 200, 3);
        }
    }

    if (garageLightOn && (millis() - doorEvent_time_milli >= GarageLightOnPeriod_ms))
    {
        String str  = "Shutting off garage light...";
        Serial.println(str);

        digitalWrite(out_GarageLight, LOW);
        garageLightOn = false;
    }

    delay( 50 );
}

void doBeep(int piezoOutput, int duration_ms, int numTimes)
{
    for (int i=0; i<numTimes; ++i)
    {
        digitalWrite(piezoOutput, HIGH);
        delay(duration_ms);
        digitalWrite(piezoOutput, LOW);
        delay(duration_ms);
    }
}
