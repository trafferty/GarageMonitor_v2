#include <Arduino.h>

#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

#include <Time.h>

#include <sierra_wifi_defs.h>

#define version_str "v0.5-2023-08-12(sorting out main loop logic)"


/*
**  global variables...
*/

enum DoorState_t {
    DOOR_OPEN   = HIGH,
    DOOR_CLOSED = LOW
};

// GPIO assignments
const int in_GarageDoor = D1;
const int out_GarageLight = D8;
const int out_KitchenLight = D7;
const int out_PiezoAlarm = D6;
const int out_HB_LED = LED_BUILTIN;

unsigned long prevMillis;
unsigned long HB_period_ms = 100;

bool garageDoorState_prev = false;

#define EEPROM_SIZE 15
#define UPDATE_VAL  23

/*
**  Network variables... 
*/
IPAddress ip(IP1, IP2, IP3, GARAGE_MONITOR_v2_IP_LAST_FIELD); // make sure IP is *outside* of DHCP pool range
IPAddress gateway(GW1, GW2, GW3, GW4);
IPAddress subnet(SN1, SN2, SN3, SN4);
IPAddress DNS(DNS1, DNS2, DNS3, DNS4);
const char *ssid = SSID;
const char *password = WIFI_PW;
int server_port = 80;
String DNS_name = GARAGE_MONITOR_v2_HOSTNAME;

// Set web server port number
ESP8266WebServer server(server_port);

WiFiUDP Udp;
unsigned int UdpPort = 8888; // local port to listen for UDP packets

// structs to hold start/end times for scheduling.
tmElements_t tmCriticalStart;
tmElements_t tmCriticalEnd;
time_t criticalStart_time;
time_t criticalEnd_time;
time_t now_time;
bool init_from_EEPROM = false;

bool NTPTimeSet = false;
char ntpServerNamePrimary[] = "pool.ntp.org";
char ntpServerNameSecondary[] = "time.nist.gov";
char *ntpServerName = ntpServerNamePrimary;

// forward declarations...
time_t secondsOfDay(const tmElements_t &tm);
time_t makeTimeToday(tmElements_t &tm);
bool insideTimePeriod(time_t currentTime, tmElements_t &tm_start, tmElements_t &tm_end);
String buildDateTimeStr(time_t t);
String buildTimeStr(time_t t);
String formatNumber(int num);

void wifi_init();
time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

enum Mem_Locs {
    update_flag,
    critStart_sec, critStart_min, critStart_hour, critEnd_sec, critEnd_min, critEnd_hour,
    s2_sec, s2_min, s2_hour, e2_sec, e2_min, e2_hour,
    blinkDelay, randomDelay
};

/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

void setup()
{
    Serial.begin(115200);
    Serial.print("\nGarageMonitor v2. Version: ");
    Serial.println(version_str);


    // Prepare the pins to fire
    pinMode(in_GarageDoor, INPUT);
    pinMode(out_GarageLight, OUTPUT);
    pinMode(out_KitchenLight, OUTPUT);
    pinMode(out_PiezoAlarm, OUTPUT);
    pinMode(out_HB_LED, OUTPUT);

    digitalWrite(out_GarageLight, LOW);
    digitalWrite(out_KitchenLight, LOW);
    digitalWrite(out_PiezoAlarm, LOW);

    // blink the heartbeat LED a few times to indicate we're starting up wifi
    for (int i = 0; i < 3; ++i)
    {
        digitalWrite(out_HB_LED, !digitalRead(out_HB_LED));
        delay(100);
    }

    wifi_init();

    Serial.println("Starting UDP");
    Udp.begin(UdpPort);
    Serial.print("UDP port: ");
    Serial.println(Udp.localPort());
    Serial.println("waiting for sync");
    setSyncProvider(getNtpTime);
    setSyncInterval(5);
    NTPTimeSet = false;

    // if analog input pin 0 is unconnected, random analog
    // noise will cause the call to randomSeed() to generate
    // different seed numbers each time the sketch runs.
    // randomSeed() will then shuffle the random function.
    randomSeed(analogRead(A0));


    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);

    Serial.println("Initializing start/end times:");
    // setup start/end time structs for scheduler
    //if (UPDATE_VAL == EEPROM.read(update_flag))
    if (false)
    {
        Serial.println("...updating from EEPROM");
        tmCriticalStart.Second = EEPROM.read(critStart_sec);
        tmCriticalStart.Minute = EEPROM.read(critStart_min);
        tmCriticalStart.Hour   = EEPROM.read(critStart_hour);
        tmCriticalEnd.Second   = EEPROM.read(critEnd_sec);
        tmCriticalEnd.Minute   = EEPROM.read(critEnd_min);
        tmCriticalEnd.Hour     = EEPROM.read(critEnd_hour);
        init_from_EEPROM = true;
    }
    else
    {
        Serial.println("...initializing to default values");
        tmCriticalStart.Second = 0;
        tmCriticalStart.Minute = 0;
        tmCriticalStart.Hour   = 18;
        tmCriticalEnd.Second   = 0;
        tmCriticalEnd.Minute   = 0;
        tmCriticalEnd.Hour     = 23;
        init_from_EEPROM = false;
    }



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

    // Next, check if NTP has set clock time.
    // If not, set sync interval and return
    if (!NTPTimeSet)
    {
        if (timeStatus() != timeNotSet)
        {
            NTPTimeSet = true;
            Serial.println("Time now set by NTP");
            setSyncInterval(3600);
            HB_period_ms = 1000;
        }
        else
        {
            //setSyncInterval(5);
            return;
        }
    }

    criticalStart_time = makeTimeToday(tmCriticalStart);
    criticalEnd_time   = makeTimeToday(tmCriticalEnd);

    bool garageDoor_current = digitalRead(in_GarageDoor);

    if (garageDoor_current == DOOR_OPEN)
    {
        String str  = "Garage door is open at: ";
        str += buildDateTimeStr(now());
        Serial.println(str);

        digitalWrite(out_GarageLight, (random(1, 100) > 50 ? HIGH : LOW));
        digitalWrite(out_KitchenLight, (random(1, 100) > 50 ? HIGH : LOW));
        digitalWrite(out_PiezoAlarm, (random(1, 100) > 50 ? HIGH : LOW));
        delay( 200 );
    }
    else
    {
        digitalWrite(out_GarageLight, LOW);
        digitalWrite(out_KitchenLight, LOW);
        digitalWrite(out_PiezoAlarm, LOW);
    }

    // String str  = "Current time: ";
    // str += buildDateTimeStr(now()) + "\n";
    // str += "criticalStart_time: " + String(criticalStart_time) + " - (" + buildDateTimeStr(criticalStart_time) + ")\n";
    // str += "criticalEnd_time  : " + String(criticalEnd_time) + " - (" + buildDateTimeStr(criticalEnd_time) + ")\n";
    // Serial.println(str);


}

/* ----- Util functions ----- */
time_t secondsOfDay(const tmElements_t &tm)
{
    int32_t seconds;
    seconds = tm.Hour * SECS_PER_HOUR;
    seconds += tm.Minute * SECS_PER_MIN;
    seconds += tm.Second;
    return (time_t)seconds;
}

time_t makeTimeToday(tmElements_t &tm)
{
    // update the year, month, day to current
    tm.Year = year() - 1970;
    tm.Month = month();
    tm.Day = day();
    return makeTime(tm);
}

bool insideTimePeriod(time_t currentTime, tmElements_t &tm_start, tmElements_t &tm_end)
{
    bool insideTimePer = false;
    time_t criticalStart_time;
    time_t criticalEnd_time;

    // Check to see if the time period spans 2 days
    time_t start_secs = secondsOfDay(tm_start);
    time_t end_secs = secondsOfDay(tm_end);

    if (end_secs > start_secs)
    {
        criticalStart_time = makeTimeToday(tm_start);
        criticalEnd_time = makeTimeToday(tm_end);
    }
    else
    {
        criticalStart_time = makeTimeToday(tm_start);
        tm_end.Day++;
        criticalEnd_time = makeTime(tm_end);
    }

    now_time = now();
    if ((now_time >= criticalStart_time) && (now_time <= criticalEnd_time))
        insideTimePer = true;

    return insideTimePer;
}

/* ----- Util functions ----- */
void wifi_init()
{
    Serial.print("Setting up network with static IP.");
    WiFi.config(ip, gateway, subnet, DNS);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    // Connect to Wi-Fi network with SSID and password
    Serial.printf("Connecting to %s", ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(200);
    }
    Serial.println();
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("Fail connecting");
        delay(5000);
        ESP.restart();
    }
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.println(buildDateTimeStr(now()));
}

String buildDateTimeStr(time_t t)
{
    String timeStr = formatNumber(hour(t));
    timeStr += ":" + formatNumber(minute(t));
    timeStr += ":" + formatNumber(second(t));
    timeStr += " " + formatNumber(month(t));
    timeStr += "_" + formatNumber(day(t));
    timeStr += "_" + formatNumber(year(t));
    return timeStr;
}
String buildTimeStr(time_t t)
{
    String timeStr = formatNumber(hour(t));
    timeStr += ":" + formatNumber(minute(t));
    timeStr += ":" + formatNumber(second(t));
    return timeStr;
}
String formatNumber(int num)
{
    String formatted_num = "";
    if (num < 10)
        formatted_num = "0";
    formatted_num += String(num);
    return formatted_num;
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48;     // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming & outgoing packets

// NTP Servers:
const int timeZone = -5; // Central Standard Time

time_t getNtpTime()
{
    IPAddress ntpServerIP; // NTP server's ip address

    while (Udp.parsePacket() > 0)
        ; // discard any previously received packets
    Serial.println("Transmit NTP Request");
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    Serial.print(ntpServerName);
    Serial.print(": ");
    Serial.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    delay(20);

    uint8_t attempt = 0;
    uint8_t maxAttempts = 3;
    while (attempt < maxAttempts)
    {
        attempt++;
        uint32_t beginWait = millis();
        while (millis() - beginWait < 3000)
        {
            int size = Udp.parsePacket();
            if (size >= NTP_PACKET_SIZE)
            {
                Serial.println(attempt);
                Serial.println("Receive NTP Response");
                Udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
                unsigned long secsSince1900;
                // convert four bytes starting at location 40 to a long integer
                secsSince1900 = (unsigned long)packetBuffer[40] << 24;
                secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
                secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
                secsSince1900 |= (unsigned long)packetBuffer[43];
                return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
            }
        }
    }
    Serial.println("No NTP Response :-(");
    ntpServerName = ntpServerNameSecondary;
    return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); // NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}

void handle_action_setup_timing()
{
//   Serial.println("Setting up new timing values...");
//   tmStart1.Second = (server.arg("s1_sec")).toInt(); 
//   tmStart1.Minute = (server.arg("s1_min")).toInt(); 
//   tmStart1.Hour   = (server.arg("s1_hour")).toInt(); 
//   tmEnd1.Second   = (server.arg("e1_sec")).toInt(); 
//   tmEnd1.Minute   = (server.arg("e1_min")).toInt(); 
//   tmEnd1.Hour     = (server.arg("e1_hour")).toInt(); 
//   tmStart2.Second = (server.arg("s2_sec")).toInt(); 
//   tmStart2.Minute = (server.arg("s2_min")).toInt(); 
//   tmStart2.Hour   = (server.arg("s2_hour")).toInt(); 
//   tmEnd2.Second   = (server.arg("e2_sec")).toInt(); 
//   tmEnd2.Minute   = (server.arg("e2_min")).toInt(); 
//   tmEnd2.Hour     = (server.arg("e2_hour")).toInt();

//   blinkDelay_ms   = (server.arg("blinkDelay_ms")).toInt();
//   randomDelay_ms  = (server.arg("randomDelay_ms")).toInt();

  EEPROM.write(critStart_sec,  tmCriticalStart.Second);
  EEPROM.write(critStart_min,  tmCriticalStart.Minute);
  EEPROM.write(critStart_hour, tmCriticalStart.Hour  );
  EEPROM.write(critEnd_sec,  tmCriticalEnd.Second  );
  EEPROM.write(critEnd_min,  tmCriticalEnd.Minute  );
  EEPROM.write(critEnd_hour, tmCriticalEnd.Hour    );
  EEPROM.commit();
}
