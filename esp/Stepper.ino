/* Antenna Tuner ESP32 Firmware
 * Controls a stepper motor for an antenna tuner via TCP commands from a host (e.g., Raspberry Pi).
 * Listens for commands like "hoch,<steps>", "tief,<steps>", "rounds,<n>", and speed settings ("langsam", "mittel", "power").
 * Executes motor movements and responds with status messages ("Setze Speed", "Fertig").
 * Optimized for Amateur Radio tuning with persistent TCP connection and power management.
 *
 * Hardware: ESP32, stepper motor (connected to dirPin and stepPin), power control (onoff pin).
 * Network: Connects to WiFi and listens on TCP port 75.
 * Usage: Flash to ESP32, connect via tuner.c (e.g., ./tuner 1.885).
 * License: MIT
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#define ARDUINOJSON_USE_LONG_LONG 1

/* Pin Definitions */
#define dirPin 4        // Stepper motor direction pin (HIGH = forward, LOW = backward)
#define stepPin 5       // Stepper motor step pin (HIGH/LOW pulses to move)
#define onoff 0         // Power control pin (HIGH = on, LOW = off)

/* Global Variables */
long int stepsPerRun = 0;   // Total steps to execute for current command
int i = 0;                  // Loop counter (unused, retained for compatibility)
long int ispeed = 0;        // Stepper motor speed (microseconds delay per step)
bool bspeed = true;         // Flag to confirm speed setting
long int calls = 0;         // Tracks remaining steps to execute
bool fertig = true;         // Flag indicating motor movement completion
bool richtung = true;       // Direction flag (true = movement, false = rounds setting)
int setedSteps = 100;       // Number of steps per motor run (default 100)
char cbuffer[20];           // Buffer for incoming TCP commands
String sbuffer;             // String buffer for command parsing
String sString1 = "hoch,";  // Command prefix for forward movement
String sString2 = "tief,";  // Command prefix for backward movement
String sString3 = "s";      // Command for stopping motor
char c;                     // Current character from TCP stream
byte aindex = 0;            // Unused index (retained for compatibility)
byte bindex = 0;            // Index for cbuffer
byte cindex = 0;            // Unused index (retained for compatibility)

/* WiFi Configuration */
const char* wifiName = "farswitch";                    // WiFi SSID
const char* wifiPass = "Your Password";                // WiFi password
WiFiServer wifiServer(75);                             // TCP server on port 75

/* Converts char array to String for command parsing.
 * @param S Input char array
 * @param D Output String
 */
void charToStringL(const char S[], String &D) {
    byte at = 0;
    const char *p = S;
    D = "";
    while (*p++) {
        D.concat(S[at++]);
    }
}

/* Sets motor speed based on command.
 * @param swert Speed setting (1 = langsam, 2 = mittel, 3 = power, 4 = stop).
 */
void setzeV(int swert) {
    switch (swert) {
        case 1:
            ispeed = 8000;  // Slow speed (8ms delay per step)
            bspeed = false;
            break;
        case 2:
            ispeed = 3500;  // Medium speed (3.5ms delay)
            bspeed = false;
            break;
        case 3:
            ispeed = 2000;  // Fast speed (2ms delay)
            bspeed = false;
            break;
        case 4:
            calls = -1;  // Stop motor
            break;
        default:
            ispeed = 3500;  // Default to medium speed
            bspeed = false;
            break;
    }
}

/* Sets motor direction.
 * @param l_r Direction (1 = hoch/forward, 2 = tief/backward).
 */
void setzeDir(int l_r) {
    switch (l_r) {
        case 1:
            digitalWrite(dirPin, HIGH);  // Set forward direction
            delay(100);
            break;
        case 2:
            digitalWrite(dirPin, LOW);   // Set backward direction
            delay(100);
            break;
        default:
            break;
    }
}

/* Sets number of steps for movement.
 * @param pos Position of comma in command string
 * @param sdata Command string (e.g., "hoch,100")
 */
void setzeSteps(long int pos, String sdata) {
    String sHelp = sdata.substring(pos + 1, sdata.length());
    stepsPerRun = sHelp.toInt();
    calls = stepsPerRun;
    fertig = false;  // Mark motor as active
}

/* Sets number of steps per run (batched execution).
 * @param pos Position of comma in command string
 * @param sdata Command string (e.g., "rounds,50")
 */
void setzeRounds(long int pos, String sdata) {
    String sHelp = sdata.substring(pos + 1, sdata.length());
    setedSteps = sHelp.toInt();
    if (setedSteps > 100) { setedSteps = 100; }  // Cap at 100 steps
    if (setedSteps < 1) { setedSteps = 1; }      // Minimum 1 step
}

/* Executes stepper motor movement for specified rounds.
 * @param rounds Number of steps to execute in one batch.
 */
void runstepper(int rounds) {
    for (int i = 0; i < rounds; i++) {
        digitalWrite(stepPin, HIGH);  // Pulse high
        delayMicroseconds(ispeed);
        digitalWrite(stepPin, LOW);   // Pulse low
        delayMicroseconds(ispeed);
    }
    yield();  // Allow ESP32 multitasking
}

/* Empty function (placeholder for future extensions). */
void doanything() {
    byte a = 0;
}

/* Initializes WiFi, TCP server, and motor pins.
 * Connects to WiFi and starts server on port 75.
 */
void setup() {
    Serial.begin(115200);
    WiFi.setOutputPower(10);  // Reduce WiFi power for efficiency
    WiFi.begin(wifiName, wifiPass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    
    Serial.print("Connected to WiFi. IP:");
    Serial.println(WiFi.localIP());
    wifiServer.begin();
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(onoff, OUTPUT);
    digitalWrite(onoff, LOW);  // Power off initially
}

/* Main loop: Handles TCP client connections and processes commands.
 * Executes motor movements and sends responses ("Fertig", "Setze Speed").
 */
void loop() {
    WiFiClient client = wifiServer.available();
    
    if (client) {
        calls = 0;
        while (client.connected()) {
            while (client.available() > 0) {
                digitalWrite(onoff, HIGH);  // Power on motor
                c = client.read();
                if (c != '\n') {
                    cbuffer[bindex] = c;
                    bindex++;
                } else {
                    if (bindex == 1) { fertig = false; }
                    cbuffer[bindex] = '\0';
                    Serial.println(cbuffer);
                    charToStringL(cbuffer, sbuffer);
                    Serial.println(sbuffer);
                    
                    if (sbuffer.indexOf("langsam") == 0) {
                        setzeV(1);
                        bspeed = true;
                    }
                    if (sbuffer.indexOf("mittel") == 0) {
                        setzeV(2);
                        bspeed = true;
                    }
                    if (sbuffer.indexOf("power") == 0) {
                        setzeV(3);
                        bspeed = true;
                    }
                    if (sbuffer.indexOf("hoch,") == 0) {
                        richtung = true;
                        setzeDir(1);
                    }
                    if (sbuffer.indexOf("tief,") == 0) {
                        richtung = true;
                        setzeDir(2);
                    }
                    if (sbuffer.indexOf("rounds,") == 0) {
                        richtung = false;
                        setzeDir(2);
                    }
                    if (sbuffer.indexOf("s") == 0) {
                        setzeV(4);
                    }
                    if ((sbuffer.indexOf(",") != -1) && !richtung) {
                        setzeRounds(sbuffer.indexOf(","), sbuffer);
                    }
                    if ((sbuffer.indexOf(",") != -1) && richtung) {
                        setzeSteps(sbuffer.indexOf(","), sbuffer);
                    }
                    bindex = 0;
                    Serial.println(stepsPerRun);
                    Serial.println(ispeed);
                    Serial.println(bspeed);
                    if (bspeed) { client.println("Setze Speed "); }
                    if (!richtung) { client.println("Setze Rounds "); }
                    richtung = true;
                    bspeed = false;
                }
            }
            if ((stepsPerRun >= 1) && (ispeed >= 2000) && (bspeed == false) && (fertig == false)) {
                runstepper(setedSteps);
                yield();
                calls = calls - 1;
                client.print(".");
                if (calls <= 0) {
                    calls = stepsPerRun;
                    fertig = true;
                    client.println();
                    client.println("Fertig ");
                }
            }
        }
        client.stop();
        digitalWrite(onoff, LOW);  // Power off motor
        Serial.println("Client disconnected");
    }
}