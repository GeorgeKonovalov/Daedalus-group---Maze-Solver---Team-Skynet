/*
  OLED Diagnostic Test — Event-Based DFS Maze Solver
  ===================================================
  Arduino Nano RP2040 Connect

  MOTORS ARE DISABLED.  This is a bench-test tool that:
    • Reads four ultrasonic sensors over UART (same as main sketch)
    • Continuously classifies the sensor pattern into event types
      (dead-end, T-junction, turn, branch, corridor …)
    • Displays live sensor data, event type, state, and heading
      on a 128×64 SSD1306 OLED via I2C
    • Mirrors all info to USB Serial for logging

  OLED DISPLAY LAYOUT (21 chars × 8 lines at font size 1)
  ─────────────────────────────────────────────────────────
  Line 0:  State + heading       e.g.  "FOLLOW        > N"
  Line 1:  Event classification  e.g.  "Ev: T-JUNCTN"
  Line 2:  (separator line)
  Line 3:  North distance              "      N: 25.3"
  Line 4:  West + East compass   e.g.  "W:12.1 [+] E: 4.2"
  Line 5:  South distance              "      S: 30.0"
  Line 6:  (separator line)
  Line 7:  Wall flags + stack    e.g.  "W[N.S.] Stk:3"

  WIRING
  ──────
  OLED SDA  →  RP2040 A4 / GPIO 12  (Wire default)
  OLED SCL  →  RP2040 A5 / GPIO 13  (Wire default)
  OLED VCC  →  3.3V
  OLED GND  →  GND

  DEPENDENCIES
  ─────────────
  Install via Library Manager:
    • Adafruit SSD1306
    • Adafruit GFX Library
    • Adafruit BusIO        (installed automatically as dependency)
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MazeTypes.h"

// =====================================================================
//  OLED CONFIGURATION
// =====================================================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =====================================================================
//  EVENT DETECTION THRESHOLDS  (must match main sketch)
// =====================================================================
const float WALL_THRESHOLD    = 5.0f;
const float FRONT_STOP_DIST   = 8.0f;
const float SIDE_WALL_PRESENT = 12.0f;

// =====================================================================
//  SENSOR READINGS (updated from UART)
// =====================================================================
float distN = 0.0f, distE = 0.0f, distS = 0.0f, distW = 0.0f;
String incomingData = "";

// =====================================================================
//  STATE TRACKING
// =====================================================================
// These mirror the main sketch's state.  Since motors are off, they
// are set manually via serial commands to test the display layout.
RobotState state       = ST_STARTUP;
Direction  heading     = NORTH;
int        stackDepth  = 0;        // simulated stack depth for display
bool       btFlag      = false;    // simulated backtracking flag

// Display refresh throttle
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 100;   // 10 Hz max

// =====================================================================
//  NAME HELPERS
// =====================================================================
const char* dirLetter(Direction d) {
    static const char* n[] = {"N", "E", "S", "W"};
    return n[d];
}

const char* stateName(RobotState s) {
    switch (s) {
        case ST_STARTUP:    return "STARTUP ";
        case ST_SCAN_EVENT: return "SCANNING";
        case ST_DECIDE:     return "DECIDING";
        case ST_FOLLOWING:  return "FOLLOW  ";
        case ST_COMPLETE:   return "COMPLETE";
    }
    return "????????";
}

// Event type name from the EventType enum (stored events only)
const char* eventTypeName(EventType t) {
    switch (t) {
        case EVT_DEAD_END:     return "DEAD END";
        case EVT_T_JUNCTION:   return "T-JUNCTN";
        case EVT_LEFT_TURN:    return "LEFT TRN";
        case EVT_RIGHT_TURN:   return "RGHT TRN";
        case EVT_LEFT_BRANCH:  return "LEFT BRN";
        case EVT_RIGHT_BRANCH: return "RGHT BRN";
        case EVT_START:        return "START   ";
    }
    return "????????";
}

// =====================================================================
//  UART SENSOR PARSING
// =====================================================================
void parseDistances(String data) {
    int idx1 = data.indexOf("D1:") + 3;
    int idx2 = data.indexOf("D2:") + 3;
    int idx3 = data.indexOf("D3:") + 3;
    int idx4 = data.indexOf("D4:") + 3;

    int comma1 = data.indexOf(',', idx1);
    int comma2 = data.indexOf(',', idx2);
    int comma3 = data.indexOf(',', idx3);

    if (idx1 > 2 && idx2 > 2 && idx3 > 2 && idx4 > 2) {
        distN = data.substring(idx1, comma1).toFloat();
        distE = data.substring(idx2, comma2).toFloat();
        distS = data.substring(idx3, comma3).toFloat();
        distW = data.substring(idx4).toFloat();
    }
}

bool readSensorsUART() {
    bool newData = false;
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            parseDistances(incomingData);
            incomingData = "";
            newData = true;
        } else if (c != '\r') {
            incomingData += c;
        }
    }
    return newData;
}

// =====================================================================
//  SENSOR-RELATIVE HELPERS
// =====================================================================
float sensorInDir(Direction d) {
    switch (d) {
        case NORTH: return distN;
        case EAST:  return distE;
        case SOUTH: return distS;
        case WEST:  return distW;
    }
    return 999.0f;
}

float frontDist() { return sensorInDir(heading); }
Direction leftOf(Direction d)  { return (Direction)((d + 3) % 4); }
Direction rightOf(Direction d) { return (Direction)((d + 1) % 4); }
float leftDist()  { return sensorInDir(leftOf(heading)); }
float rightDist() { return sensorInDir(rightOf(heading)); }

bool wallPresent(Direction d) { return sensorInDir(d) <= WALL_THRESHOLD; }

// =====================================================================
//  LIVE EVENT CLASSIFICATION FOR DISPLAY
// =====================================================================
// This classifies the CURRENT sensor snapshot for display purposes.
// It covers all possible patterns including corridor and open space,
// which the main DFS sketch never stores (the robot PID-follows
// through corridors without stopping).
//
// Returns a display string directly rather than using EventType,
// since corridor and open are display-only concepts.
const char* classifyForDisplay() {
    bool wF = wallPresent(heading);
    bool wL = wallPresent(leftOf(heading));
    bool wR = wallPresent(rightOf(heading));

    if ( wF &&  wL &&  wR) return "DEAD END";
    if ( wF && !wL && !wR) return "T-JUNCTN";
    if ( wF && !wL &&  wR) return "LEFT TRN";
    if ( wF &&  wL && !wR) return "RGHT TRN";
    if (!wF &&  wL &&  wR) return "CORRIDOR";
    if (!wF && !wL &&  wR) return "LEFT BRN";
    if (!wF &&  wL && !wR) return "RGHT BRN";
    if (!wF && !wL && !wR) return "OPEN    ";

    return "????????";
}

// =====================================================================
//  OLED DRAWING
// =====================================================================
/*
  Display layout (128×64, font 6×8 px, ~21 chars/line, 8 lines):

  ┌─────────────────────┐
  │ FOLLOW        > N   │  state + heading
  │ Ev: T-JUNCTN        │  event classification
  │─────────────────────│
  │       N: 25.3       │  north distance
  │ W:12.1 [+]  E: 4.2  │  west / east + compass
  │       S: 30.0       │  south distance
  │─────────────────────│
  │ W[N.S.] Stk:3       │  wall flags + stack depth
  └─────────────────────┘
*/
void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    char buf[22];

    // ----- LINE 0: State + Heading -----
    display.setTextSize(1);
    display.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%s      > %s",
             stateName(state), dirLetter(heading));
    display.print(buf);

    // ----- LINE 1: Event -----
    display.setCursor(0, 9);
    snprintf(buf, sizeof(buf), "Ev: %s", classifyForDisplay());
    display.print(buf);

    // ----- LINE 2: separator -----
    display.drawFastHLine(0, 18, 128, SSD1306_WHITE);

    // ----- LINE 3: North distance -----
    display.setCursor(34, 21);
    if (distN <= WALL_THRESHOLD) {
        snprintf(buf, sizeof(buf), "N: WALL");
    } else {
        snprintf(buf, sizeof(buf), "N:%5.1f", distN);
    }
    display.print(buf);

    // ----- LINE 4: West [+] East -----
    display.setCursor(0, 32);
    {
        char wBuf[8], eBuf[8];
        if (distW <= WALL_THRESHOLD) {
            snprintf(wBuf, sizeof(wBuf), "WALL");
        } else {
            snprintf(wBuf, sizeof(wBuf), "%4.1f", distW);
        }
        if (distE <= WALL_THRESHOLD) {
            snprintf(eBuf, sizeof(eBuf), "WALL");
        } else {
            snprintf(eBuf, sizeof(eBuf), "%4.1f", distE);
        }
        snprintf(buf, sizeof(buf), "W:%-5s[+] E:%-5s", wBuf, eBuf);
        display.print(buf);
    }

    // ----- LINE 5: South distance -----
    display.setCursor(34, 43);
    if (distS <= WALL_THRESHOLD) {
        snprintf(buf, sizeof(buf), "S: WALL");
    } else {
        snprintf(buf, sizeof(buf), "S:%5.1f", distS);
    }
    display.print(buf);

    // ----- LINE 6: separator -----
    display.drawFastHLine(0, 53, 128, SSD1306_WHITE);

    // ----- LINE 7: Wall flags + stack depth -----
    display.setCursor(0, 56);
    {
        char wn = wallPresent(NORTH) ? 'N' : '.';
        char we = wallPresent(EAST)  ? 'E' : '.';
        char ws = wallPresent(SOUTH) ? 'S' : '.';
        char ww = wallPresent(WEST)  ? 'W' : '.';
        snprintf(buf, sizeof(buf), "W[%c%c%c%c] Stk:%-2d %s",
                 wn, we, ws, ww, stackDepth,
                 btFlag ? "BT" : "");
        display.print(buf);
    }

    display.display();
}

// =====================================================================
//  SERIAL DEBUG MIRROR
// =====================================================================
void printSerialDebug() {
    Serial.print("N:"); Serial.print(distN, 1);
    Serial.print("  E:"); Serial.print(distE, 1);
    Serial.print("  S:"); Serial.print(distS, 1);
    Serial.print("  W:"); Serial.print(distW, 1);
    Serial.print("  | Ev:"); Serial.print(classifyForDisplay());
    Serial.print("  St:"); Serial.print(stateName(state));
    Serial.print("  Hd:"); Serial.print(dirLetter(heading));
    Serial.print("  Walls:");
    if (wallPresent(NORTH)) Serial.print("N");
    if (wallPresent(EAST))  Serial.print("E");
    if (wallPresent(SOUTH)) Serial.print("S");
    if (wallPresent(WEST))  Serial.print("W");
    Serial.println();
}

// =====================================================================
//  SERIAL COMMAND INTERFACE
// =====================================================================
/*
  Type single characters in the serial monitor to test the display:

    n / e / s / w  — change heading
    0              — state: ST_STARTUP
    1              — state: ST_FOLLOWING
    2              — state: ST_SCAN_EVENT
    3              — state: ST_DECIDE
    4              — state: ST_COMPLETE
    + / -          — increment / decrement simulated stack depth
    b              — toggle simulated backtracking flag
*/
void checkSerialCommands() {
    if (!Serial.available()) return;
    char c = Serial.read();

    switch (c) {
        case 'n': case 'N': heading = NORTH; Serial.println(">> Heading: NORTH"); break;
        case 'e': case 'E': heading = EAST;  Serial.println(">> Heading: EAST");  break;
        case 's': case 'S': heading = SOUTH; Serial.println(">> Heading: SOUTH"); break;
        case 'w': case 'W': heading = WEST;  Serial.println(">> Heading: WEST");  break;
        case '0': state = ST_STARTUP;    Serial.println(">> State: STARTUP");    break;
        case '1': state = ST_FOLLOWING;  Serial.println(">> State: FOLLOWING");  break;
        case '2': state = ST_SCAN_EVENT; Serial.println(">> State: SCAN_EVENT"); break;
        case '3': state = ST_DECIDE;     Serial.println(">> State: DECIDE");     break;
        case '4': state = ST_COMPLETE;   Serial.println(">> State: COMPLETE");   break;
        case '+': stackDepth++; Serial.print(">> Stack: "); Serial.println(stackDepth); break;
        case '-': if (stackDepth > 0) stackDepth--; Serial.print(">> Stack: "); Serial.println(stackDepth); break;
        case 'b': case 'B': btFlag = !btFlag; Serial.print(">> Backtrack: "); Serial.println(btFlag ? "ON" : "OFF"); break;
    }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("ERROR: SSD1306 OLED not found at 0x3C!");
        Serial.println("Check wiring: SDA->A4, SCL->A5, VCC->3.3V");
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(250);
        }
    }

    // Splash screen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 8);
    display.println("EVENT-BASED DFS");
    display.setCursor(10, 20);
    display.println("OLED DIAG TEST");
    display.setCursor(10, 36);
    display.println("Motors: OFF");
    display.setCursor(10, 48);
    display.println("Waiting for UART...");
    display.display();

    Serial.println("=========================================");
    Serial.println("  OLED DIAGNOSTIC TEST (event-based)");
    Serial.println("  Motors DISABLED — sensor display only");
    Serial.println("=========================================");
    Serial.println("Serial commands:");
    Serial.println("  n/e/s/w     = heading");
    Serial.println("  0-4         = state");
    Serial.println("  +/-         = stack depth");
    Serial.println("  b           = toggle backtrack flag");
    Serial.println("=========================================\n");

    delay(2000);
}

// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {
    checkSerialCommands();

    bool newData = readSensorsUART();

    if (newData) {
        printSerialDebug();
    }

    unsigned long now = millis();
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
        lastDisplayUpdate = now;
        updateDisplay();
    }
}
