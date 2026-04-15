/*
  Event-Based DFS Maze Solver with PID Corridor Following
  ========================================================
  Arduino Nano RP2040 Connect — X-drive holonomic robot

  START / FINISH CONDITIONS
  -------------------------
  The robot is placed completely OUTSIDE the maze before powering on.
  Since there are no four-way crossroads in this maze, the condition
  "no walls detected on any side" uniquely identifies being outside:

    • START:  no walls + no events recorded → drive forward into maze
    • FINISH: no walls + events recorded    → robot has exited, stop

  STATE MACHINE FLOW
  ------------------
    STARTUP → WAIT_FOR_ENTRY → ENTERING → SCAN_EVENT → DECIDE
                                              ↑           ↓
                                          FOLLOWING ←------+
                                              ↓
                                          COMPLETE (when no walls + events > 0)

  HOW THE MAZE IS REMEMBERED
  ---------------------------
  The robot treats the maze as a graph of EVENTS connected by corridors:

    [T-JUNCT] ---corridor--- [DEAD END]
        |
    corridor
        |
    [LEFT TURN] ---corridor--- [DEAD END]

  Only events (junctions, turns, dead ends) are saved.  Corridors
  are the paths between them — the robot PID-follows through them
  without recording anything.

  Each Event struct (10 bytes) stores:
    • exits[4]      — which absolute directions are open
    • explored[4]   — which exits DFS has already tried
    • entryHeading  — heading when the robot first arrived
                      (needed to compute the backtrack direction)

  HOW BACKTRACKING WORKS
  ----------------------
  When the robot reaches a dead end or has explored all exits at an
  event, it pops that event off the stack and reverses direction:

      backtrack heading = opposite( popped event's entryHeading )

  This sends the robot back through the same corridor to the previous
  event.  If the stack empties entirely, the robot has backtracked
  past its first event and will eventually exit the maze, triggering
  the finish condition.

  SENSOR INPUT
  ------------
  Distances arrive over UART (Serial1) from the sensor Arduino:
      D1:<north>,D2:<east>,D3:<south>,D4:<west>\n
*/

#include <Arduino.h>
#include <Servo.h>
#include "MazeTypes.h"

// =====================================================================
//  MODE SELECT
// =====================================================================
const RunMode MODE = MAZE_DFS;

// =====================================================================
//  DIRECTION UTILITIES
// =====================================================================
Direction opposite(Direction d)  { return (Direction)((d + 2) % 4); }
Direction leftOf(Direction d)    { return (Direction)((d + 3) % 4); }
Direction rightOf(Direction d)   { return (Direction)((d + 1) % 4); }

const char* dirName(Direction d) {
    static const char* n[] = {"NORTH", "EAST", "SOUTH", "WEST"};
    return n[d];
}

const char* dirArrow(Direction d) {
    static const char* a[] = {"^", ">", "v", "<"};
    return a[d];
}

const char* eventTypeName(EventType t) {
    switch (t) {
        case EVT_DEAD_END:     return "DEAD END";
        case EVT_T_JUNCTION:   return "T-JUNCT ";
        case EVT_LEFT_TURN:    return "LEFT TRN";
        case EVT_RIGHT_TURN:   return "RGHT TRN";
        case EVT_LEFT_BRANCH:  return "LEFT BRN";
        case EVT_RIGHT_BRANCH: return "RGHT BRN";
        case EVT_START:        return "START   ";
    }
    return "????????";
}

const char* stateName(RobotState s) {
    switch (s) {
        case ST_STARTUP:        return "STARTUP";
        case ST_WAIT_FOR_ENTRY: return "WAITING";
        case ST_ENTERING:       return "ENTER  ";
        case ST_SCAN_EVENT:     return "SCAN   ";
        case ST_DECIDE:         return "DECIDE ";
        case ST_FOLLOWING:      return "FOLLOW ";
        case ST_COMPLETE:       return "DONE   ";
        case ST_COOLDOWN:       return "COOLDN ";
    }
    return "???????";
}

// =====================================================================
//  SERVO / MOTOR PINS & CALIBRATION
// =====================================================================
const uint8_t PIN_FL = 5;
const uint8_t PIN_FR = 3;
const uint8_t PIN_RR = 9;
const uint8_t PIN_RL = 4;

Servo S_FL, S_FR, S_RR, S_RL;

int STOP_FL = 90,  STOP_FR = 90,  STOP_RR = 90,  STOP_RL = 90;
int RANGE_FL = 90,  RANGE_FR = 90,  RANGE_RR = 90,  RANGE_RL = 90;
int INV_FL = +1,   INV_FR = +1,   INV_RR = +1,   INV_RL = +1;

const float GAIN_FL =  0.951f;
const float GAIN_FR = -0.901f;
const float GAIN_RR = -1.000f;
const float GAIN_RL =  0.938f;

// =====================================================================
//  SENSOR READINGS (updated from UART)
// =====================================================================
float distN = 30.0f, distE = 30.0f, distS = 30.0f, distW = 30.0f;
String incomingData = "";

// =====================================================================
//  PID PARAMETERS
// =====================================================================
float Kp = 0.07f;
float Ki = 0.00f;
float Kd = 0.00f;

float previousError = 0;
float integral       = 0;
unsigned long previousTime = 0;

const float INTEGRAL_LIMIT   = 20.0f;
const float CORRECTION_LIMIT = 0.5f;

// =====================================================================
//  MOVEMENT PARAMETERS
// =====================================================================
const float FORWARD_SPEED = 0.4f;
const float CREEP_SPEED   = 0.20f;

// =====================================================================
//  EVENT DETECTION THRESHOLDS
// =====================================================================
const float WALL_THRESHOLD       = 5.0f;     // ≤ this = wall
const float FRONT_STOP_DIST      = 8.0f;     // stop before front collision
const float SIDE_WALL_PRESENT    = 12.0f;     // side wall detection range
const unsigned long EVENT_COOLDOWN_MS = 800;  // ignore events after direction change

// =====================================================================
//  DFS EXPLORATION PRIORITY
// =====================================================================
// When multiple unexplored exits exist, try them in this order.
// Change this to bias the robot toward a particular direction.
const Direction EXPLORE_ORDER[] = {NORTH, EAST, SOUTH, WEST};

// =====================================================================
//  EVENT STACK  (this IS the maze memory)
// =====================================================================
const int MAX_EVENTS = 64;

Event eventStack[MAX_EVENTS];
int   stackTop = -1;          // -1 = empty

// Total events ever pushed in the CURRENT run.
// Resets between runs.  Used to distinguish start from finish.
int totalEventsRecorded = 0;

bool stackEmpty() { return stackTop < 0; }
bool stackFull()  { return stackTop >= MAX_EVENTS - 1; }

void stackPush(const Event& e) {
    if (!stackFull()) {
        stackTop++;
        eventStack[stackTop] = e;
        totalEventsRecorded++;
    } else {
        Serial.println("!!! EVENT STACK FULL — maze too deep !!!");
    }
}

Event stackPop() {
    Event e = eventStack[stackTop];
    stackTop--;
    return e;
}

Event& stackPeek() { return eventStack[stackTop]; }

// =====================================================================
//  STATE MACHINE VARIABLES
// =====================================================================
RobotState state    = ST_STARTUP;
Direction  heading  = NORTH;

unsigned long stateEntryTime     = 0;
unsigned long lastDirectionChange = 0;

// Backtracking flag: when true, the next event arrival is a RETURN
// to the stack-top event (don't create a new one).
bool backtracking = false;

// Side-wall memory for junction-transition detection
bool prevLeftWall  = true;
bool prevRightWall = true;

// =====================================================================
//  MULTI-RUN COMPETITION VARIABLES
// =====================================================================
// Run 1: force LEFT  at the T-junction, DFS otherwise
// Run 2: force RIGHT at the T-junction, DFS otherwise
// Run 3: choose the faster direction from runs 1 & 2
//
// Between runs: 20-second cooldown for repositioning at start.

const int TOTAL_RUNS = 3;
const unsigned long COOLDOWN_MS = 20000;   // 20 seconds between runs

int           currentRun = 0;              // 0, 1, 2 (runs 1–3)
unsigned long runTimes[TOTAL_RUNS] = {0};  // elapsed ms for each run
unsigned long mazeStartTime = 0;           // set when entering maze

// Has the forced T-junction turn been made in this run?
// Once true, the DFS operates normally for the rest of the run.
bool tJunctionHandled = false;

// =====================================================================
//  RESET FOR NEW RUN
// =====================================================================
// Clears the event stack and per-run state so the robot can run again.
// Does NOT clear runTimes[] or currentRun — those persist across runs.
void resetForNewRun() {
    stackTop            = -1;
    totalEventsRecorded = 0;
    backtracking        = false;
    tJunctionHandled    = false;
    prevLeftWall        = true;
    prevRightWall       = true;
    heading             = NORTH;
    mazeStartTime       = 0;
}

// =====================================================================
//  MOTOR HELPERS
// =====================================================================
static inline float clamp1(float v) {
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

void writeCRServo(Servo &s, int stopVal, int rangeVal, int inv, float cmd) {
    cmd = clamp1(cmd) * inv;
    int val = constrain((int)lround(stopVal + cmd * rangeVal), 0, 180);
    s.write(val);
}

void normalize(float &fl, float &fr, float &rr, float &rl) {
    float m = max(max(fabs(fl), fabs(fr)), max(fabs(rr), fabs(rl)));
    if (m < 1e-6f) { fl = fr = rr = rl = 0; return; }
    if (m > 1.0f)  { fl /= m; fr /= m; rr /= m; rl /= m; }
}

void stopAll() {
    writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, 0);
    writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, 0);
    writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, 0);
    writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, 0);
}

void drive(float Vx, float Vy, float W) {
    float fl = Vy + Vx + W;
    float fr = Vy - Vx - W;
    float rr = Vy + Vx - W;
    float rl = Vy - Vx + W;

    normalize(fl, fr, rr, rl);
    fl *= GAIN_FL;  fr *= GAIN_FR;
    rr *= GAIN_RR;  rl *= GAIN_RL;
    normalize(fl, fr, rr, rl);

    writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, fl);
    writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, fr);
    writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, rr);
    writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, rl);
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
//  DIRECTION-AWARE SENSOR ACCESS
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

float frontDist()  { return sensorInDir(heading); }
float leftDist()   { return sensorInDir(leftOf(heading)); }
float rightDist()  { return sensorInDir(rightOf(heading)); }

bool wallPresent(Direction d) { return sensorInDir(d) <= WALL_THRESHOLD; }

// Returns true when NO walls are detected in any direction.
// Since there are no four-way crossroads in this maze, this condition
// only occurs when the robot is completely outside the maze.
bool noWallsDetected() {
    return !wallPresent(NORTH) && !wallPresent(EAST) &&
           !wallPresent(SOUTH) && !wallPresent(WEST);
}

// Returns true when at least one wall is detected.
bool anyWallDetected() {
    return wallPresent(NORTH) || wallPresent(EAST) ||
           wallPresent(SOUTH) || wallPresent(WEST);
}

// =====================================================================
//  DIRECTION-AWARE DRIVING
// =====================================================================
float getCenteringError() {
    switch (heading) {
        case NORTH: case SOUTH: return distW - distE;
        case EAST:  case WEST:  return distN - distS;
    }
    return 0;
}

void driveHeading(float correction, float speed) {
    switch (heading) {
        case NORTH: drive( correction,  speed, 0); break;
        case SOUTH: drive( correction, -speed, 0); break;
        case EAST:  drive( speed,  correction, 0); break;
        case WEST:  drive(-speed,  correction, 0); break;
    }
}

// =====================================================================
//  PID CALCULATION
// =====================================================================
float calculatePID(float error) {
    unsigned long now = millis();
    float dt = (now - previousTime) / 1000.0f;
    if (dt <= 0) dt = 0.05f;

    float P = Kp * error;

    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float I = Ki * integral;

    float derivative = (error - previousError) / dt;
    float D = Kd * derivative;

    previousError = error;
    previousTime  = now;

    return constrain(P + I + D, -CORRECTION_LIMIT, CORRECTION_LIMIT);
}

void resetPID() {
    integral      = 0;
    previousError = 0;
    previousTime  = millis();
}

// =====================================================================
//  EVENT CLASSIFICATION
// =====================================================================
// Classifies the current sensor pattern relative to the arrival heading.
// Only called when creating a new event (not during backtracking).
EventType classifyEvent() {
    bool wF = wallPresent(heading);
    bool wL = wallPresent(leftOf(heading));
    bool wR = wallPresent(rightOf(heading));

    if ( wF && wL &&  wR) return EVT_DEAD_END;
    if ( wF && !wL && !wR) return EVT_T_JUNCTION;
    if ( wF && !wL &&  wR) return EVT_LEFT_TURN;
    if ( wF &&  wL && !wR) return EVT_RIGHT_TURN;
    if (!wF && !wL &&  wR) return EVT_LEFT_BRANCH;
    if (!wF &&  wL && !wR) return EVT_RIGHT_BRANCH;

    // Fallback — shouldn't happen if corridors aren't triggering events
    return EVT_DEAD_END;
}

// =====================================================================
//  EVENT CREATION
// =====================================================================
// Reads the current sensors and builds an Event struct.
//
//   isStart = true  → first event, no arrival corridor
//                      (no exit is marked explored)
//   isStart = false → normal arrival from a corridor
//                      (the exit we entered through is marked explored
//                       because that direction leads back to the
//                       previous event, not to new territory)
Event createEvent(bool isStart) {
    Event e;

    // Record which absolute directions are open
    e.exits[NORTH] = !wallPresent(NORTH);
    e.exits[EAST]  = !wallPresent(EAST);
    e.exits[SOUTH] = !wallPresent(SOUTH);
    e.exits[WEST]  = !wallPresent(WEST);

    // Clear all explored flags
    for (int i = 0; i < 4; i++) e.explored[i] = false;

    // Mark the arrival exit as explored (we came from there —
    // going back that way leads to the parent, not to new territory)
    if (!isStart) {
        Direction arrivedFrom = opposite(heading);
        e.explored[arrivedFrom] = true;
    }

    e.entryHeading = heading;
    e.type = isStart ? EVT_START : classifyEvent();

    return e;
}

// =====================================================================
//  DFS — CHOOSE NEXT EXIT
// =====================================================================
// Scans the event's exits in priority order and picks the first one
// that is both open AND unexplored.
//
// Returns true and sets `chosen` if an exit was found.
// Returns false if all open exits have been explored (backtrack).
bool chooseNextExit(Event& e, Direction& chosen) {
    for (int i = 0; i < 4; i++) {
        Direction d = EXPLORE_ORDER[i];
        if (e.exits[d] && !e.explored[d]) {
            e.explored[d] = true;    // mark it now before we leave
            chosen = d;
            return true;
        }
    }
    return false;
}

// =====================================================================
//  EVENT / JUNCTION DETECTION
// =====================================================================
bool junctionDetected() {
    if (millis() - lastDirectionChange < EVENT_COOLDOWN_MS) return false;

    // Front wall close — must stop
    if (frontDist() <= FRONT_STOP_DIST) return true;

    // Side wall disappeared — an opening appeared
    bool curLeft  = (leftDist()  <= SIDE_WALL_PRESENT);
    bool curRight = (rightDist() <= SIDE_WALL_PRESENT);

    bool opening = false;
    if (prevLeftWall  && !curLeft)  opening = true;
    if (prevRightWall && !curRight) opening = true;

    prevLeftWall  = curLeft;
    prevRightWall = curRight;

    return opening;
}

void resetSideWallMemory() {
    prevLeftWall  = (leftDist()  <= SIDE_WALL_PRESENT);
    prevRightWall = (rightDist() <= SIDE_WALL_PRESENT);
}

// =====================================================================
//  DEBUG — PRINT EVENT
// =====================================================================
void printEvent(int index, const Event& e) {
    Serial.print("  #"); Serial.print(index);
    Serial.print("  "); Serial.print(eventTypeName(e.type));
    Serial.print("  entered heading "); Serial.print(dirName(e.entryHeading));
    Serial.print("  exits[");
    for (int i = 0; i < 4; i++) {
        if (e.exits[i]) {
            Serial.print(dirArrow((Direction)i));
            Serial.print(e.explored[i] ? "* " : "  ");
        }
    }
    Serial.println("]  (* = explored)");
}

void printStack() {
    Serial.println("\n--- EVENT STACK (current path) ---");
    if (stackEmpty()) {
        Serial.println("  (empty)");
    } else {
        for (int i = 0; i <= stackTop; i++) {
            printEvent(i, eventStack[i]);
        }
    }
    Serial.print("  Stack depth: "); Serial.print(stackTop + 1);
    Serial.print(" / "); Serial.println(MAX_EVENTS);
    Serial.println();
}

// =====================================================================
//  STATE MACHINE TRANSITIONS
// =====================================================================
void enterState(RobotState newState) {
    state = newState;
    stateEntryTime = millis();

    switch (newState) {
        case ST_STARTUP:
            Serial.println("[STATE] STARTUP");
            break;

        case ST_WAIT_FOR_ENTRY:
            stopAll();
            Serial.println("[STATE] WAITING — place robot outside maze");
            Serial.println("        Will start when no walls detected...");
            break;

        case ST_ENTERING:
            mazeStartTime = millis();
            Serial.print("[STATE] ENTERING — driving ");
            Serial.println(dirName(heading));
            Serial.println("        Waiting for first wall...");
            resetPID();
            lastDirectionChange = millis();
            break;

        case ST_SCAN_EVENT:
            stopAll();
            break;

        case ST_DECIDE:
            break;

        case ST_FOLLOWING:
            Serial.print("[DRIVE]  Heading "); Serial.print(dirName(heading));
            Serial.println(backtracking ? "  (backtracking)" : "  (exploring)");
            resetPID();
            resetSideWallMemory();
            lastDirectionChange = millis();
            break;

        case ST_COMPLETE: {
            runTimes[currentRun] = millis() - mazeStartTime;
            stopAll();

            Serial.println();
            Serial.println("=========================================");
            Serial.print("  RUN "); Serial.print(currentRun + 1);
            Serial.println(" COMPLETE — robot exited the maze");
            Serial.print("  Time: ");
            Serial.print(runTimes[currentRun] / 1000.0f, 2);
            Serial.println(" seconds");
            Serial.print("  Events encountered: ");
            Serial.println(totalEventsRecorded);
            Serial.println("=========================================");

            if (currentRun < TOTAL_RUNS - 1) {
                // More runs to do — go to cooldown
                enterState(ST_COOLDOWN);
            } else {
                // All three runs done — print final results
                Serial.println();
                Serial.println("*****************************************");
                Serial.println("  ALL 3 RUNS COMPLETE");
                Serial.print("  Run 1 (left):  ");
                Serial.print(runTimes[0] / 1000.0f, 2); Serial.println("s");
                Serial.print("  Run 2 (right): ");
                Serial.print(runTimes[1] / 1000.0f, 2); Serial.println("s");
                Serial.print("  Run 3 (best):  ");
                Serial.print(runTimes[2] / 1000.0f, 2); Serial.println("s");
                Serial.print("  Fastest path was: ");
                Serial.println(runTimes[0] <= runTimes[1] ? "LEFT" : "RIGHT");
                Serial.println("*****************************************");
            }
            break;
        }

        case ST_COOLDOWN:
            stopAll();
            currentRun++;
            Serial.println();
            Serial.print("[COOLDOWN] 20 seconds before run ");
            Serial.println(currentRun + 1);
            Serial.println("           Place robot back at start...");
            break;
    }
}

// =====================================================================
//  SENSOR TEST PRINTOUT
// =====================================================================
void printSensorTest() {
    Serial.println("----------------------------------");
    Serial.print("            [N: "); Serial.print(distN, 1); Serial.println(" cm]");
    Serial.println("                 |");
    Serial.print("  [W: "); Serial.print(distW, 1);
    Serial.print(" cm] ---+--- [E: "); Serial.print(distE, 1); Serial.println(" cm]");
    Serial.println("                 |");
    Serial.print("            [S: "); Serial.print(distS, 1); Serial.println(" cm]");
    Serial.println();
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);

    S_FL.attach(PIN_FL);
    S_FR.attach(PIN_FR);
    S_RR.attach(PIN_RR);
    S_RL.attach(PIN_RL);

    stopAll();

    Serial.println("=========================================");
    switch (MODE) {
        case SENSOR_TEST:
            Serial.println("  SENSOR TEST MODE — motors disabled");
            break;
        case CORRIDOR_PID:
            Serial.println("  PID CORRIDOR FOLLOWING (no DFS)");
            Serial.print("  Kp="); Serial.print(Kp, 3);
            Serial.print("  Ki="); Serial.print(Ki, 4);
            Serial.print("  Kd="); Serial.println(Kd, 3);
            break;
        case MAZE_DFS:
            Serial.println("  EVENT-BASED DFS MAZE SOLVER");
            Serial.println("  3-RUN COMPETITION MODE");
            Serial.print("  Wall threshold: "); Serial.print(WALL_THRESHOLD);
            Serial.println(" cm");
            Serial.println("  Run 1: LEFT at T-junction");
            Serial.println("  Run 2: RIGHT at T-junction");
            Serial.println("  Run 3: fastest direction");
            Serial.print("  Cooldown between runs: ");
            Serial.print(COOLDOWN_MS / 1000);
            Serial.println("s");
            break;
    }
    Serial.println("=========================================\n");

    delay(2000);
    previousTime = millis();

    if (MODE == MAZE_DFS) {
        enterState(ST_STARTUP);
    }
}

// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {
    bool newData = readSensorsUART();

    // ---------- SENSOR TEST MODE ----------
    if (MODE == SENSOR_TEST) {
        if (newData) printSensorTest();
        return;
    }

    // ---------- STANDALONE CORRIDOR PID ----------
    if (MODE == CORRIDOR_PID) {
        if (!newData) return;

        float error = distW - distE;
        float correction = calculatePID(error);

        if (distN < FRONT_STOP_DIST) {
            stopAll();
            Serial.println("WALL AHEAD — STOPPED");
            delay(500);
            return;
        }

        drive(correction, FORWARD_SPEED, 0);

        Serial.print("N:"); Serial.print(distN, 1);
        Serial.print(" E:"); Serial.print(distE, 1);
        Serial.print(" S:"); Serial.print(distS, 1);
        Serial.print(" W:"); Serial.print(distW, 1);
        Serial.print(" | err:"); Serial.print(error, 2);
        Serial.print(" cor:"); Serial.println(correction, 3);
        return;
    }

    // ========== MAZE DFS STATE MACHINE ==========
    if (!newData) return;

    unsigned long elapsed = millis() - stateEntryTime;

    switch (state) {

        // ---------------------------------------------------------
        //  STARTUP — boot delay, then wait for entry conditions
        // ---------------------------------------------------------
        case ST_STARTUP:
            if (elapsed > 500) {
                enterState(ST_WAIT_FOR_ENTRY);
            }
            break;

        // ---------------------------------------------------------
        //  WAIT FOR ENTRY — motors off, robot is outside the maze
        // ---------------------------------------------------------
        //  The robot has been placed outside the maze.  It waits
        //  until the sensors confirm open space on all four sides
        //  (no walls ≤ 5cm in any direction).  Once confirmed, it
        //  begins driving forward into the maze.
        //
        //  Start condition:  no walls + no events recorded
        // ---------------------------------------------------------
        case ST_WAIT_FOR_ENTRY: {
            if (noWallsDetected() && totalEventsRecorded == 0) {
                Serial.println("[START]  Open space confirmed — entering maze");
                backtracking = false;
                enterState(ST_ENTERING);
            }
            break;
        }

        // ---------------------------------------------------------
        //  ENTERING — driving into the maze, no PID centering yet
        // ---------------------------------------------------------
        //  The robot drives forward at creep speed.  There are no
        //  walls to centre on yet, so PID is not used.  As soon as
        //  a wall appears on any side, the robot has entered the
        //  maze and normal DFS begins.
        // ---------------------------------------------------------
        case ST_ENTERING: {
            // Drive straight forward (no correction — no walls to centre on)
            driveHeading(0, CREEP_SPEED);

            if (anyWallDetected()) {
                stopAll();
                Serial.println("[ENTRY]  Wall detected — inside maze");
                Serial.print("         N="); Serial.print(distN, 1);
                Serial.print("  E="); Serial.print(distE, 1);
                Serial.print("  S="); Serial.print(distS, 1);
                Serial.print("  W="); Serial.println(distW, 1);
                enterState(ST_SCAN_EVENT);
            }
            break;
        }

        // ---------------------------------------------------------
        //  SCAN EVENT — robot is stopped, sensors are being read
        // ---------------------------------------------------------
        //  Two cases:
        //    1. Exploring (backtracking == false)
        //       → Create a new Event from the sensors, push it
        //    2. Backtracking (backtracking == true)
        //       → We've returned to the event already on the stack
        //         Don't create anything — just proceed to DECIDE
        // ---------------------------------------------------------
        case ST_SCAN_EVENT: {
            if (elapsed < 300) break;    // let sensors settle

            if (backtracking) {
                // We've physically returned to the event on top of the stack.
                // Its explored[] flags already reflect what we've tried.
                Serial.print("[RETURN] Back at event #");
                Serial.println(stackTop);
                printEvent(stackTop, stackPeek());

            } else {
                // New event discovered — read sensors and save it
                bool isStart = stackEmpty();
                Event e = createEvent(isStart);
                stackPush(e);

                Serial.print("[NEW]    Event #");
                Serial.print(stackTop);
                Serial.print("  ");
                Serial.println(eventTypeName(e.type));
                printEvent(stackTop, e);

                // Print sensor snapshot
                Serial.print("         N="); Serial.print(distN, 1);
                Serial.print("  E="); Serial.print(distE, 1);
                Serial.print("  S="); Serial.print(distS, 1);
                Serial.print("  W="); Serial.println(distW, 1);
            }

            enterState(ST_DECIDE);
            break;
        }

        // ---------------------------------------------------------
        //  DECIDE — DFS picks the next direction
        // ---------------------------------------------------------
        //  COMPETITION OVERRIDE:
        //    If this is the FIRST T-junction of the current run
        //    and it hasn't been handled yet, force the direction:
        //      Run 1 → LEFT,  Run 2 → RIGHT,  Run 3 → fastest
        //    After the forced turn, DFS operates normally.
        //
        //  NORMAL DFS:
        //    • If an unexplored open exit exists → take it
        //    • If all exits explored → pop and backtrack
        // ---------------------------------------------------------
        case ST_DECIDE: {
            Direction chosen;
            Event& current = stackPeek();

            // --- Competition T-junction override ---
            if (current.type == EVT_T_JUNCTION && !tJunctionHandled) {
                tJunctionHandled = true;
                Direction entry = current.entryHeading;

                if (currentRun == 0) {
                    // Run 1: force LEFT
                    chosen = leftOf(entry);
                    Serial.println("[COMP]   Run 1 — forcing LEFT at T-junction");
                } else if (currentRun == 1) {
                    // Run 2: force RIGHT
                    chosen = rightOf(entry);
                    Serial.println("[COMP]   Run 2 — forcing RIGHT at T-junction");
                } else {
                    // Run 3: take the faster direction
                    if (runTimes[0] <= runTimes[1]) {
                        chosen = leftOf(entry);
                        Serial.println("[COMP]   Run 3 — LEFT was faster, going LEFT");
                    } else {
                        chosen = rightOf(entry);
                        Serial.println("[COMP]   Run 3 — RIGHT was faster, going RIGHT");
                    }
                }

                // Mark both the chosen exit and the arrival exit as explored
                // so DFS doesn't revisit them if we backtrack here later
                current.explored[chosen] = true;
                heading = chosen;
                backtracking = false;

                Serial.print("[EXPLORE] Taking exit ");
                Serial.print(dirName(chosen));
                Serial.print(" from event #");
                Serial.println(stackTop);

                enterState(ST_FOLLOWING);

            } else if (chooseNextExit(current, chosen)) {
                // --- Normal DFS: unexplored exit available ---
                heading = chosen;
                backtracking = false;

                Serial.print("[EXPLORE] Taking exit ");
                Serial.print(dirName(chosen));
                Serial.print(" from event #");
                Serial.println(stackTop);

                enterState(ST_FOLLOWING);

            } else {
                // --- BACKTRACK: all exits at this event are used ---
                Event popped = stackPop();

                Serial.print("[BACKTRK] All exits explored at #");
                Serial.println(stackTop + 1);

                heading = opposite(popped.entryHeading);
                backtracking = true;

                if (stackEmpty()) {
                    Serial.println("[DFS]    Stack empty — heading toward maze exit");
                    backtracking = false;
                } else {
                    Serial.print("          Reversing ");
                    Serial.print(dirName(heading));
                    Serial.print(" toward event #");
                    Serial.println(stackTop);
                }

                enterState(ST_FOLLOWING);
            }
            break;
        }

        // ---------------------------------------------------------
        //  FOLLOWING — PID corridor follow until next event
        // ---------------------------------------------------------
        case ST_FOLLOWING: {
            // ---- EXIT DETECTION ----
            // If all four sensors show open space and we have events
            // recorded, the robot has left the maze entirely.
            // Since there are no four-way crossroads in this maze,
            // "no walls anywhere" can ONLY mean we are outside.
            if (noWallsDetected() && totalEventsRecorded > 0) {
                stopAll();
                Serial.println();
                Serial.println("[EXIT]   No walls detected — robot has left the maze!");
                enterState(ST_COMPLETE);
                break;
            }

            // ---- JUNCTION DETECTION ----
            // Skip if the stack is empty — the robot is heading for
            // the maze exit and should ignore intermediate junctions.
            if (!stackEmpty() && junctionDetected()) {
                stopAll();
                Serial.println("[EVENT]  Junction/event detected — stopping");
                enterState(ST_SCAN_EVENT);
                break;
            }

            // PID centering
            float error      = getCenteringError();
            float correction = calculatePID(error);

            // Slow down near front walls
            float speed = FORWARD_SPEED;
            if (frontDist() < 15.0f) speed = CREEP_SPEED;

            driveHeading(correction, speed);

            // Throttled debug
            static uint8_t dbg = 0;
            if (++dbg >= 10) {
                dbg = 0;
                Serial.print("  ");
                Serial.print(dirArrow(heading));
                Serial.print(" F:"); Serial.print(frontDist(), 1);
                Serial.print(" L:"); Serial.print(leftDist(), 1);
                Serial.print(" R:"); Serial.print(rightDist(), 1);
                Serial.print(" err:"); Serial.print(error, 2);
                Serial.print(" cor:"); Serial.print(correction, 3);
                Serial.print(backtracking ? " [BT]" : "");
                Serial.println();
            }
            break;
        }

        // ---------------------------------------------------------
        //  COMPLETE — run finished, waiting or all done
        // ---------------------------------------------------------
        case ST_COMPLETE:
            // If more runs remain, enterState already transitioned
            // to ST_COOLDOWN.  If all runs done, robot stays here.
            break;

        // ---------------------------------------------------------
        //  COOLDOWN — 20 second pause between runs
        // ---------------------------------------------------------
        //  Motors off.  Timer counts down.  When elapsed, reset
        //  per-run state and go back to WAIT_FOR_ENTRY.
        // ---------------------------------------------------------
        case ST_COOLDOWN: {
            if (elapsed >= COOLDOWN_MS) {
                resetForNewRun();
                Serial.println();
                Serial.print("[READY]  Starting run ");
                Serial.println(currentRun + 1);
                enterState(ST_WAIT_FOR_ENTRY);
            }
            break;
        }
    }
}
