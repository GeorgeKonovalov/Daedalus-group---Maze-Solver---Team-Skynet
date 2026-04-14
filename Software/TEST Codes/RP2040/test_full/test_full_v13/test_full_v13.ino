#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>

// ======================================================
// Hardware pins
// ======================================================
constexpr uint8_t ENC_A_PIN   = 7;
constexpr uint8_t ENC_B_PIN   = 6;
constexpr uint8_t ENC_BTN_PIN = 8;

// ===== SERVO PINS =====
const uint8_t PIN_FL = 5;
const uint8_t PIN_FR = 3;
const uint8_t PIN_RR = 9;
const uint8_t PIN_RL = 4;

// ===== IR PINS =====
const uint8_t PIN_IR_FL = 16;
const uint8_t PIN_IR_FR = 17;
const uint8_t PIN_IR_RL = 14;
const uint8_t PIN_IR_RR = 15;

// OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ======================================================
// Timing / button
// ======================================================
constexpr uint32_t BUTTON_DEBOUNCE_MS = 25;
constexpr uint32_t BUTTON_LONGPRESS_MS = 700;

// ======================================================
// Settings storage (fixed-point x100)
// ======================================================
struct RuntimeSettings {
  int p_x100 = 7;
  int i_x100 = 0;
  int d_x100 = 0;
  int pidCurveB_x100 = 100;
  int pidWallDistance_x100 = 600;

  // Pair gains
  int pairFLRR_fwd_x100 = 100;
  int pairFLRR_back_x100 = 100;
  int pairFRRL_fwd_x100 = 100;
  int pairFRRL_back_x100 = 100;

  // Per-wheel gains
  int fl_fwd_x100 = 100;
  int fl_back_x100 = 100;
  int fr_fwd_x100 = 100;
  int fr_back_x100 = 100;
  int rr_fwd_x100 = 100;
  int rr_back_x100 = 100;
  int rl_fwd_x100 = 100;
  int rl_back_x100 = 100;

  int junctionStopDistance_x100 = 200;
  int wallThreshold_x100 = 600;
  int turningThreshold_x100 = 2600;
  int waitBeforeTurn_x100 = 500;
  int irStableTime_x100 = 10;
  int deadEndDistance_x100 = 1600;
  int finishDistance_x100 = 3000;
  uint8_t mode = 1;
};

RuntimeSettings activeSettings;
RuntimeSettings workingSettings;

// ======================================================
// Runtime globals
// ======================================================
float gP = 0.0f;
float gI = 0.0f;
float gD = 0.0f;
float gPidCurveB = 1.0f;
float gPidWallDistanceCm = 6.0f;
float gJunctionStopDistance = 2.0f;
float gWallThresholdCm = 6.0f;
float gTurningThresholdCm = 26.0f;
float gWaitBeforeTurnSeconds = 5.0f;
float gIRStableTimeSeconds = 0.10f;
float gDeadEndDistanceCm = 16.0f;
float gFinishDistanceCm = 30.0f;
uint8_t gMode = 1;

// Pair runtime gains
float gPairFLRRFwd = 1.00f;
float gPairFLRRBack = 1.00f;
float gPairFRRLFwd = 1.00f;
float gPairFRRLBack = 1.00f;

// Per-wheel runtime gains
float gFLFwd = 1.00f;
float gFLBack = 1.00f;
float gFRFwd = 1.00f;
float gFRBack = 1.00f;
float gRRFwd = 1.00f;
float gRRBack = 1.00f;
float gRLFwd = 1.00f;
float gRLBack = 1.00f;

// ======================================================
// Runtime / trial timing globals
// ======================================================
uint32_t executionStartMs = 0;
uint32_t frozenExecutionDurationMs = 0;
uint32_t lastAttemptDurationMs = 0;
uint32_t sequenceTotalDurationMs = 0;
uint32_t executionAttempt = 0;
uint32_t phaseStartedMs = 0;
uint32_t lastUltrasonicUpdateMs = 0;

// ======================================================
// Encoder shared state
// ======================================================
volatile int8_t g_encoderDelta = 0;
volatile bool g_irSensorInterruptFlag = false;

// ======================================================
// Button handling
// ======================================================
enum class ButtonEvent : uint8_t {
  None,
  ShortPress,
  LongPress
};

struct ButtonState {
  bool stableLevel = HIGH;
  bool lastRawLevel = HIGH;
  uint32_t lastDebounceMs = 0;
  uint32_t pressedAtMs = 0;
  bool longPressFired = false;
};

ButtonState buttonState;
bool suppressButtonUntilRelease = false;

// ======================================================
// Runtime event names
// ======================================================
enum class EventName : uint8_t {
  Idle,
  Start,
  SensorWait,
  Corridor,
  TJunction,
  DeadEnd,
  LeftTurn,
  RightTurn,
  TJunctionStraight,
  Random,
  ApproachWall,
  WaitAtTurn,
  Finish,
  RestartReady
};

// ======================================================
// Menu / screen state
// ======================================================
enum class Screen : uint8_t {
  Root,
  StartSettings,
  Settings,
  PID,
  PIDTest,
  MotorTune,
  EventTestMenu,
  EventTestConfig,
  EventTestRun,
  MazeTestRun,
  IRSensorTest,
  UltrasonicSensorTest,
  DigitEditor,
  StartConfirm,
  RunScreen
};

Screen currentScreen = Screen::Root;
Screen previousScreen = Screen::Root;

int rootIndex = 0;          // 0=Start, 1=Start Settings, 2=Settings, 3=Event Test, 4=Maze Test, 5=IR, 6=US
int startSettingsIndex = 0; // 0=Mode, 1=Wait, 2=Back
int settingsIndex = 0;      // 0=PID, 1=Motor Tune, 2=Stop Dist, 3=Wall Thr, 4=Turn Thr, 5=Save
int pidIndex = 0;           // 0=P, 1=I, 2=D, 3=b, 4=WallDist, 5=PID Test, 6=Back
int motorTuneIndex = 0;
int eventTestIndex = 0;
int eventTestConfigIndex = 0;
int irSensorIndex = 0;

constexpr int ROOT_COUNT = 7;
constexpr int START_SETTINGS_COUNT = 3;
constexpr int SETTINGS_COUNT = 6;
constexpr int PID_COUNT = 7;
constexpr int IR_SENSOR_COUNT = 6;

// ======================================================
// Setup / runtime control
// ======================================================
bool programStarted = false;
bool autoStartExecution = false;
bool Execution_state = false;
EventName currentEventName = EventName::Idle;
EventName lastFinishedEventName = EventName::Idle;

// ======================================================
// Digit editor
// ======================================================
struct DigitEditor {
  bool active = false;
  bool digitUnlocked = false;
  uint8_t selectedDigit = 0;
  int tempValue = 0;
  int* targetValue = nullptr;
  int minValue = 0;
  int maxValue = 9999;
  const char* label = "";
};

DigitEditor digitEditor;

// ======================================================
// Movement code
// ======================================================
Servo S_FL, S_FR, S_RR, S_RL;

// ===== SERVO CALIBRATION =====
int STOP_FL = 90, STOP_FR = 90, STOP_RR = 90, STOP_RL = 90;
int RANGE_FL = 90, RANGE_FR = 90, RANGE_RR = 90, RANGE_RL = 90;
int INV_FL = +1, INV_FR = -1, INV_RR = -1, INV_RL = +1;

// ===== SENSOR READINGS =====
float distN = 30.0f, distE = 30.0f, distS = 30.0f, distW = 30.0f;
float avgDistN = 30.0f, avgDistE = 30.0f, avgDistS = 30.0f, avgDistW = 30.0f;
float interpretedNS = 0.0f;
float interpretedEW = 0.0f;
float pidPreviewRawError = 0.0f;
float pidPreviewInterpretedError = 0.0f;
float pidPreviewCorrection = 0.0f;
int irFL = HIGH, irFR = HIGH, irRL = HIGH, irRR = HIGH;
bool ultrasonicDataValid = false;
String incomingData = "";

constexpr int ULTRASONIC_AVG_COUNT = 5;
float ultrasonicBufN[ULTRASONIC_AVG_COUNT] = {0.0f};
float ultrasonicBufE[ULTRASONIC_AVG_COUNT] = {0.0f};
float ultrasonicBufS[ULTRASONIC_AVG_COUNT] = {0.0f};
float ultrasonicBufW[ULTRASONIC_AVG_COUNT] = {0.0f};
int ultrasonicBufIndex = 0;
int ultrasonicSampleCount = 0;

struct SensorSnapshot {
  uint8_t irMask = 0;
  float distN = 0.0f;
  float distE = 0.0f;
  float distS = 0.0f;
  float distW = 0.0f;
  bool ultrasonicValid = false;
};

struct SensorObservation {
  uint8_t irMask = 0;
  EventName candidateEvent = EventName::SensorWait;
  bool ultrasonicValid = false;
};

SensorSnapshot lastObservedSnapshot;
SensorSnapshot stableSnapshot;
uint32_t sensorLastChangeMs = 0;
uint32_t sensorChangeVersion = 0;
uint32_t sensorStableVersion = 0;
uint32_t mainHandledStableVersion = 0;
uint32_t eventTestHandledStableVersion = 0;
uint32_t mazeTestHandledStableVersion = 0;

// ===== PID =====
float Kp = 0.07f;
float Ki = 0.00f;
float Kd = 0.00f;
float previousError = 0.0f;
float integral = 0.0f;
unsigned long previousTime = 0;

const float INTEGRAL_LIMIT   = 20.0f;
const float CORRECTION_LIMIT = 0.5f;

// ===== MOVEMENT =====
const float BASE_SPEED = 0.40f;
const float TUNE_SPEED = 0.35f;
const float APPROACH_SPEED = 0.28f;
const float FRONT_STOP_THRESHOLD = 10.0f;
const float ULTRASONIC_STALE_MS = 1200.0f;
const float SENSOR_MAX_VALID_CM = 400.0f;
const float EVENT_CONFIRM_MARGIN_CM = 0.5f;
const uint32_t EXECUTION_ACQUIRE_TIMEOUT_MS = 2500;
const uint32_t EVENT_TEST_ACQUIRE_TIMEOUT_MS = 2500;
const uint32_t MAZE_TEST_ACQUIRE_TIMEOUT_MS = 2500;

// ======================================================
// Logical heading
// ======================================================
enum class Heading : uint8_t {
  North,
  East,
  South,
  West
};

Heading currentHeading = Heading::North;

enum class ExecutionPhase : uint8_t {
  Idle,
  StartSeek,
  Corridor,
  ApproachWall,
  WaitAtTurn,
  AcquireWall,
  Finished
};

ExecutionPhase executionPhase = ExecutionPhase::Idle;
Heading pendingHeading = Heading::North;
EventName executionResolvedEvent = EventName::Idle;
bool alternateTurnRightNext = false;
bool sequenceCompleted = false;

Heading pidTestHeading = Heading::North;
float pidPreviewIntegral = 0.0f;
float pidPreviewPreviousError = 0.0f;
uint32_t pidPreviewPreviousMs = 0;

enum class EventTestCase : uint8_t {
  Corridor,
  TJunction,
  DeadEnd,
  Start,
  Finish,
  LeftTurn,
  RightTurn,
  TJunctionStraight,
  Random,
  Back
};

EventTestCase selectedEventTest = EventTestCase::Corridor;
bool eventTestActive = false;

constexpr int EVENT_TEST_COUNT = 10;
constexpr int EVENT_TEST_CONFIG_COUNT = 7;

enum class TurnChoice : uint8_t {
  Left,
  Right
};

struct EventTestConfig {
  uint8_t heading = 0;              // 0=N,1=E,2=S,3=W
  int stopThreshold_x100 = 1000;    // cm
  int sideThreshold_x100 = 1800;    // cm
  int actionTime_x100 = 150;        // s
  int moveTime_x100 = 150;          // s
  TurnChoice turnChoice = TurnChoice::Left;
};

EventTestConfig eventTestConfigs[EVENT_TEST_COUNT];

enum class EventTestPhase : uint8_t {
  Idle,
  Detect,
  Reverse,
  ApproachWall,
  WaitBeforeTurn,
  AcquireWall,
  FinishStraight,
  Hold,
  StartSeek,
  StraightToSideThreshold
};

EventTestPhase eventTestPhase = EventTestPhase::Idle;
Heading eventTestPendingHeading = Heading::North;
EventName eventTestResolvedEvent = EventName::Idle;
uint32_t eventTestStartedMs = 0;

enum class MazeTestPhase : uint8_t {
  Idle,
  StartSeek,
  Corridor,
  ApproachWall,
  WaitAtTurn,
  AcquireWall,
  Finished
};

bool mazeTestActive = false;
MazeTestPhase mazeTestPhase = MazeTestPhase::Idle;
Heading mazeTestHeading = Heading::North;
Heading mazeTestPendingHeading = Heading::North;
EventName mazeTestEvent = EventName::Idle;
EventName mazeTestResolvedEvent = EventName::Idle;
uint32_t mazeTestStartedMs = 0;
uint32_t mazeTestFrozenDurationMs = 0;
uint32_t mazeTestAttempt = 0;

// ======================================================
// Motor tune test mode
// ======================================================
enum class TuneMotion : uint8_t {
  Stop,
  North,
  South,
  East,
  West
};

TuneMotion tuneMotion = TuneMotion::Stop;

constexpr int MOTOR_TUNE_COUNT = 14;

// ======================================================
// Prototypes
// ======================================================
int clampInt(int v, int lo, int hi);
String formatX100(int value);
String formatMsAsSeconds(uint32_t ms);
String formatDistanceCm(float value);
String formatSignedFloat(float value, uint8_t decimals);
int digitPlaceValue(uint8_t digitIndex);
int getDigitAt(int value, uint8_t digitIndex);
int setDigitAt(int value, uint8_t digitIndex, int newDigit);
void incrementSelectedDigit(int step);
bool parseFloatToken(const String& token, float& outValue);
float interpretValue(float x);
void pushUltrasonicAverages(float north, float east, float south, float west);
float averagedDistanceForHeading(Heading h);
float corridorRawErrorForHeading(Heading h);
float corridorInterpretedErrorForHeading(Heading h);
void resetPIDPreview();
void updatePIDPreview();
bool isIRTriggered(int value);
uint8_t buildIRMask();
SensorSnapshot captureSensorSnapshot();
SensorObservation buildSensorObservation(const SensorSnapshot& snapshot, Heading heading, float deadEndThresholdCm,
                                        float turningThresholdCm, float wallThresholdCm, float finishThresholdCm);
bool sensorObservationChanged(const SensorObservation& a, const SensorObservation& b);
bool sensorSnapshotChanged(const SensorSnapshot& a, const SensorSnapshot& b);
void updateSensorStability();
bool consumeStableSensorSnapshot(uint32_t& handledVersion, SensorSnapshot& outSnapshot);
float distanceForHeadingInSnapshot(const SensorSnapshot& snapshot, Heading h);
bool frontPairTriggeredMask(uint8_t irMask);
bool rearPairTriggeredMask(uint8_t irMask);
bool allIRTriggeredMask(uint8_t irMask);
bool allIRClearMask(uint8_t irMask);
EventName classifySensorEventWithThresholds(const SensorSnapshot& snapshot, Heading heading, bool allowFinishEvent,
                                            float deadEndThresholdCm, float turningThresholdCm, float wallThresholdCm,
                                            float finishThresholdCm);
EventName classifyStableSensorEvent(const SensorSnapshot& snapshot, Heading heading, bool allowFinishEvent);

void onEncoderAChange();
void onAnyIRSensorChange();
int consumeEncoderDelta();
ButtonEvent pollButtonEvent();

void commitGlobalsForRuntime();
void beginExecutionAttempt();
void finishExecutionAttempt();
void resetSequenceState();
uint8_t effectiveRoutingMode();
const char* effectiveRoutingModeToString();
const char* nextAttemptLabel();
bool modeUsesSequence();
void refreshSensors();
void readIRSensors();
bool ultrasonicFresh();
Heading headingFromIndex(uint8_t idx);
uint8_t headingToIndex(Heading h);
Heading turnLeftOf(Heading h);
Heading turnRightOf(Heading h);
Heading oppositeOf(Heading h);
float distanceForHeading(Heading h);
float frontDistance();
float leftDistance();
float rightDistance();
float backDistance();
float frontDistanceForHeading(Heading h);
float leftDistanceForHeading(Heading h);
float rightDistanceForHeading(Heading h);
bool leftOpen();
bool rightOpen();
Heading chooseNextHeading(bool leftIsOpen, bool rightIsOpen);
Heading chooseNextHeadingFromSnapshot(const SensorSnapshot& snapshot, Heading heading);
const char* modeShortToString(uint8_t mode);
const char* modeToString(uint8_t mode);
const char* eventNameToString(EventName name);
EventName eventForTurnHeading(Heading targetHeading);
const char* eventTestCaseToString(EventTestCase testCase);
EventName eventTestCaseToEventName(EventTestCase testCase);
const char* turnChoiceToString(TurnChoice choice);
EventTestConfig& currentEventTestConfig();
float currentEventTestStopThreshold();
float currentEventTestSideThreshold();
uint32_t currentEventTestActionMs();
uint32_t currentEventTestMoveMs();
Heading currentEventTestHeading();
Heading currentEventTestConfiguredTurn();
bool frontIRTriggered();
bool finishIRTriggered();
bool sideUltrasonicWithin(float threshold);
bool bothWallsWithin(float threshold);
bool hasLeftWallWithin(float threshold);
bool hasRightWallWithin(float threshold);
void startEventTestRun();
void resetEventTestRun();

void openDigitEditor(const char* label, int* target, int minValue, int maxValue);
void closeDigitEditor(bool commit);

void handleRootInput(int delta, ButtonEvent btn);
void handleStartConfirmInput(ButtonEvent btn);
void handleStartSettingsInput(int delta, ButtonEvent btn);
void handleSettingsInput(int delta, ButtonEvent btn);
void handlePIDInput(int delta, ButtonEvent btn);
void handlePIDTestInput(ButtonEvent btn);
void handleMotorTuneInput(int delta, ButtonEvent btn);
void handleEventTestMenuInput(int delta, ButtonEvent btn);
void handleEventTestConfigInput(int delta, ButtonEvent btn);
void handleEventTestRunInput(ButtonEvent btn);
void handleMazeTestRunInput(ButtonEvent btn);
void handleIRSensorTestInput(int delta, ButtonEvent btn);
void handleUltrasonicSensorTestInput(ButtonEvent btn);
void handleDigitEditorInput(int delta, ButtonEvent btn);

void drawHeader(const char* title);
void drawMenuItem(int y, bool selected, const String& text);
void drawScrollableItemList(const String items[], int itemCount, int selectedIndex, int startY);
void drawRootScreen();
void drawStartConfirmScreen();
void drawStartSettingsScreen();
void drawSettingsScreen();
void drawPIDScreen();
void drawPIDTestScreen();
void drawMotorTuneScreen();
void drawEventTestMenuScreen();
void drawEventTestConfigScreen();
void drawEventTestRunScreen();
void drawMazeTestRunScreen();
void drawIRSensorTestScreen();
void drawUltrasonicSensorTestScreen();
void drawDigitEditorScreen();
void drawRunScreen();
void drawUI();

void runMenuInSetup();
void ScreenLoopUpdate(EventName event_name, bool stop_machine);

static inline float clamp1(float v);
void writeCRServo(Servo &s, int stopVal, int rangeVal, int inv, float cmd);
void stopAll();
void drive(float Vx, float Vy);
void driveHeading(float forwardCmd, float lateralCmd);
void runTuneMotion();
void parseDistances(String data);
bool readSensorsUART();
float calculatePID(float error);
void followWallAwareHeading(float forwardSpeed, float wallThresholdCm, Heading heading);
void followCorridor();
void updateExecutionLogic(bool& stopNow, EventName& displayedEvent);
void runSelectedEventTest();
void startMazeTest();
void resetMazeTest();
void updateMazeTestLogic();

const char* headingToString(Heading h);
const char* tuneMotionToString(TuneMotion t);

// ======================================================
// Helpers
// ======================================================
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

String formatX100(int value) {
  value = clampInt(value, 0, 9999);
  int whole = value / 100;
  int frac = value % 100;
  char buf[12];
  snprintf(buf, sizeof(buf), "%02d.%02d", whole, frac);
  return String(buf);
}

String formatMsAsSeconds(uint32_t ms) {
  uint32_t totalSec = ms / 1000;
  uint32_t centis = (ms % 1000) / 10;
  char buf[20];
  snprintf(buf, sizeof(buf), "%lu.%02lu s", (unsigned long)totalSec, (unsigned long)centis);
  return String(buf);
}

String formatDistanceCm(float value) {
  if (!ultrasonicDataValid || value < 0.0f || value > SENSOR_MAX_VALID_CM) {
    return "--.-";
  }

  char buf[12];
  snprintf(buf, sizeof(buf), "%04.1f", value);
  return String(buf);
}

String formatSignedFloat(float value, uint8_t decimals) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%+.*f", decimals, value);
  return String(buf);
}

float interpretValue(float x) {
  float absX = fabs(x);
  float sign = (x >= 0.0f) ? 1.0f : -1.0f;
  if (absX < 0.6f) {
    if (absX <= 0.0f) return 0.0f;
    return sign * 0.6f * pow(absX / 0.6f, gPidCurveB);
  }
  return x;
}

void pushUltrasonicAverages(float north, float east, float south, float west) {
  ultrasonicBufN[ultrasonicBufIndex] = north;
  ultrasonicBufE[ultrasonicBufIndex] = east;
  ultrasonicBufS[ultrasonicBufIndex] = south;
  ultrasonicBufW[ultrasonicBufIndex] = west;

  ultrasonicBufIndex = (ultrasonicBufIndex + 1) % ULTRASONIC_AVG_COUNT;
  if (ultrasonicSampleCount < ULTRASONIC_AVG_COUNT) ultrasonicSampleCount++;

  float sumN = 0.0f;
  float sumE = 0.0f;
  float sumS = 0.0f;
  float sumW = 0.0f;
  for (int i = 0; i < ultrasonicSampleCount; ++i) {
    sumN += ultrasonicBufN[i];
    sumE += ultrasonicBufE[i];
    sumS += ultrasonicBufS[i];
    sumW += ultrasonicBufW[i];
  }

  avgDistN = sumN / ultrasonicSampleCount;
  avgDistE = sumE / ultrasonicSampleCount;
  avgDistS = sumS / ultrasonicSampleCount;
  avgDistW = sumW / ultrasonicSampleCount;
  interpretedNS = interpretValue(avgDistN - avgDistS);
  interpretedEW = interpretValue(avgDistE - avgDistW);
}

float averagedDistanceForHeading(Heading h) {
  if (ultrasonicSampleCount <= 0) {
    return distanceForHeading(h);
  }

  switch (h) {
    case Heading::North: return avgDistN;
    case Heading::East:  return avgDistE;
    case Heading::South: return avgDistS;
    case Heading::West:  return avgDistW;
    default:             return avgDistN;
  }
}

float corridorRawErrorForHeading(Heading h) {
  float left = averagedDistanceForHeading(turnLeftOf(h));
  float right = averagedDistanceForHeading(turnRightOf(h));
  return right - left;
}

float corridorInterpretedErrorForHeading(Heading h) {
  return interpretValue(corridorRawErrorForHeading(h));
}

void resetPIDPreview() {
  pidPreviewIntegral = 0.0f;
  pidPreviewPreviousError = 0.0f;
  pidPreviewCorrection = 0.0f;
  pidPreviewRawError = 0.0f;
  pidPreviewInterpretedError = 0.0f;
  pidPreviewPreviousMs = millis();
}

void updatePIDPreview() {
  Kp = gP;
  Ki = gI;
  Kd = gD;
  bool leftWallPresent = averagedDistanceForHeading(turnLeftOf(pidTestHeading)) <= gWallThresholdCm;
  bool rightWallPresent = averagedDistanceForHeading(turnRightOf(pidTestHeading)) <= gWallThresholdCm;

  if (leftWallPresent && rightWallPresent) {
    pidPreviewRawError = corridorRawErrorForHeading(pidTestHeading);
  } else if (leftWallPresent) {
    pidPreviewRawError = gPidWallDistanceCm - averagedDistanceForHeading(turnLeftOf(pidTestHeading));
  } else if (rightWallPresent) {
    pidPreviewRawError = averagedDistanceForHeading(turnRightOf(pidTestHeading)) - gPidWallDistanceCm;
  } else {
    pidPreviewRawError = 0.0f;
  }

  pidPreviewInterpretedError = interpretValue(pidPreviewRawError);

  uint32_t now = millis();
  float dt = (now - pidPreviewPreviousMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  if (dt > 0.2f) dt = 0.2f;

  float pTerm = Kp * pidPreviewInterpretedError;
  pidPreviewIntegral += pidPreviewInterpretedError * dt;
  pidPreviewIntegral = constrain(pidPreviewIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float iTerm = Ki * pidPreviewIntegral;
  float derivative = (pidPreviewInterpretedError - pidPreviewPreviousError) / dt;
  float dTerm = Kd * derivative;

  pidPreviewCorrection = constrain(pTerm + iTerm + dTerm, -CORRECTION_LIMIT, CORRECTION_LIMIT);
  pidPreviewPreviousError = pidPreviewInterpretedError;
  pidPreviewPreviousMs = now;
}

int digitPlaceValue(uint8_t digitIndex) {
  switch (digitIndex) {
    case 0: return 1000;
    case 1: return 100;
    case 2: return 10;
    case 3: return 1;
    default: return 1;
  }
}

int getDigitAt(int value, uint8_t digitIndex) {
  value = clampInt(value, 0, 9999);
  switch (digitIndex) {
    case 0: return (value / 1000) % 10;
    case 1: return (value / 100) % 10;
    case 2: return (value / 10) % 10;
    case 3: return value % 10;
    default: return 0;
  }
}

int setDigitAt(int value, uint8_t digitIndex, int newDigit) {
  value = clampInt(value, 0, 9999);
  newDigit = clampInt(newDigit, 0, 9);
  int place = digitPlaceValue(digitIndex);
  int oldDigit = getDigitAt(value, digitIndex);
  int newValue = value + (newDigit - oldDigit) * place;
  return clampInt(newValue, 0, 9999);
}

void incrementSelectedDigit(int step) {
  if (!digitEditor.active) return;

  int oldDigit = getDigitAt(digitEditor.tempValue, digitEditor.selectedDigit);
  int newDigit = oldDigit + step;

  if (newDigit < 0) newDigit = 9;
  if (newDigit > 9) newDigit = 0;

  int newValue = setDigitAt(digitEditor.tempValue, digitEditor.selectedDigit, newDigit);
  digitEditor.tempValue = clampInt(newValue, digitEditor.minValue, digitEditor.maxValue);
}

bool parseFloatToken(const String& token, float& outValue) {
  String trimmed = token;
  trimmed.trim();

  if (trimmed.length() == 0) return false;

  bool hasDigit = false;
  for (uint16_t i = 0; i < trimmed.length(); ++i) {
    char c = trimmed.charAt(i);
    if (isDigit(c)) {
      hasDigit = true;
      continue;
    }
    if (c == '.' || c == '-' || c == '+') continue;
    return false;
  }

  if (!hasDigit) return false;

  float parsed = trimmed.toFloat();
  if (parsed < 0.0f || parsed > SENSOR_MAX_VALID_CM) return false;

  outValue = parsed;
  return true;
}

bool isIRTriggered(int value) {
  return value == LOW;
}

uint8_t buildIRMask() {
  uint8_t mask = 0;
  if (isIRTriggered(irFL)) mask |= 0x01;
  if (isIRTriggered(irFR)) mask |= 0x02;
  if (isIRTriggered(irRL)) mask |= 0x04;
  if (isIRTriggered(irRR)) mask |= 0x08;
  return mask;
}

SensorSnapshot captureSensorSnapshot() {
  SensorSnapshot snapshot;
  snapshot.irMask = buildIRMask();
  snapshot.distN = (ultrasonicSampleCount > 0) ? avgDistN : distN;
  snapshot.distE = (ultrasonicSampleCount > 0) ? avgDistE : distE;
  snapshot.distS = (ultrasonicSampleCount > 0) ? avgDistS : distS;
  snapshot.distW = (ultrasonicSampleCount > 0) ? avgDistW : distW;
  snapshot.ultrasonicValid = ultrasonicFresh();
  return snapshot;
}

SensorObservation buildSensorObservation(const SensorSnapshot& snapshot, Heading heading, float deadEndThresholdCm,
                                        float turningThresholdCm, float wallThresholdCm, float finishThresholdCm) {
  SensorObservation observation;
  observation.irMask = snapshot.irMask;
  observation.ultrasonicValid = snapshot.ultrasonicValid;
  observation.candidateEvent = classifySensorEventWithThresholds(
    snapshot, heading, true, deadEndThresholdCm, turningThresholdCm, wallThresholdCm, finishThresholdCm);
  return observation;
}

bool sensorObservationChanged(const SensorObservation& a, const SensorObservation& b) {
  return a.irMask != b.irMask ||
         a.candidateEvent != b.candidateEvent ||
         a.ultrasonicValid != b.ultrasonicValid;
}

bool sensorSnapshotChanged(const SensorSnapshot& a, const SensorSnapshot& b) {
  SensorObservation obsA = buildSensorObservation(a, currentHeading, gDeadEndDistanceCm, gTurningThresholdCm, gWallThresholdCm, gFinishDistanceCm);
  SensorObservation obsB = buildSensorObservation(b, currentHeading, gDeadEndDistanceCm, gTurningThresholdCm, gWallThresholdCm, gFinishDistanceCm);
  return sensorObservationChanged(obsA, obsB);
}

void updateSensorStability() {
  SensorSnapshot currentSnapshot = captureSensorSnapshot();
  bool interruptRaised = false;

  noInterrupts();
  if (g_irSensorInterruptFlag) {
    interruptRaised = true;
    g_irSensorInterruptFlag = false;
  }
  interrupts();

  if (interruptRaised || sensorSnapshotChanged(currentSnapshot, lastObservedSnapshot)) {
    lastObservedSnapshot = currentSnapshot;
    sensorLastChangeMs = millis();
    sensorChangeVersion++;
  }

  if (sensorStableVersion != sensorChangeVersion) {
    uint32_t stableDelayMs = (uint32_t)(gIRStableTimeSeconds * 1000.0f);
    if ((millis() - sensorLastChangeMs) >= stableDelayMs) {
      stableSnapshot = lastObservedSnapshot;
      sensorStableVersion = sensorChangeVersion;
    }
  }
}

bool consumeStableSensorSnapshot(uint32_t& handledVersion, SensorSnapshot& outSnapshot) {
  if (handledVersion == sensorStableVersion) return false;
  handledVersion = sensorStableVersion;
  outSnapshot = stableSnapshot;
  return true;
}

float distanceForHeadingInSnapshot(const SensorSnapshot& snapshot, Heading h) {
  switch (h) {
    case Heading::North: return snapshot.distN;
    case Heading::East:  return snapshot.distE;
    case Heading::South: return snapshot.distS;
    case Heading::West:  return snapshot.distW;
    default:             return snapshot.distN;
  }
}

bool confirmedStableSnapshotAvailable() {
  return sensorStableVersion != 0;
}

SensorSnapshot currentStableSensorSnapshot() {
  return stableSnapshot;
}

bool distanceWithinWallThreshold(float distanceCm, float wallThresholdCm) {
  return distanceCm <= wallThresholdCm;
}

bool distanceOpenEnough(float distanceCm, float wallThresholdCm) {
  return distanceCm >= (wallThresholdCm + EVENT_CONFIRM_MARGIN_CM);
}

bool frontInTurnWindow(float frontCm, float deadEndThresholdCm, float turningThresholdCm) {
  return frontCm > (deadEndThresholdCm + EVENT_CONFIRM_MARGIN_CM) &&
         frontCm < (turningThresholdCm - EVENT_CONFIRM_MARGIN_CM);
}

bool frontInStraightWindow(float frontCm, float turningThresholdCm) {
  return frontCm >= (turningThresholdCm + EVENT_CONFIRM_MARGIN_CM);
}

bool finishFrontClearEnough(float frontCm, float finishThresholdCm) {
  return frontCm >= finishThresholdCm;
}

EventName currentStableSensorEvent(Heading heading, bool allowFinishEvent) {
  if (!confirmedStableSnapshotAvailable()) return EventName::SensorWait;
  return classifyStableSensorEvent(stableSnapshot, heading, allowFinishEvent);
}

bool frontPairTriggeredMask(uint8_t irMask) {
  return (irMask & 0x0C) == 0x0C;
}

bool rearPairTriggeredMask(uint8_t irMask) {
  return (irMask & 0x03) == 0x03;
}

bool allIRTriggeredMask(uint8_t irMask) {
  return (irMask & 0x0F) == 0x0F;
}

bool allIRClearMask(uint8_t irMask) {
  return (irMask & 0x0F) == 0x00;
}

EventName classifySensorEventWithThresholds(const SensorSnapshot& snapshot, Heading heading, bool allowFinishEvent,
                                            float deadEndThresholdCm, float turningThresholdCm, float wallThresholdCm,
                                            float finishThresholdCm) {
  if (!snapshot.ultrasonicValid) {
    return EventName::SensorWait;
  }

  bool allTriggered = allIRTriggeredMask(snapshot.irMask);
  bool allClear = allIRClearMask(snapshot.irMask);
  bool frontPairClear = !frontPairTriggeredMask(snapshot.irMask);
  bool rearPairClear = !rearPairTriggeredMask(snapshot.irMask);
  bool irChangedFromCorridor = !allTriggered && !allClear;

  float front = distanceForHeadingInSnapshot(snapshot, heading);
  float left = distanceForHeadingInSnapshot(snapshot, turnLeftOf(heading));
  float right = distanceForHeadingInSnapshot(snapshot, turnRightOf(heading));
  bool leftOpenStable = distanceOpenEnough(left, wallThresholdCm);
  bool rightOpenStable = distanceOpenEnough(right, wallThresholdCm);
  bool leftWallPresent = distanceWithinWallThreshold(left, wallThresholdCm);
  bool rightWallPresent = distanceWithinWallThreshold(right, wallThresholdCm);

  if (allTriggered) {
    if (front <= deadEndThresholdCm) return EventName::DeadEnd;
    if (front < (deadEndThresholdCm + EVENT_CONFIRM_MARGIN_CM)) return EventName::SensorWait;
    return EventName::Corridor;
  }

  if (irChangedFromCorridor && frontInTurnWindow(front, deadEndThresholdCm, turningThresholdCm)) {
    if (leftOpenStable && rightOpenStable) return EventName::TJunction;
    if (leftOpenStable && rightWallPresent) return EventName::LeftTurn;
    if (rightOpenStable && leftWallPresent) return EventName::RightTurn;
    return EventName::SensorWait;
  }

  if (irChangedFromCorridor &&
      frontInStraightWindow(front, turningThresholdCm) &&
      leftOpenStable &&
      rightOpenStable) {
    return EventName::TJunctionStraight;
  }

  if (allowFinishEvent &&
      allClear &&
      frontPairClear &&
      rearPairClear &&
      finishFrontClearEnough(front, finishThresholdCm) &&
      leftOpenStable &&
      rightOpenStable) {
    return EventName::Finish;
  }

  return EventName::SensorWait;
}

EventName classifyStableSensorEvent(const SensorSnapshot& snapshot, Heading heading, bool allowFinishEvent) {
  return classifySensorEventWithThresholds(snapshot, heading, allowFinishEvent,
                                          gDeadEndDistanceCm, gTurningThresholdCm, gWallThresholdCm, gFinishDistanceCm);
}

// ======================================================
// Encoder / button
// ======================================================
void onEncoderAChange() {
  bool a = digitalRead(ENC_A_PIN);
  bool b = digitalRead(ENC_B_PIN);

  if (a == b) {
    if (g_encoderDelta < 100) g_encoderDelta++;
  } else {
    if (g_encoderDelta > -100) g_encoderDelta--;
  }
}

void onAnyIRSensorChange() {
  g_irSensorInterruptFlag = true;
}

int consumeEncoderDelta() {
  noInterrupts();
  int delta = g_encoderDelta;
  g_encoderDelta = 0;
  interrupts();
  return delta;
}

ButtonEvent pollButtonEvent() {
  uint32_t now = millis();
  bool raw = digitalRead(ENC_BTN_PIN);
  ButtonEvent event = ButtonEvent::None;

  if (raw != buttonState.lastRawLevel) {
    buttonState.lastRawLevel = raw;
    buttonState.lastDebounceMs = now;
  }

  if ((now - buttonState.lastDebounceMs) > BUTTON_DEBOUNCE_MS) {
    if (raw != buttonState.stableLevel) {
      buttonState.stableLevel = raw;

      if (buttonState.stableLevel == LOW) {
        buttonState.pressedAtMs = now;
        buttonState.longPressFired = false;
      } else {
        if (!buttonState.longPressFired) {
          event = ButtonEvent::ShortPress;
        }
      }
    }
  }

  if (buttonState.stableLevel == LOW && !buttonState.longPressFired) {
    if ((now - buttonState.pressedAtMs) >= BUTTON_LONGPRESS_MS) {
      buttonState.longPressFired = true;
      event = ButtonEvent::LongPress;
    }
  }

  if (suppressButtonUntilRelease) {
    if (raw == HIGH && buttonState.stableLevel == HIGH) {
      suppressButtonUntilRelease = false;
    }
    return ButtonEvent::None;
  }

  return event;
}

// ======================================================
// Runtime settings commit
// ======================================================
void commitGlobalsForRuntime() {
  activeSettings = workingSettings;

  gP = activeSettings.p_x100 / 100.0f;
  gI = activeSettings.i_x100 / 100.0f;
  gD = activeSettings.d_x100 / 100.0f;
  gPidCurveB = activeSettings.pidCurveB_x100 / 100.0f;
  gPidWallDistanceCm = activeSettings.pidWallDistance_x100 / 100.0f;

  gPairFLRRFwd = activeSettings.pairFLRR_fwd_x100 / 100.0f;
  gPairFLRRBack = activeSettings.pairFLRR_back_x100 / 100.0f;
  gPairFRRLFwd = activeSettings.pairFRRL_fwd_x100 / 100.0f;
  gPairFRRLBack = activeSettings.pairFRRL_back_x100 / 100.0f;

  gFLFwd = activeSettings.fl_fwd_x100 / 100.0f;
  gFLBack = activeSettings.fl_back_x100 / 100.0f;
  gFRFwd = activeSettings.fr_fwd_x100 / 100.0f;
  gFRBack = activeSettings.fr_back_x100 / 100.0f;
  gRRFwd = activeSettings.rr_fwd_x100 / 100.0f;
  gRRBack = activeSettings.rr_back_x100 / 100.0f;
  gRLFwd = activeSettings.rl_fwd_x100 / 100.0f;
  gRLBack = activeSettings.rl_back_x100 / 100.0f;

  gJunctionStopDistance = activeSettings.junctionStopDistance_x100 / 100.0f;
  gWallThresholdCm = activeSettings.wallThreshold_x100 / 100.0f;
  gTurningThresholdCm = activeSettings.turningThreshold_x100 / 100.0f;
  gWaitBeforeTurnSeconds = activeSettings.waitBeforeTurn_x100 / 100.0f;
  gIRStableTimeSeconds = activeSettings.irStableTime_x100 / 100.0f;
  gDeadEndDistanceCm = activeSettings.deadEndDistance_x100 / 100.0f;
  gFinishDistanceCm = activeSettings.finishDistance_x100 / 100.0f;
  gMode = activeSettings.mode;
}

bool modeUsesSequence() {
  return gMode == 1;
}

void resetSequenceState() {
  executionAttempt = 0;
  frozenExecutionDurationMs = 0;
  lastAttemptDurationMs = 0;
  sequenceTotalDurationMs = 0;
  sequenceCompleted = false;
  lastFinishedEventName = EventName::Idle;
}

uint8_t effectiveRoutingMode() {
  if (!modeUsesSequence()) return gMode;

  switch (executionAttempt) {
    case 1: return 2; // right
    case 2: return 3; // left
    case 3: return 4; // selection / widest
    default: return 2;
  }
}

const char* effectiveRoutingModeToString() {
  return modeToString(effectiveRoutingMode());
}

const char* nextAttemptLabel() {
  if (!modeUsesSequence()) {
    return modeToString(gMode);
  }

  switch (executionAttempt + 1) {
    case 1: return "Right";
    case 2: return "Left";
    case 3: return "Selection";
    default: return "Right";
  }
}

void beginExecutionAttempt() {
  if (modeUsesSequence() && sequenceCompleted) {
    resetSequenceState();
  }

  Execution_state = true;
  executionPhase = ExecutionPhase::StartSeek;
  executionAttempt++;
  executionStartMs = millis();
  frozenExecutionDurationMs = 0;
  phaseStartedMs = executionStartMs;
  currentHeading = Heading::North;
  pendingHeading = currentHeading;
  executionResolvedEvent = EventName::Start;
  currentEventName = EventName::Start;
  previousError = 0.0f;
  integral = 0.0f;
  previousTime = millis();
  mainHandledStableVersion = sensorStableVersion;
  suppressButtonUntilRelease = true;
}

void finishExecutionAttempt() {
  if (!Execution_state) return;

  frozenExecutionDurationMs = millis() - executionStartMs;
  lastAttemptDurationMs = frozenExecutionDurationMs;
  sequenceTotalDurationMs += frozenExecutionDurationMs;
  Execution_state = false;
  executionPhase = ExecutionPhase::Finished;
  lastFinishedEventName = currentEventName;
  currentEventName = EventName::Finish;
  if (modeUsesSequence()) {
    sequenceCompleted = executionAttempt >= 3;
  }
  stopAll();
}

void readIRSensors() {
  irFL = digitalRead(PIN_IR_FL);
  irFR = digitalRead(PIN_IR_FR);
  irRL = digitalRead(PIN_IR_RL);
  irRR = digitalRead(PIN_IR_RR);
}

void refreshSensors() {
  readIRSensors();
  readSensorsUART();
  updateSensorStability();
}

bool ultrasonicFresh() {
  if (!ultrasonicDataValid) return false;
  return (millis() - lastUltrasonicUpdateMs) <= (uint32_t)ULTRASONIC_STALE_MS;
}

Heading headingFromIndex(uint8_t idx) {
  switch (idx % 4) {
    case 0: return Heading::North;
    case 1: return Heading::East;
    case 2: return Heading::South;
    case 3: return Heading::West;
    default: return Heading::North;
  }
}

uint8_t headingToIndex(Heading h) {
  switch (h) {
    case Heading::North: return 0;
    case Heading::East:  return 1;
    case Heading::South: return 2;
    case Heading::West:  return 3;
    default:             return 0;
  }
}

Heading turnLeftOf(Heading h) {
  switch (h) {
    case Heading::North: return Heading::West;
    case Heading::East:  return Heading::North;
    case Heading::South: return Heading::East;
    case Heading::West:  return Heading::South;
    default:             return Heading::North;
  }
}

Heading turnRightOf(Heading h) {
  switch (h) {
    case Heading::North: return Heading::East;
    case Heading::East:  return Heading::South;
    case Heading::South: return Heading::West;
    case Heading::West:  return Heading::North;
    default:             return Heading::North;
  }
}

Heading oppositeOf(Heading h) {
  switch (h) {
    case Heading::North: return Heading::South;
    case Heading::East:  return Heading::West;
    case Heading::South: return Heading::North;
    case Heading::West:  return Heading::East;
    default:             return Heading::South;
  }
}

float distanceForHeading(Heading h) {
  switch (h) {
    case Heading::North: return distN;
    case Heading::East:  return distE;
    case Heading::South: return distS;
    case Heading::West:  return distW;
    default:             return distN;
  }
}

float frontDistanceForHeading(Heading h) { return averagedDistanceForHeading(h); }
float leftDistanceForHeading(Heading h) { return averagedDistanceForHeading(turnLeftOf(h)); }
float rightDistanceForHeading(Heading h) { return averagedDistanceForHeading(turnRightOf(h)); }
float frontDistance() { return averagedDistanceForHeading(currentHeading); }
float leftDistance() { return averagedDistanceForHeading(turnLeftOf(currentHeading)); }
float rightDistance() { return averagedDistanceForHeading(turnRightOf(currentHeading)); }
float backDistance() { return averagedDistanceForHeading(oppositeOf(currentHeading)); }

bool leftOpen() {
  return ultrasonicFresh() && distanceOpenEnough(leftDistance(), gWallThresholdCm);
}

bool rightOpen() {
  return ultrasonicFresh() && distanceOpenEnough(rightDistance(), gWallThresholdCm);
}

bool bothWallsWithin(float threshold) {
  return ultrasonicFresh() &&
         distanceWithinWallThreshold(leftDistance(), threshold) &&
         distanceWithinWallThreshold(rightDistance(), threshold);
}

bool hasLeftWallWithin(float threshold) {
  return ultrasonicFresh() && distanceWithinWallThreshold(leftDistance(), threshold);
}

bool hasRightWallWithin(float threshold) {
  return ultrasonicFresh() && distanceWithinWallThreshold(rightDistance(), threshold);
}

Heading chooseNextHeading(bool leftIsOpen, bool rightIsOpen) {
  Heading leftHeading = turnLeftOf(currentHeading);
  Heading rightHeading = turnRightOf(currentHeading);

  switch (effectiveRoutingMode()) {
    case 2:
      if (rightIsOpen) return rightHeading;
      if (leftIsOpen) return leftHeading;
      break;
    case 3:
      if (leftIsOpen) return leftHeading;
      if (rightIsOpen) return rightHeading;
      break;
    case 4:
      if (leftIsOpen && rightIsOpen) {
        return (leftDistance() >= rightDistance()) ? leftHeading : rightHeading;
      }
      if (leftIsOpen) return leftHeading;
      if (rightIsOpen) return rightHeading;
      break;
    default:
      break;
  }

  return currentHeading;
}

Heading chooseNextHeadingFromSnapshot(const SensorSnapshot& snapshot, Heading heading) {
  Heading leftHeading = turnLeftOf(heading);
  Heading rightHeading = turnRightOf(heading);
  float left = distanceForHeadingInSnapshot(snapshot, leftHeading);
  float right = distanceForHeadingInSnapshot(snapshot, rightHeading);
  bool leftIsOpen = distanceOpenEnough(left, gWallThresholdCm);
  bool rightIsOpen = distanceOpenEnough(right, gWallThresholdCm);

  switch (effectiveRoutingMode()) {
    case 2:
      if (rightIsOpen) return rightHeading;
      if (leftIsOpen) return leftHeading;
      break;
    case 3:
      if (leftIsOpen) return leftHeading;
      if (rightIsOpen) return rightHeading;
      break;
    case 4:
      if (leftIsOpen && rightIsOpen) {
        return (left >= right) ? leftHeading : rightHeading;
      }
      if (leftIsOpen) return leftHeading;
      if (rightIsOpen) return rightHeading;
      break;
    default:
      break;
  }

  return heading;
}

const char* modeShortToString(uint8_t mode) {
  switch (mode) {
    case 1: return "SEQ";
    case 2: return "R";
    case 3: return "L";
    case 4: return "SEL";
    default: return "?";
  }
}

const char* modeToString(uint8_t mode) {
  switch (mode) {
    case 1: return "Sequence";
    case 2: return "Right";
    case 3: return "Left";
    case 4: return "Selection";
    default: return "Unknown";
  }
}

const char* eventNameToString(EventName name) {
  switch (name) {
    case EventName::Idle:         return "Idle";
    case EventName::Start:        return "Start";
    case EventName::SensorWait:   return "Sensor";
    case EventName::Corridor:     return "Corridor";
    case EventName::TJunction:    return "T-Jct";
    case EventName::DeadEnd:      return "Dead-end";
    case EventName::LeftTurn:     return "L-Turn";
    case EventName::RightTurn:    return "R-Turn";
    case EventName::TJunctionStraight: return "T-Straight";
    case EventName::Random:       return "Random";
    case EventName::ApproachWall: return "Approach";
    case EventName::WaitAtTurn:   return "Wait";
    case EventName::Finish:       return "Finish";
    case EventName::RestartReady: return "Restart";
    default:                      return "?";
  }
}

EventName eventForTurnHeading(Heading targetHeading) {
  if (targetHeading == turnLeftOf(currentHeading)) return EventName::LeftTurn;
  if (targetHeading == turnRightOf(currentHeading)) return EventName::RightTurn;
  return EventName::Corridor;
}

const char* eventTestCaseToString(EventTestCase testCase) {
  switch (testCase) {
    case EventTestCase::Corridor:          return "Corridor";
    case EventTestCase::TJunction:         return "T-Junction";
    case EventTestCase::DeadEnd:           return "Dead-end";
    case EventTestCase::Start:             return "Start";
    case EventTestCase::Finish:            return "Finish";
    case EventTestCase::LeftTurn:          return "Left Turn";
    case EventTestCase::RightTurn:         return "Right Turn";
    case EventTestCase::TJunctionStraight: return "T-Jct Straight";
    case EventTestCase::Random:            return "Random";
    case EventTestCase::Back:              return "Back";
    default:                               return "?";
  }
}

EventName eventTestCaseToEventName(EventTestCase testCase) {
  switch (testCase) {
    case EventTestCase::Corridor:          return EventName::Corridor;
    case EventTestCase::TJunction:         return EventName::TJunction;
    case EventTestCase::DeadEnd:           return EventName::DeadEnd;
    case EventTestCase::Start:             return EventName::Start;
    case EventTestCase::Finish:            return EventName::Finish;
    case EventTestCase::LeftTurn:          return EventName::LeftTurn;
    case EventTestCase::RightTurn:         return EventName::RightTurn;
    case EventTestCase::TJunctionStraight: return EventName::TJunctionStraight;
    case EventTestCase::Random:            return EventName::Random;
    case EventTestCase::Back:              return EventName::Idle;
    default:                               return EventName::Idle;
  }
}

const char* turnChoiceToString(TurnChoice choice) {
  return (choice == TurnChoice::Left) ? "Left" : "Right";
}

EventTestConfig& currentEventTestConfig() {
  int index = static_cast<int>(selectedEventTest);
  if (index < 0 || index >= EVENT_TEST_COUNT) index = 0;
  return eventTestConfigs[index];
}

float currentEventTestStopThreshold() {
  return currentEventTestConfig().stopThreshold_x100 / 100.0f;
}

float currentEventTestSideThreshold() {
  return currentEventTestConfig().sideThreshold_x100 / 100.0f;
}

uint32_t currentEventTestActionMs() {
  return (uint32_t)(currentEventTestConfig().actionTime_x100 * 10UL);
}

uint32_t currentEventTestMoveMs() {
  return (uint32_t)(currentEventTestConfig().moveTime_x100 * 10UL);
}

Heading currentEventTestHeading() {
  return headingFromIndex(currentEventTestConfig().heading);
}

Heading currentEventTestConfiguredTurn() {
  return (currentEventTestConfig().turnChoice == TurnChoice::Left)
    ? turnLeftOf(currentHeading)
    : turnRightOf(currentHeading);
}

bool frontIRTriggered() {
  return isIRTriggered(irRL) && isIRTriggered(irRR);
}

bool finishIRTriggered() {
  return isIRTriggered(irFL) || isIRTriggered(irFR) ||
         isIRTriggered(irRL) || isIRTriggered(irRR);
}

bool sideUltrasonicWithin(float threshold) {
  return ultrasonicFresh() && leftDistance() <= threshold && rightDistance() <= threshold;
}

void resetEventTestRun() {
  eventTestPhase = EventTestPhase::Idle;
  eventTestResolvedEvent = EventName::Idle;
  eventTestStartedMs = millis();
  eventTestPendingHeading = currentEventTestHeading();
  currentHeading = currentEventTestHeading();
}

void startEventTestRun() {
  eventTestActive = true;
  currentHeading = currentEventTestHeading();
  eventTestPendingHeading = currentHeading;
  eventTestResolvedEvent = eventTestCaseToEventName(selectedEventTest);
  currentEventName = eventTestResolvedEvent;
  previousError = 0.0f;
  integral = 0.0f;
  previousTime = millis();
  eventTestStartedMs = millis();
  eventTestHandledStableVersion = sensorStableVersion;

  switch (selectedEventTest) {
    case EventTestCase::Start:
      eventTestPhase = EventTestPhase::StartSeek;
      break;
    case EventTestCase::Finish:
      eventTestPhase = EventTestPhase::Detect;
      break;
    case EventTestCase::Corridor:
      eventTestPhase = EventTestPhase::Detect;
      break;
    default:
      eventTestPhase = EventTestPhase::Detect;
      break;
  }
}

void resetMazeTest() {
  mazeTestActive = false;
  mazeTestPhase = MazeTestPhase::Idle;
  mazeTestHeading = Heading::North;
  currentHeading = mazeTestHeading;
  mazeTestPendingHeading = mazeTestHeading;
  mazeTestEvent = EventName::Start;
  mazeTestResolvedEvent = EventName::Start;
  mazeTestStartedMs = millis();
  mazeTestFrozenDurationMs = 0;
  mazeTestAttempt = 0;
}

void startMazeTest() {
  mazeTestActive = true;
  mazeTestPhase = MazeTestPhase::StartSeek;
  mazeTestHeading = Heading::North;
  currentHeading = mazeTestHeading;
  mazeTestPendingHeading = mazeTestHeading;
  mazeTestEvent = EventName::Start;
  mazeTestResolvedEvent = EventName::Start;
  mazeTestStartedMs = millis();
  mazeTestFrozenDurationMs = 0;
  mazeTestAttempt++;
  mazeTestHandledStableVersion = sensorStableVersion;
}

void updateMazeTestLogic() {
  if (!mazeTestActive) {
    if (mazeTestPhase == MazeTestPhase::Idle) {
      mazeTestEvent = EventName::RestartReady;
    }
    return;
  }

  SensorSnapshot stableEventSnapshot = currentStableSensorSnapshot();
  bool hasStableEvent = confirmedStableSnapshotAvailable();
  EventName stableDetectedEvent = hasStableEvent
    ? classifyStableSensorEvent(stableEventSnapshot, mazeTestHeading, true)
    : EventName::SensorWait;

  switch (mazeTestPhase) {
    case MazeTestPhase::Idle:
      mazeTestEvent = EventName::RestartReady;
      return;

    case MazeTestPhase::StartSeek:
      mazeTestEvent = EventName::Start;
      mazeTestResolvedEvent = EventName::Start;
      if (stableDetectedEvent == EventName::Corridor) {
        mazeTestPhase = MazeTestPhase::Corridor;
      }
      return;

    case MazeTestPhase::Corridor:
      mazeTestEvent = EventName::Corridor;
      if (stableDetectedEvent == EventName::Finish) {
        mazeTestEvent = EventName::Finish;
        mazeTestPhase = MazeTestPhase::Finished;
        mazeTestFrozenDurationMs = millis() - mazeTestStartedMs;
        mazeTestActive = false;
        return;
      }

      if (stableDetectedEvent == EventName::DeadEnd) {
        mazeTestEvent = EventName::DeadEnd;
        mazeTestHeading = oppositeOf(mazeTestHeading);
        currentHeading = mazeTestHeading;
        return;
      }

      if (stableDetectedEvent == EventName::TJunction ||
          stableDetectedEvent == EventName::LeftTurn ||
          stableDetectedEvent == EventName::RightTurn) {
        mazeTestEvent = stableDetectedEvent;
        mazeTestResolvedEvent = stableDetectedEvent;
        if (stableDetectedEvent == EventName::LeftTurn) {
          mazeTestPendingHeading = turnLeftOf(mazeTestHeading);
        } else if (stableDetectedEvent == EventName::RightTurn) {
          mazeTestPendingHeading = turnRightOf(mazeTestHeading);
        } else {
          mazeTestPendingHeading = chooseNextHeadingFromSnapshot(stableEventSnapshot, mazeTestHeading);
        }
        mazeTestPhase = MazeTestPhase::ApproachWall;
        phaseStartedMs = millis();
        return;
      }

      if (stableDetectedEvent == EventName::TJunctionStraight) {
        mazeTestEvent = EventName::TJunctionStraight;
        mazeTestResolvedEvent = EventName::TJunctionStraight;
        mazeTestPendingHeading = mazeTestHeading;
        mazeTestPhase = MazeTestPhase::AcquireWall;
        phaseStartedMs = millis();
      }
      return;

    case MazeTestPhase::ApproachWall: {
      mazeTestEvent = mazeTestResolvedEvent;
      if (!stableEventSnapshot.ultrasonicValid) return;

      float front = distanceForHeadingInSnapshot(stableEventSnapshot, mazeTestHeading);

      if (front > gJunctionStopDistance) return;

      if (mazeTestPendingHeading == mazeTestHeading) {
        mazeTestEvent = EventName::DeadEnd;
        mazeTestHeading = oppositeOf(mazeTestHeading);
        currentHeading = mazeTestHeading;
        mazeTestPhase = MazeTestPhase::Corridor;
        return;
      }

      mazeTestEvent = (mazeTestPendingHeading == turnLeftOf(mazeTestHeading))
        ? EventName::LeftTurn
        : EventName::RightTurn;
      mazeTestResolvedEvent = mazeTestEvent;
      phaseStartedMs = millis();
      mazeTestPhase = MazeTestPhase::WaitAtTurn;
      return;
    }

    case MazeTestPhase::WaitAtTurn:
      mazeTestEvent = EventName::WaitAtTurn;
      if ((millis() - phaseStartedMs) >= (uint32_t)(gWaitBeforeTurnSeconds * 1000.0f)) {
        mazeTestHeading = mazeTestPendingHeading;
        currentHeading = mazeTestHeading;
        phaseStartedMs = millis();
        mazeTestPhase = MazeTestPhase::AcquireWall;
      }
      return;

    case MazeTestPhase::AcquireWall:
      mazeTestEvent = mazeTestResolvedEvent;
      if (ultrasonicFresh() &&
          (distanceWithinWallThreshold(leftDistanceForHeading(mazeTestHeading), gWallThresholdCm) &&
           distanceWithinWallThreshold(rightDistanceForHeading(mazeTestHeading), gWallThresholdCm))) {
        mazeTestPhase = MazeTestPhase::Corridor;
        return;
      }

      if ((millis() - phaseStartedMs) >= MAZE_TEST_ACQUIRE_TIMEOUT_MS &&
          ultrasonicFresh() &&
          (distanceWithinWallThreshold(leftDistanceForHeading(mazeTestHeading), gWallThresholdCm) ||
           distanceWithinWallThreshold(rightDistanceForHeading(mazeTestHeading), gWallThresholdCm))) {
        mazeTestPhase = MazeTestPhase::Corridor;
      }
      return;

    case MazeTestPhase::Finished:
    default:
      return;
  }
}

// ======================================================
// Digit editor
// ======================================================
void openDigitEditor(const char* label, int* target, int minValue, int maxValue) {
  digitEditor.active = true;
  digitEditor.digitUnlocked = false;
  digitEditor.selectedDigit = 0;
  digitEditor.tempValue = *target;
  digitEditor.targetValue = target;
  digitEditor.minValue = minValue;
  digitEditor.maxValue = maxValue;
  digitEditor.label = label;

  previousScreen = currentScreen;
  currentScreen = Screen::DigitEditor;
}

void closeDigitEditor(bool commit) {
  if (commit && digitEditor.targetValue != nullptr) {
    *digitEditor.targetValue = clampInt(digitEditor.tempValue, digitEditor.minValue, digitEditor.maxValue);
    commitGlobalsForRuntime(); // apply immediately so tuning affects live motors
  }

  digitEditor.active = false;
  digitEditor.digitUnlocked = false;
  currentScreen = previousScreen;
}

// ======================================================
// Menu input handlers
// ======================================================
void handleRootInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    rootIndex++;
    if (rootIndex >= ROOT_COUNT) rootIndex = ROOT_COUNT - 1;
  } else if (delta < 0) {
    rootIndex--;
    if (rootIndex < 0) rootIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (rootIndex) {
      case 0: currentScreen = Screen::StartConfirm; break;
      case 1: startSettingsIndex = 0; currentScreen = Screen::StartSettings; break;
      case 2: settingsIndex = 0; currentScreen = Screen::Settings; break;
      case 3: eventTestIndex = 0; currentScreen = Screen::EventTestMenu; break;
      case 4: resetMazeTest(); currentScreen = Screen::MazeTestRun; break;
      case 5: irSensorIndex = 0; currentScreen = Screen::IRSensorTest; break;
      case 6: currentScreen = Screen::UltrasonicSensorTest; break;
    }
  }
}

void handleStartConfirmInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    programStarted = true;
    autoStartExecution = true;
    suppressButtonUntilRelease = true;
  }
  if (btn == ButtonEvent::LongPress) {
    resetSequenceState();
    currentScreen = Screen::Root;
  }
}

void handleStartSettingsInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    startSettingsIndex++;
    if (startSettingsIndex >= START_SETTINGS_COUNT) startSettingsIndex = START_SETTINGS_COUNT - 1;
  } else if (delta < 0) {
    startSettingsIndex--;
    if (startSettingsIndex < 0) startSettingsIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (startSettingsIndex) {
      case 0:
        workingSettings.mode++;
        if (workingSettings.mode > 4) workingSettings.mode = 1;
        commitGlobalsForRuntime();
        resetSequenceState();
        break;
      case 1:
        openDigitEditor("Wait(s)", &workingSettings.waitBeforeTurn_x100, 0, 9999);
        break;
      case 2:
        currentScreen = Screen::Root;
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handleSettingsInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    settingsIndex++;
    if (settingsIndex >= SETTINGS_COUNT) settingsIndex = SETTINGS_COUNT - 1;
  } else if (delta < 0) {
    settingsIndex--;
    if (settingsIndex < 0) settingsIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (settingsIndex) {
      case 0:
        pidIndex = 0;
        currentScreen = Screen::PID;
        break;
      case 1:
        motorTuneIndex = 0;
        currentScreen = Screen::MotorTune;
        break;
      case 2:
        openDigitEditor("Stop(cm)", &workingSettings.junctionStopDistance_x100, 0, 9999);
        break;
      case 3:
        openDigitEditor("Wall(cm)", &workingSettings.wallThreshold_x100, 0, 9999);
        break;
      case 4:
        openDigitEditor("Turn(cm)", &workingSettings.turningThreshold_x100, 0, 9999);
        break;
      case 5:
        commitGlobalsForRuntime();
        currentScreen = Screen::Root;
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handlePIDInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    pidIndex++;
    if (pidIndex >= PID_COUNT) pidIndex = PID_COUNT - 1;
  } else if (delta < 0) {
    pidIndex--;
    if (pidIndex < 0) pidIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (pidIndex) {
      case 0: openDigitEditor("P", &workingSettings.p_x100, 0, 9999); break;
      case 1: openDigitEditor("I", &workingSettings.i_x100, 0, 9999); break;
      case 2: openDigitEditor("D", &workingSettings.d_x100, 0, 9999); break;
      case 3: openDigitEditor("b", &workingSettings.pidCurveB_x100, 1, 9999); break;
      case 4: openDigitEditor("WallDst", &workingSettings.pidWallDistance_x100, 0, 9999); break;
      case 5:
        pidTestHeading = Heading::North;
        resetPIDPreview();
        currentScreen = Screen::PIDTest;
        break;
      case 6: currentScreen = Screen::Settings; break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Settings;
  }
}

void handlePIDTestInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    pidTestHeading = turnRightOf(pidTestHeading);
    resetPIDPreview();
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::PID;
  }
}

void handleMotorTuneInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    motorTuneIndex++;
    if (motorTuneIndex >= MOTOR_TUNE_COUNT) motorTuneIndex = MOTOR_TUNE_COUNT - 1;
  } else if (delta < 0) {
    motorTuneIndex--;
    if (motorTuneIndex < 0) motorTuneIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (motorTuneIndex) {
      case 0:
        tuneMotion = static_cast<TuneMotion>((static_cast<int>(tuneMotion) + 1) % 5);
        break;

      case 1:  openDigitEditor("A Fwd", &workingSettings.pairFLRR_fwd_x100, 10, 300); break;
      case 2:  openDigitEditor("A Back", &workingSettings.pairFLRR_back_x100, 10, 300); break;
      case 3:  openDigitEditor("B Fwd", &workingSettings.pairFRRL_fwd_x100, 10, 300); break;
      case 4:  openDigitEditor("B Back", &workingSettings.pairFRRL_back_x100, 10, 300); break;

      case 5:  openDigitEditor("FL Fwd", &workingSettings.fl_fwd_x100, 10, 300); break;
      case 6:  openDigitEditor("FL Back", &workingSettings.fl_back_x100, 10, 300); break;
      case 7:  openDigitEditor("FR Fwd", &workingSettings.fr_fwd_x100, 10, 300); break;
      case 8:  openDigitEditor("FR Back", &workingSettings.fr_back_x100, 10, 300); break;
      case 9:  openDigitEditor("RR Fwd", &workingSettings.rr_fwd_x100, 10, 300); break;
      case 10: openDigitEditor("RR Back", &workingSettings.rr_back_x100, 10, 300); break;
      case 11: openDigitEditor("RL Fwd", &workingSettings.rl_fwd_x100, 10, 300); break;
      case 12: openDigitEditor("RL Back", &workingSettings.rl_back_x100, 10, 300); break;

      case 13:
        tuneMotion = TuneMotion::Stop;
        stopAll();
        currentScreen = Screen::Settings;
        break;
    }
    commitGlobalsForRuntime();
  }

  if (btn == ButtonEvent::LongPress) {
    tuneMotion = TuneMotion::Stop;
    stopAll();
    currentScreen = Screen::Settings;
  }
}

void handleEventTestMenuInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    eventTestIndex++;
    if (eventTestIndex >= EVENT_TEST_COUNT) eventTestIndex = EVENT_TEST_COUNT - 1;
  } else if (delta < 0) {
    eventTestIndex--;
    if (eventTestIndex < 0) eventTestIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    selectedEventTest = static_cast<EventTestCase>(eventTestIndex);
    if (selectedEventTest == EventTestCase::Back) {
      currentScreen = Screen::Root;
      return;
    }

    currentHeading = currentEventTestHeading();
    eventTestConfigIndex = 0;
    currentScreen = Screen::EventTestConfig;
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handleEventTestConfigInput(int delta, ButtonEvent btn) {
  EventTestConfig& cfg = currentEventTestConfig();

  if (delta > 0) {
    eventTestConfigIndex++;
    if (eventTestConfigIndex >= EVENT_TEST_CONFIG_COUNT) eventTestConfigIndex = EVENT_TEST_CONFIG_COUNT - 1;
  } else if (delta < 0) {
    eventTestConfigIndex--;
    if (eventTestConfigIndex < 0) eventTestConfigIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (eventTestConfigIndex) {
      case 0:
        cfg.heading = (cfg.heading + 1) % 4;
        break;
      case 1:
        openDigitEditor("StopThr", &cfg.stopThreshold_x100, 0, 9999);
        break;
      case 2:
        openDigitEditor("SideThr", &cfg.sideThreshold_x100, 0, 9999);
        break;
      case 3:
        openDigitEditor("Action", &cfg.actionTime_x100, 0, 9999);
        break;
      case 4:
        cfg.turnChoice = (cfg.turnChoice == TurnChoice::Left) ? TurnChoice::Right : TurnChoice::Left;
        break;
      case 5:
        startEventTestRun();
        suppressButtonUntilRelease = true;
        currentScreen = Screen::EventTestRun;
        break;
      case 6:
        currentScreen = Screen::EventTestMenu;
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::EventTestMenu;
  }
}

void handleEventTestRunInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    startEventTestRun();
    suppressButtonUntilRelease = true;
  }

  if (btn == ButtonEvent::LongPress) {
    eventTestActive = false;
    resetEventTestRun();
    stopAll();
    currentScreen = Screen::EventTestConfig;
  }
}

void handleMazeTestRunInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    startMazeTest();
    suppressButtonUntilRelease = true;
  }

  if (btn == ButtonEvent::LongPress) {
    mazeTestActive = false;
    resetMazeTest();
    currentScreen = Screen::Root;
  }
}

void handleIRSensorTestInput(int delta, ButtonEvent btn) {
  if (delta > 0) {
    irSensorIndex++;
    if (irSensorIndex >= IR_SENSOR_COUNT) irSensorIndex = IR_SENSOR_COUNT - 1;
  } else if (delta < 0) {
    irSensorIndex--;
    if (irSensorIndex < 0) irSensorIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (irSensorIndex) {
      case 2:
        openDigitEditor("IR Time", &workingSettings.irStableTime_x100, 1, 9999);
        break;
      case 3:
        openDigitEditor("DeadEnd", &workingSettings.deadEndDistance_x100, 0, 9999);
        break;
      case 4:
        openDigitEditor("Finish", &workingSettings.finishDistance_x100, 0, 9999);
        break;
      case 5:
        currentScreen = Screen::Root;
        break;
      default:
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handleUltrasonicSensorTestInput(ButtonEvent btn) {
  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handleDigitEditorInput(int delta, ButtonEvent btn) {
  if (delta != 0) {
    if (!digitEditor.digitUnlocked) {
      if (delta > 0) {
        digitEditor.selectedDigit++;
        if (digitEditor.selectedDigit > 3) digitEditor.selectedDigit = 3;
      } else {
        if (digitEditor.selectedDigit > 0) digitEditor.selectedDigit--;
      }
    } else {
      incrementSelectedDigit(delta > 0 ? 1 : -1);
      commitGlobalsForRuntime();
      if (digitEditor.targetValue != nullptr) {
        *digitEditor.targetValue = clampInt(digitEditor.tempValue, digitEditor.minValue, digitEditor.maxValue);
      }
      commitGlobalsForRuntime();
    }
  }

  if (btn == ButtonEvent::ShortPress) {
    digitEditor.digitUnlocked = !digitEditor.digitUnlocked;
  }

  if (btn == ButtonEvent::LongPress) {
    closeDigitEditor(true);
  }
}

// ======================================================
// Drawing
// ======================================================
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 8, title); // before was 0, 10, title
  u8g2.drawHLine(0, 11, 128);// before was 0, 13, 128
}

void drawMenuItem(int y, bool selected, const String& text) {
  if (selected) {
    u8g2.drawBox(0, y - 9, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, y, text.c_str());
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, y, text.c_str());
  }
}

void drawScrollableItemList(const String items[], int itemCount, int selectedIndex, int startY) {
  int first = selectedIndex - 1;
  if (first < 0) first = 0;
  if (first > itemCount - 4) first = max(0, itemCount - 4);

  for (int i = 0; i < 4; ++i) {
    int idx = first + i;
    if (idx >= itemCount) break;
    drawMenuItem(startY + i * 10, idx == selectedIndex, items[idx]);
  }
}

void drawRootScreen() {
  drawHeader("Main Menu");

  String items[ROOT_COUNT];
  items[0] = "Start";
  items[1] = "Start Settings";
  items[2] = "Settings";
  items[3] = "Event Test";
  items[4] = "Maze Test";
  items[5] = "IR Test";
  items[6] = "Ultrasonic Test";
  drawScrollableItemList(items, ROOT_COUNT, rootIndex, 26);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short: select");
}

void drawStartConfirmScreen() {
  drawHeader((currentEventName == EventName::Finish) ? "Finished" : "Ready");
  u8g2.setFont(u8g2_font_5x8_tf);
  if (currentEventName == EventName::Finish) {
    if (modeUsesSequence()) {
      String line1 = String("Done: ") + String(executionAttempt) + "/3";
      String line2 = String("Last: ") + formatMsAsSeconds(lastAttemptDurationMs);
      String line3 = String("Total: ") + formatMsAsSeconds(sequenceTotalDurationMs);
      String line4 = String("Next: ") + nextAttemptLabel();

      u8g2.drawStr(0, 22, line1.c_str());
      u8g2.drawStr(0, 30, line2.c_str());
      u8g2.drawStr(0, 38, line3.c_str());
      u8g2.drawStr(0, 46, line4.c_str());
    } else {
      String line1 = String("Attempt: ") + String(executionAttempt);
      String line2 = String("Time: ") + formatMsAsSeconds(lastAttemptDurationMs);
      String line3 = String("Next: ") + nextAttemptLabel();

      u8g2.drawStr(0, 24, line1.c_str());
      u8g2.drawStr(0, 34, line2.c_str());
      u8g2.drawStr(0, 44, line3.c_str());
    }
  } else {
    String line1 = String("Next: ") + nextAttemptLabel();
    String line2 = String("Mode: ") + String(modeToString(gMode));
    u8g2.drawStr(0, 24, line1.c_str());
    u8g2.drawStr(0, 34, line2.c_str());
  }

  u8g2.drawStr(0, 54, "Short: start");
  u8g2.drawStr(0, 63, "Long: root");
}

void drawStartSettingsScreen() {
  drawHeader("Start Settings");

  String items[START_SETTINGS_COUNT];
  items[0] = "Mode: " + String(modeToString(workingSettings.mode));
  items[1] = "Wait: " + formatX100(workingSettings.waitBeforeTurn_x100);
  items[2] = "Back";
  drawScrollableItemList(items, START_SETTINGS_COUNT, startSettingsIndex, 26);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: root");
}

void drawSettingsScreen() {
  drawHeader("Settings");
  String items[SETTINGS_COUNT];
  items[0] = "PID";
  items[1] = "Motor Tune";
  items[2] = "Stop: " + formatX100(workingSettings.junctionStopDistance_x100);
  items[3] = "Wall: " + formatX100(workingSettings.wallThreshold_x100);
  items[4] = "Turn: " + formatX100(workingSettings.turningThreshold_x100);
  items[5] = "Save";
  drawScrollableItemList(items, SETTINGS_COUNT, settingsIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: root");
}

void drawPIDScreen() {
  drawHeader("PID");
  String items[PID_COUNT];
  items[0] = "P: " + formatX100(workingSettings.p_x100);
  items[1] = "I: " + formatX100(workingSettings.i_x100);
  items[2] = "D: " + formatX100(workingSettings.d_x100);
  items[3] = "b: " + formatX100(workingSettings.pidCurveB_x100);
  items[4] = "WallDst: " + formatX100(workingSettings.pidWallDistance_x100);
  items[5] = "PID Test";
  items[6] = "Back";
  drawScrollableItemList(items, PID_COUNT, pidIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: settings");
}

void drawPIDTestScreen() {
  drawHeader("PID Test");
  u8g2.setFont(u8g2_font_5x7_tf);

  String line1 = String("Dir:") + headingToString(pidTestHeading) + " WD:" + formatX100(workingSettings.pidWallDistance_x100);
  String line2 = String("b:") + formatX100(workingSettings.pidCurveB_x100) + " y=0.6*(x/0.6)^b";
  String line3 = String("L:") + formatDistanceCm(averagedDistanceForHeading(turnLeftOf(pidTestHeading))) +
                 " R:" + formatDistanceCm(averagedDistanceForHeading(turnRightOf(pidTestHeading)));
  String line4 = String("Raw:") + formatSignedFloat(pidPreviewRawError, 2);
  String line5 = String("Int:") + formatSignedFloat(pidPreviewInterpretedError, 2);
  String line6 = String("PID:") + formatSignedFloat(pidPreviewCorrection, 2);

  u8g2.drawStr(0, 21, line1.c_str());
  u8g2.drawStr(0, 28, line2.c_str());
  u8g2.drawStr(0, 35, line3.c_str());
  u8g2.drawStr(0, 42, line4.c_str());
  u8g2.drawStr(0, 49, line5.c_str());
  u8g2.drawStr(0, 56, line6.c_str());
  u8g2.drawStr(0, 63, "Short=dir Long=back");
}

const char* tuneMotionToString(TuneMotion t) {
  switch (t) {
    case TuneMotion::Stop:  return "Stop";
    case TuneMotion::North: return "North";
    case TuneMotion::South: return "South";
    case TuneMotion::East:  return "East";
    case TuneMotion::West:  return "West";
    default: return "?";
  }
}

void drawMotorTuneScreen() {
  drawHeader("Motor Tune");

  String items[MOTOR_TUNE_COUNT];
  items[0]  = String("Test: ") + tuneMotionToString(tuneMotion);
  items[1]  = "A Fwd: " + formatX100(workingSettings.pairFLRR_fwd_x100);
  items[2]  = "A Back:" + formatX100(workingSettings.pairFLRR_back_x100);
  items[3]  = "B Fwd: " + formatX100(workingSettings.pairFRRL_fwd_x100);
  items[4]  = "B Back:" + formatX100(workingSettings.pairFRRL_back_x100);
  items[5]  = "FL Fwd:" + formatX100(workingSettings.fl_fwd_x100);
  items[6]  = "FL Back:" + formatX100(workingSettings.fl_back_x100);
  items[7]  = "FR Fwd:" + formatX100(workingSettings.fr_fwd_x100);
  items[8]  = "FR Back:" + formatX100(workingSettings.fr_back_x100);
  items[9]  = "RR Fwd:" + formatX100(workingSettings.rr_fwd_x100);
  items[10] = "RR Back:" + formatX100(workingSettings.rr_back_x100);
  items[11] = "RL Fwd:" + formatX100(workingSettings.rl_fwd_x100);
  items[12] = "RL Back:" + formatX100(workingSettings.rl_back_x100);
  items[13] = "Back";

  drawScrollableItemList(items, MOTOR_TUNE_COUNT, motorTuneIndex, 26);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=edit Long=back");
}

void drawEventTestMenuScreen() {
  drawHeader("Event Test");

  String items[EVENT_TEST_COUNT];
  items[0] = "Corridor";
  items[1] = "T-Junction";
  items[2] = "Dead-end";
  items[3] = "Start";
  items[4] = "Finish";
  items[5] = "Left Turn";
  items[6] = "Right Turn";
  items[7] = "T-Straight";
  items[8] = "Random";
  items[9] = "Back";
  drawScrollableItemList(items, EVENT_TEST_COUNT, eventTestIndex, 26);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=config Long=back");
}

void drawEventTestConfigScreen() {
  drawHeader("Event Config");

  EventTestConfig& cfg = currentEventTestConfig();
  String items[EVENT_TEST_CONFIG_COUNT];
  items[0] = "Dir: " + String(headingToString(headingFromIndex(cfg.heading)));
  items[1] = "Stop: " + formatX100(cfg.stopThreshold_x100);
  items[2] = "Wall: " + formatX100(cfg.sideThreshold_x100);
  items[3] = "Act: " + formatX100(cfg.actionTime_x100);
  items[4] = "Turn: " + String(turnChoiceToString(cfg.turnChoice));
  items[5] = "Run";
  items[6] = "Back";
  drawScrollableItemList(items, EVENT_TEST_CONFIG_COUNT, eventTestConfigIndex, 26);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=edit Long=back");
}

void drawEventTestRunScreen() {
  drawHeader("Event Run");
  u8g2.setFont(u8g2_font_5x8_tf);

  String line1 = String("Case: ") + eventTestCaseToString(selectedEventTest);
  String line2 = String("Ev: ") + eventNameToString(currentEventName);
  String line3 = String("Hd: ") + headingToString(currentHeading);
  String line4 = String("US N:") + formatDistanceCm(distN) + " E:" + formatDistanceCm(distE);
  String line5 = String("US S:") + formatDistanceCm(distS) + " W:" + formatDistanceCm(distW);
  String line6 = String("IR NW:") + String(irRL) + " NE:" + String(irRR);
  String line7 = String("IR SW:") + String(irFL) + " SE:" + String(irFR);

  u8g2.drawStr(0, 21, line1.c_str());
  u8g2.drawStr(0, 28, line2.c_str());
  u8g2.drawStr(0, 35, line3.c_str());
  u8g2.drawStr(0, 42, line4.c_str());
  u8g2.drawStr(0, 49, line5.c_str());
  u8g2.drawStr(0, 56, line6.c_str());
  u8g2.drawStr(0, 63, line7.c_str());
}

void drawMazeTestRunScreen() {
  drawHeader("Maze Test");
  u8g2.setFont(u8g2_font_5x7_tf);

  uint32_t shownTime = mazeTestActive ? (millis() - mazeTestStartedMs) : mazeTestFrozenDurationMs;
  String line1 = String("A:") + String(mazeTestAttempt) + " Ev:" + eventNameToString(mazeTestEvent);
  String line2 = String("Hd:") + headingToString(mazeTestHeading) + " T:" + formatMsAsSeconds(shownTime);
  String line3 = String("US N:") + formatDistanceCm(distN) + " E:" + formatDistanceCm(distE);
  String line4 = String("US S:") + formatDistanceCm(distS) + " W:" + formatDistanceCm(distW);
  String line5 = String("IR NW:") + String(irRL) + " NE:" + String(irRR);
  String line6 = String("IR SW:") + String(irFL) + " SE:" + String(irFR);

  u8g2.drawStr(0, 21, line1.c_str());
  u8g2.drawStr(0, 28, line2.c_str());
  u8g2.drawStr(0, 35, line3.c_str());
  u8g2.drawStr(0, 42, line4.c_str());
  u8g2.drawStr(0, 49, line5.c_str());
  u8g2.drawStr(0, 56, line6.c_str());
  u8g2.drawStr(0, 63, mazeTestActive ? "Long=back" : "Short=start Long=back");
}

void drawIRSensorTestScreen() {
  drawHeader("IR Test");
  String items[IR_SENSOR_COUNT];
  items[0] = "NW:" + String(irRL) + " NE:" + String(irRR);
  items[1] = "SW:" + String(irFL) + " SE:" + String(irFR);
  items[2] = "IR Time: " + formatX100(workingSettings.irStableTime_x100);
  items[3] = "DeadEnd: " + formatX100(workingSettings.deadEndDistance_x100);
  items[4] = "Finish: " + formatX100(workingSettings.finishDistance_x100);
  items[5] = "Back";
  drawScrollableItemList(items, IR_SENSOR_COUNT, irSensorIndex, 26);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=edit Long=back");
}

void drawUltrasonicSensorTestScreen() {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(18, 8, "ULTRASONIC SENSORS");
  u8g2.drawHLine(0, 10, 128);

  u8g2.drawStr(0, 18, "Front:");
  u8g2.drawStr(72, 18, formatDistanceCm(avgDistN).c_str());
  u8g2.drawStr(106, 18, "cm");

  u8g2.drawStr(0, 28, "Right:");
  u8g2.drawStr(72, 28, formatDistanceCm(avgDistE).c_str());
  u8g2.drawStr(106, 28, "cm");

  u8g2.drawStr(0, 38, "Back:");
  u8g2.drawStr(72, 38, formatDistanceCm(avgDistS).c_str());
  u8g2.drawStr(106, 38, "cm");

  u8g2.drawStr(0, 48, "Left:");
  u8g2.drawStr(72, 48, formatDistanceCm(avgDistW).c_str());
  u8g2.drawStr(106, 48, "cm");

  u8g2.drawHLine(0, 55, 128);
  String fbText = String("FB:") + formatSignedFloat(interpretedNS, 2);
  String lrText = String("LR:") + formatSignedFloat(-interpretedEW, 2);
  u8g2.drawStr(0, 63, fbText.c_str());
  u8g2.drawStr(68, 63, lrText.c_str());
}

void drawDigitEditorScreen() {
  drawHeader(digitEditor.label);

  String valueStr = formatX100(digitEditor.tempValue);

  u8g2.setFont(u8g2_font_logisoso20_tn);
  u8g2.drawStr(12, 40, valueStr.c_str());

  const int digitX[4] = {12, 26, 50, 62};
  const int underlineY = 44;
  const int frameY = 16;

  if (!digitEditor.digitUnlocked) {
    u8g2.drawHLine(digitX[digitEditor.selectedDigit], underlineY, 12);
  } else {
    u8g2.drawFrame(digitX[digitEditor.selectedDigit] - 2, frameY, 16, 30);
  }

  u8g2.setFont(u8g2_font_5x8_tf);
  if (!digitEditor.digitUnlocked) {
    u8g2.drawStr(0, 55, "Rotate=select");
    u8g2.drawStr(0, 63, "Press=unlock");
  } else {
    u8g2.drawStr(0, 55, "Rotate=change");
    u8g2.drawStr(0, 63, "Press=lock");
  }
}

const char* headingToString(Heading h) {
  switch (h) {
    case Heading::North: return "N";
    case Heading::East:  return "E";
    case Heading::South: return "S";
    case Heading::West:  return "W";
    default: return "?";
  }
}

void drawRunScreen() {
  drawHeader("Run");
  uint32_t shownTime = Execution_state ? (millis() - executionStartMs) : frozenExecutionDurationMs;
  u8g2.setFont(u8g2_font_5x7_tf);

  String line1 = "M:" + String(gMode) + "/" + String(modeShortToString(gMode)) + " A:" + String(executionAttempt);
  String line2 = "T:" + formatMsAsSeconds(shownTime) + " " + (Execution_state ? "RUN" : "STOP");
  String line3 = "Ev:" + String(eventNameToString(currentEventName)) + " Hd:" + String(headingToString(currentHeading));
  String line4 = "US N:" + formatDistanceCm(distN) + " E:" + formatDistanceCm(distE);
  String line5 = "US S:" + formatDistanceCm(distS) + " W:" + formatDistanceCm(distW);
  String line6 = "IR NW:" + String(irRL) + " NE:" + String(irRR);
  String line7 = "IR SW:" + String(irFL) + " SE:" + String(irFR);

  u8g2.drawStr(0, 21, line1.c_str());
  u8g2.drawStr(0, 28, line2.c_str());
  u8g2.drawStr(0, 35, line3.c_str());
  u8g2.drawStr(0, 42, line4.c_str());
  u8g2.drawStr(0, 49, line5.c_str());
  u8g2.drawStr(0, 56, line6.c_str());
  u8g2.drawStr(0, 63, line7.c_str());
}

void drawUI() {
  u8g2.clearBuffer();

  switch (currentScreen) {
    case Screen::Root:          drawRootScreen(); break;
    case Screen::StartSettings: drawStartSettingsScreen(); break;
    case Screen::Settings:      drawSettingsScreen(); break;
    case Screen::PID:           drawPIDScreen(); break;
    case Screen::PIDTest:       drawPIDTestScreen(); break;
    case Screen::MotorTune:     drawMotorTuneScreen(); break;
    case Screen::EventTestMenu: drawEventTestMenuScreen(); break;
    case Screen::EventTestConfig: drawEventTestConfigScreen(); break;
    case Screen::EventTestRun:  drawEventTestRunScreen(); break;
    case Screen::MazeTestRun:   drawMazeTestRunScreen(); break;
    case Screen::IRSensorTest:  drawIRSensorTestScreen(); break;
    case Screen::UltrasonicSensorTest: drawUltrasonicSensorTestScreen(); break;
    case Screen::DigitEditor:   drawDigitEditorScreen(); break;
    case Screen::StartConfirm:  drawStartConfirmScreen(); break;
    case Screen::RunScreen:     drawRunScreen(); break;
  }

  u8g2.sendBuffer();
}

// ======================================================
// Setup menu loop
// ======================================================
void runMenuInSetup() {
  currentScreen = Screen::Root;
  workingSettings = activeSettings;
  programStarted = false;
  commitGlobalsForRuntime();

  while (!programStarted) {
    refreshSensors();
    int delta = consumeEncoderDelta();
    ButtonEvent btn = pollButtonEvent();

    switch (currentScreen) {
      case Screen::Root:          handleRootInput(delta, btn); break;
      case Screen::StartSettings: handleStartSettingsInput(delta, btn); break;
      case Screen::Settings:      handleSettingsInput(delta, btn); break;
      case Screen::PID:           handlePIDInput(delta, btn); break;
      case Screen::PIDTest:       handlePIDTestInput(btn); break;
      case Screen::MotorTune:     handleMotorTuneInput(delta, btn); break;
      case Screen::EventTestMenu: handleEventTestMenuInput(delta, btn); break;
      case Screen::EventTestConfig: handleEventTestConfigInput(delta, btn); break;
      case Screen::EventTestRun:  handleEventTestRunInput(btn); break;
      case Screen::MazeTestRun:   handleMazeTestRunInput(btn); break;
      case Screen::IRSensorTest:  handleIRSensorTestInput(delta, btn); break;
      case Screen::UltrasonicSensorTest: handleUltrasonicSensorTestInput(btn); break;
      case Screen::DigitEditor:   handleDigitEditorInput(delta, btn); break;
      case Screen::StartConfirm:  handleStartConfirmInput(btn); break;
      case Screen::RunScreen:     break;
    }

    // Live motor output while tuning
    if (currentScreen == Screen::MotorTune ||
        (currentScreen == Screen::DigitEditor && previousScreen == Screen::MotorTune)) {
      runTuneMotion();
    } else if (currentScreen == Screen::EventTestRun && eventTestActive) {
      runSelectedEventTest();
    } else if (currentScreen == Screen::PIDTest) {
      updatePIDPreview();
      stopAll();
    } else if (currentScreen == Screen::MazeTestRun) {
      updateMazeTestLogic();
      stopAll();
    } else {
      stopAll();
    }

    drawUI();
    delay(10);
  }

  stopAll();
}

// ======================================================
// Runtime updater
// ======================================================
void ScreenLoopUpdate(EventName event_name, bool stop_machine) {
  ButtonEvent btn = pollButtonEvent();
  currentScreen = Execution_state ? Screen::RunScreen : Screen::StartConfirm;

  if (Execution_state) {
    currentEventName = event_name;
    if (stop_machine) {
      finishExecutionAttempt();
      currentScreen = Screen::StartConfirm;
    }
  } else {
    currentEventName = (currentEventName == EventName::Finish) ? EventName::Finish : EventName::RestartReady;

    if (btn == ButtonEvent::ShortPress) {
      beginExecutionAttempt();
      currentScreen = Screen::RunScreen;
    } else if (btn == ButtonEvent::LongPress) {
      executionPhase = ExecutionPhase::Idle;
      currentEventName = EventName::Idle;
      eventTestActive = false;
      stopAll();
      runMenuInSetup();
      if (autoStartExecution) {
        autoStartExecution = false;
        beginExecutionAttempt();
      }
    }
  }

  drawUI();
}

// ======================================================
// Movement helpers
// ======================================================
static inline float clamp1(float v) {
  if (v >  1.0f) return  1.0f;
  if (v < -1.0f) return -1.0f;
  return v;
}

void writeCRServo(Servo &s, int stopVal, int rangeVal, int inv, float cmd) {
  cmd = clamp1(cmd) * inv;

  int usStop = map(stopVal, 0, 180, 1000, 2000);
  int usRange = (int)lround(rangeVal * (1000.0f / 180.0f));

  int us = usStop + (int)lround(cmd * usRange);
  us = constrain(us, 1000, 2000);

  s.writeMicroseconds(us);
}

void stopAll() {
  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, 0.0f);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, 0.0f);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, 0.0f);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, 0.0f);
}

// Actual drive path using runtime gains
void drive(float Vx, float Vy) {
  float pairA = Vy + Vx;  // FL + RR
  float pairB = Vy - Vx;  // FR + RL

  float m = max(fabs(pairA), fabs(pairB));
  if (m > 1.0f) {
    pairA /= m;
    pairB /= m;
  }

  // Forward/back pair gains depend on sign of pair command
  pairA *= (pairA >= 0.0f) ? gPairFLRRFwd : gPairFLRRBack;
  pairB *= (pairB >= 0.0f) ? gPairFRRLFwd : gPairFRRLBack;

  float fl = pairA * ((pairA >= 0.0f) ? gFLFwd : gFLBack);
  float rr = pairA * ((pairA >= 0.0f) ? gRRFwd : gRRBack);
  float fr = pairB * ((pairB >= 0.0f) ? gFRFwd : gFRBack);
  float rl = pairB * ((pairB >= 0.0f) ? gRLFwd : gRLBack);

  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, rl);
}

void driveHeading(float forwardCmd, float lateralCmd) {
  float vx = 0.0f;
  float vy = 0.0f;

  switch (currentHeading) {
    case Heading::North:
      vx = lateralCmd;
      vy = forwardCmd;
      break;
    case Heading::East:
      vx = forwardCmd;
      vy = -lateralCmd;
      break;
    case Heading::South:
      vx = -lateralCmd;
      vy = -forwardCmd;
      break;
    case Heading::West:
      vx = -forwardCmd;
      vy = lateralCmd;
      break;
  }

  drive(vx, vy);
}

void runTuneMotion() {
  switch (tuneMotion) {
    case TuneMotion::Stop:  stopAll(); break;
    case TuneMotion::North: currentHeading = Heading::North; driveHeading(TUNE_SPEED, 0.0f); break;
    case TuneMotion::South: currentHeading = Heading::South; driveHeading(TUNE_SPEED, 0.0f); break;
    case TuneMotion::East:  currentHeading = Heading::East;  driveHeading(TUNE_SPEED, 0.0f); break;
    case TuneMotion::West:  currentHeading = Heading::West;  driveHeading(TUNE_SPEED, 0.0f); break;
  }
}

// ======================================================
// UART sensor parsing
// D1 = S, D2 = W, D3 = N, D4 = E
// ======================================================
void parseDistances(String data) {
  int idx1 = data.indexOf("D1:");
  int idx2 = data.indexOf("D2:");
  int idx3 = data.indexOf("D3:");
  int idx4 = data.indexOf("D4:");

  if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0) return;

  idx1 += 3;
  idx2 += 3;
  idx3 += 3;
  idx4 += 3;

  int comma1 = data.indexOf(',', idx1);
  int comma2 = data.indexOf(',', idx2);
  int comma3 = data.indexOf(',', idx3);

  if (comma1 < 0 || comma2 < 0 || comma3 < 0) return;

  float d1 = 0.0f;
  float d2 = 0.0f;
  float d3 = 0.0f;
  float d4 = 0.0f;

  if (!parseFloatToken(data.substring(idx1, comma1), d1)) return;
  if (!parseFloatToken(data.substring(idx2, comma2), d2)) return;
  if (!parseFloatToken(data.substring(idx3, comma3), d3)) return;
  if (!parseFloatToken(data.substring(idx4), d4)) return;

  distN = d3;
  distE = d4;
  distS = d1;
  distW = d2;
  pushUltrasonicAverages(distN, distE, distS, distW);
  ultrasonicDataValid = true;
  lastUltrasonicUpdateMs = millis();
}

bool readSensorsUART() {
  bool newData = false;
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      parseDistances(incomingData);
      incomingData = "";
      newData = ultrasonicDataValid;
    } else if (c != '\r') {
      if (incomingData.length() < 96) {
        incomingData += c;
      } else {
        incomingData = "";
      }
    }
  }
  return newData;
}

// ======================================================
// PID
// ======================================================
float calculatePID(float error) {
  unsigned long now = millis();
  float dt = (now - previousTime) / 1000.0f;

  if (dt <= 0.0f) dt = 0.01f;
  if (dt > 0.2f)  dt = 0.2f;

  float oldError = previousError;

  float P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float I = Ki * integral;

  float derivative = (error - oldError) / dt;
  float D = Kd * derivative;

  previousError = error;
  previousTime = now;

  return constrain(P + I + D, -CORRECTION_LIMIT, CORRECTION_LIMIT);
}

// ======================================================
// Simple run-mode execution
// ======================================================
void followWallAwareHeading(float forwardSpeed, float wallThresholdCm, Heading heading) {
  bool leftWallPresent = ultrasonicFresh() && leftDistanceForHeading(heading) <= wallThresholdCm;
  bool rightWallPresent = ultrasonicFresh() && rightDistanceForHeading(heading) <= wallThresholdCm;
  float error = 0.0f;
  bool pidActive = false;

  if (leftWallPresent && rightWallPresent) {
    float left = averagedDistanceForHeading(turnLeftOf(heading));
    float right = averagedDistanceForHeading(turnRightOf(heading));
    error = interpretValue(right - left);
    pidActive = true;
  } else if (leftWallPresent) {
    float left = averagedDistanceForHeading(turnLeftOf(heading));
    error = interpretValue(gPidWallDistanceCm - left);
    pidActive = true;
  } else if (rightWallPresent) {
    float right = averagedDistanceForHeading(turnRightOf(heading));
    error = interpretValue(right - gPidWallDistanceCm);
    pidActive = true;
  }

  if (pidActive) {
    float correction = calculatePID(error);
    Heading savedHeading = currentHeading;
    currentHeading = heading;
    driveHeading(forwardSpeed, correction);
    currentHeading = savedHeading;
    return;
  }

  Heading savedHeading = currentHeading;
  currentHeading = heading;
  driveHeading(forwardSpeed, 0.0f);
  currentHeading = savedHeading;
}

void followCorridorAtSpeed(float forwardSpeed) {
  followWallAwareHeading(forwardSpeed, gWallThresholdCm, currentHeading);
}

void followCorridor() {
  followCorridorAtSpeed(BASE_SPEED);
}

void runSelectedEventTest() {
  Kp = gP;
  Ki = gI;
  Kd = gD;
  SensorSnapshot stableEventSnapshot = currentStableSensorSnapshot();
  bool hasStableEvent = confirmedStableSnapshotAvailable();
  float stopThreshold = currentEventTestStopThreshold();
  float sideThreshold = currentEventTestSideThreshold();
  EventName stableDetectedEvent = hasStableEvent
    ? classifySensorEventWithThresholds(stableEventSnapshot, currentHeading, true,
                                        gDeadEndDistanceCm, gTurningThresholdCm, sideThreshold, gFinishDistanceCm)
    : EventName::SensorWait;

  auto resolveRandomEvent = [&]() -> EventName {
    return stableDetectedEvent;
  };

  switch (eventTestPhase) {
    case EventTestPhase::Idle:
      stopAll();
      return;

    case EventTestPhase::StartSeek:
      currentEventName = EventName::Start;
      if (hasStableEvent &&
          classifySensorEventWithThresholds(stableEventSnapshot, currentHeading, false,
                                            gDeadEndDistanceCm, gTurningThresholdCm, sideThreshold, gFinishDistanceCm) == EventName::Corridor) {
        eventTestPhase = EventTestPhase::Detect;
        eventTestStartedMs = millis();
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        return;
      }
      driveHeading(APPROACH_SPEED, 0.0f);
      return;

    case EventTestPhase::Detect: {
      if (selectedEventTest == EventTestCase::Start ||
          selectedEventTest == EventTestCase::Corridor) {
        currentEventName = EventName::Corridor;
        followCorridor();
        return;
      }

      if (!hasStableEvent) {
        currentEventName = EventName::Corridor;
        followCorridor();
        return;
      }

      if (selectedEventTest == EventTestCase::Finish) {
        currentEventName = EventName::Corridor;
        if (stableDetectedEvent == EventName::Finish) {
          eventTestPhase = EventTestPhase::FinishStraight;
          eventTestStartedMs = millis();
          currentEventName = EventName::Finish;
          return;
        }
        followCorridor();
        return;
      }

      if (stableDetectedEvent == EventName::SensorWait ||
          stableDetectedEvent == EventName::Corridor) {
        currentEventName = EventName::Corridor;
        followCorridor();
        return;
      }

      if (selectedEventTest == EventTestCase::DeadEnd) {
        if (stableDetectedEvent != EventName::DeadEnd) {
          currentEventName = EventName::Corridor;
          followCorridor();
          return;
        }
        currentEventName = EventName::DeadEnd;
        currentHeading = oppositeOf(currentHeading);
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        eventTestPhase = EventTestPhase::Reverse;
        return;
      }

      if (selectedEventTest == EventTestCase::TJunctionStraight) {
        if (stableDetectedEvent != EventName::TJunctionStraight) {
          currentEventName = EventName::Corridor;
          followCorridor();
          return;
        }
        currentEventName = EventName::TJunctionStraight;
        eventTestPhase = EventTestPhase::StraightToSideThreshold;
        eventTestStartedMs = millis();
        return;
      }

      if (selectedEventTest == EventTestCase::Random) {
        eventTestResolvedEvent = resolveRandomEvent();
        if (eventTestResolvedEvent == EventName::SensorWait ||
            eventTestResolvedEvent == EventName::Corridor) {
          currentEventName = EventName::Corridor;
          followCorridor();
          return;
        }
      } else {
        if (stableDetectedEvent != eventTestCaseToEventName(selectedEventTest)) {
          currentEventName = EventName::Corridor;
          followCorridor();
          return;
        }
        eventTestResolvedEvent = stableDetectedEvent;
      }

      if (eventTestResolvedEvent == EventName::DeadEnd) {
        currentHeading = oppositeOf(currentHeading);
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        eventTestPhase = EventTestPhase::Reverse;
      } else if (eventTestResolvedEvent == EventName::Finish) {
        eventTestPhase = EventTestPhase::FinishStraight;
      } else if (eventTestResolvedEvent == EventName::TJunctionStraight) {
        eventTestPhase = EventTestPhase::StraightToSideThreshold;
      } else {
        eventTestPhase = EventTestPhase::ApproachWall;
      }
      eventTestStartedMs = millis();
      currentEventName = eventTestResolvedEvent;
      return;
    }

    case EventTestPhase::Reverse:
      currentEventName = EventName::DeadEnd;
      followCorridor();
      return;

    case EventTestPhase::ApproachWall:
      currentEventName = (eventTestResolvedEvent == EventName::RestartReady)
        ? EventName::Corridor
        : eventTestResolvedEvent;
      if (ultrasonicFresh() && frontDistance() <= stopThreshold) {
        eventTestPendingHeading = (eventTestResolvedEvent == EventName::TJunction)
          ? currentEventTestConfiguredTurn()
          : (eventTestResolvedEvent == EventName::LeftTurn ? turnLeftOf(currentHeading)
             : turnRightOf(currentHeading));
        eventTestPhase = EventTestPhase::WaitBeforeTurn;
        eventTestStartedMs = millis();
        stopAll();
        return;
      }
      driveHeading(APPROACH_SPEED, 0.0f);
      return;

    case EventTestPhase::WaitBeforeTurn:
      currentEventName = EventName::WaitAtTurn;
      if ((millis() - eventTestStartedMs) >= (uint32_t)(gWaitBeforeTurnSeconds * 1000.0f)) {
        currentHeading = eventTestPendingHeading;
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        eventTestStartedMs = millis();
        eventTestPhase = EventTestPhase::AcquireWall;
      }
      stopAll();
      return;

    case EventTestPhase::AcquireWall:
      currentEventName = (eventTestResolvedEvent == EventName::TJunction)
        ? ((currentEventTestConfig().turnChoice == TurnChoice::Left) ? EventName::LeftTurn : EventName::RightTurn)
        : eventTestResolvedEvent;
      if (bothWallsWithin(sideThreshold) ||
          (((millis() - eventTestStartedMs) >= EVENT_TEST_ACQUIRE_TIMEOUT_MS) &&
           (hasLeftWallWithin(sideThreshold) || hasRightWallWithin(sideThreshold)))) {
        eventTestPhase = EventTestPhase::Detect;
        eventTestStartedMs = millis();
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        return;
      }
      followWallAwareHeading(APPROACH_SPEED, sideThreshold, currentHeading);
      return;

    case EventTestPhase::FinishStraight:
      currentEventName = EventName::Finish;
      if ((millis() - eventTestStartedMs) >= currentEventTestActionMs()) {
        eventTestPhase = EventTestPhase::Hold;
        stopAll();
        return;
      }
      driveHeading(APPROACH_SPEED, 0.0f);
      return;

    case EventTestPhase::StraightToSideThreshold:
      currentEventName = EventName::TJunctionStraight;
      if (bothWallsWithin(sideThreshold) ||
          (((millis() - eventTestStartedMs) >= EVENT_TEST_ACQUIRE_TIMEOUT_MS) &&
           (hasLeftWallWithin(sideThreshold) || hasRightWallWithin(sideThreshold)))) {
        eventTestPhase = EventTestPhase::Detect;
        eventTestStartedMs = millis();
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        return;
      }
      followWallAwareHeading(APPROACH_SPEED, sideThreshold, currentHeading);
      return;

    case EventTestPhase::Hold:
    default:
      currentEventName = (selectedEventTest == EventTestCase::Finish) ? EventName::Finish : eventTestResolvedEvent;
      stopAll();
      return;
  }
}

void updateExecutionLogic(bool& stopNow, EventName& displayedEvent) {
  if (!Execution_state) return;

  Kp = gP;
  Ki = gI;
  Kd = gD;
  SensorSnapshot stableEventSnapshot = currentStableSensorSnapshot();
  EventName stableDetectedEvent = currentStableSensorEvent(currentHeading, true);
  float front = frontDistance();

  switch (executionPhase) {
    case ExecutionPhase::StartSeek:
      displayedEvent = EventName::Start;
      executionResolvedEvent = EventName::Start;
      if (stableDetectedEvent == EventName::Corridor) {
        executionPhase = ExecutionPhase::Corridor;
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        return;
      }
      driveHeading(APPROACH_SPEED, 0.0f);
      return;

    case ExecutionPhase::Corridor:
      if (!ultrasonicFresh()) {
        stopAll();
        displayedEvent = EventName::SensorWait;
        return;
      }

      if (stableDetectedEvent == EventName::Finish) {
        displayedEvent = EventName::Finish;
        stopNow = true;
        return;
      }

      if (stableDetectedEvent == EventName::DeadEnd) {
        currentHeading = oppositeOf(currentHeading);
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        displayedEvent = EventName::DeadEnd;
        followCorridor();
        return;
      }

      if (stableDetectedEvent == EventName::TJunction ||
          stableDetectedEvent == EventName::LeftTurn ||
          stableDetectedEvent == EventName::RightTurn) {
        executionResolvedEvent = stableDetectedEvent;
        if (stableDetectedEvent == EventName::LeftTurn) {
          pendingHeading = turnLeftOf(currentHeading);
        } else if (stableDetectedEvent == EventName::RightTurn) {
          pendingHeading = turnRightOf(currentHeading);
        } else {
          pendingHeading = chooseNextHeadingFromSnapshot(stableEventSnapshot, currentHeading);
        }
        executionPhase = ExecutionPhase::ApproachWall;
        phaseStartedMs = millis();
        displayedEvent = stableDetectedEvent;
        driveHeading(APPROACH_SPEED, 0.0f);
        return;
      }

      if (stableDetectedEvent == EventName::TJunctionStraight) {
        executionResolvedEvent = EventName::TJunctionStraight;
        executionPhase = ExecutionPhase::AcquireWall;
        pendingHeading = currentHeading;
        phaseStartedMs = millis();
        displayedEvent = EventName::TJunctionStraight;
        followWallAwareHeading(APPROACH_SPEED, gWallThresholdCm, currentHeading);
        return;
      }

      displayedEvent = EventName::Corridor;
      followCorridor();
      return;

    case ExecutionPhase::ApproachWall:
      if (!ultrasonicFresh()) {
        stopAll();
        displayedEvent = EventName::SensorWait;
        return;
      }

      displayedEvent = executionResolvedEvent;

      if (front <= gJunctionStopDistance) {
        if (pendingHeading == currentHeading) {
          displayedEvent = EventName::DeadEnd;
          currentHeading = oppositeOf(currentHeading);
          previousError = 0.0f;
          integral = 0.0f;
          previousTime = millis();
          executionPhase = ExecutionPhase::Corridor;
          return;
        }

        executionPhase = ExecutionPhase::WaitAtTurn;
        phaseStartedMs = millis();
        stopAll();
        displayedEvent = executionResolvedEvent;
        return;
      }

      driveHeading(APPROACH_SPEED, 0.0f);
      return;

    case ExecutionPhase::WaitAtTurn:
      displayedEvent = EventName::WaitAtTurn;
      stopAll();

      if ((millis() - phaseStartedMs) >= (uint32_t)(gWaitBeforeTurnSeconds * 1000.0f)) {
        currentHeading = pendingHeading;
        executionPhase = ExecutionPhase::AcquireWall;
        phaseStartedMs = millis();
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
      }
      return;

    case ExecutionPhase::AcquireWall:
      if (!ultrasonicFresh()) {
        stopAll();
        displayedEvent = EventName::SensorWait;
        return;
      }

      displayedEvent = executionResolvedEvent;

      if (bothWallsWithin(gWallThresholdCm) ||
          (((millis() - phaseStartedMs) >= EXECUTION_ACQUIRE_TIMEOUT_MS) &&
           (hasLeftWallWithin(gWallThresholdCm) || hasRightWallWithin(gWallThresholdCm)))) {
        executionPhase = ExecutionPhase::Corridor;
        previousError = 0.0f;
        integral = 0.0f;
        previousTime = millis();
        displayedEvent = EventName::Corridor;
        return;
      }

      followWallAwareHeading(APPROACH_SPEED, gWallThresholdCm, currentHeading);
      return;

    case ExecutionPhase::Finished:
      displayedEvent = EventName::Finish;
      stopAll();
      return;

    case ExecutionPhase::Idle:
    default:
      displayedEvent = EventName::Idle;
      stopAll();
      return;
  }
}

// ======================================================
// Setup / loop
// ======================================================
void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  pinMode(PIN_IR_FL, INPUT);
  pinMode(PIN_IR_FR, INPUT);
  pinMode(PIN_IR_RL, INPUT);
  pinMode(PIN_IR_RR, INPUT);

  Serial.begin(115200);
  Serial1.begin(9600);

  delay(50);

  Wire.begin();
  u8g2.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), onEncoderAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_FL), onAnyIRSensorChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_FR), onAnyIRSensorChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_RL), onAnyIRSensorChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_RR), onAnyIRSensorChange, CHANGE);

  S_FL.attach(PIN_FL);
  S_FR.attach(PIN_FR);
  S_RR.attach(PIN_RR);
  S_RL.attach(PIN_RL);

  stopAll();

  activeSettings.p_x100 = 7;
  activeSettings.i_x100 = 0;
  activeSettings.d_x100 = 0;
  activeSettings.pidCurveB_x100 = 100;
  activeSettings.pidWallDistance_x100 = 600;

  activeSettings.pairFLRR_fwd_x100 = 100;
  activeSettings.pairFLRR_back_x100 = 100;
  activeSettings.pairFRRL_fwd_x100 = 100;
  activeSettings.pairFRRL_back_x100 = 100;

  activeSettings.fl_fwd_x100 = 100;
  activeSettings.fl_back_x100 = 100;
  activeSettings.fr_fwd_x100 = 72;
  activeSettings.fr_back_x100 = 72;
  activeSettings.rr_fwd_x100 = 94;
  activeSettings.rr_back_x100 = 94;
  activeSettings.rl_fwd_x100 = 68;
  activeSettings.rl_back_x100 = 68;

  activeSettings.junctionStopDistance_x100 = 200;
  activeSettings.wallThreshold_x100 = 600;
  activeSettings.turningThreshold_x100 = 2600;
  activeSettings.waitBeforeTurn_x100 = 500;
  activeSettings.irStableTime_x100 = 10;
  activeSettings.deadEndDistance_x100 = 1600;
  activeSettings.finishDistance_x100 = 3000;
  activeSettings.mode = 1;

  for (int i = 0; i < EVENT_TEST_COUNT; ++i) {
    eventTestConfigs[i].heading = 0;
    eventTestConfigs[i].stopThreshold_x100 = 200;
    eventTestConfigs[i].sideThreshold_x100 = 600;
    eventTestConfigs[i].actionTime_x100 = 150;
    eventTestConfigs[i].moveTime_x100 = 150;
    eventTestConfigs[i].turnChoice = TurnChoice::Left;
  }

  workingSettings = activeSettings;
  commitGlobalsForRuntime();
  resetSequenceState();

  previousTime = millis();
  readIRSensors();
  lastObservedSnapshot = captureSensorSnapshot();
  stableSnapshot = lastObservedSnapshot;
  sensorLastChangeMs = millis();
  sensorChangeVersion = 1;
  sensorStableVersion = 1;
  mainHandledStableVersion = sensorStableVersion;
  eventTestHandledStableVersion = sensorStableVersion;
  mazeTestHandledStableVersion = sensorStableVersion;
  resetPIDPreview();
  resetMazeTest();

  runMenuInSetup();
  if (autoStartExecution) {
    autoStartExecution = false;
    beginExecutionAttempt();
  }

  Serial.println("=========================================");
  Serial.println("Live motor tune + run mode ready");
  Serial.println("=========================================");
}

void loop() {
  bool stopNow = false;
  EventName displayedEvent = currentEventName;
  refreshSensors();

  if (Execution_state) {
    updateExecutionLogic(stopNow, displayedEvent);
  }

  ScreenLoopUpdate(displayedEvent, stopNow);
}
