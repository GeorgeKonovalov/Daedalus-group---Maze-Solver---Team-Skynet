#if 0
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>
// The user confirmed the Nano RP2040 Connect IMU library is installed,
// so we keep the include unconditional and handle availability at runtime.
#include <Arduino_LSM6DSOX.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ======================================================
// Hardware
// ======================================================
constexpr uint8_t ENC_A_PIN   = 7;
constexpr uint8_t ENC_B_PIN   = 6;
constexpr uint8_t ENC_BTN_PIN = 8;

constexpr uint8_t PIN_FL = 5;
constexpr uint8_t PIN_FR = 3;
constexpr uint8_t PIN_RR = 9;
constexpr uint8_t PIN_RL = 4;

constexpr uint8_t PIN_IR_FL = 16;
constexpr uint8_t PIN_IR_FR = 17;
constexpr uint8_t PIN_IR_RL = 14;
constexpr uint8_t PIN_IR_RR = 15;

constexpr int STOP_FL = 90;
constexpr int STOP_FR = 90;
constexpr int STOP_RR = 90;
constexpr int STOP_RL = 90;

constexpr int RANGE_FL = 90;
constexpr int RANGE_FR = 90;
constexpr int RANGE_RR = 90;
constexpr int RANGE_RL = 90;

constexpr int INV_FL =  1;
constexpr int INV_FR = -1;
constexpr int INV_RR = -1;
constexpr int INV_RL =  1;

constexpr uint8_t OLED_ADDR = 0x3C;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ======================================================
// Timing and limits
// ======================================================
constexpr uint32_t BUTTON_DEBOUNCE_MS = 25;
constexpr uint32_t BUTTON_LONGPRESS_MS = 700;
constexpr uint32_t ULTRASONIC_STALE_MS = 500;
constexpr uint8_t ULTRASONIC_AVG_COUNT = 5;
constexpr uint8_t EVENT_CONFIRM_SAMPLES = 3;
constexpr uint8_t CORRIDOR_CONFIRM_SAMPLES = 3;
constexpr float SENSOR_MAX_VALID_CM = 60.0f;
constexpr float PID_INTERPRET_LIMIT_CM = 0.6f;
constexpr float PID_INTEGRAL_LIMIT = 40.0f;
constexpr float IMU_PID_INTEGRAL_LIMIT = 120.0f;
constexpr uint16_t IMU_REFERENCE_SAMPLES = 64;
constexpr uint16_t IMU_REFERENCE_SAMPLE_DELAY_MS = 5;
constexpr float IMU_GYRO_ZERO_RATE_LEVEL_DPS = 1.0f;   // LSM6DSOX G_TyOff
constexpr float IMU_GYRO_RMS_NOISE_DPS = 0.075f;       // LSM6DSOX RnRMS
constexpr float IMU_GYRO_SENS_TOL = 0.01f;             // LSM6DSOX G_So%
constexpr float IMU_ACCEL_ZERO_G_OFFSET_G = 0.020f;    // LSM6DSOX LA_TyOff
constexpr float IMU_ACCEL_RMS_NOISE_G = 0.0018f;       // LSM6DSOX RMS @ +/-2 g
constexpr float IMU_REFERENCE_MIN_Z_G = 0.25f;
constexpr float IMU_YAW_SIGN = 1.0f;                   // Flip if the board is mounted upside down.

// ======================================================
// Settings storage (fixed-point x100)
// ======================================================
struct RuntimeSettings {
  int p_x100 = 7;
  int i_x100 = 0;
  int d_x100 = 0;
  // Global IMU usage switch. 1 = allowed everywhere, 0 = disabled everywhere.
  uint8_t imuEnabled = 1;
  int imuP_x100 = 6;
  int imuI_x100 = 0;
  int imuD_x100 = 0;
  int imuAngleThreshold_x100 = 100;
  int curveB_x100 = 100;
  int pidWallDistance_x100 = 600;
  int pidLeftScale_x100 = 100;
  int pidRightScale_x100 = 100;
  int baseSpeed_x100 = 35;
  int approachSpeed_x100 = 28;
  int leftDriveScale_x100 = 100;
  int rightDriveScale_x100 = 100;

  int frontStopDistance_x100 = 200;
  int corridorWallThreshold_x100 = 600;
  int turnDetectDistance_x100 = 2600;
  int deadEndDistance_x100 = 1600;
  int finishDistance_x100 = 3000;
  int sensingInterval_x100 = 10;
  int eventConfirmCount = 3;
  int irStableTime_x100 = 10;
  int waitBeforeTurn_x100 = 500;

  uint8_t routeMode = 1;
  uint8_t startHeadingIndex = 0;
};

struct RuntimeConfig {
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
  bool imuEnabled = true;
  float imuP = 0.0f;
  float imuI = 0.0f;
  float imuD = 0.0f;
  float imuAngleThresholdDeg = 1.0f;
  float curveB = 1.0f;
  float pidWallDistanceCm = 6.0f;
  float pidLeftScale = 1.0f;
  float pidRightScale = 1.0f;
  float baseSpeed = 0.35f;
  float approachSpeed = 0.28f;
  float leftDriveScale = 1.0f;
  float rightDriveScale = 1.0f;

  float frontStopDistanceCm = 2.0f;
  float corridorWallThresholdCm = 6.0f;
  float turnDetectDistanceCm = 26.0f;
  float deadEndDistanceCm = 16.0f;
  float finishDistanceCm = 30.0f;
  float sensingIntervalS = 0.10f;
  uint8_t eventConfirmCount = 3;
  float irStableTimeS = 0.10f;
  float waitBeforeTurnS = 5.0f;
};

RuntimeSettings activeSettings;
RuntimeSettings workingSettings;
RuntimeConfig gCfg;

// ======================================================
// High-level architecture types
// ======================================================
enum class Heading : uint8_t {
  North,
  East,
  South,
  West
};

enum class RouteMode : uint8_t {
  Sequence = 1,
  Right = 2,
  Left = 3,
  Selection = 4
};

enum class EventType : uint8_t {
  Idle,
  Start,
  Corridor,
  LeftTurn,
  RightTurn,
  TJunction,
  TJunctionStraight,
  DeadEnd,
  Finish,
  SensorWait,
  SafetyStop
};

enum class NavState : uint8_t {
  Idle,
  StartSeek,
  Corridor,
  ApproachWall,
  WaitTurn,
  AcquireCorridor,
  SafetyStop,
  Finished
};

enum class PidSource : uint8_t {
  None,
  LeftWall,
  RightWall,
  TwoWall
};

enum class Screen : uint8_t {
  Root,
  StartSettings,
  Settings,
  PID,
  PIDTest,
  IMUPIDTest,
  MotorTune,
  EventTestMenu,
  EventTestConfig,
  EventTestRun,
  IMUTest,
  IMUYawTest,
  MazeTestRun,
  IRSensorTest,
  UltrasonicSensorTest,
  MotionSettings,
  DetectionSettings,
  SensorMonitor,
  TwinMonitor,
  DigitEditor,
  StartConfirm,
  RunScreen
};

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
  bool longPressReported = false;
};

struct RelativeIrState {
  bool frontLeft = false;
  bool frontRight = false;
  bool rearLeft = false;
  bool rearRight = false;
};

struct PerceptionFrame {
  bool ultrasonicValid = false;
  uint8_t stableIrMask = 0;
  RelativeIrState ir;
  float north = 0.0f;
  float east = 0.0f;
  float south = 0.0f;
  float west = 0.0f;
  float front = 0.0f;
  float left = 0.0f;
  float right = 0.0f;
  float back = 0.0f;
  EventType candidate = EventType::SensorWait;
  bool corridorSignature = false;
  PidSource pidSource = PidSource::None;
};

struct EventConfirmState {
  bool active = false;
  EventType candidate = EventType::Idle;
  uint8_t matches = 0;
  uint32_t nextCheckMs = 0;
};

struct CorridorConfirmState {
  bool active = false;
  uint8_t matches = 0;
  uint32_t nextCheckMs = 0;
};

enum class TurnChoice : uint8_t {
  Left,
  Right
};

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

enum class EventConfigField : uint8_t {
  Heading,
  TurnChoice,
  FrontStop,
  WallThreshold,
  TurnDetect,
  DeadEnd,
  Finish,
  TurnWait,
  CheckGap,
  MatchCount,
  IRStable,
  Run,
  Back
};

struct EventTestConfig {
  uint8_t headingIndex = 0;
  TurnChoice turnChoice = TurnChoice::Left;
};

struct PidRuntime {
  float previousError = 0.0f;
  float integral = 0.0f;
  uint32_t previousMs = 0;
};

struct ImuReference {
  bool valid = false;
  float yawDeg = 0.0f;
  float xAxisX = 1.0f;
  float xAxisY = 0.0f;
  float yAxisX = 0.0f;
  float yAxisY = 1.0f;
  float avgAx = 0.0f;
  float avgAy = 0.0f;
  float avgAz = 1.0f;
  float planarTolG = 0.0f;
  float angleTolDeg = 0.0f;
  uint32_t capturedMs = 0;
};

struct ImuState {
  bool available = false;
  bool referenceValid = false;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float gyroBiasZDps = 0.0f;
  float yawDeg = 0.0f;
  float yawRateDps = 0.0f;
  float sampleDtS = 0.01f;
  float currentXAxisX = 1.0f;
  float currentXAxisY = 0.0f;
  float yawErrorDeg = 0.0f;
  float measurementAngleTolDeg = 0.0f;
  float combinedDeadbandDeg = 0.0f;
  float lastCompensationCmd = 0.0f;
  uint32_t lastSampleMs = 0;
  ImuReference reference;
};

using ImuPidRuntime = PidRuntime;

struct PlannerState {
  uint32_t attemptCounter = 0;
  uint32_t lastAttemptDurationMs = 0;
  uint32_t totalDurationMs = 0;
  uint8_t sequenceSlot = 0;
  bool sequenceCycleComplete = false;
};

struct ExecutiveState {
  bool running = false;
  NavState state = NavState::Idle;
  Heading heading = Heading::North;
  Heading pendingHeading = Heading::North;
  EventType displayEvent = EventType::Idle;
  EventType latchedEvent = EventType::Idle;
  uint32_t runStartedMs = 0;
  uint32_t phaseStartedMs = 0;
  EventConfirmState eventConfirm;
  CorridorConfirmState corridorConfirm;
};

struct EventTestState {
  bool active = false;
  NavState state = NavState::Idle;
  Heading heading = Heading::North;
  Heading pendingHeading = Heading::North;
  EventType displayEvent = EventType::Idle;
  EventType latchedEvent = EventType::Idle;
  EventConfirmState eventConfirm;
  CorridorConfirmState corridorConfirm;
  uint32_t startedMs = 0;
  uint32_t phaseStartedMs = 0;
};

enum class TuneMotion : uint8_t {
  Stop,
  North,
  East,
  South,
  West
};

struct DigitEditor {
  bool active = false;
  bool digitUnlocked = false;
  uint8_t selectedDigit = 0;
  int tempValue = 0;
  int* targetValue = nullptr;
  int minValue = 0;
  int maxValue = 9999;
  const char* label = "";
  const char* unit = "";
  const char* description1 = "";
  const char* description2 = "";
  Screen returnScreen = Screen::Root;
};

// ======================================================
// Global runtime state
// ======================================================
Servo servoFL;
Servo servoFR;
Servo servoRR;
Servo servoRL;

volatile int16_t gEncoderDelta = 0;
volatile bool gIrInterruptFlag = false;

ButtonState gButton;
DigitEditor gDigitEditor;
PlannerState gPlanner;
ExecutiveState gExecutive;
EventTestState gEventTestRun;
PidRuntime gPid;
ImuPidRuntime gImuPid;
ImuState gImu;

Screen gCurrentScreen = Screen::Root;
Screen gPreviousScreen = Screen::Root;

int gRootIndex = 0;
int gStartSettingsIndex = 0;
int gSettingsIndex = 0;
int gPidIndex = 0;
int gMotorTuneIndex = 0;
int gMotionSettingsIndex = 0;
int gDetectionSettingsIndex = 0;
int gEventTestIndex = 0;
int gEventTestConfigIndex = 0;
int gIrSensorIndex = 0;

Heading gPreviewHeading = Heading::North;

EventTestCase gSelectedEventTest = EventTestCase::Corridor;
EventTestConfig gEventTestConfigs[10];
Heading gPidTestHeading = Heading::North;
TuneMotion gTuneMotion = TuneMotion::Stop;

int gIrFL = LOW;
int gIrFR = LOW;
int gIrRL = LOW;
int gIrRR = LOW;
uint8_t gRawIrMask = 0;
uint8_t gIrCandidateMask = 0;
uint8_t gStableIrMask = 0;
uint32_t gIrCandidateSinceMs = 0;

float gDistN = 0.0f;
float gDistE = 0.0f;
float gDistS = 0.0f;
float gDistW = 0.0f;
float gAvgN = 0.0f;
float gAvgE = 0.0f;
float gAvgS = 0.0f;
float gAvgW = 0.0f;
float gBufN[ULTRASONIC_AVG_COUNT] = {0};
float gBufE[ULTRASONIC_AVG_COUNT] = {0};
float gBufS[ULTRASONIC_AVG_COUNT] = {0};
float gBufW[ULTRASONIC_AVG_COUNT] = {0};
uint8_t gBufIndex = 0;
uint8_t gBufCount = 0;
uint32_t gLastUltrasonicMs = 0;
bool gUltrasonicValid = false;

char gUartLine[96];
uint8_t gUartPos = 0;

// ======================================================
// Forward declarations
// ======================================================
void onEncoderEdge();
void onAnyIrChange();

void commitRuntimeConfig();
void resetPid();
void resetImuPid();
bool imuHardwareReady();
bool imuUsageEnabled();
void clearImuReference();

float fixedToFloat(int value);
int clampInt(int value, int minValue, int maxValue);
float clampFloat(float value, float minValue, float maxValue);
float clampUnit(float value);
uint32_t secondsToMs(float seconds);
uint8_t wrap4(int value);
uint8_t buildIrMaskFromPins();
bool isIrTriggered(int value);
bool ultrasonicFresh();
bool parseDistancesLine(const char* line, float& d1, float& d2, float& d3, float& d4);
void pushUltrasonicAverages(float north, float east, float south, float west);
void readIRSensors();
void initImu();
void updateImu();
bool captureImuReference();
void updateImuDerivedState();
float imuReferenceAngleToleranceDeg(float captureDurationS);
float imuPlanarAngleToleranceDeg(uint16_t sampleCount, float avgAz);
float imuMeasurementAngleToleranceDeg(float angleErrorDeg, float dt);
float imuGyroRateFloorDps();
float signedAngleBetweenDeg(float ax, float ay, float bx, float by);
void yawAxisForAngle(float yawDeg, float& xOut, float& yOut);
float applyImuCompensation(ImuPidRuntime& runtime);
void refreshSensors();

Heading headingFromIndex(uint8_t index);
uint8_t headingToIndex(Heading heading);
Heading turnLeftOf(Heading heading);
Heading turnRightOf(Heading heading);
Heading oppositeOf(Heading heading);
float distanceForHeading(Heading heading);

RelativeIrState relativeIrForHeading(uint8_t mask, Heading heading);
PerceptionFrame buildPerceptionFrame(Heading heading);
EventType classifyCandidateEvent(const PerceptionFrame& frame);
bool corridorSignatureForFrame(const PerceptionFrame& frame);
PidSource choosePidSource(const PerceptionFrame& frame);

RouteMode currentRouteMode();
RouteMode effectiveRouteMode();
Heading chooseHeadingForEvent(const PerceptionFrame& frame, EventType eventType, Heading currentHeading);

bool updateEventConfirmation(const PerceptionFrame& frame, EventType& confirmedEvent);
bool updateCorridorConfirmation(const PerceptionFrame& frame);
void resetEventConfirmation();
void resetCorridorConfirmation();

float shapedMagnitude(float magnitude);
float shapeSignedError(float error);
float computePid(float error);
float computeWallError(const PerceptionFrame& frame, PidSource source);
void writeServoCommand(Servo& servo, int stopAngle, int rangeAngle, int invert, float cmd);
void stopAll();
void driveRobotFrame(float lateralCmd, float forwardCmd, float rotationCmd, Heading heading);
void followSmart(Heading heading, float forwardSpeed);

void beginExecution();
void finishExecution();
void updateExecutive();

ButtonEvent readButtonEvent(bool allowInput);
void openDigitEditor(const char* label, const char* unit, int* target, int minValue, int maxValue,
                     const char* desc1, const char* desc2, Screen returnScreen);
void closeDigitEditor(bool commitValue);
void handleDigitEditorInput(int encoderDelta, ButtonEvent buttonEvent);
void handleIdleUiInput(int encoderDelta, ButtonEvent buttonEvent);
void handleRootInput(int encoderDelta, ButtonEvent buttonEvent);
void handleStartSettingsInput(int encoderDelta, ButtonEvent buttonEvent);
void handleSettingsInput(int encoderDelta, ButtonEvent buttonEvent);
void handlePidInput(int encoderDelta, ButtonEvent buttonEvent);
void handlePidTestInput(ButtonEvent buttonEvent);
void handleImuPidTestInput(ButtonEvent buttonEvent);
void handleMotorTuneInput(int encoderDelta, ButtonEvent buttonEvent);
void handleEventTestMenuInput(int encoderDelta, ButtonEvent buttonEvent);
void handleEventTestConfigInput(int encoderDelta, ButtonEvent buttonEvent);
void handleEventTestRunInput(ButtonEvent buttonEvent);
void handleImuTestInput(ButtonEvent buttonEvent);
void handleImuYawTestInput(ButtonEvent buttonEvent);
void handleIRSensorTestInput(int encoderDelta, ButtonEvent buttonEvent);
void handleUltrasonicSensorTestInput(ButtonEvent buttonEvent);
void handleMazeTestRunInput(int encoderDelta, ButtonEvent buttonEvent);
void handleStartConfirmInput(ButtonEvent buttonEvent);

void startEventTestRun();
void stopEventTestRun();
void updateEventTestRun();
EventTestConfig& currentEventTestConfig();
Heading currentEventTestHeading();
Heading currentEventTestConfiguredTurn();
const char* eventTestCaseToString(EventTestCase testCase);
EventType selectedEventTestTarget();

const char* headingToString(Heading heading);
const char* routeModeToString(RouteMode mode);
const char* eventToString(EventType eventType);
const char* navStateToString(NavState state);
const char* pidSourceToString(PidSource source);
const char* sequenceStepToString();
const char* turnChoiceToString(TurnChoice choice);
void formatFixedValue(char* buffer, size_t size, int value);
void formatDuration(char* buffer, size_t size, uint32_t durationMs);
static bool eventConfigFieldRelevant(EventTestCase testCase, EventConfigField field);
static int currentEventConfigItemCount();
static EventConfigField currentEventConfigFieldAt(int selectedIndex);
static const char* eventConfigDesc1(EventConfigField field);
static const char* eventConfigDesc2(EventConfigField field);
static void eventConfigFieldText(EventConfigField field, char* buffer, size_t size);

void drawHeader(const char* title);
void drawMenuItem(int y, bool selected, const char* text);
void drawScrollableItemList(const char* const* items, int itemCount, int selectedIndex, int startY);
void drawRootScreen();
void drawStartSettingsScreen();
void drawSettingsScreen();
void drawPIDScreen();
void drawPIDTestScreen();
void drawIMUPIDTestScreen();
void drawMotorTuneScreen();
void drawEventTestMenuScreen();
void drawEventTestConfigScreen();
void drawEventTestRunScreen();
void drawIMUTestScreen();
void drawIMUYawTestScreen();
void drawIRSensorTestScreen();
void drawUltrasonicSensorTestScreen();
void drawMazeTestRunScreen();
void drawDigitEditorScreen();
void drawStartConfirmScreen();
void drawRunScreen();
void drawUI();

// ======================================================
// Implementation sections
// ======================================================
// Helpers
float fixedToFloat(int value) {
  return value / 100.0f;
}
#endif

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float clampUnit(float value) {
  return clampFloat(value, -1.0f, 1.0f);
}

uint32_t secondsToMs(float seconds) {
  if (seconds <= 0.0f) return 0;
  return (uint32_t)lroundf(seconds * 1000.0f);
}

uint8_t wrap4(int value) {
  while (value < 0) value += 4;
  while (value > 3) value -= 4;
  return (uint8_t)value;
}

static float wrapAngleDeg(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

float signedAngleBetweenDeg(float ax, float ay, float bx, float by) {
  float dot = clampFloat(ax * bx + ay * by, -1.0f, 1.0f);
  float det = ax * by - ay * bx;
  return atan2f(det, dot) * 180.0f / PI;
}

void yawAxisForAngle(float yawDeg, float& xOut, float& yOut) {
  float yawRad = yawDeg * DEG_TO_RAD;
  xOut = cosf(yawRad);
  yOut = sinf(yawRad);
}

float imuGyroRateFloorDps() {
  return sqrtf(IMU_GYRO_ZERO_RATE_LEVEL_DPS * IMU_GYRO_ZERO_RATE_LEVEL_DPS +
               IMU_GYRO_RMS_NOISE_DPS * IMU_GYRO_RMS_NOISE_DPS);
}

float imuReferenceAngleToleranceDeg(float captureDurationS) {
  float noiseTerm = IMU_GYRO_RMS_NOISE_DPS / sqrtf((float)max((int)IMU_REFERENCE_SAMPLES, 1));
  float rateTolDps = sqrtf(IMU_GYRO_ZERO_RATE_LEVEL_DPS * IMU_GYRO_ZERO_RATE_LEVEL_DPS +
                           noiseTerm * noiseTerm);
  return rateTolDps * max(captureDurationS, 0.0f);
}

float imuPlanarAngleToleranceDeg(uint16_t sampleCount, float avgAz) {
  float samples = max((int)sampleCount, 1);
  float noiseTerm = IMU_ACCEL_RMS_NOISE_G / sqrtf((float)samples);
  float planarTolG = sqrtf(2.0f * (IMU_ACCEL_ZERO_G_OFFSET_G * IMU_ACCEL_ZERO_G_OFFSET_G +
                                   noiseTerm * noiseTerm));
  float zReference = max(fabsf(avgAz), IMU_REFERENCE_MIN_Z_G);
  return atan2f(planarTolG, zReference) * 180.0f / PI;
}

float imuMeasurementAngleToleranceDeg(float angleErrorDeg, float dt) {
  float rateAngleTolDeg = imuGyroRateFloorDps() * max(dt, 0.0f);
  float scaleAngleTolDeg = fabsf(angleErrorDeg) * IMU_GYRO_SENS_TOL;
  return sqrtf(rateAngleTolDeg * rateAngleTolDeg +
               scaleAngleTolDeg * scaleAngleTolDeg);
}

void commitRuntimeConfig() {
  activeSettings = workingSettings;

  gCfg.p = fixedToFloat(activeSettings.p_x100);
  gCfg.i = fixedToFloat(activeSettings.i_x100);
  gCfg.d = fixedToFloat(activeSettings.d_x100);
  gCfg.imuEnabled = (activeSettings.imuEnabled != 0);
  gCfg.imuP = fixedToFloat(activeSettings.imuP_x100);
  gCfg.imuI = fixedToFloat(activeSettings.imuI_x100);
  gCfg.imuD = fixedToFloat(activeSettings.imuD_x100);
  gCfg.imuAngleThresholdDeg = fixedToFloat(activeSettings.imuAngleThreshold_x100);
  gCfg.curveB = clampFloat(fixedToFloat(activeSettings.curveB_x100), 0.10f, 20.0f);
  gCfg.pidWallDistanceCm = fixedToFloat(activeSettings.pidWallDistance_x100);
  gCfg.pidLeftScale = fixedToFloat(activeSettings.pidLeftScale_x100);
  gCfg.pidRightScale = fixedToFloat(activeSettings.pidRightScale_x100);
  gCfg.baseSpeed = clampFloat(fixedToFloat(activeSettings.baseSpeed_x100), 0.0f, 1.0f);
  gCfg.approachSpeed = clampFloat(fixedToFloat(activeSettings.approachSpeed_x100), 0.0f, 1.0f);
  gCfg.leftDriveScale = clampFloat(fixedToFloat(activeSettings.leftDriveScale_x100), 0.10f, 3.0f);
  gCfg.rightDriveScale = clampFloat(fixedToFloat(activeSettings.rightDriveScale_x100), 0.10f, 3.0f);

  gCfg.frontStopDistanceCm = fixedToFloat(activeSettings.frontStopDistance_x100);
  gCfg.corridorWallThresholdCm = fixedToFloat(activeSettings.corridorWallThreshold_x100);
  gCfg.turnDetectDistanceCm = fixedToFloat(activeSettings.turnDetectDistance_x100);
  gCfg.deadEndDistanceCm = fixedToFloat(activeSettings.deadEndDistance_x100);
  gCfg.finishDistanceCm = fixedToFloat(activeSettings.finishDistance_x100);
  gCfg.sensingIntervalS = clampFloat(fixedToFloat(activeSettings.sensingInterval_x100), 0.01f, 10.0f);
  gCfg.eventConfirmCount = (uint8_t)clampInt(activeSettings.eventConfirmCount, 1, 9);
  gCfg.irStableTimeS = clampFloat(fixedToFloat(activeSettings.irStableTime_x100), 0.01f, 10.0f);
  gCfg.waitBeforeTurnS = clampFloat(fixedToFloat(activeSettings.waitBeforeTurn_x100), 0.0f, 30.0f);

  // Turning IMU usage off must immediately remove its influence everywhere.
  if (!gCfg.imuEnabled) {
    clearImuReference();
    resetImuPid();
  }
}

void resetPid() {
  gPid.previousError = 0.0f;
  gPid.integral = 0.0f;
  gPid.previousMs = millis();
}

void resetImuPid() {
  gImuPid.previousError = 0.0f;
  gImuPid.integral = 0.0f;
  gImuPid.previousMs = millis();
  gImu.lastCompensationCmd = 0.0f;
}

bool imuHardwareReady() {
  return gImu.available;
}

bool imuUsageEnabled() {
  return gCfg.imuEnabled && imuHardwareReady();
}

void clearImuReference() {
  gImu.reference.valid = false;
  gImu.referenceValid = false;
  gImu.yawErrorDeg = 0.0f;
  gImu.measurementAngleTolDeg = 0.0f;
  gImu.combinedDeadbandDeg = 0.0f;
  gImu.lastCompensationCmd = 0.0f;
}

Heading headingFromIndex(uint8_t index) {
  static const Heading headings[4] = {
    Heading::North, Heading::East, Heading::South, Heading::West
  };
  return headings[index & 0x03];
}

uint8_t headingToIndex(Heading heading) {
  switch (heading) {
    case Heading::North: return 0;
    case Heading::East:  return 1;
    case Heading::South: return 2;
    case Heading::West:  return 3;
    default:             return 0;
  }
}

Heading turnLeftOf(Heading heading) {
  return headingFromIndex(wrap4((int)headingToIndex(heading) - 1));
}

Heading turnRightOf(Heading heading) {
  return headingFromIndex(wrap4((int)headingToIndex(heading) + 1));
}

Heading oppositeOf(Heading heading) {
  return headingFromIndex(wrap4((int)headingToIndex(heading) + 2));
}

float distanceForHeading(Heading heading) {
  float ultrasonicByIndex[4] = {gAvgN, gAvgE, gAvgS, gAvgW};
  return ultrasonicByIndex[headingToIndex(heading)];
}

const char* headingToString(Heading heading) {
  switch (heading) {
    case Heading::North: return "N";
    case Heading::East:  return "E";
    case Heading::South: return "S";
    case Heading::West:  return "W";
    default:             return "?";
  }
}

const char* routeModeToString(RouteMode mode) {
  switch (mode) {
    case RouteMode::Sequence:  return "Sequence";
    case RouteMode::Right:     return "Right";
    case RouteMode::Left:      return "Left";
    case RouteMode::Selection: return "Selection";
    default:                   return "?";
  }
}

const char* eventToString(EventType eventType) {
  switch (eventType) {
    case EventType::Idle:             return "Idle";
    case EventType::Start:            return "Start";
    case EventType::Corridor:         return "Corridor";
    case EventType::LeftTurn:         return "Left Turn";
    case EventType::RightTurn:        return "Right Turn";
    case EventType::TJunction:        return "T-Junction";
    case EventType::TJunctionStraight: return "T-Straight";
    case EventType::DeadEnd:          return "Dead-End";
    case EventType::Finish:           return "Finish";
    case EventType::SensorWait:       return "Sensor Wait";
    case EventType::SafetyStop:       return "Safety Stop";
    default:                          return "?";
  }
}

const char* navStateToString(NavState state) {
  switch (state) {
    case NavState::Idle:            return "Idle";
    case NavState::StartSeek:       return "Start";
    case NavState::Corridor:        return "Corridor";
    case NavState::ApproachWall:    return "Approach";
    case NavState::WaitTurn:        return "Wait";
    case NavState::AcquireCorridor: return "Acquire";
    case NavState::SafetyStop:      return "Stop";
    case NavState::Finished:        return "Finish";
    default:                        return "?";
  }
}

const char* pidSourceToString(PidSource source) {
  switch (source) {
    case PidSource::None:      return "None";
    case PidSource::LeftWall:  return "1W Left";
    case PidSource::RightWall: return "1W Right";
    case PidSource::TwoWall:   return "2W";
    default:                   return "?";
  }
}

const char* sequenceStepToString() {
  if (currentRouteMode() != RouteMode::Sequence) return "-";
  switch (gPlanner.sequenceSlot % 3) {
    case 0: return "Right";
    case 1: return "Left";
    default: return "Selection";
  }
}

const char* turnChoiceToString(TurnChoice choice) {
  return (choice == TurnChoice::Left) ? "Left" : "Right";
}

const char* eventTestCaseToString(EventTestCase testCase) {
  switch (testCase) {
    case EventTestCase::Corridor:          return "Corridor";
    case EventTestCase::TJunction:         return "T-Junction";
    case EventTestCase::DeadEnd:           return "Dead-end";
    case EventTestCase::Start:             return "Auto Start";
    case EventTestCase::Finish:            return "Finish";
    case EventTestCase::LeftTurn:          return "Left Turn";
    case EventTestCase::RightTurn:         return "Right Turn";
    case EventTestCase::TJunctionStraight: return "T-Jct Straight";
    case EventTestCase::Random:            return "Random";
    case EventTestCase::Back:              return "Back";
    default:                               return "?";
  }
}

EventTestConfig& currentEventTestConfig() {
  return gEventTestConfigs[(int)gSelectedEventTest];
}

Heading currentEventTestHeading() {
  return headingFromIndex(currentEventTestConfig().headingIndex);
}

Heading currentEventTestConfiguredTurn() {
  return (currentEventTestConfig().turnChoice == TurnChoice::Left)
           ? turnLeftOf(gEventTestRun.heading)
           : turnRightOf(gEventTestRun.heading);
}

EventType selectedEventTestTarget() {
  switch (gSelectedEventTest) {
    case EventTestCase::Corridor:          return EventType::Corridor;
    case EventTestCase::TJunction:         return EventType::TJunction;
    case EventTestCase::DeadEnd:           return EventType::DeadEnd;
    case EventTestCase::Start:             return EventType::Start;
    case EventTestCase::Finish:            return EventType::Finish;
    case EventTestCase::LeftTurn:          return EventType::LeftTurn;
    case EventTestCase::RightTurn:         return EventType::RightTurn;
    case EventTestCase::TJunctionStraight: return EventType::TJunctionStraight;
    case EventTestCase::Random:            return EventType::Idle;
    case EventTestCase::Back:
    default:                               return EventType::Idle;
  }
}

void formatFixedValue(char* buffer, size_t size, int value) {
  int safeValue = clampInt(value, 0, 9999);
  snprintf(buffer, size, "%2d.%02d", safeValue / 100, abs(safeValue % 100));
}

void formatDuration(char* buffer, size_t size, uint32_t durationMs) {
  uint32_t totalSeconds = durationMs / 1000UL;
  uint32_t minutes = totalSeconds / 60UL;
  uint32_t seconds = totalSeconds % 60UL;
  uint32_t centiseconds = (durationMs % 1000UL) / 10UL;
  snprintf(buffer, size, "%lu:%02lu.%02lu",
           (unsigned long)minutes,
           (unsigned long)seconds,
           (unsigned long)centiseconds);
}

// Input
void onEncoderEdge() {
  static uint8_t lastState = 0;
  uint8_t a = (uint8_t)digitalRead(ENC_A_PIN);
  uint8_t b = (uint8_t)digitalRead(ENC_B_PIN);
  uint8_t state = (a << 1) | b;
  uint8_t transition = (lastState << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      gEncoderDelta++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      gEncoderDelta--;
      break;
    default:
      break;
  }

  lastState = state;
}

void onAnyIrChange() {
  gIrInterruptFlag = true;
}

ButtonEvent readButtonEvent(bool allowInput) {
  bool rawLevel = digitalRead(ENC_BTN_PIN);
  uint32_t now = millis();

  if (!allowInput) {
    gButton.lastRawLevel = rawLevel;
    gButton.stableLevel = rawLevel;
    gButton.longPressReported = false;
    if (rawLevel == LOW) {
      gButton.pressedAtMs = now;
    }
    return ButtonEvent::None;
  }

  ButtonEvent result = ButtonEvent::None;

  if (rawLevel != gButton.lastRawLevel) {
    gButton.lastRawLevel = rawLevel;
    gButton.lastDebounceMs = now;
  }

  if ((now - gButton.lastDebounceMs) >= BUTTON_DEBOUNCE_MS) {
    if (gButton.stableLevel != rawLevel) {
      gButton.stableLevel = rawLevel;
      if (rawLevel == LOW) {
        gButton.pressedAtMs = now;
        gButton.longPressReported = false;
      } else {
        if (!gButton.longPressReported) {
          result = ButtonEvent::ShortPress;
        }
      }
    }
  }

  if (gButton.stableLevel == LOW && !gButton.longPressReported) {
    if ((now - gButton.pressedAtMs) >= BUTTON_LONGPRESS_MS) {
      gButton.longPressReported = true;
      result = ButtonEvent::LongPress;
    }
  }

  return result;
}

void openDigitEditor(const char* label, const char* unit, int* target, int minValue, int maxValue,
                     const char* desc1, const char* desc2, Screen returnScreen) {
  gDigitEditor.active = true;
  gDigitEditor.digitUnlocked = false;
  gDigitEditor.selectedDigit = 0;
  gDigitEditor.targetValue = target;
  gDigitEditor.tempValue = (target != nullptr) ? *target : 0;
  gDigitEditor.minValue = minValue;
  gDigitEditor.maxValue = maxValue;
  gDigitEditor.label = label;
  gDigitEditor.unit = unit;
  gDigitEditor.description1 = desc1;
  gDigitEditor.description2 = desc2;
  gDigitEditor.returnScreen = returnScreen;
  gPreviousScreen = returnScreen;
  gCurrentScreen = Screen::DigitEditor;
}

void closeDigitEditor(bool commitValue) {
  if (commitValue && gDigitEditor.targetValue != nullptr) {
    *gDigitEditor.targetValue = clampInt(gDigitEditor.tempValue,
                                         gDigitEditor.minValue,
                                         gDigitEditor.maxValue);
    commitRuntimeConfig();
  }
  gDigitEditor.active = false;
  gDigitEditor.digitUnlocked = false;
  gCurrentScreen = gDigitEditor.returnScreen;
}

static int placeValueForDigit(uint8_t selectedDigit) {
  switch (selectedDigit) {
    case 0: return 1000;
    case 1: return 100;
    case 2: return 10;
    default: return 1;
  }
}

static int getDigitAt(int value, uint8_t selectedDigit) {
  int place = placeValueForDigit(selectedDigit);
  return (value / place) % 10;
}

static int setDigitAt(int value, uint8_t selectedDigit, int newDigit) {
  int place = placeValueForDigit(selectedDigit);
  int currentDigit = getDigitAt(value, selectedDigit);
  return value + (newDigit - currentDigit) * place;
}

void handleDigitEditorInput(int encoderDelta, ButtonEvent buttonEvent) {
  if (encoderDelta != 0) {
    if (!gDigitEditor.digitUnlocked) {
      int nextDigit = (int)gDigitEditor.selectedDigit + encoderDelta;
      while (nextDigit < 0) nextDigit += 4;
      while (nextDigit > 3) nextDigit -= 4;
      gDigitEditor.selectedDigit = (uint8_t)nextDigit;
    } else {
      int digit = getDigitAt(gDigitEditor.tempValue, gDigitEditor.selectedDigit);
      digit += (encoderDelta > 0) ? 1 : -1;
      if (digit < 0) digit = 9;
      if (digit > 9) digit = 0;
      gDigitEditor.tempValue = setDigitAt(gDigitEditor.tempValue, gDigitEditor.selectedDigit, digit);
      gDigitEditor.tempValue = clampInt(gDigitEditor.tempValue,
                                        gDigitEditor.minValue,
                                        gDigitEditor.maxValue);
    }
  }

  if (buttonEvent == ButtonEvent::ShortPress) {
    gDigitEditor.digitUnlocked = !gDigitEditor.digitUnlocked;
  } else if (buttonEvent == ButtonEvent::LongPress) {
    closeDigitEditor(true);
  }
}

// Sensors and perception
bool isIrTriggered(int value) {
  return value == HIGH;
}

uint8_t buildIrMaskFromPins() {
  uint8_t mask = 0;
  if (isIrTriggered(gIrFL)) mask |= 0x01; // SW / FL
  if (isIrTriggered(gIrFR)) mask |= 0x02; // SE / FR
  if (isIrTriggered(gIrRL)) mask |= 0x04; // NW / RL
  if (isIrTriggered(gIrRR)) mask |= 0x08; // NE / RR
  return mask;
}

void readIRSensors() {
  gIrFL = digitalRead(PIN_IR_FL);
  gIrFR = digitalRead(PIN_IR_FR);
  gIrRL = digitalRead(PIN_IR_RL);
  gIrRR = digitalRead(PIN_IR_RR);
  gRawIrMask = buildIrMaskFromPins();
}

bool ultrasonicFresh() {
  return gUltrasonicValid && (millis() - gLastUltrasonicMs) <= ULTRASONIC_STALE_MS;
}

bool parseDistancesLine(const char* line, float& d1, float& d2, float& d3, float& d4) {
  if (line == nullptr) return false;

  float v1 = 0.0f;
  float v2 = 0.0f;
  float v3 = 0.0f;
  float v4 = 0.0f;

  int matched = sscanf(line, "D1:%f,D2:%f,D3:%f,D4:%f", &v1, &v2, &v3, &v4);
  if (matched != 4) return false;

  if (v1 < 0.0f || v1 > SENSOR_MAX_VALID_CM) return false;
  if (v2 < 0.0f || v2 > SENSOR_MAX_VALID_CM) return false;
  if (v3 < 0.0f || v3 > SENSOR_MAX_VALID_CM) return false;
  if (v4 < 0.0f || v4 > SENSOR_MAX_VALID_CM) return false;

  d1 = v1;
  d2 = v2;
  d3 = v3;
  d4 = v4;
  return true;
}

void pushUltrasonicAverages(float north, float east, float south, float west) {
  gBufN[gBufIndex] = north;
  gBufE[gBufIndex] = east;
  gBufS[gBufIndex] = south;
  gBufW[gBufIndex] = west;

  gBufIndex = (uint8_t)((gBufIndex + 1) % ULTRASONIC_AVG_COUNT);
  if (gBufCount < ULTRASONIC_AVG_COUNT) {
    gBufCount++;
  }

  float sumN = 0.0f;
  float sumE = 0.0f;
  float sumS = 0.0f;
  float sumW = 0.0f;
  for (uint8_t i = 0; i < gBufCount; ++i) {
    sumN += gBufN[i];
    sumE += gBufE[i];
    sumS += gBufS[i];
    sumW += gBufW[i];
  }

  if (gBufCount > 0) {
    float scale = 1.0f / gBufCount;
    gAvgN = sumN * scale;
    gAvgE = sumE * scale;
    gAvgS = sumS * scale;
    gAvgW = sumW * scale;
  }
}

static void serviceUartSensorStream() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      gUartLine[gUartPos] = '\0';

      float d1 = 0.0f;
      float d2 = 0.0f;
      float d3 = 0.0f;
      float d4 = 0.0f;
      if (parseDistancesLine(gUartLine, d1, d2, d3, d4)) {
        // UART sender uses D1=S, D2=W, D3=N, D4=E.
        gDistS = d1;
        gDistW = d2;
        gDistN = d3;
        gDistE = d4;
        pushUltrasonicAverages(gDistN, gDistE, gDistS, gDistW);
        gUltrasonicValid = true;
        gLastUltrasonicMs = millis();
      }
      gUartPos = 0;
      continue;
    }

    if (gUartPos < sizeof(gUartLine) - 1) {
      gUartLine[gUartPos++] = c;
    } else {
      gUartPos = 0;
    }
  }
}

static void updateStableIrMask() {
  uint32_t now = millis();
  bool interruptRaised = false;
  noInterrupts();
  if (gIrInterruptFlag) {
    interruptRaised = true;
    gIrInterruptFlag = false;
  }
  interrupts();

  if (interruptRaised || gRawIrMask != gIrCandidateMask) {
    gIrCandidateMask = gRawIrMask;
    gIrCandidateSinceMs = now;
  }

  if (gStableIrMask != gIrCandidateMask) {
    uint32_t stableDelayMs = secondsToMs(gCfg.irStableTimeS);
    if ((now - gIrCandidateSinceMs) >= stableDelayMs) {
      gStableIrMask = gIrCandidateMask;
    }
  }
}

void initImu() {
  // Hardware probing is runtime-only now. Menu toggle decides whether
  // the IMU is used, but we still try to initialize the onboard device once.
  gImu.available = IMU.begin();
  if (!gImu.available) {
    Serial.println("IMU init failed. LSM6DSOX yaw hold disabled.");
    return;
  }
  Serial.println("IMU ready. LSM6DSOX runtime control enabled.");
}

static bool sampleImuBlocking(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint32_t startMs = millis();
  while ((millis() - startMs) < 250) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      return true;
    }
    delay(2);
  }
  ax = ay = 0.0f;
  az = 1.0f;
  gx = gy = gz = 0.0f;
  return false;
}

void updateImuDerivedState() {
  yawAxisForAngle(gImu.yawDeg, gImu.currentXAxisX, gImu.currentXAxisY);

  if (!gImu.referenceValid) {
    gImu.yawErrorDeg = 0.0f;
    gImu.measurementAngleTolDeg = 0.0f;
    gImu.combinedDeadbandDeg = 0.0f;
    return;
  }

  gImu.yawErrorDeg = signedAngleBetweenDeg(gImu.reference.xAxisX, gImu.reference.xAxisY,
                                           gImu.currentXAxisX, gImu.currentXAxisY);
  gImu.measurementAngleTolDeg = imuMeasurementAngleToleranceDeg(gImu.yawErrorDeg, gImu.sampleDtS);
  gImu.combinedDeadbandDeg =
    sqrtf(gImu.reference.angleTolDeg * gImu.reference.angleTolDeg +
          gImu.measurementAngleTolDeg * gImu.measurementAngleTolDeg) +
    gCfg.imuAngleThresholdDeg;
}

void updateImu() {
  if (!imuHardwareReady()) return;

  // When IMU usage is switched off from the menu, we stop updating
  // the control state so no event handler or drive path can use it.
  if (!gCfg.imuEnabled) {
    gImu.yawRateDps = 0.0f;
    gImu.lastCompensationCmd = 0.0f;
    return;
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(gImu.ax, gImu.ay, gImu.az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gImu.gx, gImu.gy, gImu.gz);

    uint32_t now = millis();
    float dt = (gImu.lastSampleMs == 0) ? 0.01f : (now - gImu.lastSampleMs) / 1000.0f;
    dt = clampFloat(dt, 0.005f, 0.100f);
    gImu.lastSampleMs = now;
    gImu.sampleDtS = dt;

    float yawRateDps = (gImu.gz - gImu.gyroBiasZDps) * IMU_YAW_SIGN;
    if (fabsf(yawRateDps) <= imuGyroRateFloorDps()) {
      yawRateDps = 0.0f;
    }

    gImu.yawRateDps = yawRateDps;
    gImu.yawDeg = wrapAngleDeg(gImu.yawDeg + yawRateDps * dt);
  }

  updateImuDerivedState();
}

bool captureImuReference() {
  if (!imuUsageEnabled()) return false;

  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;
  float sumGz = 0.0f;
  uint16_t goodSamples = 0;
  uint32_t startMs = millis();

  for (uint16_t i = 0; i < IMU_REFERENCE_SAMPLES; ++i) {
    float ax, ay, az, gx, gy, gz;
    if (!sampleImuBlocking(ax, ay, az, gx, gy, gz)) {
      continue;
    }

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGz += gz;
    goodSamples++;
    delay(IMU_REFERENCE_SAMPLE_DELAY_MS);
  }

  if (goodSamples == 0) {
    gImu.reference.valid = false;
    gImu.referenceValid = false;
    return false;
  }

  float sampleScale = 1.0f / goodSamples;
  gImu.reference.avgAx = sumAx * sampleScale;
  gImu.reference.avgAy = sumAy * sampleScale;
  gImu.reference.avgAz = sumAz * sampleScale;
  gImu.gyroBiasZDps = sumGz * sampleScale;
  gImu.reference.yawDeg = gImu.yawDeg;
  yawAxisForAngle(gImu.reference.yawDeg, gImu.reference.xAxisX, gImu.reference.xAxisY);
  gImu.reference.yAxisX = -gImu.reference.xAxisY;
  gImu.reference.yAxisY = gImu.reference.xAxisX;
  gImu.reference.planarTolG = sqrtf(2.0f * (IMU_ACCEL_ZERO_G_OFFSET_G * IMU_ACCEL_ZERO_G_OFFSET_G +
                                            sq(IMU_ACCEL_RMS_NOISE_G / sqrtf((float)goodSamples))));

  float captureDurationS = (millis() - startMs) / 1000.0f;
  float gyroTolDeg = imuReferenceAngleToleranceDeg(captureDurationS);
  float planarTolDeg = imuPlanarAngleToleranceDeg(goodSamples, gImu.reference.avgAz);
  gImu.reference.angleTolDeg = sqrtf(gyroTolDeg * gyroTolDeg + planarTolDeg * planarTolDeg);
  gImu.reference.capturedMs = millis();
  gImu.reference.valid = true;
  gImu.referenceValid = true;

  resetImuPid();
  updateImuDerivedState();

  Serial.print("IMU reference saved. biasZ[dps]=");
  Serial.print(gImu.gyroBiasZDps, 3);
  Serial.print(" deadbandBase[deg]=");
  Serial.println(gImu.reference.angleTolDeg, 3);
  return true;
}

void refreshSensors() {
  readIRSensors();
  serviceUartSensorStream();
  updateStableIrMask();
  updateImu();
}

RelativeIrState relativeIrForHeading(uint8_t mask, Heading heading) {
  // World IR order: [NW, NE, SE, SW]
  bool worldIr[4] = {
    (mask & 0x04) != 0,
    (mask & 0x08) != 0,
    (mask & 0x02) != 0,
    (mask & 0x01) != 0
  };

  // Relative order when heading North:
  // [front-left, front-right, rear-right, rear-left] = [NW, NE, SE, SW]
  int headingIndex = headingToIndex(heading);
  RelativeIrState relative;
  relative.frontLeft = worldIr[wrap4(0 + headingIndex)];
  relative.frontRight = worldIr[wrap4(1 + headingIndex)];
  relative.rearRight = worldIr[wrap4(2 + headingIndex)];
  relative.rearLeft = worldIr[wrap4(3 + headingIndex)];
  return relative;
}

static bool frontPairBlocked(const RelativeIrState& ir) {
  return ir.frontLeft && ir.frontRight;
}

static bool frontPairOpen(const RelativeIrState& ir) {
  return !ir.frontLeft && !ir.frontRight;
}

static bool rearPairOpen(const RelativeIrState& ir) {
  return !ir.rearLeft && !ir.rearRight;
}

PidSource choosePidSource(const PerceptionFrame& frame) {
  if (!frame.ultrasonicValid) return PidSource::None;

  bool leftWall = frame.left <= gCfg.corridorWallThresholdCm;
  bool rightWall = frame.right <= gCfg.corridorWallThresholdCm;

  if (leftWall && rightWall) return PidSource::TwoWall;
  if (leftWall) return PidSource::LeftWall;
  if (rightWall) return PidSource::RightWall;
  return PidSource::None;
}

bool corridorSignatureForFrame(const PerceptionFrame& frame) {
  return frame.ultrasonicValid &&
         frame.front > gCfg.deadEndDistanceCm &&
         frame.ir.frontLeft &&
         frame.ir.frontRight;
}

EventType classifyCandidateEvent(const PerceptionFrame& frame) {
  if (!frame.ultrasonicValid) {
    return EventType::SensorWait;
  }

  bool frontRightBlocked = frame.ir.frontRight;
  bool frontLeftBlocked = frame.ir.frontLeft;
  bool rearOpen = rearPairOpen(frame.ir);
  bool leftOpenByUs = frame.left >= gCfg.corridorWallThresholdCm;
  bool rightOpenByUs = frame.right >= gCfg.corridorWallThresholdCm;

  if (frame.front <= gCfg.frontStopDistanceCm) {
    return EventType::SafetyStop;
  }

  if (frame.front <= gCfg.deadEndDistanceCm) {
    if (frontRightBlocked) {
      if (frontLeftBlocked) return EventType::DeadEnd;
      return EventType::LeftTurn;
    }
    if (frontLeftBlocked) return EventType::RightTurn;
    return EventType::TJunction;
  }

  if (frame.front <= gCfg.turnDetectDistanceCm) {
    if (frontRightBlocked) {
      if (frontLeftBlocked) return EventType::Corridor;
      return EventType::LeftTurn;
    }
    if (frontLeftBlocked) return EventType::RightTurn;
    return EventType::TJunction;
  }

  if (frontRightBlocked && frontLeftBlocked) {
    return EventType::Corridor;
  }

  if (!frontRightBlocked && !frontLeftBlocked) {
    if (rearOpen &&
        leftOpenByUs &&
        rightOpenByUs &&
        frame.front >= gCfg.finishDistanceCm) {
      return EventType::Finish;
    }
    return EventType::TJunctionStraight;
  }

  return EventType::TJunctionStraight;
}

PerceptionFrame buildPerceptionFrame(Heading heading) {
  PerceptionFrame frame;
  frame.ultrasonicValid = ultrasonicFresh();
  frame.stableIrMask = gStableIrMask;
  frame.ir = relativeIrForHeading(gStableIrMask, heading);

  frame.north = (gBufCount > 0) ? gAvgN : gDistN;
  frame.east = (gBufCount > 0) ? gAvgE : gDistE;
  frame.south = (gBufCount > 0) ? gAvgS : gDistS;
  frame.west = (gBufCount > 0) ? gAvgW : gDistW;

  frame.front = distanceForHeading(heading);
  frame.left = distanceForHeading(turnLeftOf(heading));
  frame.right = distanceForHeading(turnRightOf(heading));
  frame.back = distanceForHeading(oppositeOf(heading));

  frame.candidate = classifyCandidateEvent(frame);
  frame.corridorSignature = corridorSignatureForFrame(frame);
  frame.pidSource = choosePidSource(frame);
  return frame;
}

void resetEventConfirmation() {
  gExecutive.eventConfirm.active = false;
  gExecutive.eventConfirm.candidate = EventType::Idle;
  gExecutive.eventConfirm.matches = 0;
  gExecutive.eventConfirm.nextCheckMs = 0;
}

void resetCorridorConfirmation() {
  gExecutive.corridorConfirm.active = false;
  gExecutive.corridorConfirm.matches = 0;
  gExecutive.corridorConfirm.nextCheckMs = 0;
}

bool updateEventConfirmation(const PerceptionFrame& frame, EventType& confirmedEvent) {
  confirmedEvent = EventType::Idle;

  EventType candidate = frame.candidate;
  if (candidate == EventType::Corridor ||
      candidate == EventType::Idle ||
      candidate == EventType::Start ||
      candidate == EventType::SensorWait) {
    resetEventConfirmation();
    return false;
  }

  uint32_t now = millis();
  EventConfirmState& state = gExecutive.eventConfirm;

  if (!state.active || state.candidate != candidate) {
    state.active = true;
    state.candidate = candidate;
    state.matches = 1;
    state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);
    return false;
  }

  if (now < state.nextCheckMs) {
    return false;
  }

  state.matches++;
  state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);

  if (state.matches >= gCfg.eventConfirmCount) {
    confirmedEvent = candidate;
    resetEventConfirmation();
    return true;
  }

  return false;
}

bool updateCorridorConfirmation(const PerceptionFrame& frame) {
  if (!frame.corridorSignature) {
    resetCorridorConfirmation();
    return false;
  }

  uint32_t now = millis();
  CorridorConfirmState& state = gExecutive.corridorConfirm;

  if (!state.active) {
    state.active = true;
    state.matches = 1;
    state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);
    return false;
  }

  if (now < state.nextCheckMs) {
    return false;
  }

  state.matches++;
  state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);

  if (state.matches >= gCfg.eventConfirmCount) {
    resetCorridorConfirmation();
    return true;
  }

  return false;
}

// Planner
RouteMode currentRouteMode() {
  return (RouteMode)activeSettings.routeMode;
}

RouteMode effectiveRouteMode() {
  RouteMode mode = currentRouteMode();
  if (mode != RouteMode::Sequence) {
    return mode;
  }

  switch (gPlanner.sequenceSlot % 3) {
    case 0: return RouteMode::Right;
    case 1: return RouteMode::Left;
    default: return RouteMode::Selection;
  }
}

Heading chooseHeadingForEvent(const PerceptionFrame& frame, EventType eventType, Heading currentHeading) {
  switch (eventType) {
    case EventType::LeftTurn:
      return turnLeftOf(currentHeading);
    case EventType::RightTurn:
      return turnRightOf(currentHeading);
    case EventType::DeadEnd:
      return oppositeOf(currentHeading);
    case EventType::TJunctionStraight:
      return currentHeading;
    case EventType::TJunction: {
      RouteMode mode = effectiveRouteMode();
      if (mode == RouteMode::Right) {
        return turnRightOf(currentHeading);
      }
      if (mode == RouteMode::Left) {
        return turnLeftOf(currentHeading);
      }
      if (frame.left > frame.right) {
        return turnLeftOf(currentHeading);
      }
      return turnRightOf(currentHeading);
    }
    default:
      return currentHeading;
  }
}

// Motion control
float shapedMagnitude(float magnitude) {
  float absMagnitude = fabsf(magnitude);
  if (absMagnitude < PID_INTERPRET_LIMIT_CM) {
    float ratio = absMagnitude / PID_INTERPRET_LIMIT_CM;
    return PID_INTERPRET_LIMIT_CM * powf(ratio, gCfg.curveB);
  }
  return absMagnitude;
}

float shapeSignedError(float error) {
  if (error == 0.0f) return 0.0f;
  float sign = (error >= 0.0f) ? 1.0f : -1.0f;
  return sign * shapedMagnitude(error);
}

float computeWallError(const PerceptionFrame& frame, PidSource source) {
  switch (source) {
    case PidSource::TwoWall:
      return frame.right - frame.left;
    case PidSource::LeftWall:
      return gCfg.pidWallDistanceCm - frame.left;
    case PidSource::RightWall:
      return frame.right - gCfg.pidWallDistanceCm;
    case PidSource::None:
    default:
      return 0.0f;
  }
}

float computePid(float error) {
  uint32_t now = millis();
  float dt = (gPid.previousMs == 0) ? 0.01f : (now - gPid.previousMs) / 1000.0f;
  dt = clampFloat(dt, 0.01f, 0.20f);

  float shapedError = shapeSignedError(error);
  float derivative = (shapedError - gPid.previousError) / dt;

  gPid.integral += shapedError * dt;
  gPid.integral = clampFloat(gPid.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  float output = gCfg.p * shapedError +
                 gCfg.i * gPid.integral +
                 gCfg.d * derivative;

  gPid.previousError = shapedError;
  gPid.previousMs = now;

  if (output < 0.0f) {
    output *= gCfg.pidLeftScale;
  } else if (output > 0.0f) {
    output *= gCfg.pidRightScale;
  }

  return clampFloat(output, -1.0f, 1.0f);
}

float applyImuCompensation(ImuPidRuntime& runtime) {
  updateImuDerivedState();

  if (!imuUsageEnabled() || !gImu.referenceValid) {
    runtime.previousError = 0.0f;
    runtime.integral = 0.0f;
    runtime.previousMs = millis();
    gImu.lastCompensationCmd = 0.0f;
    return 0.0f;
  }

  float errorDeg = gImu.yawErrorDeg;
  float deadbandDeg = gImu.combinedDeadbandDeg;

  uint32_t now = millis();
  float dt = (runtime.previousMs == 0) ? 0.01f : (now - runtime.previousMs) / 1000.0f;
  dt = clampFloat(dt, 0.01f, 0.20f);
  runtime.previousMs = now;

  if (fabsf(errorDeg) <= deadbandDeg) {
    runtime.previousError = 0.0f;
    runtime.integral *= 0.85f;
    gImu.lastCompensationCmd = 0.0f;
    return 0.0f;
  }

  float effectiveErrorDeg = (errorDeg > 0.0f)
                              ? (errorDeg - deadbandDeg)
                              : (errorDeg + deadbandDeg);
  float derivative = (effectiveErrorDeg - runtime.previousError) / dt;

  runtime.integral += effectiveErrorDeg * dt;
  runtime.integral = clampFloat(runtime.integral, -IMU_PID_INTEGRAL_LIMIT, IMU_PID_INTEGRAL_LIMIT);

  float output = gCfg.imuP * effectiveErrorDeg +
                 gCfg.imuI * runtime.integral +
                 gCfg.imuD * derivative;

  runtime.previousError = effectiveErrorDeg;
  output = clampFloat(output, -1.0f, 1.0f);
  gImu.lastCompensationCmd = output;
  return output;
}

void writeServoCommand(Servo& servo, int stopAngle, int rangeAngle, int invert, float cmd) {
  cmd = clampUnit(cmd) * (float)invert;

  int usStop = 1000 + (int)lroundf((stopAngle / 180.0f) * 1000.0f);
  int usRange = (int)lroundf((rangeAngle / 180.0f) * 1000.0f);
  int usValue = usStop + (int)lroundf(cmd * usRange);
  usValue = constrain(usValue, 1000, 2000);
  servo.writeMicroseconds(usValue);
}

void stopAll() {
  writeServoCommand(servoFL, STOP_FL, RANGE_FL, INV_FL, 0.0f);
  writeServoCommand(servoFR, STOP_FR, RANGE_FR, INV_FR, 0.0f);
  writeServoCommand(servoRR, STOP_RR, RANGE_RR, INV_RR, 0.0f);
  writeServoCommand(servoRL, STOP_RL, RANGE_RL, INV_RL, 0.0f);
}

/*
  45 degree omni X-drive layout
  -----------------------------
  FL and RR share the +45 degree diagonal, FR and RL share the -45 degree diagonal.

    u+ = vy + vx   -> FL, RR translation term
    u- = vy - vx   -> FR, RL translation term

  Rotational compensation is injected as an orthogonal spin pattern:

    spin = [+w, -w, -w, +w]

  Instead of renormalizing all wheels afterward, w is clamped to the wheel headroom that
  remains after translation. That preserves the exact left/right translation pattern
  [+vx, -vx, +vx, -vx] mathematically whenever the translational command itself is valid.
*/
static void driveBodyFrame(float vx, float vy, float w) {
  float basePlus = vy + vx;
  float baseMinus = vy - vx;

  float minW = -1000.0f;
  float maxW = 1000.0f;

  auto limitPlus = [&](float base, float scale) {
    float wheelLimit = 1.0f / max(scale, 0.01f);
    minW = max(minW, -wheelLimit - base);
    maxW = min(maxW, wheelLimit - base);
  };
  auto limitMinus = [&](float base, float scale) {
    float wheelLimit = 1.0f / max(scale, 0.01f);
    minW = max(minW, base - wheelLimit);
    maxW = min(maxW, base + wheelLimit);
  };

  limitPlus(basePlus, gCfg.leftDriveScale);   // FL:  basePlus + w
  limitMinus(basePlus, gCfg.leftDriveScale);  // RR:  basePlus - w
  limitMinus(baseMinus, gCfg.rightDriveScale);// FR:  baseMinus - w
  limitPlus(baseMinus, gCfg.rightDriveScale); // RL:  baseMinus + w

  if (minW > maxW) {
    w = 0.0f;
  } else {
    w = clampFloat(w, minW, maxW);
  }

  float fl = (basePlus + w) * gCfg.leftDriveScale;
  float rr = (basePlus - w) * gCfg.leftDriveScale;
  float fr = (baseMinus - w) * gCfg.rightDriveScale;
  float rl = (baseMinus + w) * gCfg.rightDriveScale;

  writeServoCommand(servoFL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeServoCommand(servoFR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeServoCommand(servoRR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeServoCommand(servoRL, STOP_RL, RANGE_RL, INV_RL, rl);
}

void driveRobotFrame(float lateralCmd, float forwardCmd, float rotationCmd, Heading heading) {
  float vx = 0.0f;
  float vy = 0.0f;

  switch (heading) {
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

  driveBodyFrame(vx, vy, rotationCmd);
}

/*
  Runtime usage architecture
  --------------------------
  1. refreshSensors() updates IR, UART ultrasonics, and the onboard LSM6DSOX yaw state.
  2. beginExecution() / startEventTestRun() capture the current body frame as the IMU reference.
  3. followSmart() computes:
       lateral correction  <- wall PID
       rotation correction <- IMU yaw-hold PID with tolerance-aware deadband
  4. driveRobotFrame() maps maze-relative motion into body-frame vx/vy while keeping the yaw
     compensation as a separate rotational term for the 45 degree omni X-drive.
*/
void followSmart(Heading heading, float forwardSpeed) {
  PerceptionFrame frame = buildPerceptionFrame(heading);
  if (!frame.ultrasonicValid) {
    stopAll();
    return;
  }

  float correction = 0.0f;
  if (frame.pidSource != PidSource::None) {
    correction = computePid(computeWallError(frame, frame.pidSource));
  }

  float rotationComp = applyImuCompensation(gImuPid);
  driveRobotFrame(correction, forwardSpeed, rotationComp, heading);
}

// Executive
static void resetPlannerCycleIfNeeded() {
  if (currentRouteMode() == RouteMode::Sequence && gPlanner.sequenceCycleComplete) {
    gPlanner.sequenceSlot = 0;
    gPlanner.sequenceCycleComplete = false;
    gPlanner.totalDurationMs = 0;
  }
}

void beginExecution() {
  resetPlannerCycleIfNeeded();
  gPlanner.attemptCounter++;

  if (imuUsageEnabled()) {
    (void)captureImuReference();
  } else {
    clearImuReference();
  }

  gExecutive.running = true;
  gExecutive.state = NavState::StartSeek;
  gExecutive.heading = headingFromIndex(activeSettings.startHeadingIndex);
  gExecutive.pendingHeading = gExecutive.heading;
  gExecutive.displayEvent = EventType::Start;
  gExecutive.latchedEvent = EventType::Start;
  gExecutive.runStartedMs = millis();
  gExecutive.phaseStartedMs = gExecutive.runStartedMs;

  resetEventConfirmation();
  resetCorridorConfirmation();
  resetPid();
  resetImuPid();

  gCurrentScreen = Screen::RunScreen;
}

void finishExecution() {
  stopAll();

  uint32_t durationMs = millis() - gExecutive.runStartedMs;
  gPlanner.lastAttemptDurationMs = durationMs;

  if (currentRouteMode() == RouteMode::Sequence) {
    gPlanner.totalDurationMs += durationMs;
    if (gPlanner.sequenceSlot >= 2) {
      gPlanner.sequenceCycleComplete = true;
      gPlanner.sequenceSlot = 0;
    } else {
      gPlanner.sequenceSlot++;
    }
  }

  gExecutive.running = false;
  gExecutive.state = NavState::Finished;
  gExecutive.displayEvent = EventType::Finish;
  gExecutive.latchedEvent = EventType::Finish;

  gCurrentScreen = Screen::StartConfirm;
}

static void enterPhase(NavState state, EventType displayEvent) {
  gExecutive.state = state;
  gExecutive.displayEvent = displayEvent;
  gExecutive.phaseStartedMs = millis();
  resetCorridorConfirmation();
  if (state == NavState::Corridor || state == NavState::StartSeek) {
    resetEventConfirmation();
  }
}

void updateExecutive() {
  if (!gExecutive.running) return;

  PerceptionFrame frame = buildPerceptionFrame(gExecutive.heading);
  gExecutive.displayEvent = frame.candidate;

  switch (gExecutive.state) {
    case NavState::StartSeek:
      gExecutive.displayEvent = EventType::Start;
      if (frame.candidate == EventType::SafetyStop) {
        enterPhase(NavState::SafetyStop, EventType::SafetyStop);
        stopAll();
        return;
      }
      if (updateCorridorConfirmation(frame)) {
        enterPhase(NavState::Corridor, EventType::Corridor);
        return;
      }
      followSmart(gExecutive.heading, gCfg.approachSpeed);
      return;

    case NavState::Corridor: {
      if (frame.candidate == EventType::SensorWait) {
        stopAll();
        gExecutive.displayEvent = EventType::SensorWait;
        return;
      }

      if (frame.candidate == EventType::SafetyStop) {
        enterPhase(NavState::SafetyStop, EventType::SafetyStop);
        stopAll();
        return;
      }

      EventType confirmed = EventType::Idle;
      if (updateEventConfirmation(frame, confirmed)) {
        gExecutive.latchedEvent = confirmed;

        if (confirmed == EventType::Finish) {
          finishExecution();
          return;
        }

        if (confirmed == EventType::DeadEnd) {
          gExecutive.heading = oppositeOf(gExecutive.heading);
          resetPid();
          enterPhase(NavState::AcquireCorridor, EventType::DeadEnd);
          return;
        }

        if (confirmed == EventType::TJunctionStraight) {
          resetPid();
          enterPhase(NavState::AcquireCorridor, EventType::TJunctionStraight);
          return;
        }

        if (confirmed == EventType::LeftTurn ||
            confirmed == EventType::RightTurn ||
            confirmed == EventType::TJunction) {
          gExecutive.pendingHeading = chooseHeadingForEvent(frame, confirmed, gExecutive.heading);
          enterPhase(NavState::ApproachWall, confirmed);
          return;
        }
      }

      gExecutive.displayEvent = EventType::Corridor;
      followSmart(gExecutive.heading, gCfg.baseSpeed);
      return;
    }

    case NavState::ApproachWall:
      gExecutive.displayEvent = gExecutive.latchedEvent;
      if (!frame.ultrasonicValid) {
        stopAll();
        return;
      }
      if (frame.front <= gCfg.frontStopDistanceCm) {
        enterPhase(NavState::WaitTurn, gExecutive.latchedEvent);
        stopAll();
        return;
      }
      followSmart(gExecutive.heading, gCfg.approachSpeed);
      return;

    case NavState::WaitTurn:
      gExecutive.displayEvent = gExecutive.latchedEvent;
      stopAll();
      if ((millis() - gExecutive.phaseStartedMs) >= secondsToMs(gCfg.waitBeforeTurnS)) {
        gExecutive.heading = gExecutive.pendingHeading;
        resetPid();
        enterPhase(NavState::AcquireCorridor, gExecutive.latchedEvent);
      }
      return;

    case NavState::AcquireCorridor:
      gExecutive.displayEvent = gExecutive.latchedEvent;
      if (frame.candidate == EventType::SafetyStop) {
        enterPhase(NavState::SafetyStop, EventType::SafetyStop);
        stopAll();
        return;
      }
      if (updateCorridorConfirmation(frame)) {
        resetPid();
        enterPhase(NavState::Corridor, EventType::Corridor);
        return;
      }
      followSmart(gExecutive.heading, gCfg.approachSpeed);
      return;

    case NavState::SafetyStop:
      gExecutive.displayEvent = EventType::SafetyStop;
      stopAll();
      if (updateCorridorConfirmation(frame)) {
        resetPid();
        enterPhase(NavState::Corridor, EventType::Corridor);
      }
      return;

    case NavState::Finished:
    case NavState::Idle:
    default:
      stopAll();
      return;
  }
}

// UI and drawing
static bool updateEventConfirmState(EventConfirmState& state, EventType candidate, EventType& confirmedEvent) {
  confirmedEvent = EventType::Idle;

  if (candidate == EventType::Idle ||
      candidate == EventType::Start ||
      candidate == EventType::SensorWait ||
      candidate == EventType::Corridor) {
    state.active = false;
    state.candidate = EventType::Idle;
    state.matches = 0;
    state.nextCheckMs = 0;
    return false;
  }

  uint32_t now = millis();
  if (!state.active || state.candidate != candidate) {
    state.active = true;
    state.candidate = candidate;
    state.matches = 1;
    state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);
    return false;
  }

  if (now < state.nextCheckMs) {
    return false;
  }

  state.matches++;
  state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);
  if (state.matches >= gCfg.eventConfirmCount) {
    confirmedEvent = candidate;
    state.active = false;
    state.candidate = EventType::Idle;
    state.matches = 0;
    state.nextCheckMs = 0;
    return true;
  }

  return false;
}

static bool updateCorridorConfirmState(CorridorConfirmState& state, bool corridorSignature) {
  if (!corridorSignature) {
    state.active = false;
    state.matches = 0;
    state.nextCheckMs = 0;
    return false;
  }

  uint32_t now = millis();
  if (!state.active) {
    state.active = true;
    state.matches = 1;
    state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);
    return false;
  }

  if (now < state.nextCheckMs) {
    return false;
  }

  state.matches++;
  state.nextCheckMs = now + secondsToMs(gCfg.sensingIntervalS);
  if (state.matches >= gCfg.eventConfirmCount) {
    state.active = false;
    state.matches = 0;
    state.nextCheckMs = 0;
    return true;
  }
  return false;
}

static void runTuneMotion() {
  switch (gTuneMotion) {
    case TuneMotion::Stop:
      stopAll();
      return;
    default:
      break;
  }

  float rotationComp = applyImuCompensation(gImuPid);
  switch (gTuneMotion) {
    case TuneMotion::North: driveRobotFrame(0.0f, gCfg.baseSpeed, rotationComp, Heading::North); break;
    case TuneMotion::East:  driveRobotFrame(0.0f, gCfg.baseSpeed, rotationComp, Heading::East); break;
    case TuneMotion::South: driveRobotFrame(0.0f, gCfg.baseSpeed, rotationComp, Heading::South); break;
    case TuneMotion::West:  driveRobotFrame(0.0f, gCfg.baseSpeed, rotationComp, Heading::West); break;
    case TuneMotion::Stop:  stopAll(); break;
  }
}

static void startEventPhase(NavState state, EventType displayEvent) {
  gEventTestRun.state = state;
  gEventTestRun.displayEvent = displayEvent;
  gEventTestRun.phaseStartedMs = millis();
  gEventTestRun.eventConfirm.active = false;
  gEventTestRun.eventConfirm.candidate = EventType::Idle;
  gEventTestRun.eventConfirm.matches = 0;
  gEventTestRun.eventConfirm.nextCheckMs = 0;
  gEventTestRun.corridorConfirm.active = false;
  gEventTestRun.corridorConfirm.matches = 0;
  gEventTestRun.corridorConfirm.nextCheckMs = 0;
}

void startEventTestRun() {
  if (imuUsageEnabled()) {
    (void)captureImuReference();
  } else {
    clearImuReference();
  }

  gEventTestRun.active = true;
  gEventTestRun.heading = currentEventTestHeading();
  gEventTestRun.pendingHeading = gEventTestRun.heading;
  gEventTestRun.displayEvent = (gSelectedEventTest == EventTestCase::Start) ? EventType::Start : EventType::Corridor;
  gEventTestRun.latchedEvent = gEventTestRun.displayEvent;
  gEventTestRun.startedMs = millis();
  gEventTestRun.phaseStartedMs = gEventTestRun.startedMs;
  gEventTestRun.state = (gSelectedEventTest == EventTestCase::Start) ? NavState::StartSeek : NavState::Corridor;
  gEventTestRun.eventConfirm.active = false;
  gEventTestRun.corridorConfirm.active = false;
  resetPid();
  resetImuPid();
}

void stopEventTestRun() {
  gEventTestRun.active = false;
  gEventTestRun.state = NavState::Idle;
  stopAll();
}

void handleRootInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 9;
  if (encoderDelta != 0) {
    gRootIndex += encoderDelta;
    if (gRootIndex < 0) gRootIndex = 0;
    if (gRootIndex >= itemCount) gRootIndex = itemCount - 1;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gRootIndex) {
    case 0: beginExecution(); break;
    case 1: gCurrentScreen = Screen::StartSettings; break;
    case 2: gCurrentScreen = Screen::Settings; break;
    case 3: gCurrentScreen = Screen::EventTestMenu; break;
    case 4: gCurrentScreen = Screen::IMUTest; break;
    case 5: gCurrentScreen = Screen::IMUYawTest; break;
    case 6: gPreviewHeading = headingFromIndex(activeSettings.startHeadingIndex); gCurrentScreen = Screen::MazeTestRun; break;
    case 7: gCurrentScreen = Screen::IRSensorTest; break;
    case 8: gCurrentScreen = Screen::UltrasonicSensorTest; break;
    default: break;
  }
}

void handleStartSettingsInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 4;

  if (encoderDelta != 0) {
    gStartSettingsIndex += encoderDelta;
    if (gStartSettingsIndex < 0) gStartSettingsIndex = 0;
    if (gStartSettingsIndex >= itemCount) gStartSettingsIndex = itemCount - 1;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gStartSettingsIndex) {
    case 0:
      workingSettings.routeMode++;
      if (workingSettings.routeMode > (uint8_t)RouteMode::Selection) {
        workingSettings.routeMode = (uint8_t)RouteMode::Sequence;
      }
      commitRuntimeConfig();
      break;
    case 1:
      workingSettings.startHeadingIndex = wrap4(workingSettings.startHeadingIndex + 1);
      commitRuntimeConfig();
      break;
    case 2:
      openDigitEditor("Turn wait", "s", &workingSettings.waitBeforeTurn_x100, 0, 3000,
                      "Stop duration before", "changing heading", Screen::StartSettings);
      break;
    case 3:
      gCurrentScreen = Screen::Root;
      break;
    default:
      break;
  }
}

void handleSettingsInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 12;

  if (encoderDelta != 0) {
    gSettingsIndex += encoderDelta;
    if (gSettingsIndex < 0) gSettingsIndex = 0;
    if (gSettingsIndex >= itemCount) gSettingsIndex = itemCount - 1;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gSettingsIndex) {
    case 0: gCurrentScreen = Screen::PID; break;
    case 1: gCurrentScreen = Screen::MotorTune; break;
    case 2:
      workingSettings.imuEnabled = workingSettings.imuEnabled ? 0 : 1;
      commitRuntimeConfig();
      break;
    case 3: openDigitEditor("Front stop", "cm", &workingSettings.frontStopDistance_x100, 0, 9999, "Emergency stop if", "front gets too close", Screen::Settings); break;
    case 4: openDigitEditor("Wall thr", "cm", &workingSettings.corridorWallThreshold_x100, 0, 9999, "Wall limit for PID", "and finish detection", Screen::Settings); break;
    case 5: openDigitEditor("Turn detect", "cm", &workingSettings.turnDetectDistance_x100, 0, 9999, "Upper front range", "that still means turn", Screen::Settings); break;
    case 6: openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999, "Front distance limit", "for dead-end event", Screen::Settings); break;
    case 7: openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999, "Front distance needed", "before finish counts", Screen::Settings); break;
    case 8: openDigitEditor("Check gap", "s", &workingSettings.sensingInterval_x100, 1, 9999, "Time between event", "verification checks", Screen::Settings); break;
    case 9: openDigitEditor("Match N", "x", &workingSettings.eventConfirmCount, 1, 9, "How many identical", "events confirm state", Screen::Settings); break;
    case 10: openDigitEditor("IR stable", "s", &workingSettings.irStableTime_x100, 1, 9999, "How long IR mask", "must stay unchanged", Screen::Settings); break;
    case 11: gCurrentScreen = Screen::Root; break;
    default: break;
  }
}

void handlePidInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 14;
  if (encoderDelta != 0) {
    gPidIndex += encoderDelta;
    if (gPidIndex < 0) gPidIndex = 0;
    if (gPidIndex >= itemCount) gPidIndex = itemCount - 1;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Settings;
    return;
  }
  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gPidIndex) {
    case 0: openDigitEditor("P gain", "x", &workingSettings.p_x100, 0, 9999, "Proportional reaction", "to wall error", Screen::PID); break;
    case 1: openDigitEditor("I gain", "x", &workingSettings.i_x100, 0, 9999, "Integral correction", "for constant drift", Screen::PID); break;
    case 2: openDigitEditor("D gain", "x", &workingSettings.d_x100, 0, 9999, "Derivative damping", "for fast changes", Screen::PID); break;
    case 3: openDigitEditor("Curve b", "x", &workingSettings.curveB_x100, 1, 9999, "Shapes small errors", "before PID acts", Screen::PID); break;
    case 4: openDigitEditor("1-wall", "cm", &workingSettings.pidWallDistance_x100, 0, 9999, "Target distance when", "only one wall exists", Screen::PID); break;
    case 5: openDigitEditor("Left out", "x", &workingSettings.pidLeftScale_x100, 10, 300, "Extra scale for", "left correction", Screen::PID); break;
    case 6: openDigitEditor("Right out", "x", &workingSettings.pidRightScale_x100, 10, 300, "Extra scale for", "right correction", Screen::PID); break;
    case 7: gPidTestHeading = headingFromIndex(activeSettings.startHeadingIndex); gCurrentScreen = Screen::PIDTest; break;
    case 8: openDigitEditor("IMU P", "x", &workingSettings.imuP_x100, 0, 9999, "Yaw-hold proportional", "gain on angle error", Screen::PID); break;
    case 9: openDigitEditor("IMU I", "x", &workingSettings.imuI_x100, 0, 9999, "Yaw-hold integral", "gain for slow drift", Screen::PID); break;
    case 10: openDigitEditor("IMU D", "x", &workingSettings.imuD_x100, 0, 9999, "Yaw-hold derivative", "gain for twist damping", Screen::PID); break;
    case 11: openDigitEditor("IMU thr", "deg", &workingSettings.imuAngleThreshold_x100, 0, 4500, "Extra angle above", "sensor deadband", Screen::PID); break;
    case 12: resetImuPid(); gCurrentScreen = Screen::IMUPIDTest; break;
    case 13: gCurrentScreen = Screen::Settings; break;
    default: break;
  }
}

void handlePidTestInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::ShortPress) {
    gPidTestHeading = turnRightOf(gPidTestHeading);
  } else if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::PID;
  }
}

void handleImuPidTestInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::ShortPress) {
    (void)captureImuReference();
  } else if (buttonEvent == ButtonEvent::LongPress) {
    resetImuPid();
    gCurrentScreen = Screen::PID;
  }
}

void handleMotorTuneInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 6;
  if (encoderDelta != 0) {
    gMotorTuneIndex += encoderDelta;
    if (gMotorTuneIndex < 0) gMotorTuneIndex = 0;
    if (gMotorTuneIndex >= itemCount) gMotorTuneIndex = itemCount - 1;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gTuneMotion = TuneMotion::Stop;
    stopAll();
    gCurrentScreen = Screen::Settings;
    return;
  }

  if (buttonEvent == ButtonEvent::ShortPress) {
    gTuneMotion = (TuneMotion)gMotorTuneIndex;
  }
}

static bool eventConfigFieldRelevant(EventTestCase testCase, EventConfigField field) {
  switch (field) {
    case EventConfigField::Heading:
    case EventConfigField::WallThreshold:
    case EventConfigField::DeadEnd:
    case EventConfigField::Finish:
    case EventConfigField::CheckGap:
    case EventConfigField::MatchCount:
    case EventConfigField::IRStable:
    case EventConfigField::Run:
    case EventConfigField::Back:
      return true;
    case EventConfigField::TurnChoice:
      return testCase == EventTestCase::TJunction;
    case EventConfigField::FrontStop:
      return testCase == EventTestCase::LeftTurn ||
             testCase == EventTestCase::RightTurn ||
             testCase == EventTestCase::TJunction ||
             testCase == EventTestCase::TJunctionStraight ||
             testCase == EventTestCase::Random;
    case EventConfigField::TurnDetect:
      return testCase != EventTestCase::Start &&
             testCase != EventTestCase::Finish &&
             testCase != EventTestCase::DeadEnd;
    case EventConfigField::TurnWait:
      return testCase == EventTestCase::LeftTurn ||
             testCase == EventTestCase::RightTurn ||
             testCase == EventTestCase::TJunction ||
             testCase == EventTestCase::TJunctionStraight ||
             testCase == EventTestCase::Random;
    default:
      return false;
  }
}

static int currentEventConfigItemCount() {
  int count = 0;
  for (int i = 0; i <= (int)EventConfigField::Back; ++i) {
    if (eventConfigFieldRelevant(gSelectedEventTest, (EventConfigField)i)) count++;
  }
  return count;
}

static EventConfigField currentEventConfigFieldAt(int selectedIndex) {
  int count = 0;
  for (int i = 0; i <= (int)EventConfigField::Back; ++i) {
    EventConfigField field = (EventConfigField)i;
    if (!eventConfigFieldRelevant(gSelectedEventTest, field)) continue;
    if (count == selectedIndex) return field;
    count++;
  }
  return EventConfigField::Back;
}

static const char* eventConfigDesc1(EventConfigField field) {
  switch (field) {
    case EventConfigField::Heading: return "Robot heading for";
    case EventConfigField::TurnChoice: return "Branch used for";
    case EventConfigField::FrontStop: return "Front distance to";
    case EventConfigField::WallThreshold: return "Wall presence limit";
    case EventConfigField::TurnDetect: return "Upper front range";
    case EventConfigField::DeadEnd: return "Front dead-end limit";
    case EventConfigField::Finish: return "Front finish limit";
    case EventConfigField::TurnWait: return "Wait before new";
    case EventConfigField::CheckGap: return "Time between";
    case EventConfigField::MatchCount: return "How many identical";
    case EventConfigField::IRStable: return "How long IR must";
    case EventConfigField::Run: return "Start selected";
    case EventConfigField::Back:
    default: return "Return to event";
  }
}

static const char* eventConfigDesc2(EventConfigField field) {
  switch (field) {
    case EventConfigField::Heading: return "this test case.";
    case EventConfigField::TurnChoice: return "T-junction test.";
    case EventConfigField::FrontStop: return "enter stop stage.";
    case EventConfigField::WallThreshold: return "for PID selection.";
    case EventConfigField::TurnDetect: return "that still means turn.";
    case EventConfigField::DeadEnd: return "before dead-end triggers.";
    case EventConfigField::Finish: return "before finish triggers.";
    case EventConfigField::TurnWait: return "heading command.";
    case EventConfigField::CheckGap: return "event rechecks [s].";
    case EventConfigField::MatchCount: return "reads must match.";
    case EventConfigField::IRStable: return "stay unchanged [s].";
    case EventConfigField::Run: return "event execution.";
    case EventConfigField::Back:
    default: return "test list.";
  }
}

void handleEventTestMenuInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 10;
  if (encoderDelta != 0) {
    gEventTestIndex += encoderDelta;
    if (gEventTestIndex < 0) gEventTestIndex = 0;
    if (gEventTestIndex >= itemCount) gEventTestIndex = itemCount - 1;
  }
  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }
  if (buttonEvent != ButtonEvent::ShortPress) return;
  gSelectedEventTest = (EventTestCase)gEventTestIndex;
  if (gSelectedEventTest == EventTestCase::Back) {
    gCurrentScreen = Screen::Root;
  } else {
    gEventTestConfigIndex = 0;
    gCurrentScreen = Screen::EventTestConfig;
  }
}

void handleImuTestInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
  }
}

void handleImuYawTestInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::ShortPress) {
    if (imuUsageEnabled()) {
      gImu.yawDeg = 0.0f;
      (void)captureImuReference();
    }
  } else if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
  }
}

void handleEventTestConfigInput(int encoderDelta, ButtonEvent buttonEvent) {
  int itemCount = currentEventConfigItemCount();
  if (encoderDelta != 0) {
    gEventTestConfigIndex += encoderDelta;
    if (gEventTestConfigIndex < 0) gEventTestConfigIndex = 0;
    if (gEventTestConfigIndex >= itemCount) gEventTestConfigIndex = itemCount - 1;
  }
  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::EventTestMenu;
    return;
  }
  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (currentEventConfigFieldAt(gEventTestConfigIndex)) {
    case EventConfigField::Heading:
      currentEventTestConfig().headingIndex = wrap4(currentEventTestConfig().headingIndex + 1);
      break;
    case EventConfigField::TurnChoice:
      currentEventTestConfig().turnChoice = (currentEventTestConfig().turnChoice == TurnChoice::Left) ? TurnChoice::Right : TurnChoice::Left;
      break;
    case EventConfigField::FrontStop:
      openDigitEditor("Front stop", "cm", &workingSettings.frontStopDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::FrontStop), eventConfigDesc2(EventConfigField::FrontStop), Screen::EventTestConfig);
      break;
    case EventConfigField::WallThreshold:
      openDigitEditor("Wall thr", "cm", &workingSettings.corridorWallThreshold_x100, 0, 9999, eventConfigDesc1(EventConfigField::WallThreshold), eventConfigDesc2(EventConfigField::WallThreshold), Screen::EventTestConfig);
      break;
    case EventConfigField::TurnDetect:
      openDigitEditor("Turn detect", "cm", &workingSettings.turnDetectDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::TurnDetect), eventConfigDesc2(EventConfigField::TurnDetect), Screen::EventTestConfig);
      break;
    case EventConfigField::DeadEnd:
      openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::DeadEnd), eventConfigDesc2(EventConfigField::DeadEnd), Screen::EventTestConfig);
      break;
    case EventConfigField::Finish:
      openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::Finish), eventConfigDesc2(EventConfigField::Finish), Screen::EventTestConfig);
      break;
    case EventConfigField::TurnWait:
      openDigitEditor("Turn wait", "s", &workingSettings.waitBeforeTurn_x100, 0, 3000, eventConfigDesc1(EventConfigField::TurnWait), eventConfigDesc2(EventConfigField::TurnWait), Screen::EventTestConfig);
      break;
    case EventConfigField::CheckGap:
      openDigitEditor("Check gap", "s", &workingSettings.sensingInterval_x100, 1, 9999, eventConfigDesc1(EventConfigField::CheckGap), eventConfigDesc2(EventConfigField::CheckGap), Screen::EventTestConfig);
      break;
    case EventConfigField::MatchCount:
      openDigitEditor("Match N", "x", &workingSettings.eventConfirmCount, 1, 9, eventConfigDesc1(EventConfigField::MatchCount), eventConfigDesc2(EventConfigField::MatchCount), Screen::EventTestConfig);
      break;
    case EventConfigField::IRStable:
      openDigitEditor("IR stable", "s", &workingSettings.irStableTime_x100, 1, 9999, eventConfigDesc1(EventConfigField::IRStable), eventConfigDesc2(EventConfigField::IRStable), Screen::EventTestConfig);
      break;
    case EventConfigField::Run:
      startEventTestRun();
      gCurrentScreen = Screen::EventTestRun;
      break;
    case EventConfigField::Back:
    default:
      gCurrentScreen = Screen::EventTestMenu;
      break;
  }
}

void handleEventTestRunInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::LongPress) {
    stopEventTestRun();
    gCurrentScreen = Screen::EventTestConfig;
  }
}

void handleIRSensorTestInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 9;
  if (encoderDelta != 0) {
    gIrSensorIndex += encoderDelta;
    if (gIrSensorIndex < 0) gIrSensorIndex = 0;
    if (gIrSensorIndex >= itemCount) gIrSensorIndex = itemCount - 1;
  }
  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }
  if (buttonEvent != ButtonEvent::ShortPress) return;
  if (gIrSensorIndex == 4) openDigitEditor("Check gap", "s", &workingSettings.sensingInterval_x100, 1, 9999, "Time between event", "verification checks", Screen::IRSensorTest);
  if (gIrSensorIndex == 5) openDigitEditor("Match N", "x", &workingSettings.eventConfirmCount, 1, 9, "How many identical", "events confirm state", Screen::IRSensorTest);
  if (gIrSensorIndex == 6) openDigitEditor("IR stable", "s", &workingSettings.irStableTime_x100, 1, 9999, "How long IR mask", "must stay unchanged", Screen::IRSensorTest);
  if (gIrSensorIndex == 7) openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999, "Front distance limit", "for dead-end event", Screen::IRSensorTest);
  if (gIrSensorIndex == 8) openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999, "Front distance needed", "before finish counts", Screen::IRSensorTest);
}

void handleUltrasonicSensorTestInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
  }
}

void handleMazeTestRunInput(int encoderDelta, ButtonEvent buttonEvent) {
  if (encoderDelta != 0) {
    gPreviewHeading = headingFromIndex(wrap4((int)headingToIndex(gPreviewHeading) + encoderDelta));
  }
  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
  }
}

void updateEventTestRun() {
  if (!gEventTestRun.active) return;

  PerceptionFrame frame = buildPerceptionFrame(gEventTestRun.heading);
  EventType target = selectedEventTestTarget();

  switch (gEventTestRun.state) {
    case NavState::StartSeek:
      gEventTestRun.displayEvent = EventType::Start;
      if (frame.candidate == EventType::SafetyStop) {
        startEventPhase(NavState::SafetyStop, EventType::SafetyStop);
        stopAll();
        return;
      }
      if (updateCorridorConfirmState(gEventTestRun.corridorConfirm, frame.corridorSignature)) {
        startEventPhase(NavState::Corridor, EventType::Corridor);
        return;
      }
      followSmart(gEventTestRun.heading, gCfg.approachSpeed);
      return;

    case NavState::Corridor: {
      gEventTestRun.displayEvent = frame.candidate;
      if (gSelectedEventTest == EventTestCase::Corridor) {
        followSmart(gEventTestRun.heading, gCfg.baseSpeed);
        return;
      }
      if (frame.candidate == EventType::SafetyStop) {
        startEventPhase(NavState::SafetyStop, EventType::SafetyStop);
        stopAll();
        return;
      }

      EventType observed = frame.candidate;
      if (gSelectedEventTest != EventTestCase::Random && target != EventType::Idle) {
        if (observed != target) observed = EventType::Corridor;
      }

      EventType confirmed = EventType::Idle;
      if (updateEventConfirmState(gEventTestRun.eventConfirm, observed, confirmed)) {
        gEventTestRun.latchedEvent = confirmed;
        if (confirmed == EventType::Finish) {
          gEventTestRun.state = NavState::Finished;
          gEventTestRun.displayEvent = EventType::Finish;
          stopAll();
          return;
        }
        if (confirmed == EventType::DeadEnd) {
          gEventTestRun.heading = oppositeOf(gEventTestRun.heading);
          resetPid();
          startEventPhase(NavState::AcquireCorridor, EventType::DeadEnd);
          return;
        }
        if (confirmed == EventType::TJunctionStraight) {
          resetPid();
          startEventPhase(NavState::AcquireCorridor, EventType::TJunctionStraight);
          return;
        }
        if (confirmed == EventType::LeftTurn ||
            confirmed == EventType::RightTurn ||
            confirmed == EventType::TJunction) {
          if (confirmed == EventType::TJunction && gSelectedEventTest == EventTestCase::TJunction) {
            gEventTestRun.pendingHeading = (currentEventTestConfig().turnChoice == TurnChoice::Left)
                                             ? turnLeftOf(gEventTestRun.heading)
                                             : turnRightOf(gEventTestRun.heading);
          } else {
            gEventTestRun.pendingHeading = chooseHeadingForEvent(frame, confirmed, gEventTestRun.heading);
          }
          startEventPhase(NavState::ApproachWall, confirmed);
          return;
        }
      }

      followSmart(gEventTestRun.heading, gCfg.baseSpeed);
      return;
    }

    case NavState::ApproachWall:
      gEventTestRun.displayEvent = gEventTestRun.latchedEvent;
      if (!frame.ultrasonicValid) {
        stopAll();
        return;
      }
      if (frame.front <= gCfg.frontStopDistanceCm) {
        startEventPhase(NavState::WaitTurn, gEventTestRun.latchedEvent);
        stopAll();
        return;
      }
      followSmart(gEventTestRun.heading, gCfg.approachSpeed);
      return;

    case NavState::WaitTurn:
      gEventTestRun.displayEvent = gEventTestRun.latchedEvent;
      stopAll();
      if ((millis() - gEventTestRun.phaseStartedMs) >= secondsToMs(gCfg.waitBeforeTurnS)) {
        gEventTestRun.heading = gEventTestRun.pendingHeading;
        resetPid();
        startEventPhase(NavState::AcquireCorridor, gEventTestRun.latchedEvent);
      }
      return;

    case NavState::AcquireCorridor:
      gEventTestRun.displayEvent = gEventTestRun.latchedEvent;
      if (frame.candidate == EventType::SafetyStop) {
        startEventPhase(NavState::SafetyStop, EventType::SafetyStop);
        stopAll();
        return;
      }
      if (updateCorridorConfirmState(gEventTestRun.corridorConfirm, frame.corridorSignature)) {
        startEventPhase(NavState::Corridor, EventType::Corridor);
        return;
      }
      followSmart(gEventTestRun.heading, gCfg.approachSpeed);
      return;

    case NavState::SafetyStop:
      gEventTestRun.displayEvent = EventType::SafetyStop;
      stopAll();
      if (updateCorridorConfirmState(gEventTestRun.corridorConfirm, frame.corridorSignature)) {
        startEventPhase(NavState::Corridor, EventType::Corridor);
      }
      return;

    case NavState::Finished:
      gEventTestRun.displayEvent = EventType::Finish;
      stopAll();
      return;
    case NavState::Idle:
    default:
      stopAll();
      return;
  }
}

void handleStartConfirmInput(ButtonEvent buttonEvent) {
  if (buttonEvent == ButtonEvent::ShortPress) {
    beginExecution();
  } else if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
  }
}

void handleIdleUiInput(int encoderDelta, ButtonEvent buttonEvent) {
  switch (gCurrentScreen) {
    case Screen::Root:                 handleRootInput(encoderDelta, buttonEvent); break;
    case Screen::StartSettings:        handleStartSettingsInput(encoderDelta, buttonEvent); break;
    case Screen::Settings:             handleSettingsInput(encoderDelta, buttonEvent); break;
    case Screen::PID:                  handlePidInput(encoderDelta, buttonEvent); break;
    case Screen::PIDTest:              handlePidTestInput(buttonEvent); break;
    case Screen::IMUTest:              handleImuTestInput(buttonEvent); break;
    case Screen::IMUYawTest:           handleImuYawTestInput(buttonEvent); break;
    case Screen::MotorTune:            handleMotorTuneInput(encoderDelta, buttonEvent); break;
    case Screen::EventTestMenu:        handleEventTestMenuInput(encoderDelta, buttonEvent); break;
    case Screen::EventTestConfig:      handleEventTestConfigInput(encoderDelta, buttonEvent); break;
    case Screen::EventTestRun:         handleEventTestRunInput(buttonEvent); break;
    case Screen::MazeTestRun:          handleMazeTestRunInput(encoderDelta, buttonEvent); break;
    case Screen::IRSensorTest:         handleIRSensorTestInput(encoderDelta, buttonEvent); break;
    case Screen::UltrasonicSensorTest: handleUltrasonicSensorTestInput(buttonEvent); break;
    case Screen::StartConfirm:         handleStartConfirmInput(buttonEvent); break;
    case Screen::DigitEditor:          handleDigitEditorInput(encoderDelta, buttonEvent); break;
    default:                           break;
  }
}

void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 8, title);
  u8g2.drawHLine(0, 11, 128);
}

void drawMenuItem(int y, bool selected, const char* text) {
  if (selected) {
    u8g2.drawBox(0, y - 9, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, y, text);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, y, text);
  }
}

static void drawSimpleMenu(const char* title, const char* const* items, int itemCount, int selectedIndex) {
  drawHeader(title);
  drawScrollableItemList(items, itemCount, selectedIndex, 26);
}

void drawScrollableItemList(const char* const* items, int itemCount, int selectedIndex, int startY) {
  constexpr int visibleCount = 4;
  int first = selectedIndex - 1;
  if (first < 0) first = 0;
  if (first > itemCount - visibleCount) first = max(0, itemCount - visibleCount);

  for (int i = 0; i < visibleCount; ++i) {
    int idx = first + i;
    if (idx >= itemCount) break;
    drawMenuItem(startY + i * 10, idx == selectedIndex, items[idx]);
  }

  if (itemCount > visibleCount) {
    if (first > 0) {
      u8g2.drawLine(122, 14, 124, 11);
      u8g2.drawLine(124, 11, 126, 14);
    }
    if (first + visibleCount < itemCount) {
      u8g2.drawLine(122, 58, 124, 61);
      u8g2.drawLine(124, 61, 126, 58);
    }
  }
}

void drawRootScreen() {
  drawHeader("Main Menu");
  static const char* const items[] = {
    "Start",
    "Start Settings",
    "Settings",
    "Event Test",
    "IMU Test",
    "Yaw Test",
    "Execution State",
    "IR Test",
    "Ultrasonic Test"
  };
  drawScrollableItemList(items, 9, gRootIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short: select");
}

void drawStartSettingsScreen() {
  char line0[24];
  char line1[24];
  char line2[24];

  snprintf(line0, sizeof(line0), "Route: %s", routeModeToString(currentRouteMode()));
  snprintf(line1, sizeof(line1), "Start dir: %s", headingToString(headingFromIndex(activeSettings.startHeadingIndex)));
  snprintf(line2, sizeof(line2), "Turn wait: %d.%02d s",
           activeSettings.waitBeforeTurn_x100 / 100,
           activeSettings.waitBeforeTurn_x100 % 100);

  const char* items[] = {
    line0,
    line1,
    line2,
    "Back"
  };
  drawSimpleMenu("Start Settings", items, 4, gStartSettingsIndex);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: root");
}

void drawSettingsScreen() {
  drawHeader("Settings");
  char items[12][28];
  snprintf(items[0], sizeof(items[0]), "PID");
  snprintf(items[1], sizeof(items[1]), "Motor Tune");
  snprintf(items[2], sizeof(items[2]), "IMU:%s", activeSettings.imuEnabled ? "ON" : "OFF");
  snprintf(items[3], sizeof(items[3]), "FrontStop:%d.%02d", activeSettings.frontStopDistance_x100 / 100, activeSettings.frontStopDistance_x100 % 100);
  snprintf(items[4], sizeof(items[4]), "WallThr:%d.%02d", activeSettings.corridorWallThreshold_x100 / 100, activeSettings.corridorWallThreshold_x100 % 100);
  snprintf(items[5], sizeof(items[5]), "TurnDet:%d.%02d", activeSettings.turnDetectDistance_x100 / 100, activeSettings.turnDetectDistance_x100 % 100);
  snprintf(items[6], sizeof(items[6]), "DeadEnd:%d.%02d", activeSettings.deadEndDistance_x100 / 100, activeSettings.deadEndDistance_x100 % 100);
  snprintf(items[7], sizeof(items[7]), "Finish:%d.%02d", activeSettings.finishDistance_x100 / 100, activeSettings.finishDistance_x100 % 100);
  snprintf(items[8], sizeof(items[8]), "CheckGap:%d.%02d", activeSettings.sensingInterval_x100 / 100, activeSettings.sensingInterval_x100 % 100);
  snprintf(items[9], sizeof(items[9]), "Match N:%d", activeSettings.eventConfirmCount);
  snprintf(items[10], sizeof(items[10]), "IRStable:%d.%02d", activeSettings.irStableTime_x100 / 100, activeSettings.irStableTime_x100 % 100);
  snprintf(items[11], sizeof(items[11]), "Back");
  const char* ptrs[12];
  for (int i = 0; i < 12; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 12, gSettingsIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: root");
}

void drawPIDScreen() {
  drawHeader("PID");
  char items[14][28];
  snprintf(items[0], sizeof(items[0]), "P:%d.%02d", activeSettings.p_x100 / 100, activeSettings.p_x100 % 100);
  snprintf(items[1], sizeof(items[1]), "I:%d.%02d", activeSettings.i_x100 / 100, activeSettings.i_x100 % 100);
  snprintf(items[2], sizeof(items[2]), "D:%d.%02d", activeSettings.d_x100 / 100, activeSettings.d_x100 % 100);
  snprintf(items[3], sizeof(items[3]), "Curve:%d.%02d", activeSettings.curveB_x100 / 100, activeSettings.curveB_x100 % 100);
  snprintf(items[4], sizeof(items[4]), "1Wall:%d.%02d", activeSettings.pidWallDistance_x100 / 100, activeSettings.pidWallDistance_x100 % 100);
  snprintf(items[5], sizeof(items[5]), "LeftOut:%d.%02d", activeSettings.pidLeftScale_x100 / 100, activeSettings.pidLeftScale_x100 % 100);
  snprintf(items[6], sizeof(items[6]), "RightOut:%d.%02d", activeSettings.pidRightScale_x100 / 100, activeSettings.pidRightScale_x100 % 100);
  snprintf(items[7], sizeof(items[7]), "PID Test");
  snprintf(items[8], sizeof(items[8]), "IMUP:%d.%02d", activeSettings.imuP_x100 / 100, activeSettings.imuP_x100 % 100);
  snprintf(items[9], sizeof(items[9]), "IMUI:%d.%02d", activeSettings.imuI_x100 / 100, activeSettings.imuI_x100 % 100);
  snprintf(items[10], sizeof(items[10]), "IMUD:%d.%02d", activeSettings.imuD_x100 / 100, activeSettings.imuD_x100 % 100);
  snprintf(items[11], sizeof(items[11]), "IMUThr:%d.%02d", activeSettings.imuAngleThreshold_x100 / 100, activeSettings.imuAngleThreshold_x100 % 100);
  snprintf(items[12], sizeof(items[12]), "IMU PID Test");
  snprintf(items[13], sizeof(items[13]), "Back");
  const char* ptrs[14];
  for (int i = 0; i < 14; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 14, gPidIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: settings");
}

void drawPIDTestScreen() {
  drawHeader("PID Test");
  PerceptionFrame frame = buildPerceptionFrame(gPidTestHeading);
  float rawError = computeWallError(frame, frame.pidSource);
  float shaped = shapeSignedError(rawError);
  u8g2.setFont(u8g2_font_5x7_tf);
  char line[32];
  snprintf(line, sizeof(line), "Dir:%s Src:%s", headingToString(gPidTestHeading), pidSourceToString(frame.pidSource));
  u8g2.drawStr(0, 21, line);
  snprintf(line, sizeof(line), "L:%2.1f R:%2.1f", frame.left, frame.right);
  u8g2.drawStr(0, 28, line);
  snprintf(line, sizeof(line), "Raw:%2.2f", rawError);
  u8g2.drawStr(0, 35, line);
  snprintf(line, sizeof(line), "Shp:%2.2f", shaped);
  u8g2.drawStr(0, 42, line);
  u8g2.drawStr(0, 63, "Short=rotate Hold=back");
}

void drawIMUTestScreen() {
  drawHeader("IMU Test");
  u8g2.setFont(u8g2_font_5x7_tf);

  if (!activeSettings.imuEnabled) {
    u8g2.drawStr(0, 22, "IMU is OFF");
    u8g2.drawStr(0, 31, "Enable it in");
    u8g2.drawStr(0, 40, "Settings -> IMU");
    u8g2.drawStr(0, 49, "to use it globally");
    u8g2.drawStr(0, 63, "Long=back");
    return;
  }

  if (!gImu.available) {
    u8g2.drawStr(0, 22, "IMU init failed");
    u8g2.drawStr(0, 31, "Library is used,");
    u8g2.drawStr(0, 40, "but IMU.begin()");
    u8g2.drawStr(0, 49, "returned false");
    u8g2.drawStr(0, 63, "Long=back");
    return;
  }

  char line[32];
  snprintf(line, sizeof(line), "Ax:%+1.3f Ay:%+1.3f", gImu.ax, gImu.ay);
  u8g2.drawStr(0, 18, line);
  snprintf(line, sizeof(line), "Az:%+1.3f", gImu.az);
  u8g2.drawStr(0, 27, line);
  snprintf(line, sizeof(line), "Gx:%+2.2f Gy:%+2.2f", gImu.gx, gImu.gy);
  u8g2.drawStr(0, 36, line);
  snprintf(line, sizeof(line), "Gz:%+2.2f Bz:%+2.2f", gImu.gz, gImu.gyroBiasZDps);
  u8g2.drawStr(0, 45, line);
  snprintf(line, sizeof(line), "Yaw:%+3.2f Ref:%c", gImu.yawDeg, gImu.referenceValid ? 'Y' : 'N');
  u8g2.drawStr(0, 54, line);
  u8g2.drawStr(0, 63, "Hold=back");
}

void drawIMUYawTestScreen() {
  drawHeader("Yaw Test");
  u8g2.setFont(u8g2_font_5x8_tf);

  if (!activeSettings.imuEnabled) {
    u8g2.drawStr(0, 22, "IMU is OFF");
    u8g2.drawStr(0, 31, "Enable in Settings");
    u8g2.drawStr(0, 63, "Hold=back");
    return;
  }

  if (!gImu.available) {
    u8g2.drawStr(0, 22, "IMU init failed");
    u8g2.drawStr(0, 63, "Hold=back");
    return;
  }

  float shownYawDeg = gImu.referenceValid ? gImu.yawErrorDeg : gImu.yawDeg;

  char line[32];
  snprintf(line, sizeof(line), "Relative yaw");
  u8g2.drawStr(0, 18, line);

  u8g2.setFont(u8g2_font_logisoso20_tn);
  snprintf(line, sizeof(line), "%+05.1f", shownYawDeg);
  u8g2.drawStr(8, 44, line);

  u8g2.setFont(u8g2_font_5x8_tf);
  snprintf(line, sizeof(line), "Rate:%+2.2f dps", gImu.yawRateDps);
  u8g2.drawStr(0, 54, line);
  snprintf(line, sizeof(line), "Db:%2.2f  Ref:%c", gImu.combinedDeadbandDeg, gImu.referenceValid ? 'Y' : 'N');
  u8g2.drawStr(0, 63, line);
}

void drawIMUPIDTestScreen() {
  drawHeader("IMU PID");
  u8g2.setFont(u8g2_font_5x7_tf);

  if (!activeSettings.imuEnabled) {
    u8g2.drawStr(0, 24, "IMU is OFF");
    u8g2.drawStr(0, 33, "Enable in Settings");
    u8g2.drawStr(0, 63, "Long=back");
    return;
  }

  if (!gImu.available) {
    u8g2.drawStr(0, 24, "IMU init failed");
    u8g2.drawStr(0, 63, "Long=back");
    return;
  }

  char line[32];
  if (gImu.referenceValid) {
    snprintf(line, sizeof(line), "Ref:(%+1.2f,%+1.2f)", gImu.reference.xAxisX, gImu.reference.xAxisY);
  } else {
    snprintf(line, sizeof(line), "Ref:not saved");
  }
  u8g2.drawStr(0, 18, line);
  snprintf(line, sizeof(line), "Cur:(%+1.2f,%+1.2f)", gImu.currentXAxisX, gImu.currentXAxisY);
  u8g2.drawStr(0, 27, line);
  snprintf(line, sizeof(line), "Err:%+2.2f Db:%2.2f", gImu.yawErrorDeg, gImu.combinedDeadbandDeg);
  u8g2.drawStr(0, 36, line);
  snprintf(line, sizeof(line), "Cmd:%+1.3f Bias:%+2.2f", gImu.lastCompensationCmd, gImu.gyroBiasZDps);
  u8g2.drawStr(0, 45, line);
  snprintf(line, sizeof(line), "A0:(%+1.3f,%+1.3f)", gImu.reference.avgAx, gImu.reference.avgAy);
  u8g2.drawStr(0, 54, line);
  u8g2.drawStr(0, 63, "Short=save Hold=back");
}

void drawMotorTuneScreen() {
  drawHeader("Motor Tune");
  const char* items[] = { "Stop", "North", "East", "South", "West", "Back" };
  drawScrollableItemList(items, 6, gMotorTuneIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=run Long=back");
}

void drawMotionSettingsScreen() {
  char items[12][28];
  snprintf(items[0], sizeof(items[0]), "P gain: %d.%02d", activeSettings.p_x100 / 100, activeSettings.p_x100 % 100);
  snprintf(items[1], sizeof(items[1]), "I gain: %d.%02d", activeSettings.i_x100 / 100, activeSettings.i_x100 % 100);
  snprintf(items[2], sizeof(items[2]), "D gain: %d.%02d", activeSettings.d_x100 / 100, activeSettings.d_x100 % 100);
  snprintf(items[3], sizeof(items[3]), "Curve b: %d.%02d", activeSettings.curveB_x100 / 100, activeSettings.curveB_x100 % 100);
  snprintf(items[4], sizeof(items[4]), "1-wall: %d.%02dcm", activeSettings.pidWallDistance_x100 / 100, activeSettings.pidWallDistance_x100 % 100);
  snprintf(items[5], sizeof(items[5]), "L corr: %d.%02d", activeSettings.pidLeftScale_x100 / 100, activeSettings.pidLeftScale_x100 % 100);
  snprintf(items[6], sizeof(items[6]), "R corr: %d.%02d", activeSettings.pidRightScale_x100 / 100, activeSettings.pidRightScale_x100 % 100);
  snprintf(items[7], sizeof(items[7]), "Base: %d.%02d", activeSettings.baseSpeed_x100 / 100, activeSettings.baseSpeed_x100 % 100);
  snprintf(items[8], sizeof(items[8]), "Approach: %d.%02d", activeSettings.approachSpeed_x100 / 100, activeSettings.approachSpeed_x100 % 100);
  snprintf(items[9], sizeof(items[9]), "L drive: %d.%02d", activeSettings.leftDriveScale_x100 / 100, activeSettings.leftDriveScale_x100 % 100);
  snprintf(items[10], sizeof(items[10]), "R drive: %d.%02d", activeSettings.rightDriveScale_x100 / 100, activeSettings.rightDriveScale_x100 % 100);
  snprintf(items[11], sizeof(items[11]), "Back");

  const char* pointers[12];
  for (int i = 0; i < 12; ++i) pointers[i] = items[i];
  drawSimpleMenu("Motion Settings", pointers, 12, gMotionSettingsIndex);
}

void drawDetectionSettingsScreen() {
  char items[8][28];
  snprintf(items[0], sizeof(items[0]), "Front stop: %d.%02d", activeSettings.frontStopDistance_x100 / 100, activeSettings.frontStopDistance_x100 % 100);
  snprintf(items[1], sizeof(items[1]), "Wall lim: %d.%02d", activeSettings.corridorWallThreshold_x100 / 100, activeSettings.corridorWallThreshold_x100 % 100);
  snprintf(items[2], sizeof(items[2]), "Turn det: %d.%02d", activeSettings.turnDetectDistance_x100 / 100, activeSettings.turnDetectDistance_x100 % 100);
  snprintf(items[3], sizeof(items[3]), "Dead-end: %d.%02d", activeSettings.deadEndDistance_x100 / 100, activeSettings.deadEndDistance_x100 % 100);
  snprintf(items[4], sizeof(items[4]), "Finish: %d.%02d", activeSettings.finishDistance_x100 / 100, activeSettings.finishDistance_x100 % 100);
  snprintf(items[5], sizeof(items[5]), "Recheck: %d.%02d", activeSettings.sensingInterval_x100 / 100, activeSettings.sensingInterval_x100 % 100);
  snprintf(items[6], sizeof(items[6]), "IR stable: %d.%02d", activeSettings.irStableTime_x100 / 100, activeSettings.irStableTime_x100 % 100);
  snprintf(items[7], sizeof(items[7]), "Back");

  const char* pointers[8];
  for (int i = 0; i < 8; ++i) pointers[i] = items[i];
  drawSimpleMenu("Detection Set", pointers, 8, gDetectionSettingsIndex);
}

void drawSensorMonitorScreen() {
  Heading heading = gExecutive.running ? gExecutive.heading : gPreviewHeading;
  PerceptionFrame frame = buildPerceptionFrame(heading);

  drawHeader("Sensor Monitor");
  u8g2.setFont(u8g2_font_5x8_tr);

  char line[32];
  snprintf(line, sizeof(line), "Dir:%s  Event:%s", headingToString(heading), eventToString(frame.candidate));
  u8g2.drawStr(0, 20, line);

  snprintf(line, sizeof(line), "IR fL:%d fR:%d rL:%d rR:%d",
           frame.ir.frontLeft ? 1 : 0,
           frame.ir.frontRight ? 1 : 0,
           frame.ir.rearLeft ? 1 : 0,
           frame.ir.rearRight ? 1 : 0);
  u8g2.drawStr(0, 29, line);

  snprintf(line, sizeof(line), "F:%2.1f L:%2.1f R:%2.1f", frame.front, frame.left, frame.right);
  u8g2.drawStr(0, 38, line);

  snprintf(line, sizeof(line), "B:%2.1f  PID:%s", frame.back, pidSourceToString(frame.pidSource));
  u8g2.drawStr(0, 47, line);

  snprintf(line, sizeof(line), "Stable mask:%02X", frame.stableIrMask);
  u8g2.drawStr(0, 56, line);

  u8g2.drawStr(0, 64, "Enc:dir  Hold:back");
}

static void drawRobotCorner(int x, int y, bool triggered) {
  if (triggered) {
    u8g2.drawDisc(x, y, 2);
  } else {
    u8g2.drawCircle(x, y, 2);
  }
}

static void drawTwinSideStubs(int left, int top, int right, int bottom, const RelativeIrState& ir) {
  if (ir.frontLeft)  u8g2.drawVLine(left - 7, top + 1, 6);
  if (ir.frontRight) u8g2.drawVLine(right + 7, top + 1, 6);
  if (ir.rearLeft)   u8g2.drawVLine(left - 7, bottom - 6, 6);
  if (ir.rearRight)  u8g2.drawVLine(right + 7, bottom - 6, 6);
}

void drawTwinMonitorScreen() {
  Heading heading = gExecutive.running ? gExecutive.heading : gPreviewHeading;
  PerceptionFrame frame = buildPerceptionFrame(heading);

  drawHeader(eventToString(frame.candidate));
  u8g2.setFont(u8g2_font_5x8_tr);

  int left = 46;
  int top = 16;
  int width = 36;
  int height = 24;
  int right = left + width;
  int bottom = top + height;
  int cx = left + width / 2;

  // Main symmetric body
  u8g2.drawFrame(left, top, width, height);
  drawRobotCorner(left, top, frame.ir.frontLeft);
  drawRobotCorner(right, top, frame.ir.frontRight);
  drawRobotCorner(left, bottom, frame.ir.rearLeft);
  drawRobotCorner(right, bottom, frame.ir.rearRight);

  // Inner direction labels
  u8g2.drawStr(cx - 2, top + 7, headingToString(heading));
  u8g2.drawStr(left + 3, top + 14, headingToString(turnLeftOf(heading)));
  u8g2.drawStr(right - 5, top + 14, headingToString(turnRightOf(heading)));
  u8g2.drawStr(cx - 2, bottom - 3, headingToString(oppositeOf(heading)));

  // Side walls from ultrasonic
  if (frame.left <= gCfg.corridorWallThresholdCm) {
    u8g2.drawVLine(left - 14, top, height + 1);
  }
  if (frame.right <= gCfg.corridorWallThresholdCm) {
    u8g2.drawVLine(right + 14, top, height + 1);
  }

  // Front wall from front ultrasonic
  if (frame.front <= gCfg.turnDetectDistanceCm) {
    u8g2.drawHLine(left + 6, top - 8, width - 12);
  }

  // IR stubs around the robot
  drawTwinSideStubs(left, top, right, bottom, frame.ir);

  char line[24];
  snprintf(line, sizeof(line), "N:%04.1f E:%04.1f", frame.north, frame.east);
  u8g2.drawStr(0, 55, line);
  snprintf(line, sizeof(line), "S:%04.1f W:%04.1f", frame.south, frame.west);
  u8g2.drawStr(0, 64, line);
}

static void eventConfigFieldText(EventConfigField field, char* buffer, size_t size) {
  switch (field) {
    case EventConfigField::Heading:
      snprintf(buffer, size, "Heading: %s", headingToString(currentEventTestHeading()));
      break;
    case EventConfigField::TurnChoice:
      snprintf(buffer, size, "T-Choice: %s", turnChoiceToString(currentEventTestConfig().turnChoice));
      break;
    case EventConfigField::FrontStop:
      snprintf(buffer, size, "FrontStop:%d.%02d", activeSettings.frontStopDistance_x100 / 100, activeSettings.frontStopDistance_x100 % 100);
      break;
    case EventConfigField::WallThreshold:
      snprintf(buffer, size, "WallThr:%d.%02d", activeSettings.corridorWallThreshold_x100 / 100, activeSettings.corridorWallThreshold_x100 % 100);
      break;
    case EventConfigField::TurnDetect:
      snprintf(buffer, size, "TurnDet:%d.%02d", activeSettings.turnDetectDistance_x100 / 100, activeSettings.turnDetectDistance_x100 % 100);
      break;
    case EventConfigField::DeadEnd:
      snprintf(buffer, size, "DeadEnd:%d.%02d", activeSettings.deadEndDistance_x100 / 100, activeSettings.deadEndDistance_x100 % 100);
      break;
    case EventConfigField::Finish:
      snprintf(buffer, size, "Finish:%d.%02d", activeSettings.finishDistance_x100 / 100, activeSettings.finishDistance_x100 % 100);
      break;
    case EventConfigField::TurnWait:
      snprintf(buffer, size, "TurnWait:%d.%02d", activeSettings.waitBeforeTurn_x100 / 100, activeSettings.waitBeforeTurn_x100 % 100);
      break;
    case EventConfigField::CheckGap:
      snprintf(buffer, size, "CheckGap:%d.%02d", activeSettings.sensingInterval_x100 / 100, activeSettings.sensingInterval_x100 % 100);
      break;
    case EventConfigField::MatchCount:
      snprintf(buffer, size, "Match N:%d", activeSettings.eventConfirmCount);
      break;
    case EventConfigField::IRStable:
      snprintf(buffer, size, "IRStable:%d.%02d", activeSettings.irStableTime_x100 / 100, activeSettings.irStableTime_x100 % 100);
      break;
    case EventConfigField::Run:
      snprintf(buffer, size, "Run This Test");
      break;
    case EventConfigField::Back:
    default:
      snprintf(buffer, size, "Back");
      break;
  }
}

void drawEventTestMenuScreen() {
  drawHeader("Event Test");
  const char* items[] = {
    "Corridor", "T-Junction", "Dead-end", "Auto Start", "Finish",
    "Left Turn", "Right Turn", "T-Jct Straight", "Random", "Back"
  };
  drawScrollableItemList(items, 10, gEventTestIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=config Long=back");
}

void drawEventTestConfigScreen() {
  drawHeader(eventTestCaseToString(gSelectedEventTest));
  int itemCount = currentEventConfigItemCount();
  char items[13][28];
  const char* ptrs[13];
  for (int i = 0; i < itemCount; ++i) {
    eventConfigFieldText(currentEventConfigFieldAt(i), items[i], sizeof(items[i]));
    ptrs[i] = items[i];
  }
  drawScrollableItemList(ptrs, itemCount, gEventTestConfigIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Behavior fields only");
}

void drawEventTestRunScreen() {
  drawHeader("Event Run");
  PerceptionFrame frame = buildPerceptionFrame(gEventTestRun.heading);
  u8g2.setFont(u8g2_font_5x8_tr);
  char line[32];
  snprintf(line, sizeof(line), "Case:%s", eventTestCaseToString(gSelectedEventTest));
  u8g2.drawStr(0, 20, line);
  snprintf(line, sizeof(line), "State:%s", navStateToString(gEventTestRun.state));
  u8g2.drawStr(0, 29, line);
  snprintf(line, sizeof(line), "Event:%s", eventToString(gEventTestRun.displayEvent));
  u8g2.drawStr(0, 38, line);
  snprintf(line, sizeof(line), "Dir:%s PID:%s", headingToString(gEventTestRun.heading), pidSourceToString(frame.pidSource));
  u8g2.drawStr(0, 47, line);
  snprintf(line, sizeof(line), "F:%2.1f L:%2.1f R:%2.1f", frame.front, frame.left, frame.right);
  u8g2.drawStr(0, 56, line);
  u8g2.drawStr(0, 63, "Long=back");
}

void drawIRSensorTestScreen() {
  drawHeader("IR Test");
  char items[9][28];
  snprintf(items[0], sizeof(items[0]), "NW(RL): %d", isIrTriggered(gIrRL) ? 1 : 0);
  snprintf(items[1], sizeof(items[1]), "NE(RR): %d", isIrTriggered(gIrRR) ? 1 : 0);
  snprintf(items[2], sizeof(items[2]), "SW(FL): %d", isIrTriggered(gIrFL) ? 1 : 0);
  snprintf(items[3], sizeof(items[3]), "SE(FR): %d", isIrTriggered(gIrFR) ? 1 : 0);
  snprintf(items[4], sizeof(items[4]), "CheckGap:%d.%02d", activeSettings.sensingInterval_x100 / 100, activeSettings.sensingInterval_x100 % 100);
  snprintf(items[5], sizeof(items[5]), "Match N:%d", activeSettings.eventConfirmCount);
  snprintf(items[6], sizeof(items[6]), "IRStable:%d.%02d", activeSettings.irStableTime_x100 / 100, activeSettings.irStableTime_x100 % 100);
  snprintf(items[7], sizeof(items[7]), "DeadEnd:%d.%02d", activeSettings.deadEndDistance_x100 / 100, activeSettings.deadEndDistance_x100 % 100);
  snprintf(items[8], sizeof(items[8]), "Finish:%d.%02d", activeSettings.finishDistance_x100 / 100, activeSettings.finishDistance_x100 % 100);
  const char* ptrs[9];
  for (int i = 0; i < 9; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 9, gIrSensorIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=edit Long=back");
}

void drawUltrasonicSensorTestScreen() {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(18, 8, "ULTRASONIC SENSORS");
  u8g2.drawHLine(0, 10, 128);
  char line[24];
  u8g2.drawStr(0, 18, "Front:");
  snprintf(line, sizeof(line), "%4.1f cm", gAvgN);
  u8g2.drawStr(72, 18, line);
  u8g2.drawStr(0, 28, "Right:");
  snprintf(line, sizeof(line), "%4.1f cm", gAvgE);
  u8g2.drawStr(72, 28, line);
  u8g2.drawStr(0, 38, "Back:");
  snprintf(line, sizeof(line), "%4.1f cm", gAvgS);
  u8g2.drawStr(72, 38, line);
  u8g2.drawStr(0, 48, "Left:");
  snprintf(line, sizeof(line), "%4.1f cm", gAvgW);
  u8g2.drawStr(72, 48, line);
  u8g2.drawHLine(0, 55, 127);
  snprintf(line, sizeof(line), "FB:%2.2f", gAvgN - gAvgS);
  u8g2.drawStr(0, 63, line);
  snprintf(line, sizeof(line), "LR:%2.2f", gAvgW - gAvgE);
  u8g2.drawStr(68, 63, line);
}

void drawMazeTestRunScreen() {
  drawTwinMonitorScreen();
}

void drawDigitEditorScreen() {
  drawHeader(gDigitEditor.label);
  u8g2.setFont(u8g2_font_logisoso20_tn);

  char numeric[12];
  formatFixedValue(numeric, sizeof(numeric), gDigitEditor.tempValue);
  u8g2.drawStr(12, 40, numeric);

  int digitX[4] = {14, 22, 34, 42};
  int selectedX = digitX[gDigitEditor.selectedDigit];
  if (gDigitEditor.selectedDigit >= 2) selectedX += 6;

  if (gDigitEditor.digitUnlocked) {
    u8g2.drawFrame(selectedX - 1, 18, 8, 23);
  } else {
    u8g2.drawLine(selectedX, 44, selectedX + 2, 40);
    u8g2.drawLine(selectedX + 2, 40, selectedX + 4, 44);
  }

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 49, gDigitEditor.description1);
  u8g2.drawStr(0, 56, gDigitEditor.description2);
  u8g2.drawStr(0, 63, "Short:edit Long:save");
}

void drawStartConfirmScreen() {
  drawHeader("Start Confirmation");
  u8g2.setFont(u8g2_font_5x8_tr);

  char line[32];
  snprintf(line, sizeof(line), "Mode:%s", routeModeToString(currentRouteMode()));
  u8g2.drawStr(0, 20, line);

  snprintf(line, sizeof(line), "Next:%s  Dir:%s",
           sequenceStepToString(),
           headingToString(headingFromIndex(activeSettings.startHeadingIndex)));
  u8g2.drawStr(0, 29, line);

  char duration[20];
  formatDuration(duration, sizeof(duration), gPlanner.lastAttemptDurationMs);
  snprintf(line, sizeof(line), "Last:%s", duration);
  u8g2.drawStr(0, 40, line);

  formatDuration(duration, sizeof(duration), gPlanner.totalDurationMs);
  snprintf(line, sizeof(line), "Total:%s", duration);
  u8g2.drawStr(0, 49, line);

  u8g2.drawStr(0, 64, "Short:start  Hold:root");
}

void drawRunScreen() {
  PerceptionFrame frame = buildPerceptionFrame(gExecutive.heading);
  drawHeader("Execution");
  u8g2.setFont(u8g2_font_5x8_tr);

  char line[32];
  snprintf(line, sizeof(line), "State:%s", navStateToString(gExecutive.state));
  u8g2.drawStr(0, 20, line);

  snprintf(line, sizeof(line), "Event:%s", eventToString(gExecutive.displayEvent));
  u8g2.drawStr(0, 29, line);

  snprintf(line, sizeof(line), "Dir:%s  PID:%s",
           headingToString(gExecutive.heading),
           pidSourceToString(frame.pidSource));
  u8g2.drawStr(0, 38, line);

  snprintf(line, sizeof(line), "F:%2.1f L:%2.1f R:%2.1f", frame.front, frame.left, frame.right);
  u8g2.drawStr(0, 47, line);

  char duration[20];
  formatDuration(duration, sizeof(duration), millis() - gExecutive.runStartedMs);
  snprintf(line, sizeof(line), "T:%s", duration);
  u8g2.drawStr(0, 56, line);

  snprintf(line, sizeof(line), "Try:%lu",
           (unsigned long)gPlanner.attemptCounter);
  u8g2.drawStr(92, 56, line);
}

void drawUI() {
  u8g2.clearBuffer();

  switch (gCurrentScreen) {
    case Screen::Root:                 drawRootScreen(); break;
    case Screen::StartSettings:        drawStartSettingsScreen(); break;
    case Screen::Settings:             drawSettingsScreen(); break;
    case Screen::PID:                  drawPIDScreen(); break;
    case Screen::PIDTest:              drawPIDTestScreen(); break;
    case Screen::IMUPIDTest:           drawIMUPIDTestScreen(); break;
    case Screen::MotorTune:            drawMotorTuneScreen(); break;
    case Screen::EventTestMenu:        drawEventTestMenuScreen(); break;
    case Screen::EventTestConfig:      drawEventTestConfigScreen(); break;
    case Screen::EventTestRun:         drawEventTestRunScreen(); break;
    case Screen::IMUTest:              drawIMUTestScreen(); break;
    case Screen::IMUYawTest:           drawIMUYawTestScreen(); break;
    case Screen::MazeTestRun:          drawMazeTestRunScreen(); break;
    case Screen::IRSensorTest:         drawIRSensorTestScreen(); break;
    case Screen::UltrasonicSensorTest: drawUltrasonicSensorTestScreen(); break;
    case Screen::DigitEditor:          drawDigitEditorScreen(); break;
    case Screen::StartConfirm:         drawStartConfirmScreen(); break;
    case Screen::RunScreen:            drawRunScreen(); break;
    default:                           drawRootScreen(); break;
  }

  u8g2.sendBuffer();
}

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
  Wire.begin();
  initImu();
  u8g2.begin();
  u8g2.setContrast(255);

  servoFL.attach(PIN_FL, 1000, 2000);
  servoFR.attach(PIN_FR, 1000, 2000);
  servoRR.attach(PIN_RR, 1000, 2000);
  servoRL.attach(PIN_RL, 1000, 2000);
  stopAll();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), onEncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), onEncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_FL), onAnyIrChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_FR), onAnyIrChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_RL), onAnyIrChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_RR), onAnyIrChange, CHANGE);

  workingSettings = activeSettings;
  commitRuntimeConfig();

  readIRSensors();
  gIrCandidateMask = gRawIrMask;
  gStableIrMask = gRawIrMask;
  gIrCandidateSinceMs = millis();
  gPreviewHeading = headingFromIndex(activeSettings.startHeadingIndex);
  resetPid();
  resetImuPid();
}

void loop() {
  refreshSensors();

  int encoderDelta = 0;
  noInterrupts();
  encoderDelta = gEncoderDelta;
  gEncoderDelta = 0;
  interrupts();

  if (gExecutive.running) {
    (void)readButtonEvent(false);
    updateExecutive();
    gCurrentScreen = Screen::RunScreen;
  } else if (gCurrentScreen == Screen::EventTestRun) {
    ButtonEvent buttonEvent = readButtonEvent(true);
    handleEventTestRunInput(buttonEvent);
    if (gCurrentScreen == Screen::EventTestRun) {
      updateEventTestRun();
    }
  } else if (gCurrentScreen == Screen::MotorTune) {
    ButtonEvent buttonEvent = readButtonEvent(true);
    handleMotorTuneInput(encoderDelta, buttonEvent);
    if (gCurrentScreen == Screen::MotorTune) {
      runTuneMotion();
    } else {
      stopAll();
    }
  } else if (gCurrentScreen == Screen::IMUPIDTest) {
    ButtonEvent buttonEvent = readButtonEvent(true);
    handleImuPidTestInput(buttonEvent);
    if (gCurrentScreen == Screen::IMUPIDTest) {
      (void)applyImuCompensation(gImuPid);
    }
  } else {
    ButtonEvent buttonEvent = readButtonEvent(true);
    handleIdleUiInput(encoderDelta, buttonEvent);
  }

  drawUI();
}
