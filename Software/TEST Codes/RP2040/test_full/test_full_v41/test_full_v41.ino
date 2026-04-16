#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>
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
constexpr uint32_t BUTTON_DEBOUNCE_MS = 15;
constexpr uint32_t BUTTON_LONGPRESS_MS = 700;
constexpr uint32_t ULTRASONIC_STALE_MS = 200;
constexpr uint8_t ULTRASONIC_AVG_COUNT_MAX = 5;
constexpr uint8_t EVENT_CONFIRM_SAMPLES = 3;
constexpr uint8_t CORRIDOR_CONFIRM_SAMPLES = 3;
constexpr uint32_t UI_REFRESH_INTERVAL_MS = 40;
constexpr float SENSOR_MAX_VALID_CM = 60.0f;
constexpr float PID_INTERPRET_LIMIT_CM = 0.6f;
constexpr float PID_INTEGRAL_LIMIT = 40.0f;
constexpr float PID_SOURCE_HYSTERESIS_CM = 0.75f;
constexpr float IMU_PID_INTEGRAL_LIMIT = 120.0f;
constexpr uint16_t IMU_REFERENCE_SAMPLES = 64;
constexpr uint16_t IMU_REFERENCE_SAMPLE_DELAY_MS = 5;
constexpr float IMU_GYRO_ZERO_RATE_LEVEL_DPS = 1.0f;   // LSM6DSOX G_TyOff
constexpr float IMU_GYRO_RMS_NOISE_DPS = 0.075f;       // LSM6DSOX RnRMS
constexpr float IMU_GYRO_SENS_TOL = 0.01f;             // LSM6DSOX G_So%
constexpr float IMU_ACCEL_ZERO_G_OFFSET_G = 0.020f;    // LSM6DSOX LA_TyOff
constexpr float IMU_ACCEL_RMS_NOISE_G = 0.0018f;       // LSM6DSOX RMS @ +/-2 g
constexpr float IMU_REFERENCE_MIN_Z_G = 0.25f;
constexpr float IMU_CAPTURE_MAX_GYRO_DPS = 2.5f;       // Reject reference capture while the robot is still rotating.
constexpr float IMU_CAPTURE_MAX_NORM_ERR_G = 0.15f;    // Reject reference capture if the board is being shaken.
constexpr float IMU_GYRO_SOFT_ZONE_DPS = 0.35f;        // Low-rate smoothing zone to avoid stick-slip yaw integration.
constexpr float IMU_ERROR_FULL_SCALE_DEG = 45.0f;      // IMU PID works on normalized yaw error instead of raw degrees.
constexpr float IMU_OUTPUT_SLEW_PER_S = 2.0f;          // Limit yaw-hold output steps so the chassis does not snap.
constexpr float IMU_REANCHOR_HOLD_S = 0.35f;           // Briefly pause compensation after a fresh reference capture.
constexpr uint32_t IMU_SAMPLE_TIMEOUT_MS = 80;
constexpr uint8_t IMU_SAMPLE_POLL_DELAY_MS = 1;

// ======================================================
// Settings storage (fixed-point x100)
// ======================================================
struct RuntimeSettings {    
  int p_x1000 = 100;   //   
  int i_x1000 = 0;     //   
  int d_x1000 = 20;    //   
  // Global IMU usage switch. 1 = allowed everywhere, 0 = disabled everywhere.
  uint8_t imuEnabled = 0;
  int imuSign_x100 = 100;
  int imuP_x10000 = 0;
  int imuI_x10000 = 0;
  int imuD_x10000 = 0;
  int imuAngleThreshold_x100 = 100;
  int imuPidTestYawError_x100 = 300;
  int curveB_x100 = 100;
  int pidWallDistance_x100 = 600;
  int pidLeftScale_x100 = 100;
  int pidRightScale_x100 = 80;
  int overallSpeedScale_x100 = 100;
  int baseSpeed_x100 = 35;
  int approachSpeed_x100 = 28;
  int leftDriveScale_x100 = 100;
  int rightDriveScale_x100 = 100;

  int frontStopDistance_x100 = 400;
  int corridorWallThreshold_x100 = 600;
  int turnDetectDistance_x100 = 2600;
  int deadEndDistance_x100 = 1600;
  int finishDistance_x100 = 3000;
  int sensingInterval_x100 = 10;
  int eventConfirmCount = 3;
  int ultrasonicAvgCount = 5;
  int waitBeforeTurn_x100 = 500;

  uint8_t routeMode = 1;
  uint8_t startHeadingIndex = 0;
};

struct RuntimeConfig {
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
  bool imuEnabled = false;
  float imuSign = 1.0f;
  float imuP = 0.0f;
  float imuI = 0.0f;
  float imuD = 0.0f;
  float imuAngleThresholdDeg = 1.0f;
  float imuPidTestYawErrorDeg = 3.0f;
  float curveB = 1.0f;
  float pidWallDistanceCm = 6.0f;
  float pidLeftScale = 1.0f;
  float pidRightScale = 1.0f;
  float overallSpeedScale = 1.0f;
  float baseSpeed = 0.35f;
  float approachSpeed = 0.28f;
  float leftDriveScale = 1.0f;
  float rightDriveScale = 1.0f;

  float frontStopDistanceCm = 4.0f;
  float corridorWallThresholdCm = 6.0f;
  float turnDetectDistanceCm = 26.0f;
  float deadEndDistanceCm = 16.0f;
  float finishDistanceCm = 30.0f;
  float sensingIntervalS = 0.10f;
  uint8_t eventConfirmCount = 3;
  uint8_t ultrasonicAvgCount = 5;
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
  IMUPIDMenu,
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
  uint32_t ultrasonicSampleId = 0;
  uint8_t stableIrMask = 0;
  RelativeIrState ir;
  float north = 0.0f;
  float east = 0.0f;
  float south = 0.0f;
  float west = 0.0f;
  float northRaw = 0.0f;
  float eastRaw = 0.0f;
  float southRaw = 0.0f;
  float westRaw = 0.0f;
  float front = 0.0f;
  float left = 0.0f;
  float right = 0.0f;
  float back = 0.0f;
  float frontRaw = 0.0f;
  float leftRaw = 0.0f;
  float rightRaw = 0.0f;
  float backRaw = 0.0f;
  EventType candidate = EventType::SensorWait;
  bool corridorSignature = false;
  PidSource pidSource = PidSource::None;
};

struct EventConfirmState {
  bool active = false;
  EventType candidate = EventType::Idle;
  uint8_t matches = 0;
  uint32_t nextCheckMs = 0;
  uint32_t lastSampleId = 0;
};

struct CorridorConfirmState {
  bool active = false;
  uint8_t matches = 0;
  uint32_t nextCheckMs = 0;
  uint32_t lastSampleId = 0;
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
  OverallSpeed,
  FrontStop,
  WallThreshold,
  TurnDetect,
  DeadEnd,
  Finish,
  TurnWait,
  CheckGap,
  MatchCount,
  WallP,
  WallI,
  WallD,
  CurveB,
  OneWallTarget,
  LeftOutput,
  RightOutput,
  ImuToggle,
  ImuP,
  ImuI,
  ImuD,
  ImuThreshold,
  ImuTestYawError,
  ImuSign,
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

struct WallReferenceState {
  // These remembered side-wall distances make the 1-wall PID inherit the
  // last real corridor geometry instead of snapping back to a fixed menu target.
  bool leftValid = false;
  bool rightValid = false;
  float leftCm = 0.0f;
  float rightCm = 0.0f;
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
  uint32_t compensationHoldUntilMs = 0;
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
  uint8_t integerDigits = 2;
  uint8_t decimals = 2;
  uint8_t digitCount = 4;
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

volatile int32_t gEncoderRawDelta = 0;
volatile uint8_t gEncoderLastState = 0;

ButtonState gButton;
DigitEditor gDigitEditor;
PlannerState gPlanner;
ExecutiveState gExecutive;
EventTestState gEventTestRun;
PidRuntime gPid;
WallReferenceState gWallRef;
ImuPidRuntime gImuPid;
ImuState gImu;
PidSource gActivePidSource = PidSource::None;
bool gImuPidTestArmed = false;

Screen gCurrentScreen = Screen::Root;
Screen gPreviousScreen = Screen::Root;

int gRootIndex = 0;
int gStartSettingsIndex = 0;
int gSettingsIndex = 0;
int gPidIndex = 0;
int gImuPidMenuIndex = 0;
int gMotorTuneIndex = 0;
int gMotionSettingsIndex = 0;
int gDetectionSettingsIndex = 0;
int gEventTestIndex = 0;
int gEventTestConfigIndex = 0;
int gIrSensorIndex = 0;
int8_t gEncoderStepCarry = 0;

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
uint8_t gStableIrMask = 0;

float gDistN = 0.0f;
float gDistE = 0.0f;
float gDistS = 0.0f;
float gDistW = 0.0f;
float gAvgN = 0.0f;
float gAvgE = 0.0f;
float gAvgS = 0.0f;
float gAvgW = 0.0f;
float gBufN[ULTRASONIC_AVG_COUNT_MAX] = {0};
float gBufE[ULTRASONIC_AVG_COUNT_MAX] = {0};
float gBufS[ULTRASONIC_AVG_COUNT_MAX] = {0};
float gBufW[ULTRASONIC_AVG_COUNT_MAX] = {0};
uint8_t gBufIndex = 0;
uint8_t gBufCount = 0;
uint32_t gLastUltrasonicMs = 0;
uint32_t gUltrasonicSampleId = 0;
bool gUltrasonicValid = false;

char gUartLine[96];
uint8_t gUartPos = 0;
uint32_t gLastUiDrawMs = 0;
Screen gLastDrawnScreen = Screen::RunScreen;

// ======================================================
// Forward declarations
// ======================================================
void onEncoderEdge();

void commitRuntimeConfig();
void resetPid();
void resetDrivePidControl();
void resetImuPid();
bool imuHardwareReady();
bool imuUsageEnabled();
void clearImuReference();
void holdImuCompensation(float seconds);
bool imuCaptureSampleLooksStill(float ax, float ay, float az, float gx, float gy, float gz);

float fixedToFloat2(int value);
float fixedToFloat3(int value);
float fixedToFloat4(int value);
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
float currentImuRotationCompensation(ImuPidRuntime& runtime);
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
void resetWallReference();
void updateWallReference(const PerceptionFrame& frame);
PidSource stabilizedPidSource(const PerceptionFrame& frame, PidSource previousSource);
void writeServoCommand(Servo& servo, int stopAngle, int rangeAngle, int invert, float cmd);
void stopAll();
void driveRobotFrame(float lateralCmd, float forwardCmd, float rotationCmd, Heading heading);
void followSmart(Heading heading, float forwardSpeed);

void beginExecution();
void finishExecution();
void updateExecutive();

ButtonEvent readButtonEvent(bool allowInput);
void openDigitEditor(const char* label, const char* unit, int* target, int minValue, int maxValue,
                     const char* desc1, const char* desc2, Screen returnScreen,
                     uint8_t integerDigits = 2, uint8_t decimals = 2);
void closeDigitEditor(bool commitValue);
void handleDigitEditorInput(int encoderDelta, ButtonEvent buttonEvent);
void handleIdleUiInput(int encoderDelta, ButtonEvent buttonEvent);
void handleRootInput(int encoderDelta, ButtonEvent buttonEvent);
void handleStartSettingsInput(int encoderDelta, ButtonEvent buttonEvent);
void handleSettingsInput(int encoderDelta, ButtonEvent buttonEvent);
void handlePidInput(int encoderDelta, ButtonEvent buttonEvent);
void handleImuPidMenuInput(int encoderDelta, ButtonEvent buttonEvent);
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
void formatCompactValue(char* buffer, size_t size, int value, uint8_t decimals);
void formatPaddedValue(char* buffer, size_t size, int value, uint8_t integerDigits, uint8_t decimals);
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
void drawImuPidMenuScreen();
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
float fixedToFloat2(int value) {
  return value / 100.0f;
}

float fixedToFloat3(int value) {
  return value / 1000.0f;
}

float fixedToFloat4(int value) {
  return value / 10000.0f;
}

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
  uint8_t previousAvgCount = gCfg.ultrasonicAvgCount;
  activeSettings = workingSettings;

  gCfg.p = fixedToFloat3(activeSettings.p_x1000);
  gCfg.i = fixedToFloat3(activeSettings.i_x1000);
  gCfg.d = fixedToFloat3(activeSettings.d_x1000);
  gCfg.imuEnabled = (activeSettings.imuEnabled != 0);
  gCfg.imuSign = (activeSettings.imuSign_x100 < 0) ? -1.0f : 1.0f;
  gCfg.imuP = fixedToFloat4(activeSettings.imuP_x10000);
  gCfg.imuI = fixedToFloat4(activeSettings.imuI_x10000);
  gCfg.imuD = fixedToFloat4(activeSettings.imuD_x10000);
  gCfg.imuAngleThresholdDeg = fixedToFloat2(activeSettings.imuAngleThreshold_x100);
  gCfg.imuPidTestYawErrorDeg = fixedToFloat2(activeSettings.imuPidTestYawError_x100);
  gCfg.curveB = clampFloat(fixedToFloat2(activeSettings.curveB_x100), 0.10f, 20.0f);
  gCfg.pidWallDistanceCm = fixedToFloat2(activeSettings.pidWallDistance_x100);
  gCfg.pidLeftScale = fixedToFloat2(activeSettings.pidLeftScale_x100);
  gCfg.pidRightScale = fixedToFloat2(activeSettings.pidRightScale_x100);
  gCfg.overallSpeedScale = clampFloat(fixedToFloat2(activeSettings.overallSpeedScale_x100), 0.0f, 1.0f);
  gCfg.baseSpeed = clampFloat(fixedToFloat2(activeSettings.baseSpeed_x100), 0.0f, 1.0f);
  gCfg.approachSpeed = clampFloat(fixedToFloat2(activeSettings.approachSpeed_x100), 0.0f, 1.0f);
  gCfg.leftDriveScale = clampFloat(fixedToFloat2(activeSettings.leftDriveScale_x100), 0.10f, 3.0f);
  gCfg.rightDriveScale = clampFloat(fixedToFloat2(activeSettings.rightDriveScale_x100), 0.10f, 3.0f);

  gCfg.frontStopDistanceCm = fixedToFloat2(activeSettings.frontStopDistance_x100);
  gCfg.corridorWallThresholdCm = fixedToFloat2(activeSettings.corridorWallThreshold_x100);
  gCfg.turnDetectDistanceCm = fixedToFloat2(activeSettings.turnDetectDistance_x100);
  gCfg.deadEndDistanceCm = fixedToFloat2(activeSettings.deadEndDistance_x100);
  gCfg.finishDistanceCm = fixedToFloat2(activeSettings.finishDistance_x100);
  gCfg.sensingIntervalS = clampFloat(fixedToFloat2(activeSettings.sensingInterval_x100), 0.01f, 10.0f);
  gCfg.eventConfirmCount = (uint8_t)clampInt(activeSettings.eventConfirmCount, 1, 9);
  gCfg.ultrasonicAvgCount = (uint8_t)clampInt(activeSettings.ultrasonicAvgCount, 1, ULTRASONIC_AVG_COUNT_MAX);
  gCfg.waitBeforeTurnS = clampFloat(fixedToFloat2(activeSettings.waitBeforeTurn_x100), 0.0f, 30.0f);

  if (gCfg.ultrasonicAvgCount != previousAvgCount) {
    gBufCount = 0;
    gBufIndex = 0;
    gAvgN = 0.0f;
    gAvgE = 0.0f;
    gAvgS = 0.0f;
    gAvgW = 0.0f;
  }

  // Turning IMU usage off must immediately remove its influence everywhere.
  if (!gCfg.imuEnabled) {
    clearImuReference();
    resetImuPid();
    gImuPidTestArmed = false;
  }
}

void resetPid() {
  gPid.previousError = 0.0f;
  gPid.integral = 0.0f;
  gPid.previousMs = millis();
}

void resetDrivePidControl() {
  // A wall-source change means the PID error now has a different physical meaning,
  // so both controller memory and the held source selection need a clean reset.
  resetPid();
  gActivePidSource = PidSource::None;
}

void resetWallReference() {
  gWallRef.leftValid = false;
  gWallRef.rightValid = false;
  gWallRef.leftCm = 0.0f;
  gWallRef.rightCm = 0.0f;
}

void updateWallReference(const PerceptionFrame& frame) {
  // Smoothly remember the last reliable side-wall distance so a 2-wall -> 1-wall
  // transition does not suddenly jump to a fixed target from the menu.
  constexpr float memoryBlend = 0.25f;

  if (!frame.ultrasonicValid) return;

  if (frame.left <= gCfg.corridorWallThresholdCm) {
    if (!gWallRef.leftValid) {
      gWallRef.leftCm = frame.left;
      gWallRef.leftValid = true;
    } else {
      gWallRef.leftCm = (1.0f - memoryBlend) * gWallRef.leftCm + memoryBlend * frame.left;
    }
  }

  if (frame.right <= gCfg.corridorWallThresholdCm) {
    if (!gWallRef.rightValid) {
      gWallRef.rightCm = frame.right;
      gWallRef.rightValid = true;
    } else {
      gWallRef.rightCm = (1.0f - memoryBlend) * gWallRef.rightCm + memoryBlend * frame.right;
    }
  }
}

static bool wallStillPresent(float distanceCm, bool wasPresent) {
  float thresholdCm = wasPresent
                        ? (gCfg.corridorWallThresholdCm + PID_SOURCE_HYSTERESIS_CM)
                        : gCfg.corridorWallThresholdCm;
  return distanceCm <= thresholdCm;
}

PidSource stabilizedPidSource(const PerceptionFrame& frame, PidSource previousSource) {
  if (!frame.ultrasonicValid) {
    return PidSource::None;
  }

  bool previousLeftWall =
    previousSource == PidSource::LeftWall || previousSource == PidSource::TwoWall;
  bool previousRightWall =
    previousSource == PidSource::RightWall || previousSource == PidSource::TwoWall;

  bool leftWall = wallStillPresent(frame.left, previousLeftWall);
  bool rightWall = wallStillPresent(frame.right, previousRightWall);

  if (leftWall && rightWall) return PidSource::TwoWall;
  if (leftWall) return PidSource::LeftWall;
  if (rightWall) return PidSource::RightWall;
  return PidSource::None;
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
  gImu.compensationHoldUntilMs = 0;
}

void holdImuCompensation(float seconds) {
  // A short hold keeps the controller from fighting a heading that we just changed on purpose.
  uint32_t holdUntilMs = millis() + secondsToMs(seconds);
  if (holdUntilMs > gImu.compensationHoldUntilMs) {
    gImu.compensationHoldUntilMs = holdUntilMs;
  }
  resetImuPid();
}

bool imuCaptureSampleLooksStill(float ax, float ay, float az, float gx, float gy, float gz) {
  // Reference capture is only trusted while the rover is stationary enough that gyro bias is meaningful.
  float accelNormG = sqrtf(ax * ax + ay * ay + az * az);
  float gyroNormDps = sqrtf(gx * gx + gy * gy + gz * gz);
  return fabsf(accelNormG - 1.0f) <= IMU_CAPTURE_MAX_NORM_ERR_G &&
         gyroNormDps <= IMU_CAPTURE_MAX_GYRO_DPS;
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
  float ultrasonicByIndex[4] = {
    (gBufCount > 0) ? gAvgN : gDistN,
    (gBufCount > 0) ? gAvgE : gDistE,
    (gBufCount > 0) ? gAvgS : gDistS,
    (gBufCount > 0) ? gAvgW : gDistW
  };
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

const char* nextTrialModeToString() {
  return routeModeToString(effectiveRouteMode());
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

static int pow10i(uint8_t exponent) {
  int value = 1;
  for (uint8_t i = 0; i < exponent; ++i) {
    value *= 10;
  }
  return value;
}

void formatCompactValue(char* buffer, size_t size, int value, uint8_t decimals) {
  int scale = pow10i(decimals);
  int safeValue = max(value, 0);
  int whole = (scale > 0) ? (safeValue / scale) : safeValue;
  int fraction = (scale > 0) ? abs(safeValue % scale) : 0;

  if (decimals == 0) {
    snprintf(buffer, size, "%d", safeValue);
  } else {
    snprintf(buffer, size, "%d.%0*d", whole, decimals, fraction);
  }
}

void formatPaddedValue(char* buffer, size_t size, int value, uint8_t integerDigits, uint8_t decimals) {
  int scale = pow10i(decimals);
  int safeValue = max(value, 0);
  int whole = (scale > 0) ? (safeValue / scale) : safeValue;
  int fraction = (scale > 0) ? abs(safeValue % scale) : 0;

  if (decimals == 0) {
    snprintf(buffer, size, "%0*d", integerDigits, whole);
  } else {
    snprintf(buffer, size, "%0*d.%0*d", integerDigits, whole, decimals, fraction);
  }
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
  uint8_t a = (uint8_t)digitalRead(ENC_A_PIN);
  uint8_t b = (uint8_t)digitalRead(ENC_B_PIN);
  uint8_t state = (a << 1) | b;
  uint8_t transition = (gEncoderLastState << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      gEncoderRawDelta++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      gEncoderRawDelta--;
      break;
    default:
      break;
  }

  gEncoderLastState = state;
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
                     const char* desc1, const char* desc2, Screen returnScreen,
                     uint8_t integerDigits, uint8_t decimals) {
  gDigitEditor.active = true;
  gDigitEditor.digitUnlocked = false;
  gDigitEditor.selectedDigit = 0;
  gDigitEditor.integerDigits = integerDigits;
  gDigitEditor.decimals = decimals;
  gDigitEditor.digitCount = integerDigits + decimals;
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
  int exponent = (int)gDigitEditor.digitCount - 1 - (int)selectedDigit;
  return pow10i((uint8_t)max(exponent, 0));
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
      while (nextDigit < 0) nextDigit += gDigitEditor.digitCount;
      while (nextDigit >= gDigitEditor.digitCount) nextDigit -= gDigitEditor.digitCount;
      gDigitEditor.selectedDigit = (uint8_t)nextDigit;
    } else {
      int digit = getDigitAt(gDigitEditor.tempValue, gDigitEditor.selectedDigit);
      digit += encoderDelta;
      while (digit < 0) digit += 10;
      while (digit > 9) digit -= 10;
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
  gStableIrMask = gRawIrMask;
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
  uint8_t avgCount = max((int)gCfg.ultrasonicAvgCount, 1);
  gBufN[gBufIndex] = north;
  gBufE[gBufIndex] = east;
  gBufS[gBufIndex] = south;
  gBufW[gBufIndex] = west;

  gBufIndex = (uint8_t)((gBufIndex + 1) % avgCount);
  if (gBufCount < avgCount) {
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
        // UART sender uses D1=N, D2=E, D3=S, D4=W.
        gDistN = d1;
        gDistE = d2;
        gDistS = d3;
        gDistW = d4;
        pushUltrasonicAverages(gDistN, gDistE, gDistS, gDistW);
        gUltrasonicSampleId++;
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

void initImu() {
  // Hardware probing is runtime-only now. Menu toggle decides whether
  // the IMU is used, but we still try to initialize the onboard device once.
  gImu.available = IMU.begin();
  if (!gImu.available) {
    //Serial.println("IMU init failed. LSM6DSOX yaw hold disabled.");
    return;
  }
  //Serial.println("IMU ready. LSM6DSOX runtime control enabled.");
}

static bool sampleImuBlocking(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint32_t startMs = millis();
  while ((millis() - startMs) < IMU_SAMPLE_TIMEOUT_MS) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      return true;
    }
    delay(IMU_SAMPLE_POLL_DELAY_MS);
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
    gImu.yawErrorDeg = 0.0f;
    gImu.measurementAngleTolDeg = 0.0f;
    gImu.combinedDeadbandDeg = 0.0f;
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

    float yawRateDps = (gImu.gz - gImu.gyroBiasZDps) * gCfg.imuSign;
    float absYawRateDps = fabsf(yawRateDps);
    if (absYawRateDps < IMU_GYRO_SOFT_ZONE_DPS) {
      // Use a smooth low-rate taper instead of a hard dead zone so slow real turns do not accumulate late.
      float scale = absYawRateDps / IMU_GYRO_SOFT_ZONE_DPS;
      yawRateDps *= scale * scale;
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

  // Allow extra attempts so we can reject moving samples instead of storing a bad reference.
  for (uint16_t i = 0; i < IMU_REFERENCE_SAMPLES * 3 && goodSamples < IMU_REFERENCE_SAMPLES; ++i) {
    float ax, ay, az, gx, gy, gz;
    if (!sampleImuBlocking(ax, ay, az, gx, gy, gz)) {
      continue;
    }

    if (!imuCaptureSampleLooksStill(ax, ay, az, gx, gy, gz)) {
      delay(IMU_REFERENCE_SAMPLE_DELAY_MS);
      continue;
    }

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGz += gz;
    goodSamples++;
    delay(IMU_REFERENCE_SAMPLE_DELAY_MS);
  }

  if (goodSamples < (IMU_REFERENCE_SAMPLES / 2)) {
    gImu.reference.valid = false;
    gImu.referenceValid = false;
    //Serial.println("IMU reference rejected. Keep robot still during capture.");
    return false;
  }

  float sampleScale = 1.0f / goodSamples;
  gImu.reference.avgAx = sumAx * sampleScale;
  gImu.reference.avgAy = sumAy * sampleScale;
  gImu.reference.avgAz = sumAz * sampleScale;
  gImu.gyroBiasZDps = sumGz * sampleScale;
  // Re-zero yaw at capture time so the controller works on fresh relative rotation only.
  gImu.yawDeg = 0.0f;
  gImu.reference.yawDeg = 0.0f;
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
  holdImuCompensation(IMU_REANCHOR_HOLD_S);
  updateImuDerivedState();

  //Serial.print("IMU reference saved. biasZ[dps]=");
  //Serial.print(gImu.gyroBiasZDps, 3);
  //Serial.print(" deadbandBase[deg]=");
  //Serial.println(gImu.reference.angleTolDeg, 3);
  return true;
}

void refreshSensors() {
  readIRSensors();
  serviceUartSensorStream();
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
  // The displayed source uses the same averaged ultrasonic geometry as the
  // runtime event and wall-following logic. The live controller still adds
  // extra hysteresis in stabilizedPidSource().
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

  float frontCm = frame.front;
  float leftCm = frame.left;
  float rightCm = frame.right;
  bool frontRightBlocked = frame.ir.frontRight;
  bool frontLeftBlocked = frame.ir.frontLeft;
  bool oneFrontBlocked = (frontRightBlocked != frontLeftBlocked);
  bool bothFrontBlocked = frontRightBlocked && frontLeftBlocked;
  bool bothFrontOpen = !frontRightBlocked && !frontLeftBlocked;
  bool rearOpen = rearPairOpen(frame.ir);
  bool leftOpenByUs = leftCm >= gCfg.corridorWallThresholdCm;
  bool rightOpenByUs = rightCm >= gCfg.corridorWallThresholdCm;

  if (frontCm <= gCfg.frontStopDistanceCm) {
    return EventType::SafetyStop;
  }

  if (frontCm <= gCfg.deadEndDistanceCm) {
    if (frontRightBlocked) {
      if (frontLeftBlocked) return EventType::DeadEnd;
      return EventType::LeftTurn;
    }
    if (frontLeftBlocked) return EventType::RightTurn;
    return EventType::TJunction;
  }

  if (frontCm <= gCfg.turnDetectDistanceCm) {
    if (frontRightBlocked) {
      if (frontLeftBlocked) return EventType::Corridor;
      return EventType::LeftTurn;
    }
    if (frontLeftBlocked) return EventType::RightTurn;
    return EventType::TJunction;
  }

  if (bothFrontBlocked) {
    return EventType::Corridor;
  }

  if (bothFrontOpen) {
    if (rearOpen &&
        leftOpenByUs &&
        rightOpenByUs &&
        frontCm >= gCfg.finishDistanceCm) {
      return EventType::Finish;
    }
    return EventType::TJunction;
  }

  if (oneFrontBlocked && frontCm > gCfg.turnDetectDistanceCm) {
    return EventType::TJunctionStraight;
  }

  return EventType::Corridor;
}

PerceptionFrame buildPerceptionFrame(Heading heading) {
  PerceptionFrame frame;
  frame.ultrasonicValid = ultrasonicFresh();
  frame.ultrasonicSampleId = gUltrasonicSampleId;
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
  // Keep the event path fully aligned with the older v37 behavior: every
  // consumer sees the same averaged ultrasonic geometry instead of mixing
  // raw detection with filtered handling.
  frame.northRaw = frame.north;
  frame.eastRaw = frame.east;
  frame.southRaw = frame.south;
  frame.westRaw = frame.west;
  frame.frontRaw = frame.front;
  frame.leftRaw = frame.left;
  frame.rightRaw = frame.right;
  frame.backRaw = frame.back;

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
  gExecutive.eventConfirm.lastSampleId = 0;
}

void resetCorridorConfirmation() {
  gExecutive.corridorConfirm.active = false;
  gExecutive.corridorConfirm.matches = 0;
  gExecutive.corridorConfirm.nextCheckMs = 0;
  gExecutive.corridorConfirm.lastSampleId = 0;
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
    case PidSource::LeftWall: {
      float targetLeftCm = gWallRef.leftValid ? gWallRef.leftCm : gCfg.pidWallDistanceCm;
      return targetLeftCm - frame.left;
    }
    case PidSource::RightWall: {
      float targetRightCm = gWallRef.rightValid ? gWallRef.rightCm : gCfg.pidWallDistanceCm;
      return frame.right - targetRightCm;
    }
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

  if (now < gImu.compensationHoldUntilMs) {
    // Intentional turns get a short quiet window so the IMU controller does not kick during settling.
    runtime.previousError = 0.0f;
    runtime.integral *= 0.80f;
    gImu.lastCompensationCmd = 0.0f;
    return 0.0f;
  }

  if (fabsf(errorDeg) <= deadbandDeg) {
    runtime.previousError = 0.0f;
    runtime.integral *= 0.85f;
    gImu.lastCompensationCmd = 0.0f;
    return 0.0f;
  }

  float effectiveErrorDeg = (errorDeg > 0.0f)
                              ? (errorDeg - deadbandDeg)
                              : (errorDeg + deadbandDeg);
  float normalizedError = effectiveErrorDeg / IMU_ERROR_FULL_SCALE_DEG;
  float derivative = (normalizedError - runtime.previousError) / dt;

  runtime.integral += normalizedError * dt;
  runtime.integral = clampFloat(runtime.integral, -IMU_PID_INTEGRAL_LIMIT, IMU_PID_INTEGRAL_LIMIT);

  float output = gCfg.imuP * normalizedError +
                 gCfg.imuI * runtime.integral +
                 gCfg.imuD * derivative;

  runtime.previousError = normalizedError;
  float maxDelta = IMU_OUTPUT_SLEW_PER_S * dt;
  output = clampFloat(output,
                      gImu.lastCompensationCmd - maxDelta,
                      gImu.lastCompensationCmd + maxDelta);
  output = clampFloat(output, -1.0f, 1.0f);
  gImu.lastCompensationCmd = output;
  return output;
}

float currentImuRotationCompensation(ImuPidRuntime& runtime) {
  // Keep the IMU-off path explicit everywhere so PID tuning cannot accidentally
  // inherit stale rotational correction from a previous enabled state.
  if (!imuUsageEnabled()) {
    runtime.previousError = 0.0f;
    runtime.integral = 0.0f;
    runtime.previousMs = millis();
    gImu.lastCompensationCmd = 0.0f;
    return 0.0f;
  }

  return applyImuCompensation(runtime);
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

static void normalizeWheelEnvelope(float& fl, float& fr, float& rr, float& rl) {
  float maxAbs =
    max(max(fabsf(fl), fabsf(fr)),
        max(fabsf(rr), fabsf(rl)));

  if (maxAbs > 1.0f) {
    float scale = 1.0f / maxAbs;
    fl *= scale;
    fr *= scale;
    rr *= scale;
    rl *= scale;
  }
}

static float clampTowardZero(float candidate, float baseCmd) {
  if (baseCmd > 0.0f) {
    return clampFloat(candidate, 0.0f, baseCmd);
  }
  if (baseCmd < 0.0f) {
    return clampFloat(candidate, baseCmd, 0.0f);
  }
  return candidate;
}

/*
  Limited base-motion mixer
  -------------------------
  Base translation comes from the commanded travel direction only and is scaled
  by the global speed factor. Lateral/rotational compensation is then allowed
  to reduce that base motion toward zero, but not reverse it while the robot is
  already translating. Pure rotation is still allowed whenever the base command
  for a wheel is zero, which keeps the isolated IMU PID test working.
*/
static void driveBodyFrameLimited(float baseVx, float baseVy,
                                  float compVx, float compVy,
                                  float w) {
  float basePlus = baseVy + baseVx;
  float baseMinus = baseVy - baseVx;
  w = clampUnit(w);

  float baseTranslationMax =
    max(fabsf(basePlus) * gCfg.leftDriveScale,
        fabsf(baseMinus) * gCfg.rightDriveScale);
  if (baseTranslationMax > 1.0f) {
    float baseScale = 1.0f / baseTranslationMax;
    basePlus *= baseScale;
    baseMinus *= baseScale;
  }

  basePlus *= gCfg.overallSpeedScale;
  baseMinus *= gCfg.overallSpeedScale;

  float compPlus = compVy + compVx;
  float compMinus = compVy - compVx;

  float pairPlus = clampTowardZero(basePlus + compPlus, basePlus);
  float pairMinus = clampTowardZero(baseMinus + compMinus, baseMinus);

  float flBase = pairPlus * gCfg.leftDriveScale;
  float rrBase = pairPlus * gCfg.leftDriveScale;
  float frBase = pairMinus * gCfg.rightDriveScale;
  float rlBase = pairMinus * gCfg.rightDriveScale;

  float fl = flBase + w * gCfg.leftDriveScale;
  float rr = rrBase - w * gCfg.leftDriveScale;
  float fr = frBase - w * gCfg.rightDriveScale;
  float rl = rlBase + w * gCfg.rightDriveScale;

  if (fabsf(flBase) > 0.0001f) fl = clampTowardZero(fl, flBase);
  if (fabsf(rrBase) > 0.0001f) rr = clampTowardZero(rr, rrBase);
  if (fabsf(frBase) > 0.0001f) fr = clampTowardZero(fr, frBase);
  if (fabsf(rlBase) > 0.0001f) rl = clampTowardZero(rl, rlBase);

  normalizeWheelEnvelope(fl, fr, rr, rl);

  writeServoCommand(servoFL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeServoCommand(servoFR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeServoCommand(servoRR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeServoCommand(servoRL, STOP_RL, RANGE_RL, INV_RL, rl);
}

/*
  45 degree omni X-drive layout
  -----------------------------
  FL and RR share the +45 degree diagonal, FR and RL share the -45 degree diagonal.

    u+ = vy + vx   -> FL, RR translation term
    u- = vy - vx   -> FR, RL translation term

  Rotational compensation is injected as an orthogonal spin pattern:

    spin = [+w, -w, -w, +w]

  Robust saturation policy:
  1. Keep pure translation inside the wheel envelope first.
  2. Add rotational compensation on top.
  3. Renormalize the final four wheel commands together if needed.

  This avoids impossible translation requests and keeps partial yaw authority
  instead of dropping it to zero when the drive is heavily loaded.
*/
static void driveBodyFrame(float vx, float vy, float w) {
  float basePlus = vy + vx;
  float baseMinus = vy - vx;
  w = clampUnit(w);

  float translationMax =
    max(fabsf(basePlus) * gCfg.leftDriveScale,
        fabsf(baseMinus) * gCfg.rightDriveScale);
  if (translationMax > 1.0f) {
    float translationScale = 1.0f / translationMax;
    basePlus *= translationScale;
    baseMinus *= translationScale;
  }

  float fl = (basePlus + w) * gCfg.leftDriveScale;
  float rr = (basePlus - w) * gCfg.leftDriveScale;
  float fr = (baseMinus - w) * gCfg.rightDriveScale;
  float rl = (baseMinus + w) * gCfg.rightDriveScale;

  normalizeWheelEnvelope(fl, fr, rr, rl);

  writeServoCommand(servoFL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeServoCommand(servoFR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeServoCommand(servoRR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeServoCommand(servoRL, STOP_RL, RANGE_RL, INV_RL, rl);
}

void driveRobotFrame(float lateralCmd, float forwardCmd, float rotationCmd, Heading heading) {
  float baseVx = 0.0f;
  float baseVy = 0.0f;
  float compVx = 0.0f;
  float compVy = 0.0f;

  switch (heading) {
    case Heading::North:
      baseVy = forwardCmd;
      compVx = lateralCmd;
      break;
    case Heading::East:
      baseVx = forwardCmd;
      compVy = -lateralCmd;
      break;
    case Heading::South:
      baseVy = -forwardCmd;
      compVx = -lateralCmd;
      break;
    case Heading::West:
      baseVx = -forwardCmd;
      compVy = lateralCmd;
      break;
  }

  driveBodyFrameLimited(baseVx, baseVy, compVx, compVy, rotationCmd);
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
    resetDrivePidControl();
    stopAll();
    return;
  }

  updateWallReference(frame);
  PidSource controlSource = stabilizedPidSource(frame, gActivePidSource);
  if (controlSource != gActivePidSource) {
    gActivePidSource = controlSource;
    resetPid();
  }

  float correction = 0.0f;
  if (controlSource != PidSource::None) {
    correction = computePid(computeWallError(frame, controlSource));
  }

  float rotationComp = currentImuRotationCompensation(gImuPid);
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
  resetWallReference();
  gImuPidTestArmed = false;

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
  resetDrivePidControl();
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
  resetEventConfirmation();
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
          resetDrivePidControl();
          enterPhase(NavState::AcquireCorridor, EventType::DeadEnd);
          return;
        }

        if (confirmed == EventType::TJunctionStraight) {
          resetDrivePidControl();
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
        resetDrivePidControl();
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
        resetDrivePidControl();
        enterPhase(NavState::Corridor, EventType::Corridor);
        return;
      }
      {
        PerceptionFrame finishFrame = frame;
        finishFrame.candidate = (frame.candidate == EventType::Finish) ? EventType::Finish : EventType::Idle;
        EventType finishConfirmed = EventType::Idle;
        if (updateEventConfirmation(finishFrame, finishConfirmed) && finishConfirmed == EventType::Finish) {
          finishExecution();
          return;
        }
      }
      followSmart(gExecutive.heading, gCfg.approachSpeed);
      return;

    case NavState::SafetyStop:
      gExecutive.displayEvent = EventType::SafetyStop;
      stopAll();
      if (updateCorridorConfirmation(frame)) {
        resetDrivePidControl();
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
    state.lastSampleId = 0;
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
    state.lastSampleId = 0;
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

  float rotationComp = currentImuRotationCompensation(gImuPid);
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
  resetWallReference();
  gImuPidTestArmed = false;
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
  resetDrivePidControl();
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
    case 0: gCurrentScreen = Screen::StartConfirm; break;
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
                      "Stop duration before", "changing heading", Screen::StartSettings, 2, 2);
      break;
    case 3:
      gCurrentScreen = Screen::Root;
      break;
    default:
      break;
  }
}

void handleSettingsInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 13;

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
    case 3: openDigitEditor("Overall spd", "x", &workingSettings.overallSpeedScale_x100, 0, 100, "Scales base travel", "keeps correction full", Screen::Settings, 1, 2); break;
    case 4: openDigitEditor("Front stop", "cm", &workingSettings.frontStopDistance_x100, 0, 9999, "Emergency stop if", "front gets too close", Screen::Settings, 2, 2); break;
    case 5: openDigitEditor("Wall thr", "cm", &workingSettings.corridorWallThreshold_x100, 0, 9999, "Wall limit for PID", "and finish detection", Screen::Settings, 2, 2); break;
    case 6: openDigitEditor("Turn detect", "cm", &workingSettings.turnDetectDistance_x100, 0, 9999, "Upper front range", "that still means turn", Screen::Settings, 2, 2); break;
    case 7: openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999, "Front distance limit", "for dead-end event", Screen::Settings, 2, 2); break;
    case 8: openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999, "Front distance needed", "before finish counts", Screen::Settings, 2, 2); break;
    case 9: openDigitEditor("Check gap", "s", &workingSettings.sensingInterval_x100, 1, 9999, "Time between event", "verification checks", Screen::Settings, 2, 2); break;
    case 10: openDigitEditor("US avg k", "x", &workingSettings.ultrasonicAvgCount, 1, ULTRASONIC_AVG_COUNT_MAX, "How many ultrasonic", "samples are averaged", Screen::Settings, 1, 0); break;
    case 11: openDigitEditor("Match N", "x", &workingSettings.eventConfirmCount, 1, 9, "How many identical", "events confirm state", Screen::Settings, 1, 0); break;
    case 12: gCurrentScreen = Screen::Root; break;
    default: break;
  }
}

void handlePidInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 10;
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
    case 0: openDigitEditor("P gain", "x", &workingSettings.p_x1000, 0, 9999, "Proportional reaction", "to wall error", Screen::PID, 1, 3); break;
    case 1: openDigitEditor("I gain", "x", &workingSettings.i_x1000, 0, 9999, "Integral correction", "for constant drift", Screen::PID, 1, 3); break;
    case 2: openDigitEditor("D gain", "x", &workingSettings.d_x1000, 0, 9999, "Derivative damping", "for fast changes", Screen::PID, 1, 3); break;
    case 3: openDigitEditor("Curve b", "x", &workingSettings.curveB_x100, 1, 9999, "Shapes small errors", "before PID acts", Screen::PID, 1, 2); break;
    case 4: openDigitEditor("1-wall", "cm", &workingSettings.pidWallDistance_x100, 0, 9999, "Target distance when", "only one wall exists", Screen::PID, 2, 2); break;
    case 5: openDigitEditor("Left out", "x", &workingSettings.pidLeftScale_x100, 10, 300, "Extra scale for", "left correction", Screen::PID, 1, 2); break;
    case 6: openDigitEditor("Right out", "x", &workingSettings.pidRightScale_x100, 10, 300, "Extra scale for", "right correction", Screen::PID, 1, 2); break;
    case 7: gPidTestHeading = headingFromIndex(activeSettings.startHeadingIndex); gCurrentScreen = Screen::PIDTest; break;
    case 8: gCurrentScreen = Screen::IMUPIDMenu; break;
    case 9: gCurrentScreen = Screen::Settings; break;
    default: break;
  }
}

void handleImuPidMenuInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 9;
  if (encoderDelta != 0) {
    gImuPidMenuIndex += encoderDelta;
    if (gImuPidMenuIndex < 0) gImuPidMenuIndex = 0;
    if (gImuPidMenuIndex >= itemCount) gImuPidMenuIndex = itemCount - 1;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::PID;
    return;
  }
  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gImuPidMenuIndex) {
    case 0:
      workingSettings.imuEnabled = workingSettings.imuEnabled ? 0 : 1;
      commitRuntimeConfig();
      break;
    case 1: openDigitEditor("IMU P", "x", &workingSettings.imuP_x10000, 0, 99999, "Yaw-hold proportional", "gain for rotation", Screen::IMUPIDMenu, 1, 4); break;
    case 2: openDigitEditor("IMU I", "x", &workingSettings.imuI_x10000, 0, 99999, "Yaw-hold integral", "gain for drift", Screen::IMUPIDMenu, 1, 4); break;
    case 3: openDigitEditor("IMU D", "x", &workingSettings.imuD_x10000, 0, 99999, "Yaw-hold derivative", "gain for damping", Screen::IMUPIDMenu, 1, 4); break;
    case 4: openDigitEditor("IMU thr", "deg", &workingSettings.imuAngleThreshold_x100, 0, 4500, "Extra angle above", "sensor deadband", Screen::IMUPIDMenu, 2, 2); break;
    case 5: openDigitEditor("Yaw err", "deg", &workingSettings.imuPidTestYawError_x100, 0, 4500, "If yaw stays inside", "this range stop", Screen::IMUPIDMenu, 2, 2); break;
    case 6:
      workingSettings.imuSign_x100 = (workingSettings.imuSign_x100 < 0) ? 100 : -100;
      commitRuntimeConfig();
      resetImuPid();
      break;
    case 7:
      gImuPidTestArmed = false;
      resetImuPid();
      stopAll();
      if (imuUsageEnabled()) {
        gImuPidTestArmed = captureImuReference();
      }
      gCurrentScreen = Screen::IMUPIDTest;
      break;
    case 8:
    default:
      gCurrentScreen = Screen::PID;
      break;
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
    gImuPidTestArmed = imuUsageEnabled() && captureImuReference();
    if (!gImuPidTestArmed) {
      resetImuPid();
      stopAll();
    }
  } else if (buttonEvent == ButtonEvent::LongPress) {
    gImuPidTestArmed = false;
    resetImuPid();
    stopAll();
    gCurrentScreen = Screen::IMUPIDMenu;
  }
}

void handleMotorTuneInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 11;
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
    switch (gMotorTuneIndex) {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
        gTuneMotion = (TuneMotion)gMotorTuneIndex;
        break;
      case 5:
        openDigitEditor("Overall spd", "x", &workingSettings.overallSpeedScale_x100, 0, 100,
                        "Scales base travel", "keeps correction full", Screen::MotorTune, 1, 2);
        break;
      case 6:
        openDigitEditor("Travel spd", "x", &workingSettings.baseSpeed_x100, 0, 100,
                        "Forward speed used", "for normal motion", Screen::MotorTune, 1, 2);
        break;
      case 7:
        openDigitEditor("Approach", "x", &workingSettings.approachSpeed_x100, 0, 100,
                        "Forward speed used", "near wall events", Screen::MotorTune, 1, 2);
        break;
      case 8:
        openDigitEditor("Left drv", "x", &workingSettings.leftDriveScale_x100, 10, 300,
                        "Left diagonal drive", "scale calibration", Screen::MotorTune, 1, 2);
        break;
      case 9:
        openDigitEditor("Right drv", "x", &workingSettings.rightDriveScale_x100, 10, 300,
                        "Right diagonal drive", "scale calibration", Screen::MotorTune, 1, 2);
        break;
      case 10:
      default:
        gTuneMotion = TuneMotion::Stop;
        stopAll();
        gCurrentScreen = Screen::Settings;
        break;
    }
  }
}

static bool eventConfigFieldRelevant(EventTestCase testCase, EventConfigField field) {
  switch (field) {
    case EventConfigField::Heading:
    case EventConfigField::OverallSpeed:
    case EventConfigField::WallThreshold:
    case EventConfigField::DeadEnd:
    case EventConfigField::Finish:
    case EventConfigField::CheckGap:
    case EventConfigField::MatchCount:
    case EventConfigField::WallP:
    case EventConfigField::WallI:
    case EventConfigField::WallD:
    case EventConfigField::CurveB:
    case EventConfigField::OneWallTarget:
    case EventConfigField::LeftOutput:
    case EventConfigField::RightOutput:
    case EventConfigField::ImuToggle:
    case EventConfigField::ImuP:
    case EventConfigField::ImuI:
    case EventConfigField::ImuD:
    case EventConfigField::ImuThreshold:
    case EventConfigField::ImuTestYawError:
    case EventConfigField::ImuSign:
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
    case EventConfigField::OverallSpeed: return "Scales base travel";
    case EventConfigField::FrontStop: return "Front distance to";
    case EventConfigField::WallThreshold: return "Wall presence limit";
    case EventConfigField::TurnDetect: return "Upper front range";
    case EventConfigField::DeadEnd: return "Front dead-end limit";
    case EventConfigField::Finish: return "Front finish limit";
    case EventConfigField::TurnWait: return "Wait before new";
    case EventConfigField::CheckGap: return "Time between";
    case EventConfigField::MatchCount: return "How many identical";
    case EventConfigField::WallP: return "Wall PID proportional";
    case EventConfigField::WallI: return "Wall PID integral";
    case EventConfigField::WallD: return "Wall PID derivative";
    case EventConfigField::CurveB: return "Wall error shaping";
    case EventConfigField::OneWallTarget: return "Fallback 1-wall";
    case EventConfigField::LeftOutput: return "Left correction";
    case EventConfigField::RightOutput: return "Right correction";
    case EventConfigField::ImuToggle: return "Enable or disable";
    case EventConfigField::ImuP: return "IMU proportional";
    case EventConfigField::ImuI: return "IMU integral";
    case EventConfigField::ImuD: return "IMU derivative";
    case EventConfigField::ImuThreshold: return "Extra yaw angle";
    case EventConfigField::ImuTestYawError: return "Yaw error window";
    case EventConfigField::ImuSign: return "Flip IMU rotation";
    case EventConfigField::Run: return "Start selected";
    case EventConfigField::Back:
    default: return "Return to event";
  }
}

static const char* eventConfigDesc2(EventConfigField field) {
  switch (field) {
    case EventConfigField::Heading: return "this test case.";
    case EventConfigField::TurnChoice: return "T-junction test.";
    case EventConfigField::OverallSpeed: return "not correction.";
    case EventConfigField::FrontStop: return "enter stop stage.";
    case EventConfigField::WallThreshold: return "for PID selection.";
    case EventConfigField::TurnDetect: return "that still means turn.";
    case EventConfigField::DeadEnd: return "before dead-end triggers.";
    case EventConfigField::Finish: return "before finish triggers.";
    case EventConfigField::TurnWait: return "heading command.";
    case EventConfigField::CheckGap: return "event rechecks [s].";
    case EventConfigField::MatchCount: return "reads must match.";
    case EventConfigField::WallP: return "gain for wall error.";
    case EventConfigField::WallI: return "gain for steady drift.";
    case EventConfigField::WallD: return "gain for damping.";
    case EventConfigField::CurveB: return "before PID acts.";
    case EventConfigField::OneWallTarget: return "target distance.";
    case EventConfigField::LeftOutput: return "output scale.";
    case EventConfigField::RightOutput: return "output scale.";
    case EventConfigField::ImuToggle: return "IMU PID globally.";
    case EventConfigField::ImuP: return "gain for yaw error.";
    case EventConfigField::ImuI: return "gain for yaw drift.";
    case EventConfigField::ImuD: return "gain for yaw damping.";
    case EventConfigField::ImuThreshold: return "above deadband.";
    case EventConfigField::ImuTestYawError: return "for pure yaw test.";
    case EventConfigField::ImuSign: return "direction globally.";
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
    case EventConfigField::OverallSpeed:
      openDigitEditor("Overall spd", "x", &workingSettings.overallSpeedScale_x100, 0, 100, eventConfigDesc1(EventConfigField::OverallSpeed), eventConfigDesc2(EventConfigField::OverallSpeed), Screen::EventTestConfig, 1, 2);
      break;
    case EventConfigField::FrontStop:
      openDigitEditor("Front stop", "cm", &workingSettings.frontStopDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::FrontStop), eventConfigDesc2(EventConfigField::FrontStop), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::WallThreshold:
      openDigitEditor("Wall thr", "cm", &workingSettings.corridorWallThreshold_x100, 0, 9999, eventConfigDesc1(EventConfigField::WallThreshold), eventConfigDesc2(EventConfigField::WallThreshold), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::TurnDetect:
      openDigitEditor("Turn detect", "cm", &workingSettings.turnDetectDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::TurnDetect), eventConfigDesc2(EventConfigField::TurnDetect), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::DeadEnd:
      openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::DeadEnd), eventConfigDesc2(EventConfigField::DeadEnd), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::Finish:
      openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::Finish), eventConfigDesc2(EventConfigField::Finish), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::TurnWait:
      openDigitEditor("Turn wait", "s", &workingSettings.waitBeforeTurn_x100, 0, 3000, eventConfigDesc1(EventConfigField::TurnWait), eventConfigDesc2(EventConfigField::TurnWait), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::CheckGap:
      openDigitEditor("Check gap", "s", &workingSettings.sensingInterval_x100, 1, 9999, eventConfigDesc1(EventConfigField::CheckGap), eventConfigDesc2(EventConfigField::CheckGap), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::MatchCount:
      openDigitEditor("Match N", "x", &workingSettings.eventConfirmCount, 1, 9, eventConfigDesc1(EventConfigField::MatchCount), eventConfigDesc2(EventConfigField::MatchCount), Screen::EventTestConfig, 1, 0);
      break;
    case EventConfigField::WallP:
      openDigitEditor("P gain", "x", &workingSettings.p_x1000, 0, 9999, eventConfigDesc1(EventConfigField::WallP), eventConfigDesc2(EventConfigField::WallP), Screen::EventTestConfig, 1, 3);
      break;
    case EventConfigField::WallI:
      openDigitEditor("I gain", "x", &workingSettings.i_x1000, 0, 9999, eventConfigDesc1(EventConfigField::WallI), eventConfigDesc2(EventConfigField::WallI), Screen::EventTestConfig, 1, 3);
      break;
    case EventConfigField::WallD:
      openDigitEditor("D gain", "x", &workingSettings.d_x1000, 0, 9999, eventConfigDesc1(EventConfigField::WallD), eventConfigDesc2(EventConfigField::WallD), Screen::EventTestConfig, 1, 3);
      break;
    case EventConfigField::CurveB:
      openDigitEditor("Curve b", "x", &workingSettings.curveB_x100, 1, 9999, eventConfigDesc1(EventConfigField::CurveB), eventConfigDesc2(EventConfigField::CurveB), Screen::EventTestConfig, 1, 2);
      break;
    case EventConfigField::OneWallTarget:
      openDigitEditor("1-wall", "cm", &workingSettings.pidWallDistance_x100, 0, 9999, eventConfigDesc1(EventConfigField::OneWallTarget), eventConfigDesc2(EventConfigField::OneWallTarget), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::LeftOutput:
      openDigitEditor("Left out", "x", &workingSettings.pidLeftScale_x100, 10, 300, eventConfigDesc1(EventConfigField::LeftOutput), eventConfigDesc2(EventConfigField::LeftOutput), Screen::EventTestConfig, 1, 2);
      break;
    case EventConfigField::RightOutput:
      openDigitEditor("Right out", "x", &workingSettings.pidRightScale_x100, 10, 300, eventConfigDesc1(EventConfigField::RightOutput), eventConfigDesc2(EventConfigField::RightOutput), Screen::EventTestConfig, 1, 2);
      break;
    case EventConfigField::ImuToggle:
      workingSettings.imuEnabled = workingSettings.imuEnabled ? 0 : 1;
      commitRuntimeConfig();
      break;
    case EventConfigField::ImuP:
      openDigitEditor("IMU P", "x", &workingSettings.imuP_x10000, 0, 99999, eventConfigDesc1(EventConfigField::ImuP), eventConfigDesc2(EventConfigField::ImuP), Screen::EventTestConfig, 1, 4);
      break;
    case EventConfigField::ImuI:
      openDigitEditor("IMU I", "x", &workingSettings.imuI_x10000, 0, 99999, eventConfigDesc1(EventConfigField::ImuI), eventConfigDesc2(EventConfigField::ImuI), Screen::EventTestConfig, 1, 4);
      break;
    case EventConfigField::ImuD:
      openDigitEditor("IMU D", "x", &workingSettings.imuD_x10000, 0, 99999, eventConfigDesc1(EventConfigField::ImuD), eventConfigDesc2(EventConfigField::ImuD), Screen::EventTestConfig, 1, 4);
      break;
    case EventConfigField::ImuThreshold:
      openDigitEditor("IMU thr", "deg", &workingSettings.imuAngleThreshold_x100, 0, 4500, eventConfigDesc1(EventConfigField::ImuThreshold), eventConfigDesc2(EventConfigField::ImuThreshold), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::ImuTestYawError:
      openDigitEditor("Yaw err", "deg", &workingSettings.imuPidTestYawError_x100, 0, 4500, eventConfigDesc1(EventConfigField::ImuTestYawError), eventConfigDesc2(EventConfigField::ImuTestYawError), Screen::EventTestConfig, 2, 2);
      break;
    case EventConfigField::ImuSign:
      workingSettings.imuSign_x100 = (workingSettings.imuSign_x100 < 0) ? 100 : -100;
      commitRuntimeConfig();
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
  constexpr int itemCount = 8;
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
  if (gIrSensorIndex == 4) openDigitEditor("Check gap", "s", &workingSettings.sensingInterval_x100, 1, 9999, "Time between event", "verification checks", Screen::IRSensorTest, 2, 2);
  if (gIrSensorIndex == 5) openDigitEditor("Match N", "x", &workingSettings.eventConfirmCount, 1, 9, "How many identical", "events confirm state", Screen::IRSensorTest, 1, 0);
  if (gIrSensorIndex == 6) openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999, "Front distance limit", "for dead-end event", Screen::IRSensorTest, 2, 2);
  if (gIrSensorIndex == 7) openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999, "Front distance needed", "before finish counts", Screen::IRSensorTest, 2, 2);
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
          resetDrivePidControl();
          startEventPhase(NavState::AcquireCorridor, EventType::DeadEnd);
          return;
        }
        if (confirmed == EventType::TJunctionStraight) {
          resetDrivePidControl();
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
        resetDrivePidControl();
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
      {
        EventType finishObserved = (frame.candidate == EventType::Finish) ? EventType::Finish : EventType::Idle;
        EventType finishConfirmed = EventType::Idle;
        if (updateEventConfirmState(gEventTestRun.eventConfirm, finishObserved, finishConfirmed) &&
            finishConfirmed == EventType::Finish) {
          gEventTestRun.state = NavState::Finished;
          gEventTestRun.displayEvent = EventType::Finish;
          stopAll();
          return;
        }
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
    case Screen::IMUPIDMenu:           handleImuPidMenuInput(encoderDelta, buttonEvent); break;
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
  char items[13][28];
  snprintf(items[0], sizeof(items[0]), "PID");
  snprintf(items[1], sizeof(items[1]), "Motor Tune");
  snprintf(items[2], sizeof(items[2]), "IMU:%s", activeSettings.imuEnabled ? "ON" : "OFF");
  char value[16];
  formatCompactValue(value, sizeof(value), activeSettings.overallSpeedScale_x100, 2);
  snprintf(items[3], sizeof(items[3]), "Overall:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.frontStopDistance_x100, 2);
  snprintf(items[4], sizeof(items[4]), "FrontStop:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.corridorWallThreshold_x100, 2);
  snprintf(items[5], sizeof(items[5]), "WallThr:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.turnDetectDistance_x100, 2);
  snprintf(items[6], sizeof(items[6]), "TurnDet:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.deadEndDistance_x100, 2);
  snprintf(items[7], sizeof(items[7]), "DeadEnd:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.finishDistance_x100, 2);
  snprintf(items[8], sizeof(items[8]), "Finish:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.sensingInterval_x100, 2);
  snprintf(items[9], sizeof(items[9]), "CheckGap:%s", value);
  snprintf(items[10], sizeof(items[10]), "USavg:%d", activeSettings.ultrasonicAvgCount);
  snprintf(items[11], sizeof(items[11]), "Match N:%d", activeSettings.eventConfirmCount);
  snprintf(items[12], sizeof(items[12]), "Back");
  const char* ptrs[13];
  for (int i = 0; i < 13; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 13, gSettingsIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: root");
}

void drawPIDScreen() {
  drawHeader("PID");
  char items[10][28];
  char value[16];
  formatCompactValue(value, sizeof(value), activeSettings.p_x1000, 3);
  snprintf(items[0], sizeof(items[0]), "P:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.i_x1000, 3);
  snprintf(items[1], sizeof(items[1]), "I:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.d_x1000, 3);
  snprintf(items[2], sizeof(items[2]), "D:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.curveB_x100, 2);
  snprintf(items[3], sizeof(items[3]), "Curve:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.pidWallDistance_x100, 2);
  snprintf(items[4], sizeof(items[4]), "1Wall:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.pidLeftScale_x100, 2);
  snprintf(items[5], sizeof(items[5]), "LeftOut:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.pidRightScale_x100, 2);
  snprintf(items[6], sizeof(items[6]), "RightOut:%s", value);
  snprintf(items[7], sizeof(items[7]), "PID Test");
  snprintf(items[8], sizeof(items[8]), "IMU PID");
  snprintf(items[9], sizeof(items[9]), "Back");
  const char* ptrs[10];
  for (int i = 0; i < 10; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 10, gPidIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: settings");
}

void drawImuPidMenuScreen() {
  drawHeader("IMU PID");
  char items[9][28];
  char value[16];
  snprintf(items[0], sizeof(items[0]), "IMU:%s", activeSettings.imuEnabled ? "ON" : "OFF");
  formatCompactValue(value, sizeof(value), activeSettings.imuP_x10000, 4);
  snprintf(items[1], sizeof(items[1]), "P:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.imuI_x10000, 4);
  snprintf(items[2], sizeof(items[2]), "I:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.imuD_x10000, 4);
  snprintf(items[3], sizeof(items[3]), "D:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.imuAngleThreshold_x100, 2);
  snprintf(items[4], sizeof(items[4]), "ExtraThr:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.imuPidTestYawError_x100, 2);
  snprintf(items[5], sizeof(items[5]), "YawErr:%s", value);
  snprintf(items[6], sizeof(items[6]), "Sign:%+d", (activeSettings.imuSign_x100 < 0) ? -1 : 1);
  snprintf(items[7], sizeof(items[7]), "IMU PID Test");
  snprintf(items[8], sizeof(items[8]), "Back");
  const char* ptrs[9];
  for (int i = 0; i < 9; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 9, gImuPidMenuIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: PID");
}

void drawPIDTestScreen() {
  drawHeader("PID Test");
  PerceptionFrame frame = buildPerceptionFrame(gPidTestHeading);
  updateWallReference(frame);
  PidSource displaySource = stabilizedPidSource(frame, frame.pidSource);
  float rawError = computeWallError(frame, displaySource);
  float shaped = shapeSignedError(rawError);
  u8g2.setFont(u8g2_font_5x7_tf);
  char line[32];
  snprintf(line, sizeof(line), "Dir:%s Src:%s", headingToString(gPidTestHeading), pidSourceToString(displaySource));
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
  snprintf(line, sizeof(line), "Armed:%c Ref:%c", gImuPidTestArmed ? 'Y' : 'N', gImu.referenceValid ? 'Y' : 'N');
  u8g2.drawStr(0, 18, line);
  if (gImu.referenceValid) {
    snprintf(line, sizeof(line), "Ref:(%+1.2f,%+1.2f)", gImu.reference.xAxisX, gImu.reference.xAxisY);
  } else {
    snprintf(line, sizeof(line), "Ref:not saved");
  }
  u8g2.drawStr(0, 27, line);
  snprintf(line, sizeof(line), "Cur:(%+1.2f,%+1.2f)", gImu.currentXAxisX, gImu.currentXAxisY);
  u8g2.drawStr(0, 36, line);
  snprintf(line, sizeof(line), "Err:%+2.2f Win:%2.2f", gImu.yawErrorDeg, gCfg.imuPidTestYawErrorDeg);
  u8g2.drawStr(0, 45, line);
  snprintf(line, sizeof(line), "Cmd:%+1.2f Db:%2.2f", gImu.lastCompensationCmd, gImu.combinedDeadbandDeg);
  u8g2.drawStr(0, 54, line);
  u8g2.drawStr(0, 63, "Short=renew Hold=back");
}

void drawMotorTuneScreen() {
  drawHeader("Motor Tune");
  char items[11][28];
  char value[16];
  snprintf(items[0], sizeof(items[0]), "Stop");
  snprintf(items[1], sizeof(items[1]), "North");
  snprintf(items[2], sizeof(items[2]), "East");
  snprintf(items[3], sizeof(items[3]), "South");
  snprintf(items[4], sizeof(items[4]), "West");
  formatCompactValue(value, sizeof(value), activeSettings.overallSpeedScale_x100, 2);
  snprintf(items[5], sizeof(items[5]), "Overall:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.baseSpeed_x100, 2);
  snprintf(items[6], sizeof(items[6]), "Travel:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.approachSpeed_x100, 2);
  snprintf(items[7], sizeof(items[7]), "Approach:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.leftDriveScale_x100, 2);
  snprintf(items[8], sizeof(items[8]), "LeftDrv:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.rightDriveScale_x100, 2);
  snprintf(items[9], sizeof(items[9]), "RightDrv:%s", value);
  snprintf(items[10], sizeof(items[10]), "Back");
  const char* ptrs[11];
  for (int i = 0; i < 11; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 11, gMotorTuneIndex, 26);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=run/edit Hold=back");
}

void drawMotionSettingsScreen() {
  char items[12][28];
  char value[16];
  formatCompactValue(value, sizeof(value), activeSettings.p_x1000, 3);
  snprintf(items[0], sizeof(items[0]), "P gain: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.i_x1000, 3);
  snprintf(items[1], sizeof(items[1]), "I gain: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.d_x1000, 3);
  snprintf(items[2], sizeof(items[2]), "D gain: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.curveB_x100, 2);
  snprintf(items[3], sizeof(items[3]), "Curve b: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.pidWallDistance_x100, 2);
  snprintf(items[4], sizeof(items[4]), "1-wall: %scm", value);
  formatCompactValue(value, sizeof(value), activeSettings.pidLeftScale_x100, 2);
  snprintf(items[5], sizeof(items[5]), "L corr: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.pidRightScale_x100, 2);
  snprintf(items[6], sizeof(items[6]), "R corr: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.baseSpeed_x100, 2);
  snprintf(items[7], sizeof(items[7]), "Travel: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.approachSpeed_x100, 2);
  snprintf(items[8], sizeof(items[8]), "Approach: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.leftDriveScale_x100, 2);
  snprintf(items[9], sizeof(items[9]), "L drive: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.rightDriveScale_x100, 2);
  snprintf(items[10], sizeof(items[10]), "R drive: %s", value);
  snprintf(items[11], sizeof(items[11]), "Back");

  const char* pointers[12];
  for (int i = 0; i < 12; ++i) pointers[i] = items[i];
  drawSimpleMenu("Motion Settings", pointers, 12, gMotionSettingsIndex);
}

void drawDetectionSettingsScreen() {
  char items[7][28];
  char value[16];
  formatCompactValue(value, sizeof(value), activeSettings.frontStopDistance_x100, 2);
  snprintf(items[0], sizeof(items[0]), "Front stop: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.corridorWallThreshold_x100, 2);
  snprintf(items[1], sizeof(items[1]), "Wall lim: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.turnDetectDistance_x100, 2);
  snprintf(items[2], sizeof(items[2]), "Turn det: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.deadEndDistance_x100, 2);
  snprintf(items[3], sizeof(items[3]), "Dead-end: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.finishDistance_x100, 2);
  snprintf(items[4], sizeof(items[4]), "Finish: %s", value);
  formatCompactValue(value, sizeof(value), activeSettings.sensingInterval_x100, 2);
  snprintf(items[5], sizeof(items[5]), "Recheck: %s", value);
  snprintf(items[6], sizeof(items[6]), "Back");

  const char* pointers[7];
  for (int i = 0; i < 7; ++i) pointers[i] = items[i];
  drawSimpleMenu("Detection Set", pointers, 7, gDetectionSettingsIndex);
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

  snprintf(line, sizeof(line), "IR mask:%02X", frame.stableIrMask);
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
  char value[16];
  switch (field) {
    case EventConfigField::Heading:
      snprintf(buffer, size, "Heading: %s", headingToString(currentEventTestHeading()));
      break;
    case EventConfigField::TurnChoice:
      snprintf(buffer, size, "T-Choice: %s", turnChoiceToString(currentEventTestConfig().turnChoice));
      break;
    case EventConfigField::OverallSpeed:
      formatCompactValue(value, sizeof(value), activeSettings.overallSpeedScale_x100, 2);
      snprintf(buffer, size, "Overall:%s", value);
      break;
    case EventConfigField::FrontStop:
      formatCompactValue(value, sizeof(value), activeSettings.frontStopDistance_x100, 2);
      snprintf(buffer, size, "FrontStop:%s", value);
      break;
    case EventConfigField::WallThreshold:
      formatCompactValue(value, sizeof(value), activeSettings.corridorWallThreshold_x100, 2);
      snprintf(buffer, size, "WallThr:%s", value);
      break;
    case EventConfigField::TurnDetect:
      formatCompactValue(value, sizeof(value), activeSettings.turnDetectDistance_x100, 2);
      snprintf(buffer, size, "TurnDet:%s", value);
      break;
    case EventConfigField::DeadEnd:
      formatCompactValue(value, sizeof(value), activeSettings.deadEndDistance_x100, 2);
      snprintf(buffer, size, "DeadEnd:%s", value);
      break;
    case EventConfigField::Finish:
      formatCompactValue(value, sizeof(value), activeSettings.finishDistance_x100, 2);
      snprintf(buffer, size, "Finish:%s", value);
      break;
    case EventConfigField::TurnWait:
      formatCompactValue(value, sizeof(value), activeSettings.waitBeforeTurn_x100, 2);
      snprintf(buffer, size, "TurnWait:%s", value);
      break;
    case EventConfigField::CheckGap:
      formatCompactValue(value, sizeof(value), activeSettings.sensingInterval_x100, 2);
      snprintf(buffer, size, "CheckGap:%s", value);
      break;
    case EventConfigField::MatchCount:
      snprintf(buffer, size, "Match N:%d", activeSettings.eventConfirmCount);
      break;
    case EventConfigField::WallP:
      formatCompactValue(value, sizeof(value), activeSettings.p_x1000, 3);
      snprintf(buffer, size, "P:%s", value);
      break;
    case EventConfigField::WallI:
      formatCompactValue(value, sizeof(value), activeSettings.i_x1000, 3);
      snprintf(buffer, size, "I:%s", value);
      break;
    case EventConfigField::WallD:
      formatCompactValue(value, sizeof(value), activeSettings.d_x1000, 3);
      snprintf(buffer, size, "D:%s", value);
      break;
    case EventConfigField::CurveB:
      formatCompactValue(value, sizeof(value), activeSettings.curveB_x100, 2);
      snprintf(buffer, size, "Curve:%s", value);
      break;
    case EventConfigField::OneWallTarget:
      formatCompactValue(value, sizeof(value), activeSettings.pidWallDistance_x100, 2);
      snprintf(buffer, size, "1Wall:%s", value);
      break;
    case EventConfigField::LeftOutput:
      formatCompactValue(value, sizeof(value), activeSettings.pidLeftScale_x100, 2);
      snprintf(buffer, size, "LeftOut:%s", value);
      break;
    case EventConfigField::RightOutput:
      formatCompactValue(value, sizeof(value), activeSettings.pidRightScale_x100, 2);
      snprintf(buffer, size, "RightOut:%s", value);
      break;
    case EventConfigField::ImuToggle:
      snprintf(buffer, size, "IMU:%s", activeSettings.imuEnabled ? "ON" : "OFF");
      break;
    case EventConfigField::ImuP:
      formatCompactValue(value, sizeof(value), activeSettings.imuP_x10000, 4);
      snprintf(buffer, size, "IMUP:%s", value);
      break;
    case EventConfigField::ImuI:
      formatCompactValue(value, sizeof(value), activeSettings.imuI_x10000, 4);
      snprintf(buffer, size, "IMUI:%s", value);
      break;
    case EventConfigField::ImuD:
      formatCompactValue(value, sizeof(value), activeSettings.imuD_x10000, 4);
      snprintf(buffer, size, "IMUD:%s", value);
      break;
    case EventConfigField::ImuThreshold:
      formatCompactValue(value, sizeof(value), activeSettings.imuAngleThreshold_x100, 2);
      snprintf(buffer, size, "IMUThr:%s", value);
      break;
    case EventConfigField::ImuTestYawError:
      formatCompactValue(value, sizeof(value), activeSettings.imuPidTestYawError_x100, 2);
      snprintf(buffer, size, "YawErr:%s", value);
      break;
    case EventConfigField::ImuSign:
      snprintf(buffer, size, "IMUSign:%+d", (activeSettings.imuSign_x100 < 0) ? -1 : 1);
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
  char items[28][32];
  const char* ptrs[28];
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
  char items[8][28];
  snprintf(items[0], sizeof(items[0]), "NW(RL): %d", isIrTriggered(gIrRL) ? 1 : 0);
  snprintf(items[1], sizeof(items[1]), "NE(RR): %d", isIrTriggered(gIrRR) ? 1 : 0);
  snprintf(items[2], sizeof(items[2]), "SW(FL): %d", isIrTriggered(gIrFL) ? 1 : 0);
  snprintf(items[3], sizeof(items[3]), "SE(FR): %d", isIrTriggered(gIrFR) ? 1 : 0);
  char value[16];
  formatCompactValue(value, sizeof(value), activeSettings.sensingInterval_x100, 2);
  snprintf(items[4], sizeof(items[4]), "CheckGap:%s", value);
  snprintf(items[5], sizeof(items[5]), "Match N:%d", activeSettings.eventConfirmCount);
  formatCompactValue(value, sizeof(value), activeSettings.deadEndDistance_x100, 2);
  snprintf(items[6], sizeof(items[6]), "DeadEnd:%s", value);
  formatCompactValue(value, sizeof(value), activeSettings.finishDistance_x100, 2);
  snprintf(items[7], sizeof(items[7]), "Finish:%s", value);
  const char* ptrs[8];
  for (int i = 0; i < 8; ++i) ptrs[i] = items[i];
  drawScrollableItemList(ptrs, 8, gIrSensorIndex, 26);
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

  char numeric[16];
  formatPaddedValue(numeric, sizeof(numeric), gDigitEditor.tempValue, gDigitEditor.integerDigits, gDigitEditor.decimals);
  int originX = (128 - u8g2.getStrWidth(numeric)) / 2;
  u8g2.drawStr(originX, 40, numeric);

  int digitX[8] = {0};
  int digitW[8] = {0};
  uint8_t digitIndex = 0;
  for (uint8_t i = 0; numeric[i] != '\0' && digitIndex < 8; ++i) {
    if (numeric[i] < '0' || numeric[i] > '9') continue;
    char prefix[16];
    memcpy(prefix, numeric, i);
    prefix[i] = '\0';
    char digitStr[2] = {numeric[i], '\0'};
    digitX[digitIndex] = originX + u8g2.getStrWidth(prefix);
    digitW[digitIndex] = u8g2.getStrWidth(digitStr);
    digitIndex++;
  }

  if (digitIndex > 0 && gDigitEditor.selectedDigit < digitIndex) {
    int selectedX = digitX[gDigitEditor.selectedDigit];
    int selectedW = digitW[gDigitEditor.selectedDigit];
    int underlineY = 45;

    u8g2.drawHLine(selectedX - 1, underlineY, selectedW + 2);
    if (gDigitEditor.digitUnlocked) {
      u8g2.drawHLine(selectedX - 1, underlineY + 2, selectedW + 2);
    } else {
      u8g2.drawHLine(selectedX - 1, underlineY + 1, selectedW + 2);
    }
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
           nextTrialModeToString(),
           headingToString(headingFromIndex(activeSettings.startHeadingIndex)));
  u8g2.drawStr(0, 29, line);

  char duration[20];
  formatDuration(duration, sizeof(duration), gPlanner.lastAttemptDurationMs);
  snprintf(line, sizeof(line), "Last:%s", duration);
  u8g2.drawStr(0, 40, line);

  formatDuration(duration, sizeof(duration), gPlanner.totalDurationMs);
  snprintf(line, sizeof(line), "Total:%s", duration);
  u8g2.drawStr(0, 49, line);

  u8g2.drawStr(0, 64, "Short:go  Hold:back");
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

static bool screenUsesUiThrottle(Screen screen) {
  switch (screen) {
    case Screen::RunScreen:
    case Screen::EventTestRun:
    case Screen::UltrasonicSensorTest:
    case Screen::IMUTest:
    case Screen::IMUYawTest:
      return true;
    default:
      return false;
  }
}

void drawUI() {
  uint32_t now = millis();
  bool forceDraw = (gCurrentScreen != gLastDrawnScreen);
  if (!forceDraw &&
      screenUsesUiThrottle(gCurrentScreen) &&
      (now - gLastUiDrawMs) < UI_REFRESH_INTERVAL_MS) {
    return;
  }

  u8g2.clearBuffer();

  switch (gCurrentScreen) {
    case Screen::Root:                 drawRootScreen(); break;
    case Screen::StartSettings:        drawStartSettingsScreen(); break;
    case Screen::Settings:             drawSettingsScreen(); break;
    case Screen::PID:                  drawPIDScreen(); break;
    case Screen::IMUPIDMenu:           drawImuPidMenuScreen(); break;
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
  gLastUiDrawMs = now;
  gLastDrawnScreen = gCurrentScreen;
}

void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  pinMode(PIN_IR_FL, INPUT);
  pinMode(PIN_IR_FR, INPUT);
  pinMode(PIN_IR_RL, INPUT);
  pinMode(PIN_IR_RR, INPUT);

  // Seed the encoder state before enabling interrupts so the first edge after boot
  // is measured relative to the real hardware position instead of an implicit zero state.
  gEncoderLastState =
    (((uint8_t)digitalRead(ENC_A_PIN)) << 1) |
    ((uint8_t)digitalRead(ENC_B_PIN));

  //Serial.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
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

  workingSettings = activeSettings;
  commitRuntimeConfig();

  readIRSensors();
  gStableIrMask = gRawIrMask;
  gPreviewHeading = headingFromIndex(activeSettings.startHeadingIndex);
  resetDrivePidControl();
  resetImuPid();
}

void loop() {
  refreshSensors();

  int32_t rawEncoderDelta = 0;
  noInterrupts();
  rawEncoderDelta = gEncoderRawDelta;
  gEncoderRawDelta = 0;
  interrupts();

  int32_t combinedEncoderDelta = rawEncoderDelta + gEncoderStepCarry;
  int encoderDelta = 0;
  while (combinedEncoderDelta >= 2) {
    encoderDelta++;
    combinedEncoderDelta -= 2;
  }
  while (combinedEncoderDelta <= -2) {
    encoderDelta--;
    combinedEncoderDelta += 2;
  }
  gEncoderStepCarry = (int8_t)combinedEncoderDelta;

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
      if (gImuPidTestArmed && imuUsageEnabled() && gImu.referenceValid) {
        // The IMU PID test intentionally isolates pure yaw recovery from the
        // maze-heading logic so only rotational compensation is being tuned here.
        float allowedYawErrorDeg = max(gCfg.imuPidTestYawErrorDeg, gImu.combinedDeadbandDeg);
        if (fabsf(gImu.yawErrorDeg) <= allowedYawErrorDeg) {
          resetImuPid();
          stopAll();
        } else {
          float rotationComp = currentImuRotationCompensation(gImuPid);
          driveBodyFrame(0.0f, 0.0f, rotationComp);
        }
      } else {
        stopAll();
      }
    } else {
      stopAll();
    }
  } else {
    ButtonEvent buttonEvent = readButtonEvent(true);
    handleIdleUiInput(encoderDelta, buttonEvent);
  }

  drawUI();
}
