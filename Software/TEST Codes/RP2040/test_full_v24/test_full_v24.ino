#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>
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

// ======================================================
// Settings storage (fixed-point x100)
// ======================================================
struct RuntimeSettings {
  int p_x100 = 7;
  int i_x100 = 0;
  int d_x100 = 0;
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
  int irStableTime_x100 = 10;
  int waitBeforeTurn_x100 = 500;

  uint8_t routeMode = 1;
  uint8_t startHeadingIndex = 0;
};

struct RuntimeConfig {
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
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

struct PidRuntime {
  float previousError = 0.0f;
  float integral = 0.0f;
  uint32_t previousMs = 0;
};

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
PidRuntime gPid;

Screen gCurrentScreen = Screen::Root;
Screen gPreviousScreen = Screen::Root;

int gRootIndex = 0;
int gStartSettingsIndex = 0;
int gMotionSettingsIndex = 0;
int gDetectionSettingsIndex = 0;

Heading gPreviewHeading = Heading::North;

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

float fixedToFloat(int value);
int clampInt(int value, int minValue, int maxValue);
float clampFloat(float value, float minValue, float maxValue);
float clampUnit(float value);
uint32_t secondsToMs(float seconds);
uint8_t buildIrMaskFromPins();
bool isIrTriggered(int value);
bool ultrasonicFresh();
bool parseDistancesLine(const char* line, float& d1, float& d2, float& d3, float& d4);
void pushUltrasonicAverages(float north, float east, float south, float west);
void readIRSensors();
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
void driveRobotFrame(float lateralCmd, float forwardCmd, Heading heading);
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
void handleMotionSettingsInput(int encoderDelta, ButtonEvent buttonEvent);
void handleDetectionSettingsInput(int encoderDelta, ButtonEvent buttonEvent);
void handleSensorMonitorInput(int encoderDelta, ButtonEvent buttonEvent);
void handleTwinMonitorInput(int encoderDelta, ButtonEvent buttonEvent);
void handleStartConfirmInput(ButtonEvent buttonEvent);

const char* headingToString(Heading heading);
const char* routeModeToString(RouteMode mode);
const char* eventToString(EventType eventType);
const char* navStateToString(NavState state);
const char* pidSourceToString(PidSource source);
const char* sequenceStepToString();
void formatFixedValue(char* buffer, size_t size, int value);
void formatDuration(char* buffer, size_t size, uint32_t durationMs);

void drawHeader(const char* title);
void drawMenuItem(int y, bool selected, const char* text);
void drawRootScreen();
void drawStartSettingsScreen();
void drawMotionSettingsScreen();
void drawDetectionSettingsScreen();
void drawSensorMonitorScreen();
void drawTwinMonitorScreen();
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

void commitRuntimeConfig() {
  activeSettings = workingSettings;

  gCfg.p = fixedToFloat(activeSettings.p_x100);
  gCfg.i = fixedToFloat(activeSettings.i_x100);
  gCfg.d = fixedToFloat(activeSettings.d_x100);
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
  gCfg.irStableTimeS = clampFloat(fixedToFloat(activeSettings.irStableTime_x100), 0.01f, 10.0f);
  gCfg.waitBeforeTurnS = clampFloat(fixedToFloat(activeSettings.waitBeforeTurn_x100), 0.0f, 30.0f);
}

void resetPid() {
  gPid.previousError = 0.0f;
  gPid.integral = 0.0f;
  gPid.previousMs = millis();
}

Heading headingFromIndex(uint8_t index) {
  switch (index & 0x03) {
    case 0: return Heading::North;
    case 1: return Heading::East;
    case 2: return Heading::South;
    default: return Heading::West;
  }
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
  switch (heading) {
    case Heading::North: return Heading::West;
    case Heading::East:  return Heading::North;
    case Heading::South: return Heading::East;
    case Heading::West:  return Heading::South;
    default:             return Heading::North;
  }
}

Heading turnRightOf(Heading heading) {
  switch (heading) {
    case Heading::North: return Heading::East;
    case Heading::East:  return Heading::South;
    case Heading::South: return Heading::West;
    case Heading::West:  return Heading::North;
    default:             return Heading::North;
  }
}

Heading oppositeOf(Heading heading) {
  switch (heading) {
    case Heading::North: return Heading::South;
    case Heading::East:  return Heading::West;
    case Heading::South: return Heading::North;
    case Heading::West:  return Heading::East;
    default:             return Heading::North;
  }
}

float distanceForHeading(Heading heading) {
  switch (heading) {
    case Heading::North: return gAvgN;
    case Heading::East:  return gAvgE;
    case Heading::South: return gAvgS;
    case Heading::West:  return gAvgW;
    default:             return gAvgN;
  }
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

void refreshSensors() {
  readIRSensors();
  serviceUartSensorStream();
  updateStableIrMask();
}

RelativeIrState relativeIrForHeading(uint8_t mask, Heading heading) {
  bool sw = (mask & 0x01) != 0;
  bool se = (mask & 0x02) != 0;
  bool nw = (mask & 0x04) != 0;
  bool ne = (mask & 0x08) != 0;

  RelativeIrState relative;
  switch (heading) {
    case Heading::North:
      relative.frontLeft = nw;
      relative.frontRight = ne;
      relative.rearLeft = sw;
      relative.rearRight = se;
      break;
    case Heading::East:
      relative.frontLeft = ne;
      relative.frontRight = se;
      relative.rearLeft = nw;
      relative.rearRight = sw;
      break;
    case Heading::South:
      relative.frontLeft = se;
      relative.frontRight = sw;
      relative.rearLeft = ne;
      relative.rearRight = nw;
      break;
    case Heading::West:
      relative.frontLeft = sw;
      relative.frontRight = nw;
      relative.rearLeft = se;
      relative.rearRight = ne;
      break;
  }
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

  if (state.matches >= EVENT_CONFIRM_SAMPLES) {
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

  if (state.matches >= CORRIDOR_CONFIRM_SAMPLES) {
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

static void driveBodyFrame(float vx, float vy) {
  float fl = (vy + vx) * gCfg.leftDriveScale;
  float rr = (vy + vx) * gCfg.leftDriveScale;
  float fr = (vy - vx) * gCfg.rightDriveScale;
  float rl = (vy - vx) * gCfg.rightDriveScale;

  float maxAbs = max(max(fabsf(fl), fabsf(fr)), max(fabsf(rr), fabsf(rl)));
  if (maxAbs > 1.0f) {
    fl /= maxAbs;
    fr /= maxAbs;
    rr /= maxAbs;
    rl /= maxAbs;
  }

  writeServoCommand(servoFL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeServoCommand(servoFR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeServoCommand(servoRR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeServoCommand(servoRL, STOP_RL, RANGE_RL, INV_RL, rl);
}

void driveRobotFrame(float lateralCmd, float forwardCmd, Heading heading) {
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

  driveBodyFrame(vx, vy);
}

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

  driveRobotFrame(correction, forwardSpeed, heading);
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
void handleRootInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 6;

  if (encoderDelta != 0) {
    gRootIndex += encoderDelta;
    while (gRootIndex < 0) gRootIndex += itemCount;
    while (gRootIndex >= itemCount) gRootIndex -= itemCount;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gRootIndex) {
    case 0:
      beginExecution();
      break;
    case 1:
      gCurrentScreen = Screen::StartSettings;
      break;
    case 2:
      gCurrentScreen = Screen::MotionSettings;
      break;
    case 3:
      gCurrentScreen = Screen::DetectionSettings;
      break;
    case 4:
      gPreviewHeading = headingFromIndex(activeSettings.startHeadingIndex);
      gCurrentScreen = Screen::SensorMonitor;
      break;
    case 5:
      gPreviewHeading = headingFromIndex(activeSettings.startHeadingIndex);
      gCurrentScreen = Screen::TwinMonitor;
      break;
    default:
      break;
  }
}

void handleStartSettingsInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 4;

  if (encoderDelta != 0) {
    gStartSettingsIndex += encoderDelta;
    while (gStartSettingsIndex < 0) gStartSettingsIndex += itemCount;
    while (gStartSettingsIndex >= itemCount) gStartSettingsIndex -= itemCount;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gStartSettingsIndex) {
    case 0:
      activeSettings.routeMode++;
      if (activeSettings.routeMode > (uint8_t)RouteMode::Selection) {
        activeSettings.routeMode = (uint8_t)RouteMode::Sequence;
      }
      workingSettings.routeMode = activeSettings.routeMode;
      commitRuntimeConfig();
      break;
    case 1:
      activeSettings.startHeadingIndex = (activeSettings.startHeadingIndex + 1) & 0x03;
      workingSettings.startHeadingIndex = activeSettings.startHeadingIndex;
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

void handleMotionSettingsInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 12;

  if (encoderDelta != 0) {
    gMotionSettingsIndex += encoderDelta;
    while (gMotionSettingsIndex < 0) gMotionSettingsIndex += itemCount;
    while (gMotionSettingsIndex >= itemCount) gMotionSettingsIndex -= itemCount;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gMotionSettingsIndex) {
    case 0:
      openDigitEditor("P gain", "x", &workingSettings.p_x100, 0, 9999,
                      "Proportional reaction", "to wall error", Screen::MotionSettings);
      break;
    case 1:
      openDigitEditor("I gain", "x", &workingSettings.i_x100, 0, 9999,
                      "Integral correction", "for constant drift", Screen::MotionSettings);
      break;
    case 2:
      openDigitEditor("D gain", "x", &workingSettings.d_x100, 0, 9999,
                      "Derivative damping", "for quick changes", Screen::MotionSettings);
      break;
    case 3:
      openDigitEditor("Curve b", "x", &workingSettings.curveB_x100, 10, 500,
                      "Shapes small errors", "before PID acts", Screen::MotionSettings);
      break;
    case 4:
      openDigitEditor("1-wall target", "cm", &workingSettings.pidWallDistance_x100, 0, 9999,
                      "Desired wall distance", "for single-wall PID", Screen::MotionSettings);
      break;
    case 5:
      openDigitEditor("Left corr", "x", &workingSettings.pidLeftScale_x100, 10, 300,
                      "Extra scale when", "correcting left", Screen::MotionSettings);
      break;
    case 6:
      openDigitEditor("Right corr", "x", &workingSettings.pidRightScale_x100, 10, 300,
                      "Extra scale when", "correcting right", Screen::MotionSettings);
      break;
    case 7:
      openDigitEditor("Base speed", "x", &workingSettings.baseSpeed_x100, 0, 100,
                      "Forward speed during", "corridor running", Screen::MotionSettings);
      break;
    case 8:
      openDigitEditor("Approach spd", "x", &workingSettings.approachSpeed_x100, 0, 100,
                      "Forward speed during", "event handlers", Screen::MotionSettings);
      break;
    case 9:
      openDigitEditor("Left drive", "x", &workingSettings.leftDriveScale_x100, 10, 300,
                      "Output scale for", "FL and RR pair", Screen::MotionSettings);
      break;
    case 10:
      openDigitEditor("Right drive", "x", &workingSettings.rightDriveScale_x100, 10, 300,
                      "Output scale for", "FR and RL pair", Screen::MotionSettings);
      break;
    case 11:
      gCurrentScreen = Screen::Root;
      break;
    default:
      break;
  }
}

void handleDetectionSettingsInput(int encoderDelta, ButtonEvent buttonEvent) {
  constexpr int itemCount = 8;

  if (encoderDelta != 0) {
    gDetectionSettingsIndex += encoderDelta;
    while (gDetectionSettingsIndex < 0) gDetectionSettingsIndex += itemCount;
    while (gDetectionSettingsIndex >= itemCount) gDetectionSettingsIndex -= itemCount;
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
    return;
  }

  if (buttonEvent != ButtonEvent::ShortPress) return;

  switch (gDetectionSettingsIndex) {
    case 0:
      openDigitEditor("Front stop", "cm", &workingSettings.frontStopDistance_x100, 0, 9999,
                      "Emergency stop if", "front gets too close", Screen::DetectionSettings);
      break;
    case 1:
      openDigitEditor("Wall limit", "cm", &workingSettings.corridorWallThreshold_x100, 0, 9999,
                      "Wall present for PID", "and finish checks", Screen::DetectionSettings);
      break;
    case 2:
      openDigitEditor("Turn detect", "cm", &workingSettings.turnDetectDistance_x100, 0, 9999,
                      "Upper front distance", "for turn detection", Screen::DetectionSettings);
      break;
    case 3:
      openDigitEditor("Dead-end", "cm", &workingSettings.deadEndDistance_x100, 0, 9999,
                      "Front distance limit", "for dead-end logic", Screen::DetectionSettings);
      break;
    case 4:
      openDigitEditor("Finish", "cm", &workingSettings.finishDistance_x100, 0, 9999,
                      "Front distance needed", "for finish detection", Screen::DetectionSettings);
      break;
    case 5:
      openDigitEditor("Recheck T", "s", &workingSettings.sensingInterval_x100, 1, 9999,
                      "Interval between", "event rechecks", Screen::DetectionSettings);
      break;
    case 6:
      openDigitEditor("IR stable", "s", &workingSettings.irStableTime_x100, 1, 9999,
                      "How long IR state", "must stay unchanged", Screen::DetectionSettings);
      break;
    case 7:
      gCurrentScreen = Screen::Root;
      break;
    default:
      break;
  }
}

void handleSensorMonitorInput(int encoderDelta, ButtonEvent buttonEvent) {
  if (encoderDelta != 0) {
    int next = (int)headingToIndex(gPreviewHeading) + encoderDelta;
    while (next < 0) next += 4;
    while (next > 3) next -= 4;
    gPreviewHeading = headingFromIndex((uint8_t)next);
  }

  if (buttonEvent == ButtonEvent::LongPress) {
    gCurrentScreen = Screen::Root;
  }
}

void handleTwinMonitorInput(int encoderDelta, ButtonEvent buttonEvent) {
  handleSensorMonitorInput(encoderDelta, buttonEvent);
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
    case Screen::Root:
      handleRootInput(encoderDelta, buttonEvent);
      break;
    case Screen::StartSettings:
      handleStartSettingsInput(encoderDelta, buttonEvent);
      break;
    case Screen::MotionSettings:
      handleMotionSettingsInput(encoderDelta, buttonEvent);
      break;
    case Screen::DetectionSettings:
      handleDetectionSettingsInput(encoderDelta, buttonEvent);
      break;
    case Screen::SensorMonitor:
      handleSensorMonitorInput(encoderDelta, buttonEvent);
      break;
    case Screen::TwinMonitor:
      handleTwinMonitorInput(encoderDelta, buttonEvent);
      break;
    case Screen::DigitEditor:
      handleDigitEditorInput(encoderDelta, buttonEvent);
      break;
    case Screen::StartConfirm:
      handleStartConfirmInput(buttonEvent);
      break;
    case Screen::RunScreen:
    default:
      break;
  }
}

void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 8, title);
  u8g2.drawHLine(0, 10, 128);
}

void drawMenuItem(int y, bool selected, const char* text) {
  if (selected) {
    u8g2.drawBox(0, y - 8, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, y, text);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, y, text);
  }
}

static void drawSimpleMenu(const char* title, const char* const* items, int itemCount, int selectedIndex) {
  drawHeader(title);
  constexpr int visibleCount = 5;
  int startIndex = 0;

  if (selectedIndex >= visibleCount) {
    startIndex = selectedIndex - visibleCount + 1;
  }
  if (startIndex > itemCount - visibleCount) {
    startIndex = max(0, itemCount - visibleCount);
  }

  int endIndex = min(itemCount, startIndex + visibleCount);
  int row = 0;
  for (int i = startIndex; i < endIndex; ++i) {
    drawMenuItem(20 + row * 9, i == selectedIndex, items[i]);
    row++;
  }

  if (itemCount > visibleCount) {
    if (startIndex > 0) {
      u8g2.drawLine(122, 14, 124, 11);
      u8g2.drawLine(124, 11, 126, 14);
    }
    if (endIndex < itemCount) {
      u8g2.drawLine(122, 58, 124, 61);
      u8g2.drawLine(124, 61, 126, 58);
    }
  }
}

void drawRootScreen() {
  static const char* const items[] = {
    "Start",
    "Start Settings",
    "Motion Settings",
    "Detection Settings",
    "Sensor Monitor",
    "Digital Twin"
  };
  drawSimpleMenu("Maze Solver v24", items, 6, gRootIndex);
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

void drawDigitEditorScreen() {
  drawHeader(gDigitEditor.label);
  u8g2.setFont(u8g2_font_6x12_tr);

  char valueText[24];
  char numeric[12];
  formatFixedValue(numeric, sizeof(numeric), gDigitEditor.tempValue);
  snprintf(valueText, sizeof(valueText), "%s %s", numeric, gDigitEditor.unit);
  u8g2.drawStr(14, 30, valueText);

  int digitX[4] = {14, 22, 34, 42};
  int selectedX = digitX[gDigitEditor.selectedDigit];
  if (gDigitEditor.selectedDigit >= 2) selectedX += 6;

  if (gDigitEditor.digitUnlocked) {
    u8g2.drawFrame(selectedX - 1, 17, 8, 14);
  } else {
    u8g2.drawLine(selectedX, 38, selectedX + 2, 34);
    u8g2.drawLine(selectedX + 2, 34, selectedX + 4, 38);
  }

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 45, gDigitEditor.description1);
  u8g2.drawStr(0, 54, gDigitEditor.description2);
  u8g2.drawStr(0, 64, "Short:edit  Hold:save");
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
    case Screen::Root:
      drawRootScreen();
      break;
    case Screen::StartSettings:
      drawStartSettingsScreen();
      break;
    case Screen::MotionSettings:
      drawMotionSettingsScreen();
      break;
    case Screen::DetectionSettings:
      drawDetectionSettingsScreen();
      break;
    case Screen::SensorMonitor:
      drawSensorMonitorScreen();
      break;
    case Screen::TwinMonitor:
      drawTwinMonitorScreen();
      break;
    case Screen::DigitEditor:
      drawDigitEditorScreen();
      break;
    case Screen::StartConfirm:
      drawStartConfirmScreen();
      break;
    case Screen::RunScreen:
      drawRunScreen();
      break;
    default:
      drawRootScreen();
      break;
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
  } else {
    ButtonEvent buttonEvent = readButtonEvent(true);
    handleIdleUiInput(encoderDelta, buttonEvent);
  }

  drawUI();
}
