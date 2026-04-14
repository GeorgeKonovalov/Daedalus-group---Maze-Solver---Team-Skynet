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

  // Pair gains
  int pairFLRR_fwd_x100 = 100;
  int pairFLRR_back_x100 = 100;
  int pairFRRL_fwd_x100 = 100;
  int pairFRRL_back_x100 = 100;

  // Per-wheel gains
  int fl_fwd_x100 = 100;
  int fl_back_x100 = 100;
  int fr_fwd_x100 = 72;
  int fr_back_x100 = 72;
  int rr_fwd_x100 = 94;
  int rr_back_x100 = 94;
  int rl_fwd_x100 = 68;
  int rl_back_x100 = 68;

  int thresholdTime_x100 = 150;
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
float gThresholdTime = 0.0f;
uint8_t gMode = 1;

// Pair runtime gains
float gPairFLRRFwd = 1.00f;
float gPairFLRRBack = 1.00f;
float gPairFRRLFwd = 1.00f;
float gPairFRRLBack = 1.00f;

// Per-wheel runtime gains
float gFLFwd = 1.00f;
float gFLBack = 1.00f;
float gFRFwd = 0.72f;
float gFRBack = 0.72f;
float gRRFwd = 0.94f;
float gRRBack = 0.94f;
float gRLFwd = 0.68f;
float gRLBack = 0.68f;

// ======================================================
// Runtime / trial timing globals
// ======================================================
uint32_t executionStartMs = 0;
uint32_t frozenExecutionDurationMs = 0;

// ======================================================
// Encoder shared state
// ======================================================
volatile int8_t g_encoderDelta = 0;

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

// ======================================================
// Runtime event names
// ======================================================
enum class EventName : uint8_t {
  Start,
  Finish,
  Straight
};

// ======================================================
// Menu / screen state
// ======================================================
enum class Screen : uint8_t {
  Root,
  StartSettings,
  Settings,
  PID,
  MotorTune,
  DigitEditor,
  StartConfirm,
  RunScreen
};

Screen currentScreen = Screen::Root;
Screen previousScreen = Screen::Root;

int rootIndex = 0;          // 0=Start, 1=Start Settings, 2=Settings
int settingsIndex = 0;      // 0=PID, 1=Motor Tune, 2=Threshold, 3=Save
int pidIndex = 0;           // 0=P, 1=I, 2=D, 3=Back
int motorTuneIndex = 0;

constexpr int ROOT_COUNT = 3;
constexpr int SETTINGS_COUNT = 4;
constexpr int PID_COUNT = 4;

// ======================================================
// Setup / runtime control
// ======================================================
bool programStarted = false;
bool Execution_state = false;
EventName currentEventName = EventName::Start;

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
String incomingData = "";

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
const float FRONT_STOP_THRESHOLD = 10.0f;

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
int digitPlaceValue(uint8_t digitIndex);
int getDigitAt(int value, uint8_t digitIndex);
int setDigitAt(int value, uint8_t digitIndex, int newDigit);
void incrementSelectedDigit(int step);

void onEncoderAChange();
int consumeEncoderDelta();
ButtonEvent pollButtonEvent();

void commitGlobalsForRuntime();

void openDigitEditor(const char* label, int* target, int minValue, int maxValue);
void closeDigitEditor(bool commit);

void handleRootInput(int delta, ButtonEvent btn);
void handleStartConfirmInput(ButtonEvent btn);
void handleStartSettingsInput(int delta, ButtonEvent btn);
void handleSettingsInput(int delta, ButtonEvent btn);
void handlePIDInput(int delta, ButtonEvent btn);
void handleMotorTuneInput(int delta, ButtonEvent btn);
void handleDigitEditorInput(int delta, ButtonEvent btn);

void drawHeader(const char* title);
void drawMenuItem(int y, bool selected, const String& text);
void drawScrollableItemList(const String items[], int itemCount, int selectedIndex, int startY);
void drawRootScreen();
void drawStartConfirmScreen();
void drawStartSettingsScreen();
void drawSettingsScreen();
void drawPIDScreen();
void drawMotorTuneScreen();
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
void followCorridor();
void updateExecutionLogic(bool& stopNow, EventName& displayedEvent);

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
          return ButtonEvent::ShortPress;
        }
      }
    }
  }

  if (buttonState.stableLevel == LOW && !buttonState.longPressFired) {
    if ((now - buttonState.pressedAtMs) >= BUTTON_LONGPRESS_MS) {
      buttonState.longPressFired = true;
      return ButtonEvent::LongPress;
    }
  }

  return ButtonEvent::None;
}

// ======================================================
// Runtime settings commit
// ======================================================
void commitGlobalsForRuntime() {
  activeSettings = workingSettings;

  gP = activeSettings.p_x100 / 100.0f;
  gI = activeSettings.i_x100 / 100.0f;
  gD = activeSettings.d_x100 / 100.0f;

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

  gThresholdTime = activeSettings.thresholdTime_x100 / 100.0f;
  gMode = activeSettings.mode;
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
      case 1: currentScreen = Screen::StartSettings; break;
      case 2: settingsIndex = 0; currentScreen = Screen::Settings; break;
    }
  }
}

void handleStartConfirmInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    programStarted = true;
  }
  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handleStartSettingsInput(int delta, ButtonEvent btn) {
  (void)delta;

  if (btn == ButtonEvent::ShortPress) {
    workingSettings.mode++;
    if (workingSettings.mode > 4) workingSettings.mode = 1;
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
        openDigitEditor("Threshold", &workingSettings.thresholdTime_x100, 0, 9999);
        break;
      case 3:
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
      case 3: currentScreen = Screen::Settings; break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Settings;
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
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, title);
  u8g2.drawHLine(0, 13, 128);
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
  drawMenuItem(30, rootIndex == 0, "Start");
  drawMenuItem(42, rootIndex == 1, "Start Settings");
  drawMenuItem(54, rootIndex == 2, "Settings");
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short: select");
}

void drawStartConfirmScreen() {
  drawHeader("Ready");
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(20, 36, "START");
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 54, "Short press: run");
  u8g2.drawStr(0, 63, "Long press: root");
}

void drawStartSettingsScreen() {
  drawHeader("Start Settings");
  drawMenuItem(34, true, "Mode: " + String(workingSettings.mode));
  drawMenuItem(50, false, "Short=change");
  drawMenuItem(60, false, "Long=quit");
}

void drawSettingsScreen() {
  drawHeader("Settings");
  drawMenuItem(24, settingsIndex == 0, "PID");
  drawMenuItem(34, settingsIndex == 1, "Motor Tune");
  drawMenuItem(44, settingsIndex == 2, "Threshold: " + formatX100(workingSettings.thresholdTime_x100));
  drawMenuItem(56, settingsIndex == 3, "Save");
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: root");
}

void drawPIDScreen() {
  drawHeader("PID");
  drawMenuItem(26, pidIndex == 0, "P: " + formatX100(workingSettings.p_x100));
  drawMenuItem(36, pidIndex == 1, "I: " + formatX100(workingSettings.i_x100));
  drawMenuItem(46, pidIndex == 2, "D: " + formatX100(workingSettings.d_x100));
  drawMenuItem(56, pidIndex == 3, "Back");
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Long: settings");
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

  drawScrollableItemList(items, MOTOR_TUNE_COUNT, motorTuneIndex, 24);

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=edit Long=back");
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
  drawMenuItem(24, false, "Time: " + formatMsAsSeconds(shownTime));
  drawMenuItem(35, false, "Event: " + String(currentEventName == EventName::Finish ? "Finish" :
                                              currentEventName == EventName::Start ? "Start" : "Straight"));
  drawMenuItem(46, false, "Head: " + String(headingToString(currentHeading)));
  drawMenuItem(57, false, String("State: ") + (Execution_state ? "RUN" : "STOP"));
}

void drawUI() {
  u8g2.clearBuffer();

  switch (currentScreen) {
    case Screen::Root:          drawRootScreen(); break;
    case Screen::StartSettings: drawStartSettingsScreen(); break;
    case Screen::Settings:      drawSettingsScreen(); break;
    case Screen::PID:           drawPIDScreen(); break;
    case Screen::MotorTune:     drawMotorTuneScreen(); break;
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
    int delta = consumeEncoderDelta();
    ButtonEvent btn = pollButtonEvent();

    switch (currentScreen) {
      case Screen::Root:          handleRootInput(delta, btn); break;
      case Screen::StartSettings: handleStartSettingsInput(delta, btn); break;
      case Screen::Settings:      handleSettingsInput(delta, btn); break;
      case Screen::PID:           handlePIDInput(delta, btn); break;
      case Screen::MotorTune:     handleMotorTuneInput(delta, btn); break;
      case Screen::DigitEditor:   handleDigitEditorInput(delta, btn); break;
      case Screen::StartConfirm:  handleStartConfirmInput(btn); break;
      case Screen::RunScreen:     break;
    }

    // Live motor output while tuning
    if (currentScreen == Screen::MotorTune ||
        (currentScreen == Screen::DigitEditor && previousScreen == Screen::MotorTune)) {
      runTuneMotion();
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

  if (!Execution_state) {
    currentScreen = Screen::StartConfirm;

    if (btn == ButtonEvent::ShortPress) {
      Execution_state = true;
      executionStartMs = millis();
      frozenExecutionDurationMs = 0;
      currentScreen = Screen::RunScreen;
      currentEventName = EventName::Start;
      previousError = 0.0f;
      integral = 0.0f;
      previousTime = millis();
    }

    if (btn == ButtonEvent::LongPress) {
      currentScreen = Screen::Root;
    }

    drawUI();
    return;
  }

  currentScreen = Screen::RunScreen;
  currentEventName = event_name;

  if (stop_machine) {
    currentEventName = EventName::Finish;
    frozenExecutionDurationMs = millis() - executionStartMs;
    Execution_state = false;
    stopAll();
    currentScreen = Screen::StartConfirm;
    drawUI();
    return;
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

  float d1 = data.substring(idx1, comma1).toFloat();
  float d2 = data.substring(idx2, comma2).toFloat();
  float d3 = data.substring(idx3, comma3).toFloat();
  float d4 = data.substring(idx4).toFloat();

  distN = d3;
  distE = d4;
  distS = d1;
  distW = d2;
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
void followCorridor() {
  // North-heading corridor follow
  currentHeading = Heading::North;

  // positive correction = move right/east
  float error = distE - distW;
  float correction = calculatePID(error);

  driveHeading(BASE_SPEED, correction);
}

void updateExecutionLogic(bool& stopNow, EventName& displayedEvent) {
  if (!Execution_state) return;

  Kp = gP;
  Ki = gI;
  Kd = gD;

  readSensorsUART();

  displayedEvent = EventName::Straight;

  if (distN <= FRONT_STOP_THRESHOLD) {
    stopAll();
    displayedEvent = EventName::Finish;
    stopNow = true;
    return;
  }

  followCorridor();
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

  S_FL.attach(PIN_FL);
  S_FR.attach(PIN_FR);
  S_RR.attach(PIN_RR);
  S_RL.attach(PIN_RL);

  stopAll();

  activeSettings.p_x100 = 7;
  activeSettings.i_x100 = 0;
  activeSettings.d_x100 = 0;

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

  activeSettings.thresholdTime_x100 = 150;
  activeSettings.mode = 1;

  workingSettings = activeSettings;
  commitGlobalsForRuntime();

  previousTime = millis();

  runMenuInSetup();

  Serial.println("=========================================");
  Serial.println("Live motor tune + run mode ready");
  Serial.println("=========================================");
}

void loop() {
  bool stopNow = false;
  EventName displayedEvent = currentEventName;

  if (Execution_state) {
    updateExecutionLogic(stopNow, displayedEvent);
  }

  ScreenLoopUpdate(displayedEvent, stopNow);
}
