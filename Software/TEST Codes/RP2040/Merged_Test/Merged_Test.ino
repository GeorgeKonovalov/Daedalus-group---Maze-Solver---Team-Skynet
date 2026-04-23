#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>
#include <WiFiNINA.h>
#include <ArduinoOTA.h>

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
  int p_x100 = 7;              // 1.25
  int i_x100 = 0;                // 0.05
  int d_x100 = 0;               // 0.20
  int thresholdTime_x100 = 150;  // 1.50
  uint8_t mode = 1;              // 1..4
};

RuntimeSettings activeSettings;
RuntimeSettings workingSettings;

// ======================================================
// Runtime globals used in loop()
// ======================================================
float gP = 0.0f;
float gI = 0.0f;
float gD = 0.0f;
float gThresholdTime = 0.0f;
uint8_t gMode = 1;

// ======================================================
// Runtime / trial timing globals
// ======================================================
uint32_t executionStartMs = 0;
uint32_t cycleStartMs = 0;
uint32_t frozenExecutionDurationMs = 0;

uint32_t time_first_trial = 0;
uint32_t time_second_trial = 0;
uint32_t time_third_trial = 0;
uint32_t total_cycle_time = 0;

bool modeSequenceStarted = false;
bool modeSequenceFinished = false;

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
  LeftTurn,
  RightTurn,
  TJunction,
  DeadEnds,
  Straight
};

// ======================================================
// Mode 1 sub-sequence
// ======================================================
enum class Mode1SubMode : uint8_t {
  RightTurn,
  LeftTurn,
  Selection
};

Mode1SubMode mode1SubMode = Mode1SubMode::RightTurn;

// ======================================================
// Menu / screen state
// ======================================================
enum class Screen : uint8_t {
  Root,
  StartSettings,
  Settings,
  PID,
  DigitEditor,
  StartConfirm,
  RunScreen
};

Screen currentScreen = Screen::Root;
Screen previousScreen = Screen::Root;

int rootIndex = 0;          // 0=Start, 1=Start Settings, 2=Settings
int settingsIndex = 0;      // 0=PID, 1=Threshold, 2=Save
int pidIndex = 0;           // 0=P, 1=I, 2=D, 3=Back

constexpr int ROOT_COUNT = 3;
constexpr int SETTINGS_COUNT = 3;
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
  uint8_t selectedDigit = 0;  // 0..3
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

// ===== CALIBRATION =====
int STOP_FL = 90,  STOP_FR = 90,  STOP_RR = 90,  STOP_RL = 90;
int RANGE_FL = 90, RANGE_FR = 90, RANGE_RR = 90, RANGE_RL = 90;
int INV_FL = +1,   INV_FR = +1,   INV_RR = +1,   INV_RL = +1;

// ===== SPEED EQUALIZATION GAINS =====
const float GAIN_FL =  0.964f;
const float GAIN_FR = -0.707f;
const float GAIN_RR = -1.000f;
const float GAIN_RL =  0.688f;

// ===== SENSOR READINGS =====
float distN = 30.0f, distE = 30.0f, distS = 30.0f, distW = 30.0f;
String incomingData = "";

// ===== PID PARAMETERS =====
float Kp = 0.07f;
float Ki = 0.00f;
float Kd = 0.00f;
float T_threshold = 0.00f;

float previousError = 0.0f;
float integral = 0.0f;
unsigned long previousTime = 0;

const float INTEGRAL_LIMIT   = 20.0f;
const float CORRECTION_LIMIT = 0.5f;

// ===== MOVEMENT =====
const float FORWARD_SPEED = 0.4f;

// ======================================================
// Manual prototypes to avoid Arduino .ino auto-prototype issues
// ======================================================
const char* eventNameToString(EventName ev);
Mode1SubMode fixedSubmodeForMode(uint8_t mode);
const char* mode1SubmodeToString(Mode1SubMode sm);
EventName eventForMode1Submode(Mode1SubMode sm);

int clampInt(int v, int lo, int hi);
String formatX100(int value);
String formatMsAsSeconds(uint32_t ms);
int digitPlaceValue(uint8_t digitIndex);
int getDigitAt(int value, uint8_t digitIndex);
int setDigitAt(int value, uint8_t digitIndex, int newDigit);
void incrementSelectedDigit(int step);

void resetCycleTimes();

void onEncoderAChange();
int consumeEncoderDelta();
ButtonEvent pollButtonEvent();

void commitGlobalsForRuntime();
const char* getModeTopText();
void setExecutionEvent(EventName ev);
void prepareNextRun();
void startExecutionNow();
void stopExecutionNow();

void openDigitEditor(const char* label, int* target, int minValue, int maxValue);
void closeDigitEditor(bool commit);

void handleRootInput(int delta, ButtonEvent btn);
void handleStartConfirmInput(ButtonEvent btn);
void handleStartSettingsInput(int delta, ButtonEvent btn);
void handleSettingsInput(int delta, ButtonEvent btn);
void handlePIDInput(int delta, ButtonEvent btn);
void handleDigitEditorInput(int delta, ButtonEvent btn);

void drawHeader(const char* title);
void drawMenuItem(int y, bool selected, const String& text);
void drawRootScreen();
void drawStartConfirmScreen();
void drawStartSettingsScreen();
void drawSettingsScreen();
void drawPIDScreen();
void drawDigitEditorScreen();
void drawRunScreen();
void drawUI();

void runMenuInSetup();
void ScreenLoopUpdate(EventName event_name, bool stop_machine);
void updateExecutionLogic(bool& stopNow, EventName& displayedEvent);

// movement helpers
static inline float clamp1(float v);
void writeCRServo(Servo &s, int stopVal, int rangeVal, int inv, float cmd);
void normalize(float &fl, float &fr, float &rr, float &rl);
void stopAll();
void drive(float Vx, float Vy, float W);
void parseDistances(String data);
bool readSensorsUART();
float calculatePID(float error);

// ======================================================
// General helpers
// ======================================================
const char* eventNameToString(EventName ev) {
  switch (ev) {
    case EventName::Start:     return "Start";
    case EventName::Finish:    return "Finish";
    case EventName::LeftTurn:  return "Left Turn";
    case EventName::RightTurn: return "Right Turn";
    case EventName::TJunction: return "T-Junction";
    case EventName::DeadEnds:  return "Dead Ends";
    case EventName::Straight:  return "Straight";
    default:                   return "Unknown";
  }
}

Mode1SubMode fixedSubmodeForMode(uint8_t mode) {
  switch (mode) {
    case 2: return Mode1SubMode::RightTurn;
    case 3: return Mode1SubMode::LeftTurn;
    case 4: return Mode1SubMode::Selection;
    default: return Mode1SubMode::RightTurn;
  }
}

const char* mode1SubmodeToString(Mode1SubMode sm) {
  switch (sm) {
    case Mode1SubMode::RightTurn: return "Right Turn";
    case Mode1SubMode::LeftTurn:  return "Left Turn";
    case Mode1SubMode::Selection: return "Selection";
    default:                      return "Unknown";
  }
}

EventName eventForMode1Submode(Mode1SubMode sm) {
  switch (sm) {
    case Mode1SubMode::RightTurn: return EventName::RightTurn;
    case Mode1SubMode::LeftTurn:  return EventName::LeftTurn;
    case Mode1SubMode::Selection: return EventName::Start; // placeholder for selection
    default:                      return EventName::Start;
  }
}

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

String formatX100(int value) {
  value = clampInt(value, 0, 9999);
  int whole = value / 100;
  int frac = value % 100;

  char buf[8];
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

void resetCycleTimes() {
  time_first_trial = 0;
  time_second_trial = 0;
  time_third_trial = 0;
  total_cycle_time = 0;
  frozenExecutionDurationMs = 0;
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
// Runtime / execution
// ======================================================
void commitGlobalsForRuntime() {
  activeSettings = workingSettings;

  gP = activeSettings.p_x100 / 100.0f;
  gI = activeSettings.i_x100 / 100.0f;
  gD = activeSettings.d_x100 / 100.0f;
  gThresholdTime = activeSettings.thresholdTime_x100 / 100.0f;
  gMode = activeSettings.mode;
}

const char* getModeTopText() {
  uint8_t shownMode = Execution_state ? gMode : workingSettings.mode;

  switch (shownMode) {
    case 1:
      switch (mode1SubMode) {
        case Mode1SubMode::RightTurn: return "Mode 1: Right Turn";
        case Mode1SubMode::LeftTurn:  return "Mode 1: Left Turn";
        case Mode1SubMode::Selection: return "Mode 1: Selection";
      }
      return "Mode 1";
    case 2: return "Mode 2: Right Turn";
    case 3: return "Mode 3: Left Turn";
    case 4: return "Mode 4: Selection";
    default: return "Mode ?";
  }
}

void setExecutionEvent(EventName ev) {
  currentEventName = ev;
}

void prepareNextRun() {
  commitGlobalsForRuntime();

  if (gMode == 1) {
    if (!modeSequenceStarted || modeSequenceFinished) {
      resetCycleTimes();
      modeSequenceStarted = true;
      modeSequenceFinished = false;
      cycleStartMs = millis();
      mode1SubMode = Mode1SubMode::RightTurn;
    }
  } else {
    resetCycleTimes();
    modeSequenceStarted = true;
    modeSequenceFinished = false;
    cycleStartMs = millis();
    mode1SubMode = fixedSubmodeForMode(gMode);
  }
}

void startExecutionNow() {
  Execution_state = true;
  executionStartMs = millis();
  frozenExecutionDurationMs = 0;
  currentScreen = Screen::RunScreen;
  setExecutionEvent(eventForMode1Submode(mode1SubMode));
}

void stopExecutionNow() {
  if (Execution_state) {
    uint32_t finishTime = millis() - executionStartMs;
    frozenExecutionDurationMs = finishTime;

    switch (gMode) {
      case 1:
        switch (mode1SubMode) {
          case Mode1SubMode::RightTurn:
            time_first_trial = finishTime;
            mode1SubMode = Mode1SubMode::LeftTurn;
            break;

          case Mode1SubMode::LeftTurn:
            time_second_trial = finishTime;
            mode1SubMode = Mode1SubMode::Selection;
            break;

          case Mode1SubMode::Selection:
            time_third_trial = finishTime;
            total_cycle_time = millis() - cycleStartMs;
            modeSequenceFinished = true;
            mode1SubMode = Mode1SubMode::RightTurn;
            break;
        }
        break;

      case 2:
        time_first_trial = finishTime;
        total_cycle_time = finishTime;
        modeSequenceFinished = true;
        break;

      case 3:
        time_second_trial = finishTime;
        total_cycle_time = finishTime;
        modeSequenceFinished = true;
        break;

      case 4:
        time_third_trial = finishTime;
        total_cycle_time = finishTime;
        modeSequenceFinished = true;
        break;
    }
  }

  Execution_state = false;
  currentScreen = Screen::StartConfirm;
  stopAll();
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
      case 0:
        currentScreen = Screen::StartConfirm;
        break;
      case 1:
        currentScreen = Screen::StartSettings;
        break;
      case 2:
        settingsIndex = 0;
        currentScreen = Screen::Settings;
        break;
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
    if (workingSettings.mode > 4) {
      workingSettings.mode = 1;
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
        openDigitEditor("Threshold", &workingSettings.thresholdTime_x100, 0, 9999);
        break;
      case 2:
        activeSettings = workingSettings;
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
      case 0:
        openDigitEditor("P", &workingSettings.p_x100, 0, 9999);
        break;
      case 1:
        openDigitEditor("I", &workingSettings.i_x100, 0, 9999);
        break;
      case 2:
        openDigitEditor("D", &workingSettings.d_x100, 0, 9999);
        break;
      case 3:
        currentScreen = Screen::Settings;
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
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

void drawRootScreen() {
  drawHeader("Main Menu");

  drawMenuItem(30, rootIndex == 0, "Start");
  drawMenuItem(42, rootIndex == 1, "Start Settings");
  drawMenuItem(54, rootIndex == 2, "Settings");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short: select");
}

void drawStartConfirmScreen() {
  drawHeader(getModeTopText());

  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(20, 36, "START");

  u8g2.setFont(u8g2_font_5x8_tf);

  if (workingSettings.mode == 1 && modeSequenceStarted && !modeSequenceFinished) {
    String nextLine = "Next: " + String(mode1SubmodeToString(mode1SubMode));
    u8g2.drawStr(0, 42, nextLine.c_str());
  }

  if (modeSequenceFinished) {
    String totalLine = "Total: " + formatMsAsSeconds(total_cycle_time);
    u8g2.drawStr(0, 42, totalLine.c_str());
  }

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

  drawMenuItem(30, settingsIndex == 0, "PID");
  drawMenuItem(42, settingsIndex == 1, "Threshold: " + formatX100(workingSettings.thresholdTime_x100));
  drawMenuItem(54, settingsIndex == 2, "Save");

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

void drawRunScreen() {
  drawHeader(getModeTopText());

  uint32_t shownTime = Execution_state ? (millis() - executionStartMs) : frozenExecutionDurationMs;

  drawMenuItem(28, false, "Time: " + formatMsAsSeconds(shownTime));
  drawMenuItem(40, false, "Event: " + String(eventNameToString(currentEventName)));
  drawMenuItem(52, false, String("State: ") + (Execution_state ? "RUNNING" : "STOPPED"));

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Execution");
}

void drawUI() {
  u8g2.clearBuffer();

  switch (currentScreen) {
    case Screen::Root:
      drawRootScreen();
      break;
    case Screen::StartSettings:
      drawStartSettingsScreen();
      break;
    case Screen::Settings:
      drawSettingsScreen();
      break;
    case Screen::PID:
      drawPIDScreen();
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

  while (!programStarted) {
    int delta = consumeEncoderDelta();
    ButtonEvent btn = pollButtonEvent();

    switch (currentScreen) {
      case Screen::Root:
        handleRootInput(delta, btn);
        break;
      case Screen::StartSettings:
        handleStartSettingsInput(delta, btn);
        break;
      case Screen::Settings:
        handleSettingsInput(delta, btn);
        break;
      case Screen::PID:
        handlePIDInput(delta, btn);
        break;
      case Screen::DigitEditor:
        handleDigitEditorInput(delta, btn);
        break;
      case Screen::StartConfirm:
        handleStartConfirmInput(btn);
        break;
      case Screen::RunScreen:
        break;
    }

    drawUI();
    delay(10);
  }
}

// ======================================================
// Screen/runtime updater
// ======================================================
void ScreenLoopUpdate(EventName event_name, bool stop_machine) {
  ButtonEvent btn = pollButtonEvent();

  if (!Execution_state) {
    currentScreen = Screen::StartConfirm;

    if (btn == ButtonEvent::ShortPress) {
      prepareNextRun();
      startExecutionNow();
    }

    if (btn == ButtonEvent::LongPress) {
      currentScreen = Screen::Root;
    }

    drawUI();
    return;
  }

  currentScreen = Screen::RunScreen;
  setExecutionEvent(event_name);

  if (stop_machine) {
    setExecutionEvent(EventName::Finish);
    stopExecutionNow();
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
  int val = constrain((int)lround(stopVal + cmd * rangeVal), 0, 180);
  s.write(val);
}

void normalize(float &fl, float &fr, float &rr, float &rl) {
  float m = max(max(fabs(fl), fabs(fr)), max(fabs(rr), fabs(rl)));
  if (m < 1e-6f) {
    fl = fr = rr = rl = 0;
    return;
  }
  if (m > 1.0f) {
    fl /= m; fr /= m; rr /= m; rl /= m;
  }
}

void stopAll() {
  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, 0);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, 0);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, 0);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, 0);
}

void drive(float Vx, float Vy, float W) {
  float FlRr = Vy + Vx + W;
  float FrRl = Vy - Vx - W;

  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, FlRr*GAIN_FL);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, FrRl*GAIN_FR);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, FlRr*GAIN_RR);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, FrRl*GAIN_RL);
}

// ======================================================
// UART sensor parsing
// Expected line: D1:xx,D2:xx,D3:xx,D4:xx
// ======================================================
void parseDistances(String data) {
  int idx1 = data.indexOf("D1:") + 3;
  int idx2 = data.indexOf("D2:") + 3;
  int idx3 = data.indexOf("D3:") + 3;
  int idx4 = data.indexOf("D4:") + 3;

  int comma1 = data.indexOf(',', idx1);
  int comma2 = data.indexOf(',', idx2);
  int comma3 = data.indexOf(',', idx3);

  if (idx1 > 2 && idx2 > 2 && idx3 > 2 && idx4 > 2 && comma1 > 0 && comma2 > 0 && comma3 > 0) {
    distS = data.substring(idx1, comma1).toFloat();
    distW = data.substring(idx2, comma2).toFloat();
    distN = data.substring(idx3, comma3).toFloat(); 
    distE = data.substring(idx4).toFloat();
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

// ======================================================
// PID
// ======================================================
float calculatePID(float error) {
  unsigned long now = millis();
  float dt = (now - previousTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.05f;

  float P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float I = Ki * integral;

  float derivative = (error - previousError) / dt;
  float D = Kd * derivative;

  previousError = error;
  previousTime = now;

  return constrain(P + I + D, -CORRECTION_LIMIT, CORRECTION_LIMIT);
}

// ======================================================
// Execution logic for movement test
// ======================================================
void updateExecutionLogic(bool& stopNow, EventName& displayedEvent) {
  if (!Execution_state) return;

  // Use current mode/submode for displayed event
  displayedEvent = eventForMode1Submode(mode1SubMode);

  // Load current gains from menu-committed globals
  Kp = gP;
  Ki = gI;
  Kd = gD;
  T_threshold = gThresholdTime;

  // No sensor update yet -> keep screen alive, no stop
  if (!readSensorsUART()) {
    return;
  }

  // Corridor centering: positive = too far west
  float error = distE - distW;
  float correction = calculatePID(error);

  // Example stop condition: wall ahead
  if (distN <= 8.0f) {
    stopAll();
    displayedEvent = EventName::Finish;
    stopNow = true;
    return;
  }

  // Otherwise keep driving forward with lateral correction
  drive(correction, FORWARD_SPEED, 0);

  // Optional debug
  Serial.print("E:"); Serial.print(distE, 1);
  Serial.print(" W:"); Serial.print(distW, 1);
  Serial.print(" N:"); Serial.print(distN, 1);
  Serial.print(" | err:"); Serial.print(error, 2);
  Serial.print(" cor:"); Serial.print(correction, 3);
  Serial.print(" | P:"); Serial.print(Kp * error, 3);
  Serial.print(" I:"); Serial.print(Ki * integral, 4);
  Serial.print(" D:"); Serial.println(Kd * ((error - previousError)), 3);
}

// ======================================================
// Setup / loop
// ======================================================
void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

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

  // initial menu values
  activeSettings.p_x100 = 7;
  activeSettings.i_x100 = 0;
  activeSettings.d_x100 = 0;
  activeSettings.thresholdTime_x100 = 150;
  activeSettings.mode = 1;

  runMenuInSetup();
  commitGlobalsForRuntime();

  previousTime = millis();

  Serial.println("=========================================");
  Serial.println("Merged menu + movement test ready");
  Serial.print("Kp="); Serial.print(gP, 3);
  Serial.print(" Ki="); Serial.print(gI, 3);
  Serial.print(" Kd="); Serial.print(gD, 3);
  Serial.print(" Threshold="); Serial.println(gThresholdTime, 3);
  Serial.println("=========================================");
}

void loop() {
  Kp = gP;
  Ki = gI;
  Kd = gD;
  T_threshold = gThresholdTime;
  if (!readSensorsUART()) return;

  // Centering error: positive = robot is too far west
  float error = distE - distW;

    // PID correction applied as lateral (Vx) movement
  float correction = calculatePID(error);

  // Stop if front wall detected
  if (distN <= 8.0f) {
    stopAll();
    Serial.print("WALL AHEAD — STOPPED  N:");
    Serial.println(distN, 1);
    delay(500);
    return;
  }

    // Drive north with lateral correction
    drive(correction, FORWARD_SPEED, 0);

    // Debug output
    Serial.print("E:"); Serial.print(distE, 1);
    Serial.print(" W:"); Serial.print(distW, 1);
    Serial.print(" | err:"); Serial.print(error, 2);
    Serial.print(" cor:"); Serial.print(correction, 3);
    Serial.print(" | P:"); Serial.print(Kp * error, 3);
    Serial.print(" I:"); Serial.print(Ki * integral, 4);
    Serial.print(" D:"); Serial.println(Kd * (error - previousError), 3);
}
