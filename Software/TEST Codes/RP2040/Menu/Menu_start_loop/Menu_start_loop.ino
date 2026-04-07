#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
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
// Hardware pins
// ======================================================
constexpr uint8_t ENC_A_PIN   = 7;
constexpr uint8_t ENC_B_PIN   = 6;
constexpr uint8_t ENC_BTN_PIN = 8;

// OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ======================================================
// Button timing
// ======================================================
constexpr uint32_t BUTTON_DEBOUNCE_MS = 25;
constexpr uint32_t BUTTON_LONGPRESS_MS = 700;

// ======================================================
// Settings storage
// Internal fixed-point x100
// ======================================================
struct RuntimeSettings {
  int p_x100 = 125;              // 1.25
  int i_x100 = 5;                // 0.05
  int d_x100 = 20;               // 0.20
  int thresholdTime_x100 = 150;  // 1.50
  uint8_t mode = 1;              // 1..4
};

RuntimeSettings activeSettings;   // committed settings
RuntimeSettings workingSettings;  // edited in menu

// ======================================================
// Global runtime variables for loop()
// These are committed when Start is pressed on root
// ======================================================
float gP = 0.0f;
float gI = 0.0f;
float gD = 0.0f;
float gThresholdTime = 0.0f;
uint8_t gMode = 1;

// ======================================================
// Encoder shared state
// No smoothing
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

// ======================================================
// Mode 1 sub-sequence:
// Right Turn -> Left Turn -> Selection
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
// Start/Setup control
// ======================================================
bool programStarted = false;

// ======================================================
// Runtime control
// ======================================================
bool Execution_state = false;              // must be global, not local in loop()
uint32_t executionStartMs = 0;
uint32_t frozenExecutionDurationMs = 0;
EventName currentEventName = EventName::Start;

// ======================================================
// Digit editor
// digits: [thousands][hundreds].[tens][ones]
// value 12.34 = 1234
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
// Utility
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

// ======================================================
// Encoder ISR
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

// ======================================================
// Button polling
// ======================================================
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
// Commit globals for runtime use in loop()
// Called when execution is started
// ======================================================
void commitGlobalsForRuntime() {
  activeSettings = workingSettings;

  gP = activeSettings.p_x100 / 100.0f;
  gI = activeSettings.i_x100 / 100.0f;
  gD = activeSettings.d_x100 / 100.0f;
  gThresholdTime = activeSettings.thresholdTime_x100 / 100.0f;
  gMode = activeSettings.mode;
}

// ======================================================
// Mode text logic
// ======================================================
const char* getModeTopText() {
  switch (gMode) {
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

// ======================================================
// Runtime execution helpers
// ======================================================
void setExecutionEvent(EventName ev) {
  currentEventName = ev;
}

void stopExecutionNow() {
  if (Execution_state) {
    frozenExecutionDurationMs = millis() - executionStartMs;
  }
  Execution_state = false;
  currentScreen = Screen::StartConfirm;
}

void startExecutionNow() {
  commitGlobalsForRuntime();

  Execution_state = true;
  executionStartMs = millis();
  frozenExecutionDurationMs = 0;
  currentEventName = EventName::Start;
  currentScreen = Screen::RunScreen;
}

// ======================================================
// Digit editor helpers
// ======================================================
void openDigitEditor(const char* label, int* target, int minValue = 0, int maxValue = 9999) {
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
// Input handlers
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
      case 0: // Start
        currentScreen = Screen::StartConfirm;
        break;

      case 1: // Start Settings
        currentScreen = Screen::StartSettings;
        break;

      case 2: // Settings
        settingsIndex = 0;
        currentScreen = Screen::Settings;
        break;
    }
  }
}

void handleStartConfirmInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    // Mode 1 cycles per new start press
    if (workingSettings.mode == 1) {
      Mode1SubMode currentStage = mode1SubMode;
      startExecutionNow();

      switch (currentStage) {
        case Mode1SubMode::RightTurn:
          setExecutionEvent(EventName::RightTurn);
          mode1SubMode = Mode1SubMode::LeftTurn;
          break;
        case Mode1SubMode::LeftTurn:
          setExecutionEvent(EventName::LeftTurn);
          mode1SubMode = Mode1SubMode::Selection;
          break;
        case Mode1SubMode::Selection:
          setExecutionEvent(EventName::Start); // selection placeholder
          mode1SubMode = Mode1SubMode::RightTurn;
          break;
      }
    } else {
      startExecutionNow();

      if (workingSettings.mode == 2) setExecutionEvent(EventName::RightTurn);
      if (workingSettings.mode == 3) setExecutionEvent(EventName::LeftTurn);
      if (workingSettings.mode == 4) setExecutionEvent(EventName::Start); // selection placeholder
    }

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
      case 0: // PID
        pidIndex = 0;
        currentScreen = Screen::PID;
        break;

      case 1: // Threshold
        openDigitEditor("Threshold", &workingSettings.thresholdTime_x100, 0, 9999);
        break;

      case 2: // Save
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
// Drawing helpers
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

// ======================================================
// Screens
// ======================================================
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
  u8g2.drawStr(20, 30, "START");

  u8g2.setFont(u8g2_font_5x8_tf);

  if (gMode == 1 && modeSequenceStarted && !modeSequenceFinished) {
    String nextLine = "Next: " + String(submodeToString(mode1SubMode));
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

  // Shift digit highlight here
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
// Menu loop only inside setup()
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
// Example execution logic
// Replace with your real logic
// ======================================================
void updateExecutionLogic() {
  // Example auto-stop by threshold time
  if (Execution_state) {
    uint32_t elapsedMs = millis() - executionStartMs;

    //if (elapsedMs >= (uint32_t)(gThresholdTime * 1000.0f)) { Kamal - this part triggers Finish on the screen, make sure when the Finish is reached this commented code is executed
    setExecutionEvent(EventName::Finish);// ## This part is to display events
    stopExecutionNow();// - NO STOP WHEN NOT FINISH + save the time elapsedMS in global and add to the sum
    //}
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

const char* submodeToString(Mode1SubMode sm) {
  switch (sm) {
    case Mode1SubMode::RightTurn: return "Right Turn";
    case Mode1SubMode::LeftTurn:  return "Left Turn";
    case Mode1SubMode::Selection: return "Selection";
    default: return "Unknown";
  }
}

EventName eventForSubmode(Mode1SubMode sm) {
  switch (sm) {
    case Mode1SubMode::RightTurn: return EventName::RightTurn;
    case Mode1SubMode::LeftTurn:  return EventName::LeftTurn;
    case Mode1SubMode::Selection: return EventName::Start; // no Selection event in your EventName list
    default:                      return EventName::Start;
  }
}

void resetCycleTimes() {
  time_first_trial = 0;
  time_second_trial = 0;
  time_third_trial = 0;
  total_cycle_time = 0;
  frozenExecutionDurationMs = 0;
}

void ScreenLoopUpdate(EventName event_name = EventName::Start, bool stop_machine = false) {
  ButtonEvent btn = pollButtonEvent();

  // --------------------------------------------------
  // CASE 1: machine is NOT running
  // stay on StartConfirm and wait for button press
  // --------------------------------------------------
  if (!Execution_state) {
    currentScreen = Screen::StartConfirm;

    if (btn == ButtonEvent::ShortPress) {
      // Start / restart next trial
      if (gMode == 1) {
        // Start a new 3-step cycle only if not already in progress
        // or if the previous one fully finished
        if (!modeSequenceStarted || modeSequenceFinished) {
          resetCycleTimes();
          modeSequenceStarted = true;
          modeSequenceFinished = false;
          cycleStartMs = millis();
          mode1SubMode = Mode1SubMode::RightTurn;
        }
        // else: keep current mode1SubMode as the next pending leg
      } else {
        // Modes 2,3,4 are single-stage cycles
        resetCycleTimes();
        modeSequenceStarted = true;
        modeSequenceFinished = false;
        cycleStartMs = millis();
        mode1SubMode = fixedSubmodeForMode(gMode);
      }

      executionStartMs = millis();
      frozenExecutionDurationMs = 0;
      Execution_state = true;
      currentScreen = Screen::RunScreen;

      // Set initial event for the leg being started
      setExecutionEvent(eventForSubmode(mode1SubMode));
    }

    if (btn == ButtonEvent::LongPress) {
      currentScreen = Screen::Root;
    }

    drawUI();
    return;
  }

  // --------------------------------------------------
  // CASE 2: machine is RUNNING
  // update displayed event and run screen
  // --------------------------------------------------
  currentScreen = Screen::RunScreen;
  setExecutionEvent(event_name);

  // If stop is requested, freeze timer immediately,
  // store trial time(s), go to StartConfirm and wait.
  if (stop_machine) {
    uint32_t finishTime = millis() - executionStartMs;
    frozenExecutionDurationMs = finishTime;

    switch (gMode) {
      case 1:
        switch (mode1SubMode) {
          case Mode1SubMode::RightTurn:
            time_first_trial = finishTime;
            mode1SubMode = Mode1SubMode::LeftTurn;   // next leg
            break;

          case Mode1SubMode::LeftTurn:
            time_second_trial = finishTime;
            mode1SubMode = Mode1SubMode::Selection;  // next leg
            break;

          case Mode1SubMode::Selection:
            time_third_trial = finishTime;
            total_cycle_time = millis() - cycleStartMs;
            modeSequenceFinished = true;
            mode1SubMode = Mode1SubMode::RightTurn;  // prepare next full cycle
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

    // Show finish event on stop
    setExecutionEvent(EventName::Finish);

    // Stop immediately and go back to start screen
    Execution_state = false;
    currentScreen = Screen::StartConfirm;

    drawUI();
    return;
  }

  // Still running
  drawUI();
}

// ======================================================
// Arduino setup / loop
// ======================================================

void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(50);

  Wire.begin();
  u8g2.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), onEncoderAChange, CHANGE);

  // initial values
  activeSettings.p_x100 = 125;
  activeSettings.i_x100 = 5;
  activeSettings.d_x100 = 20;
  activeSettings.thresholdTime_x100 = 150;
  activeSettings.mode = 1;

  runMenuInSetup();
}

void loop() {
  // --------------------------------------------------
  // YOUR machine logic determines:
  // 1. current event to display
  // 2. whether machine should stop now
  // --------------------------------------------------

  EventName displayedEvent = EventName::Straight;
  bool stopNow = false;

  // Example logic:
  // displayedEvent = EventName::LeftTurn;
  // if (some_finish_condition) stopNow = true;

  ScreenLoopUpdate(displayedEvent, stopNow);

  // Optional debug
  Serial.print("Exec=");
  Serial.print(Execution_state ? "1" : "0");
  Serial.print(" Mode=");
  Serial.print(gMode);
  Serial.print(" CurrentEvent=");
  Serial.print(eventNameToString(currentEventName));
  Serial.print(" T1=");
  Serial.print(time_first_trial);
  Serial.print(" T2=");
  Serial.print(time_second_trial);
  Serial.print(" T3=");
  Serial.print(time_third_trial);
  Serial.print(" Total=");
  Serial.println(total_cycle_time);

  delay(20);
}