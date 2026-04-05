#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ======================================================
// Hardware pins
// ======================================================
constexpr uint8_t ENC_A_PIN   = 2;
constexpr uint8_t ENC_B_PIN   = 3;
constexpr uint8_t ENC_BTN_PIN = 4;

// OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ======================================================
// Tuning
// ======================================================
constexpr int MENU_STEPS_PER_MOVE = 2;
constexpr int DIGIT_STEPS_PER_MOVE = 2;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 25;
constexpr uint32_t BUTTON_LONGPRESS_MS = 700;

// ======================================================
// Saved parameters used globally by runtime
// ======================================================
struct RuntimeSettings {
  int p_x100 = 125;               // 1.25
  int i_x100 = 5;                 // 0.05
  int d_x100 = 20;                // 0.20
  int thresholdTime_x100 = 150;   // 1.50
  uint8_t testMode = 1;
};

RuntimeSettings savedSettings;    // committed values used by runtime
RuntimeSettings workingSettings;  // edited in menu before save

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
// UI / app states
// ======================================================
enum class AppMode : uint8_t {
  MenuSetup,
  Running
};

enum class Screen : uint8_t {
  Root,
  Settings,
  PID,
  DigitEditor,
  StartConfirm
};

AppMode appMode = AppMode::MenuSetup;
Screen currentScreen = Screen::Root;
Screen previousScreen = Screen::Root;

// ======================================================
// Generic menu state
// ======================================================
struct MenuState {
  int index = 0;
  int stepAccum = 0;
};

MenuState rootMenu;
MenuState settingsMenu;
MenuState pidMenu;

// ======================================================
// Digit editor
// ======================================================
struct DigitEditor {
  bool active = false;
  bool digitUnlocked = false;
  uint8_t selectedDigit = 0;   // 0..3
  int tempValue = 0;
  int* targetValue = nullptr;
  int minValue = 0;
  int maxValue = 9999;
  const char* label = "";

  int selectStepAccum = 0;
  int changeStepAccum = 0;
};

DigitEditor digitEditor;

// ======================================================
// Runtime execution state
// ======================================================
bool settingsUpdated = false;
bool startRequested = false;

bool runActive = false;
bool triggerOccurred = false;

uint32_t runStartMs = 0;
uint32_t triggerMs = 0;

// ======================================================
// Menu sizes
// ======================================================
constexpr int ROOT_COUNT = 2;      // Start, Settings
constexpr int SETTINGS_COUNT = 3;  // PID, Threshold, Save
constexpr int PID_COUNT = 4;       // P, I, D, Back

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

  char buf[16];
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

void resetMenuState(MenuState& menu) {
  menu.index = 0;
  menu.stepAccum = 0;
}

void resetDigitEditorAccums() {
  digitEditor.selectStepAccum = 0;
  digitEditor.changeStepAccum = 0;
}

void applySmoothClampedDelta(int delta, int stepsPerMove, MenuState& menu, int itemCount) {
  if (delta == 0) return;

  menu.stepAccum += delta;

  while (menu.stepAccum >= stepsPerMove) {
    if (menu.index < itemCount - 1) menu.index++;
    menu.stepAccum -= stepsPerMove;
  }

  while (menu.stepAccum <= -stepsPerMove) {
    if (menu.index > 0) menu.index--;
    menu.stepAccum += stepsPerMove;
  }
}

void applySmoothClampedDigitSelect(int delta) {
  if (delta == 0) return;

  digitEditor.selectStepAccum += delta;

  while (digitEditor.selectStepAccum >= DIGIT_STEPS_PER_MOVE) {
    if (digitEditor.selectedDigit < 3) digitEditor.selectedDigit++;
    digitEditor.selectStepAccum -= DIGIT_STEPS_PER_MOVE;
  }

  while (digitEditor.selectStepAccum <= -DIGIT_STEPS_PER_MOVE) {
    if (digitEditor.selectedDigit > 0) digitEditor.selectedDigit--;
    digitEditor.selectStepAccum += DIGIT_STEPS_PER_MOVE;
  }
}

void incrementSelectedDigit(int step) {
  if (!digitEditor.active) return;

  int oldDigit = getDigitAt(digitEditor.tempValue, digitEditor.selectedDigit);
  int newDigit = oldDigit + step;

  while (newDigit < 0) newDigit += 10;
  while (newDigit > 9) newDigit -= 10;

  int newValue = setDigitAt(digitEditor.tempValue, digitEditor.selectedDigit, newDigit);
  digitEditor.tempValue = clampInt(newValue, digitEditor.minValue, digitEditor.maxValue);
}

void applySmoothDigitChange(int delta) {
  if (delta == 0) return;

  digitEditor.changeStepAccum += delta;

  while (digitEditor.changeStepAccum >= DIGIT_STEPS_PER_MOVE) {
    incrementSelectedDigit(1);
    digitEditor.changeStepAccum -= DIGIT_STEPS_PER_MOVE;
  }

  while (digitEditor.changeStepAccum <= -DIGIT_STEPS_PER_MOVE) {
    incrementSelectedDigit(-1);
    digitEditor.changeStepAccum += DIGIT_STEPS_PER_MOVE;
  }
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
  const uint32_t now = millis();
  const bool raw = digitalRead(ENC_BTN_PIN);

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
// Settings/session
// ======================================================
void beginSettingsSession() {
  workingSettings = savedSettings;
}

void saveSettingsSession() {
  savedSettings = workingSettings;
  settingsUpdated = true;
}

// ======================================================
// Digit editor
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
  resetDigitEditorAccums();

  previousScreen = currentScreen;
  currentScreen = Screen::DigitEditor;
}

void closeDigitEditor(bool commit) {
  if (commit && digitEditor.targetValue != nullptr) {
    *digitEditor.targetValue = clampInt(digitEditor.tempValue, digitEditor.minValue, digitEditor.maxValue);
  }

  digitEditor.active = false;
  digitEditor.digitUnlocked = false;
  resetDigitEditorAccums();
  currentScreen = previousScreen;
}

// ======================================================
// Runtime control
// ======================================================
void startRun() {
  runActive = true;
  triggerOccurred = false;
  runStartMs = millis();
  triggerMs = 0;
  appMode = AppMode::Running;
}

void handleTriggerEvent() {
  if (!runActive) return;
  if (!triggerOccurred) {
    triggerOccurred = true;
    triggerMs = millis();
  }
}

// ======================================================
// Screen transitions
// ======================================================
void goToRoot() {
  currentScreen = Screen::Root;
}

void goToSettings() {
  beginSettingsSession();
  resetMenuState(settingsMenu);
  currentScreen = Screen::Settings;
}

void goToPID() {
  resetMenuState(pidMenu);
  currentScreen = Screen::PID;
}

// ======================================================
// Menu input
// ======================================================
void handleRootInput(int delta, ButtonEvent btn) {
  applySmoothClampedDelta(delta, MENU_STEPS_PER_MOVE, rootMenu, ROOT_COUNT);

  if (btn == ButtonEvent::ShortPress) {
    switch (rootMenu.index) {
      case 0:
        currentScreen = Screen::StartConfirm;
        break;
      case 1:
        goToSettings();
        break;
    }
  }
}

void handleSettingsInput(int delta, ButtonEvent btn) {
  applySmoothClampedDelta(delta, MENU_STEPS_PER_MOVE, settingsMenu, SETTINGS_COUNT);

  if (btn == ButtonEvent::ShortPress) {
    switch (settingsMenu.index) {
      case 0:
        goToPID();
        break;
      case 1:
        openDigitEditor("Threshold", &workingSettings.thresholdTime_x100, 0, 9999);
        break;
      case 2:
        saveSettingsSession();
        goToRoot();
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    goToRoot();
  }
}

void handlePIDInput(int delta, ButtonEvent btn) {
  applySmoothClampedDelta(delta, MENU_STEPS_PER_MOVE, pidMenu, PID_COUNT);

  if (btn == ButtonEvent::ShortPress) {
    switch (pidMenu.index) {
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
      applySmoothClampedDigitSelect(delta);
    } else {
      applySmoothDigitChange(delta);
    }
  }

  if (btn == ButtonEvent::ShortPress) {
    digitEditor.digitUnlocked = !digitEditor.digitUnlocked;
    resetDigitEditorAccums();
  }

  if (btn == ButtonEvent::LongPress) {
    closeDigitEditor(true);
  }
}

void handleStartConfirmInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    startRun();
  }
  if (btn == ButtonEvent::LongPress) {
    goToRoot();
  }
}

void handleMenuMode(int delta, ButtonEvent btn) {
  switch (currentScreen) {
    case Screen::Root:
      handleRootInput(delta, btn);
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
  }
}

void handleRunMode(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress) {
    handleTriggerEvent();
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

void drawVerticalMenu(
  const char* title,
  const String items[],
  int itemCount,
  int selectedIndex,
  int firstY,
  int rowStep,
  const char* footer = nullptr
) {
  drawHeader(title);

  for (int i = 0; i < itemCount; ++i) {
    drawMenuItem(firstY + i * rowStep, i == selectedIndex, items[i]);
  }

  if (footer != nullptr) {
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 63, footer);
  }
}

// ======================================================
// Menu/setup screens
// ======================================================
void drawRootScreen() {
  const String items[ROOT_COUNT] = {
    "Start",
    "Settings"
  };
  drawVerticalMenu("Setup", items, ROOT_COUNT, rootMenu.index, 30, 12, "Short: select");
}

void drawSettingsScreen() {
  const String items[SETTINGS_COUNT] = {
    "PID",
    "Threshold: " + formatX100(workingSettings.thresholdTime_x100),
    "Save"
  };
  drawVerticalMenu("Settings", items, SETTINGS_COUNT, settingsMenu.index, 28, 11);
}

void drawPIDScreen() {
  const String items[PID_COUNT] = {
    "P: " + formatX100(workingSettings.p_x100),
    "I: " + formatX100(workingSettings.i_x100),
    "D: " + formatX100(workingSettings.d_x100),
    "Back"
  };
  drawVerticalMenu("PID", items, PID_COUNT, pidMenu.index, 26, 9);
}

void drawDigitEditorScreen() {
  drawHeader(digitEditor.label);

  String valueStr = formatX100(digitEditor.tempValue);

  u8g2.setFont(u8g2_font_logisoso20_tn);
  u8g2.drawStr(12, 40, valueStr.c_str());

  const int xPos[4] = {16, 34, 64, 82};
  const int underlineY = 44;

  if (!digitEditor.digitUnlocked) {
    u8g2.drawHLine(xPos[digitEditor.selectedDigit], underlineY, 12);
  } else {
    u8g2.drawFrame(xPos[digitEditor.selectedDigit] - 2, 16, 16, 30);
  }

  u8g2.setFont(u8g2_font_5x8_tf);
  if (!digitEditor.digitUnlocked) {
    u8g2.drawStr(0, 63, "Rotate=select, press=unlock");
  } else {
    u8g2.drawStr(0, 63, "Rotate=change, press=lock");
  }
}

void drawStartConfirmScreen() {
  drawHeader("Ready to Start");

  drawMenuItem(30, false, "P: " + formatX100(savedSettings.p_x100));
  drawMenuItem(42, false, "I: " + formatX100(savedSettings.i_x100));
  drawMenuItem(54, false, "D: " + formatX100(savedSettings.d_x100));

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "Short=start, long=back");
}

// ======================================================
// Run screen
// ======================================================
void drawRunScreen() {
  drawHeader("Running");

  uint32_t now = millis();
  uint32_t elapsed = now - runStartMs;

  String totalTime = "Total: " + formatMsAsSeconds(elapsed);
  drawMenuItem(26, false, totalTime);

  String threshold = "Thr: " + formatX100(savedSettings.thresholdTime_x100);
  drawMenuItem(38, false, threshold);

  if (!triggerOccurred) {
    drawMenuItem(50, false, "Trigger: waiting");
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 63, "Press knob to trigger");
  } else {
    uint32_t sinceTrigger = now - triggerMs;
    drawMenuItem(50, false, "Trigger: done");
    drawMenuItem(62, false, "After: " + formatMsAsSeconds(sinceTrigger));
  }
}

// ======================================================
// Draw dispatcher
// ======================================================
void drawUI() {
  u8g2.clearBuffer();

  if (appMode == AppMode::MenuSetup) {
    switch (currentScreen) {
      case Screen::Root:
        drawRootScreen();
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
    }
  } else {
    drawRunScreen();
  }

  u8g2.sendBuffer();
}

// ======================================================
// Runtime hook
// ======================================================
void applyUpdatedSettingsIfNeeded() {
  if (!settingsUpdated) return;

  settingsUpdated = false;

  Serial.println("Saved parameters:");
  Serial.print("P = "); Serial.println(formatX100(savedSettings.p_x100));
  Serial.print("I = "); Serial.println(formatX100(savedSettings.i_x100));
  Serial.print("D = "); Serial.println(formatX100(savedSettings.d_x100));
  Serial.print("Threshold = "); Serial.println(formatX100(savedSettings.thresholdTime_x100));
}

// ======================================================
// Setup / loop
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

  workingSettings = savedSettings;

  resetMenuState(rootMenu);
  resetMenuState(settingsMenu);
  resetMenuState(pidMenu);
}

void loop() {
  const int delta = consumeEncoderDelta();
  const ButtonEvent btn = pollButtonEvent();

  if (appMode == AppMode::MenuSetup) {
    handleMenuMode(delta, btn);
  } else {
    handleRunMode(btn);
  }

  applyUpdatedSettingsIfNeeded();
  drawUI();

  delay(10);
}
