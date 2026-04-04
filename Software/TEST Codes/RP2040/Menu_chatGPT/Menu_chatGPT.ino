#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ======================================================
// Hardware pins - CHANGE THESE TO MATCH YOUR WIRING
// ======================================================
constexpr uint8_t ENC_A_PIN   = 6;
constexpr uint8_t ENC_B_PIN   = 7;
constexpr uint8_t ENC_BTN_PIN = 8;

// OLED I2C 128x64 SSD1306 example D18 - SDA
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ======================================================
// Settings model
// Scaled x100 for exact editing of values shown as 00.00
// ======================================================
struct RuntimeSettings {
  int p_x100 = 100;               // 1.00
  int i_x100 = 0;                 // 0.00
  int d_x100 = 0;                 // 0.00
  int thresholdTime_x100 = 100;   // 1.00
  uint8_t testMode = 1;           // 1..4
};

RuntimeSettings activeSettings;
RuntimeSettings workingSettings;

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
  bool stableLevel = HIGH;             // INPUT_PULLUP => HIGH not pressed
  bool lastRawLevel = HIGH;
  uint32_t lastDebounceMs = 0;
  uint32_t pressedAtMs = 0;
  bool longPressFired = false;
};

ButtonState buttonState;

constexpr uint32_t BUTTON_DEBOUNCE_MS = 25;
constexpr uint32_t BUTTON_LONGPRESS_MS = 700;

// ======================================================
// Menu definitions
// ======================================================
enum class Screen : uint8_t {
  Root,
  TestStart,
  Settings,
  PID,
  DigitEditor,
  StartAction,
  IMUTemplate
};

enum class EditTarget : uint8_t {
  None,
  P,
  I,
  D,
  ThresholdTime,
  TestMode
};

Screen currentScreen = Screen::Root;
Screen previousScreen = Screen::Root;

int rootIndex = 0;
int testStartIndex = 0;
int settingsIndex = 0;
int pidIndex = 0;
bool settingsSessionActive = false;

constexpr int ROOT_COUNT = 3;
constexpr int TESTSTART_COUNT = 2;  // Mode, Back
constexpr int SETTINGS_COUNT = 5;   // PID, Threshold, IMU, Save, Cancel
constexpr int PID_COUNT = 4;        // P, I, D, Back

// ======================================================
// Digit editor state
// For values 00.00 => integer 0..9999
// digits: [thousands][hundreds].[tens][ones]
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
// Main execution flags / demo state
// ======================================================
bool settingsUpdated = false;
bool startRequested = false;
uint8_t startMode = 0;      // "Start" sets start mode 0
uint8_t selectedTestMode = 1;

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
  int frac  = value % 100;

  char buf[8];
  snprintf(buf, sizeof(buf), "%02d.%02d", whole, frac);
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
  newValue = clampInt(newValue, digitEditor.minValue, digitEditor.maxValue);
  digitEditor.tempValue = newValue;
}

// ======================================================
// Encoder ISR
// Simple quadrature handling
// ======================================================
void onEncoderAChange() 
{
  bool a = digitalRead(ENC_A_PIN);
  bool b = digitalRead(ENC_B_PIN);

  // Determine direction on A edge
  if (a == b) {
    if (g_encoderDelta < 100) g_encoderDelta++;
  } else {
    if (g_encoderDelta > -100) g_encoderDelta--;
  }
}

// ======================================================
// Button polling
// ======================================================
ButtonEvent pollButtonEvent() {
  uint32_t now = millis();
  bool raw = digitalRead(ENC_BTN_PIN);

  if (raw != buttonState.lastRawLevel) {
    buttonState.lastDebounceMs = now;
    buttonState.lastRawLevel = raw;
  }

  if ((now - buttonState.lastDebounceMs) > BUTTON_DEBOUNCE_MS) {
    if (raw != buttonState.stableLevel) {
      buttonState.stableLevel = raw;

      // pressed
      if (buttonState.stableLevel == LOW) {
        buttonState.pressedAtMs = now;
        buttonState.longPressFired = false;
      }
      // released
      else {
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
// Settings helpers
// ======================================================
void beginSettingsSession() {
  workingSettings = activeSettings;
  settingsSessionActive = true;
}

void cancelSettingsSession() {
  workingSettings = activeSettings;
  settingsSessionActive = false;
}

void saveSettingsSession() {
  activeSettings = workingSettings;
  settingsUpdated = true;
  settingsSessionActive = false;
}

// ======================================================
// Editor control
// ======================================================
void openDigitEditor(const char* label, int* target, int minValue = 0, int maxValue = 9999) {
  digitEditor.active = true;
  digitEditor.digitUnlocked = false;
  digitEditor.selectedDigit = 0;
  digitEditor.targetValue = target;
  digitEditor.tempValue = *target;
  digitEditor.minValue = minValue;
  digitEditor.maxValue = maxValue;
  digitEditor.label = label;

  previousScreen = currentScreen;
  currentScreen = Screen::DigitEditor;
}

void commitDigitEditor() {
  if (digitEditor.targetValue != nullptr) {
    *digitEditor.targetValue = clampInt(digitEditor.tempValue, digitEditor.minValue, digitEditor.maxValue);
  }
  digitEditor.active = false;
  digitEditor.digitUnlocked = false;
  currentScreen = previousScreen;
}

void cancelDigitEditor() {
  digitEditor.active = false;
  digitEditor.digitUnlocked = false;
  currentScreen = previousScreen;
}

// ======================================================
// Actions
// ======================================================
void doStartMode0() {
  startMode = 0;
  startRequested = true;
  currentScreen = Screen::StartAction;
}

void doStartTestMode() {
  startMode = activeSettings.testMode;
  startRequested = true;
  currentScreen = Screen::StartAction;
}

// ======================================================
// Input handling
// ======================================================
int consumeEncoderDelta() {
  noInterrupts();
  int delta = g_encoderDelta;
  g_encoderDelta = 0;
  interrupts();
  return delta;
}

void handleRootInput(int delta, ButtonEvent btn) {
  if (delta != 0) {
    rootIndex += (delta > 0) ? 1 : -1;
    if (rootIndex < 0) rootIndex = ROOT_COUNT - 1;
    if (rootIndex >= ROOT_COUNT) rootIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (rootIndex) {
      case 0:
        doStartMode0();
        break;
      case 1:
        testStartIndex = 0;
        currentScreen = Screen::TestStart;
        break;
      case 2:
        beginSettingsSession();
        settingsIndex = 0;
        currentScreen = Screen::Settings;
        break;
    }
  }
}

void handleTestStartInput(int delta, ButtonEvent btn) {
  if (delta != 0) {
    testStartIndex += (delta > 0) ? 1 : -1;
    if (testStartIndex < 0) testStartIndex = TESTSTART_COUNT - 1;
    if (testStartIndex >= TESTSTART_COUNT) testStartIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (testStartIndex) {
      case 0:
        // directly edit mode with encoder; short press confirms start
        activeSettings.testMode++;
        if (activeSettings.testMode > 4) activeSettings.testMode = 1;
        break;
      case 1:
        currentScreen = Screen::Root;
        break;
    }
  }

  if (delta != 0 && testStartIndex == 0) {
    int mode = activeSettings.testMode;
    mode += (delta > 0) ? 1 : -1;
    if (mode < 1) mode = 4;
    if (mode > 4) mode = 1;
    activeSettings.testMode = static_cast<uint8_t>(mode);
  }

  if (btn == ButtonEvent::LongPress) {
    doStartTestMode();
  }
}

void handleSettingsInput(int delta, ButtonEvent btn) {
  if (delta != 0) {
    settingsIndex += (delta > 0) ? 1 : -1;
    if (settingsIndex < 0) settingsIndex = SETTINGS_COUNT - 1;
    if (settingsIndex >= SETTINGS_COUNT) settingsIndex = 0;
  }

  if (btn == ButtonEvent::ShortPress) {
    switch (settingsIndex) {
      case 0: // PID
        pidIndex = 0;
        currentScreen = Screen::PID;
        break;
      case 1: // Threshold Time
        openDigitEditor("Threshold", &workingSettings.thresholdTime_x100, 0, 9999);
        break;
      case 2: // IMU
        currentScreen = Screen::IMUTemplate;
        break;
      case 3: // Save
        saveSettingsSession();
        currentScreen = Screen::Root;
        break;
      case 4: // Cancel
        cancelSettingsSession();
        currentScreen = Screen::Root;
        break;
    }
  }

  if (btn == ButtonEvent::LongPress) {
    cancelSettingsSession();
    currentScreen = Screen::Root;
  }
}

void handlePIDInput(int delta, ButtonEvent btn) {
  if (delta != 0) {
    pidIndex += (delta > 0) ? 1 : -1;
    if (pidIndex < 0) pidIndex = PID_COUNT - 1;
    if (pidIndex >= PID_COUNT) pidIndex = 0;
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
        digitEditor.selectedDigit = (digitEditor.selectedDigit + 1) % 4;
      } else {
        digitEditor.selectedDigit = (digitEditor.selectedDigit == 0) ? 3 : (digitEditor.selectedDigit - 1);
      }
    } else {
      incrementSelectedDigit(delta > 0 ? 1 : -1);
    }
  }

  if (btn == ButtonEvent::ShortPress) {
    digitEditor.digitUnlocked = !digitEditor.digitUnlocked;
  }

  if (btn == ButtonEvent::LongPress) {
    // Save current temp value and return
    commitDigitEditor();
  }
}

void handleStartActionInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress || btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Root;
  }
}

void handleIMUTemplateInput(ButtonEvent btn) {
  if (btn == ButtonEvent::ShortPress || btn == ButtonEvent::LongPress) {
    currentScreen = Screen::Settings;
  }
}

// ======================================================
// Drawing
// ======================================================
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, title);
  u8g2.drawHLine(0, 12, 128);
}

void drawMenuItem(int y, bool selected, const String& text) {
  if (selected) {
    u8g2.drawBox(0, y - 10, 128, 12);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, y, text.c_str());
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, y, text.c_str());
  }
}

void drawRootScreen() {
  drawHeader("Main Menu");

  drawMenuItem(26, rootIndex == 0, "Start");
  drawMenuItem(38, rootIndex == 1, "Test Start");
  drawMenuItem(50, rootIndex == 2, "Settings");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 62, "Short: select");
}

void drawTestStartScreen() {
  drawHeader("Test Start");

  String modeLine = "Mode: " + String(activeSettings.testMode);
  drawMenuItem(26, testStartIndex == 0, modeLine);
  drawMenuItem(38, testStartIndex == 1, "Back");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 62, "Rotate mode / Long: start");
}

void drawSettingsScreen() {
  drawHeader("Settings");

  drawMenuItem(22, settingsIndex == 0, "PID");
  drawMenuItem(32, settingsIndex == 1, "Threshold: " + formatX100(workingSettings.thresholdTime_x100));
  drawMenuItem(42, settingsIndex == 2, "IMU");
  drawMenuItem(52, settingsIndex == 3, "Save Settings");
  drawMenuItem(62, settingsIndex == 4, "Cancel");
}

void drawPIDScreen() {
  drawHeader("PID");

  drawMenuItem(22, pidIndex == 0, "P: " + formatX100(workingSettings.p_x100));
  drawMenuItem(32, pidIndex == 1, "I: " + formatX100(workingSettings.i_x100));
  drawMenuItem(42, pidIndex == 2, "D: " + formatX100(workingSettings.d_x100));
  drawMenuItem(52, pidIndex == 3, "Back");
}

void drawDigitEditorScreen() {
  drawHeader(digitEditor.label);

  String valueStr = formatX100(digitEditor.tempValue);

  // Big-ish display of value
  u8g2.setFont(u8g2_font_logisoso20_tn);
  u8g2.drawStr(12, 38, valueStr.c_str());

  // Digit selector underline
  // positions approximated for "00.00" in this font
  int xPos[4] = {16, 34, 64, 82};
  int underlineY = 42;

  if (!digitEditor.digitUnlocked) {
    u8g2.drawHLine(xPos[digitEditor.selectedDigit], underlineY, 12);
  } else {
    u8g2.drawFrame(xPos[digitEditor.selectedDigit] - 2, 14, 16, 30);
  }

  u8g2.setFont(u8g2_font_5x8_tf);
  if (!digitEditor.digitUnlocked) {
    u8g2.drawStr(0, 62, "Rotate=select digit, Press=unlock");
  } else {
    u8g2.drawStr(0, 62, "Rotate=change digit, Press=lock");
  }
}

void drawStartActionScreen() {
  drawHeader("Start Requested");

  String line1 = "Start mode: " + String(startMode);
  drawMenuItem(30, false, line1);
  drawMenuItem(44, false, "Main loop will use it");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 62, "Press to return");
}

void drawIMUTemplateScreen() {
  drawHeader("IMU");

  drawMenuItem(28, false, "Template page");
  drawMenuItem(42, false, "Add IMU options here");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 62, "Press to return");
}

void drawUI() {
  u8g2.clearBuffer();

  switch (currentScreen) {
    case Screen::Root:        drawRootScreen(); break;
    case Screen::TestStart:   drawTestStartScreen(); break;
    case Screen::Settings:    drawSettingsScreen(); break;
    case Screen::PID:         drawPIDScreen(); break;
    case Screen::DigitEditor: drawDigitEditorScreen(); break;
    case Screen::StartAction: drawStartActionScreen(); break;
    case Screen::IMUTemplate: drawIMUTemplateScreen(); break;
  }

  u8g2.sendBuffer();
}

// ======================================================
// Example main control / execution loop usage
// Replace this with your real application logic
// ======================================================
void applyUpdatedSettingsIfNeeded() {
  if (settingsUpdated) {
    settingsUpdated = false;

    // Example: recalculate controller coefficients here
    // float p = activeSettings.p_x100 / 100.0f;
    // float i = activeSettings.i_x100 / 100.0f;
    // float d = activeSettings.d_x100 / 100.0f;

    Serial.println("Settings committed:");
    Serial.print("P = "); Serial.println(formatX100(activeSettings.p_x100));
    Serial.print("I = "); Serial.println(formatX100(activeSettings.i_x100));
    Serial.print("D = "); Serial.println(formatX100(activeSettings.d_x100));
    Serial.print("Threshold = "); Serial.println(formatX100(activeSettings.thresholdTime_x100));
    Serial.print("Test Mode = "); Serial.println(activeSettings.testMode);
  }
}

void handleStartRequestIfNeeded() {
  if (startRequested) {
    startRequested = false;

    Serial.print("START requested, mode = ");
    Serial.println(startMode);

    // Here call your actual start / test execution code
    // startMode == 0 -> normal start
    // startMode == 1..4 -> chosen test mode
  }
}

// ======================================================
// Arduino setup/loop
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
  activeSettings.p_x100 = 125;              // 1.25
  activeSettings.i_x100 = 5;                // 0.05
  activeSettings.d_x100 = 20;               // 0.20
  activeSettings.thresholdTime_x100 = 150;  // 1.50
  activeSettings.testMode = 1;

  workingSettings = activeSettings;
}

void loop() {
  int delta = consumeEncoderDelta();
  ButtonEvent btn = pollButtonEvent();

  switch (currentScreen) {
    case Screen::Root:
      handleRootInput(delta, btn);
      break;

    case Screen::TestStart:
      handleTestStartInput(delta, btn);
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

    case Screen::StartAction:
      handleStartActionInput(btn);
      break;

    case Screen::IMUTemplate:
      handleIMUTemplateInput(btn);
      break;
  }

  applyUpdatedSettingsIfNeeded();
  handleStartRequestIfNeeded();
  drawUI();

  delay(10);
}
