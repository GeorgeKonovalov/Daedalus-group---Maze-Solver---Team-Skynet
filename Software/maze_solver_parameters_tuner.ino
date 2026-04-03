// ===== PID TUNER WITH POT + BUTTON =====
// Button cycles: KP → KI → KD → EVENT DELAY → START

// ===== PIN DEFINITIONS =====
#define POT_PIN     A0
#define BUTTON_PIN  2

// ===== PID PARAMETERS =====
float Kp = 0.05;
float Ki = 0.00;
float Kd = 0.00;

// ===== EVENT DELAY =====
float eventDelay = 0.0;   // seconds — adjust range below

// ===== TUNING RANGES (adjust freely) =====
const float KP_MIN = 0.0,  KP_MAX = 1.0;
const float KI_MIN = 0.0,  KI_MAX = 0.5;
const float KD_MIN = 0.0,  KD_MAX = 0.2;
const float ED_MIN = 0.0,  ED_MAX = 5.0;   // event delay range (seconds)

// ===== START TRIGGER THRESHOLD =====
const float START_THRESHOLD = 0.90;

// ===== MENU STATES =====
enum MenuState { MENU_KP, MENU_KI, MENU_KD, MENU_DELAY, MENU_START };
MenuState currentMenu = MENU_KP;

// ===== BUTTON DEBOUNCE =====
bool lastButtonState     = HIGH;
bool buttonState         = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// ===== SERIAL UPDATE RATE =====
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 150;

bool robotRunning = false;

// ===================================================
void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  printMenu();
}

// ===================================================
void loop() {
  handleButton();
  handlePot();
  printLive();
}

// ===================================================
float readPotNorm() {
  return analogRead(POT_PIN) / 1023.0;
}

float mapFloat(float norm, float minVal, float maxVal) {
  return minVal + norm * (maxVal - minVal);
}

// ===================================================
void handlePot() {
  float norm = readPotNorm();

  switch (currentMenu) {
    case MENU_KP:    Kp         = mapFloat(norm, KP_MIN, KP_MAX); break;
    case MENU_KI:    Ki         = mapFloat(norm, KI_MIN, KI_MAX); break;
    case MENU_KD:    Kd         = mapFloat(norm, KD_MIN, KD_MAX); break;
    case MENU_DELAY: eventDelay = mapFloat(norm, ED_MIN, ED_MAX); break;

    case MENU_START:
      if (norm >= START_THRESHOLD && !robotRunning) {
        robotRunning = true;
        Serial.println(F("\n============================="));
        Serial.println(F("  ✓ ROBOT STARTED"));
        Serial.print(F("  Kp="));         Serial.println(Kp, 4);
        Serial.print(F("  Ki="));         Serial.println(Ki, 4);
        Serial.print(F("  Kd="));         Serial.println(Kd, 4);
        Serial.print(F("  EventDelay=")); Serial.print(eventDelay, 2); Serial.println(F("s"));
        Serial.println(F("=============================\n"));
        startRobot();
      }
      break;
  }
}

// ===================================================
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) advanceMenu();
    }
  }

  lastButtonState = reading;
}

// ===================================================
void advanceMenu() {
  robotRunning = false;

  switch (currentMenu) {
    case MENU_KP:    currentMenu = MENU_KI;    break;
    case MENU_KI:    currentMenu = MENU_KD;    break;
    case MENU_KD:    currentMenu = MENU_DELAY; break;   // <-- new step
    case MENU_DELAY: currentMenu = MENU_START; break;
    case MENU_START: currentMenu = MENU_KP;    break;
  }

  printMenu();
}

// ===================================================
void printMenu() {
  Serial.println(F("\n-----------------------------"));
  switch (currentMenu) {
    case MENU_KP:
      Serial.println(F("  MENU: Adjust  Kp"));
      Serial.print(F("  Range: ")); Serial.print(KP_MIN); Serial.print(F(" – ")); Serial.println(KP_MAX);
      break;
    case MENU_KI:
      Serial.println(F("  MENU: Adjust  Ki"));
      Serial.print(F("  Range: ")); Serial.print(KI_MIN); Serial.print(F(" – ")); Serial.println(KI_MAX);
      break;
    case MENU_KD:
      Serial.println(F("  MENU: Adjust  Kd"));
      Serial.print(F("  Range: ")); Serial.print(KD_MIN); Serial.print(F(" – ")); Serial.println(KD_MAX);
      break;
    case MENU_DELAY:
      Serial.println(F("  MENU: Adjust  Event Delay"));
      Serial.print(F("  Range: ")); Serial.print(ED_MIN); Serial.print(F("s – ")); Serial.print(ED_MAX); Serial.println(F("s"));
      break;
    case MENU_START:
      Serial.println(F("  MENU: START"));
      Serial.println(F("  Rotate pot >90% right to launch"));
      break;
  }
  Serial.println(F("  [Press button for next menu]"));
  Serial.println(F("-----------------------------"));
}

// ===================================================
void printLive() {
  if (millis() - lastPrintTime < PRINT_INTERVAL) return;
  lastPrintTime = millis();

  float norm = readPotNorm();

  Serial.print(F("Kp="));    Serial.print(Kp, 4);
  Serial.print(F("  Ki="));  Serial.print(Ki, 4);
  Serial.print(F("  Kd="));  Serial.print(Kd, 4);
  Serial.print(F("  Delay=")); Serial.print(eventDelay, 2); Serial.print(F("s"));
  Serial.print(F("  | Pot=")); Serial.print((int)(norm * 100)); Serial.print(F("%"));

  if (currentMenu == MENU_START) {
    Serial.print(norm >= START_THRESHOLD ? F("  >> RELEASE TO START") : F("  >> turn right to start"));
  }

  Serial.println();
}

// ===================================================
void startRobot() {
  // e.g. delay((unsigned long)(eventDelay * 1000));
  //      myPID.SetTunings(Kp, Ki, Kd);
  //      motorsEnable();
}
