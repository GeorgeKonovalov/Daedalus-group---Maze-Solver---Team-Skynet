// Pins
const int pinA = 7;
const int pinB = 6;
const int buttonPin = 8;

// Encoder state
volatile int encoderDelta = 0;
volatile bool moved = false;

// Button state
bool lastButtonState = HIGH;

// Timing for multiplier
unsigned long lastTickTime = 0;

// Menu cursor
int cursor = 0;

// Menu item structure
struct MenuItem {
  const char* name;
  int value;          // current value
  int minValue;
  int maxValue;
  int step;           // normal increment
  void (*action)();   // optional function to execute
};

// Example menu
MenuItem menu[] = {
  {"Item 1", 0, 0, 100, 1, nullptr},
  {"Item 2", 50, 0, 200, 5, nullptr},
  {"Item 3", 10, 0, 50, 1, nullptr},
};

const int menuSize = sizeof(menu) / sizeof(menu[0]);

// ------------------- Setup -------------------
void setup() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP); 

  Serial.begin(115200);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, CHANGE);

  Serial.println("Menu Ready!");
}

// ------------------- Main Loop -------------------
void loop() {
  // ----- Handle encoder rotation -----
  if (moved) {
    noInterrupts();
    int delta = encoderDelta;
    encoderDelta = 0;
    moved = false;
    interrupts();

    unsigned long now = millis();
    int multiplier = getStepMultiplier(now - lastTickTime);
    lastTickTime = now;

    if (delta != 0) {
      // If rotating while in menu list, move cursor
      cursor += delta * multiplier;

      // Wrap around
      if (cursor < 0) cursor = menuSize - 1;
      if (cursor >= menuSize) cursor = 0;

      displayMenu();
    }
  }

  // ----- Button logic handled in ISR -----
  // Nothing extra needed in loop
}

// ------------------- Encoder ISR -------------------
void handleEncoder() {
  bool A = digitalRead(pinA);
  bool B = digitalRead(pinB);

  if (A == B) {
    encoderDelta++;
  } else {
    encoderDelta--;
  }

  moved = true;
}

// ------------------- Button ISR -------------------
void buttonPressed() {
  if (digitalRead(buttonPin) == LOW) {
    lastButtonState = LOW;
    // Example action: print current value
    Serial.print("Selected: ");
    Serial.print(menu[cursor].name);
    Serial.print(" = ");
    Serial.println(menu[cursor].value);
    // Optionally call menu action
    if (menu[cursor].action != nullptr) menu[cursor].action();
  } else {
    lastButtonState = HIGH;
  }
}

// ------------------- Multiplier function -------------------
int getStepMultiplier(unsigned long deltaTime) {
  if (deltaTime < 50) return 10;
  if (deltaTime < 150) return 5;
  return 1;
}

// ------------------- Display function -------------------
void displayMenu() {
  Serial.println("---- MENU ----");
  for (int i = 0; i < menuSize; i++) {
    if (i == cursor) Serial.print("> ");
    else Serial.print("  ");
    Serial.print(menu[i].name);
    Serial.print(": ");
    Serial.println(menu[i].value);
  }
  Serial.println("--------------");
}
