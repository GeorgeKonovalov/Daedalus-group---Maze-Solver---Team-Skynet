/*
  Parallax Continuous Rotation Servo — “max speed” command tester

  What this does:
  - Lets you command each of 4 CR servos to full forward / full reverse / stop
  - Lets you *sweep pulse width* to see where speed saturates (your practical max)
  - Because CR servos have NO speed feedback, this code can only COMMAND max speed,
    not MEASURE it. To measure RPM you need an encoder or a feedback servo.

  Serial Monitor (115200 baud, No line ending recommended)

  Select servo:
    1 = FL, 2 = FR, 3 = RR, 4 = RL, 0 = all

  Motion commands:
    f = full forward (STOP + RANGE)
    b = full backward (STOP - RANGE)
    s = stop (STOP)
    + = increase pulse by step (default 5 us)
    - = decrease pulse by step
    p = print current selected servo pulse value
    w = sweep (STOP -> STOP+RANGE -> STOP -> STOP-RANGE -> STOP) to see response
*/

#include <Arduino.h>
#include <Servo.h>

// --------- EDIT PINS ----------
const uint8_t PIN_FL = 10;
const uint8_t PIN_FR = 11;
const uint8_t PIN_RR = 12;
const uint8_t PIN_RL = 13;

// --------- Servo objects ----------
Servo S_FL, S_FR, S_RR, S_RL;

// --------- Calibration defaults ----------
int STOP_US[4]  = {1500, 1500, 1500, 1500}; // tune each so it really stops
int RANGE_US[4] = {250,  250,  250,  250};  // full-speed command offset
int INV[4]      = {+1,   +1,   +1,   +1};   // flip if direction is wrong

// Safety clamp (typical servo pulse limits)
const int MIN_US = 1000;
const int MAX_US = 2000;

// Step for +/- commands
int stepUs = 5;

// Latched pulse value per servo (so you can tweak & hold)
int pulseNow[4];

int selected = 0; // 0=all, 1..4 = one servo

Servo* servos[4];

int clampUs(int us) {
  if (us < MIN_US) return MIN_US;
  if (us > MAX_US) return MAX_US;
  return us;
}

void writeServoUs(int idx, int us) {
  us = clampUs(us);
  servos[idx]->writeMicroseconds(us);
  pulseNow[idx] = us;
}

void applySelected(int us) {
  if (selected == 0) {
    for (int i = 0; i < 4; i++) writeServoUs(i, us);
  } else {
    writeServoUs(selected - 1, us);
  }
}

int stopForSelected() {
  if (selected == 0) return 1500; // not used directly
  return STOP_US[selected - 1];
}

int rangeForSelected() {
  if (selected == 0) return 250;  // not used directly
  return RANGE_US[selected - 1];
}

void stopAll() {
  for (int i = 0; i < 4; i++) {
    int us = STOP_US[i];
    writeServoUs(i, us);
  }
}

void fullForwardSelected() {
  if (selected == 0) {
    for (int i = 0; i < 4; i++) {
      int us = STOP_US[i] + INV[i] * RANGE_US[i];
      writeServoUs(i, us);
    }
  } else {
    int i = selected - 1;
    int us = STOP_US[i] + INV[i] * RANGE_US[i];
    writeServoUs(i, us);
  }
}

void fullBackwardSelected() {
  if (selected == 0) {
    for (int i = 0; i < 4; i++) {
      int us = STOP_US[i] - INV[i] * RANGE_US[i];
      writeServoUs(i, us);
    }
  } else {
    int i = selected - 1;
    int us = STOP_US[i] - INV[i] * RANGE_US[i];
    writeServoUs(i, us);
  }
}

void printStatus() {
  Serial.println("\n--- Status ---");
  Serial.print("Selected: ");
  if (selected == 0) Serial.println("ALL");
  else Serial.println(selected);

  const char* names[4] = {"FL","FR","RR","RL"};
  for (int i = 0; i < 4; i++) {
    Serial.print(names[i]);
    Serial.print("  STOP=");
    Serial.print(STOP_US[i]);
    Serial.print("  RANGE=");
    Serial.print(RANGE_US[i]);
    Serial.print("  INV=");
    Serial.print(INV[i]);
    Serial.print("  NOW=");
    Serial.println(pulseNow[i]);
  }
  Serial.print("Step(us): "); Serial.println(stepUs);
  Serial.println("--------------\n");
}

void sweepSelected() {
  // For ALL: sweep each servo independently in sequence (simpler, safer)
  if (selected == 0) {
    for (int i = 0; i < 4; i++) {
      Serial.print("Sweeping servo "); Serial.println(i + 1);
      int stop = STOP_US[i];
      int range = RANGE_US[i] * INV[i];

      writeServoUs(i, stop); delay(800);
      writeServoUs(i, stop + range); delay(1200);
      writeServoUs(i, stop); delay(800);
      writeServoUs(i, stop - range); delay(1200);
      writeServoUs(i, stop); delay(800);
    }
    return;
  }

  int i = selected - 1;
  int stop = STOP_US[i];
  int range = RANGE_US[i] * INV[i];

  writeServoUs(i, stop); delay(800);
  writeServoUs(i, stop + range); delay(1200);
  writeServoUs(i, stop); delay(800);
  writeServoUs(i, stop - range); delay(1200);
  writeServoUs(i, stop); delay(800);
}

void setup() {
  Serial.begin(115200);

  S_FL.attach(PIN_FL);
  S_FR.attach(PIN_FR);
  S_RR.attach(PIN_RR);
  S_RL.attach(PIN_RL);

  servos[0] = &S_FL;
  servos[1] = &S_FR;
  servos[2] = &S_RR;
  servos[3] = &S_RL;

  // Init pulses to STOP
  for (int i = 0; i < 4; i++) pulseNow[i] = STOP_US[i];
  stopAll();

  Serial.println("CR Servo max-speed COMMAND tester ready.");
  Serial.println("Select: 1..4 (servo), 0 (all).");
  Serial.println("Commands: f full forward, b full backward, s stop, +/-, p print, w sweep.");
  Serial.println("Note: This COMMANDS max speed; it does NOT MEASURE speed (needs encoder).");

  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  char c = (char)Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  // Selection
  if (c >= '0' && c <= '4') {
    selected = c - '0';
    Serial.print("Selected: ");
    if (selected == 0) Serial.println("ALL");
    else Serial.println(selected);
    return;
  }

  switch (c) {
    case 'f':
      fullForwardSelected();
      Serial.println("Full forward (commanded max).");
      break;

    case 'b':
      fullBackwardSelected();
      Serial.println("Full backward (commanded max).");
      break;

    case 's':
      stopAll();
      Serial.println("STOP.");
      break;

    case '+': {
      if (selected == 0) {
        Serial.println("Select a single servo (1..4) for +/- pulse tweaking.");
        break;
      }
      int i = selected - 1;
      writeServoUs(i, pulseNow[i] + stepUs);
      Serial.print("Pulse now: "); Serial.println(pulseNow[i]);
      break;
    }

    case '-': {
      if (selected == 0) {
        Serial.println("Select a single servo (1..4) for +/- pulse tweaking.");
        break;
      }
      int i = selected - 1;
      writeServoUs(i, pulseNow[i] - stepUs);
      Serial.print("Pulse now: "); Serial.println(pulseNow[i]);
      break;
    }

    case 'w':
      Serial.println("Sweep...");
      sweepSelected();
      Serial.println("Sweep done.");
      break;

    case 'p':
      printStatus();
      break;

    default:
      Serial.print("Unknown: "); Serial.println(c);
      break;
  }
}
