#include <Servo.h>

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Servo pins (UPDATED)
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 3;
const int SERVO3_PIN = 4;
const int SERVO4_PIN = 5;

// Continuous rotation servo control values
const int STOP_SPEED = 90;

// Adjust if needed
const int FORWARD_MAX = 120;
const int REVERSE_MAX = 60;

// Timing
const int RAMP_DELAY = 50;      
const int HOLD_TIME = 5000;     // UPDATED to 5 seconds
const int STEP_SIZE = 1;

void setup() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  stopAllServos();
  delay(1000);
}

void loop() {
  // Forward direction
  rampServos(STOP_SPEED, FORWARD_MAX, STEP_SIZE);
  delay(HOLD_TIME);
  rampServos(FORWARD_MAX, STOP_SPEED, STEP_SIZE);

  delay(500);

  // Reverse direction
  rampServos(STOP_SPEED, REVERSE_MAX, STEP_SIZE);
  delay(HOLD_TIME);
  rampServos(REVERSE_MAX, STOP_SPEED, STEP_SIZE);

  delay(500);
}

void setAllServos(int speedValue) {
  servo1.write(speedValue);
  servo2.write(speedValue);
  servo3.write(speedValue);
  servo4.write(speedValue);
}

void stopAllServos() {
  setAllServos(STOP_SPEED);
}

void rampServos(int startSpeed, int endSpeed, int stepAmount) {
  if (startSpeed < endSpeed) {
    for (int speedVal = startSpeed; speedVal <= endSpeed; speedVal += stepAmount) {
      setAllServos(speedVal);
      delay(RAMP_DELAY);
    }
  } else {
    for (int speedVal = startSpeed; speedVal >= endSpeed; speedVal -= stepAmount) {
      setAllServos(speedVal);
      delay(RAMP_DELAY);
    }
  }
}
