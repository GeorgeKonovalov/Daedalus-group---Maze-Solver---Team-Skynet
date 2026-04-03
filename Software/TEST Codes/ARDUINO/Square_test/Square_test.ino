/*
  X-drive omni robot — AUTONOMOUS NON-STOP SQUARE PATH
  No Serial, starts immediately on power-up

  Path forever:
    forward A
    right A
    backward A
    left A

  IMPORTANT:
  Time-based (open loop). Distance depends on calibration.
*/

#include <Arduino.h>
#include <Servo.h>

// ----- Servo pins -----
const uint8_t PIN_FL = 13;
const uint8_t PIN_FR = 10;
const uint8_t PIN_RR = 12;
const uint8_t PIN_RL = 11;

Servo S_FL, S_FR, S_RR, S_RL;

// ----- Calibration -----
int STOP_FL = 1500, STOP_FR = 1500, STOP_RR = 1500, STOP_RL = 1500;
int RANGE_FL = 250, RANGE_FR = 250, RANGE_RR = 250, RANGE_RL = 250;
int INV_FL = +1, INV_FR = +1, INV_RR = +1, INV_RL = +1;

// Speed equalization gains (your tested values)
const float GAIN_FL = -0.859f;
const float GAIN_FR =  1.000f;
const float GAIN_RR =  0.907f;
const float GAIN_RL = -0.956f;

// ----- Motion parameters -----
const float speedLevel = 6.0; // fixed speed (0..9 equivalent)
const float speedScale = speedLevel / 9.0;

// Square parameters (EDIT THESE)
const float A = 0.50;        // side length in meters
const float sec_per_meter_fb = 3.2; // forward/back calibration
const float sec_per_meter_st = 3.4; // strafe calibration

// Path states
enum State { FWD, RIGHT, BACK, LEFT };
State state = FWD;

unsigned long stepStart;
unsigned long stepDuration;

// ---------- Drive helpers ----------
void writeCRServo(Servo &s, int stopUs, int rangeUs, int inv, float cmd) {
  cmd = constrain(cmd, -1, 1) * inv;
  s.writeMicroseconds(stopUs + cmd * rangeUs);
}

void normalize(float &fl, float &fr, float &rr, float &rl) {
  float m = max(max(abs(fl), abs(fr)), max(abs(rr), abs(rl)));
  if (m > 1) { fl/=m; fr/=m; rr/=m; rl/=m; }
}

void drive(float Vx, float Vy) {
  float fl = Vy + Vx;
  float fr = Vy - Vx;
  float rr = Vy + Vx;
  float rl = Vy - Vx;

  normalize(fl, fr, rr, rl);

  fl *= GAIN_FL;
  fr *= GAIN_FR;
  rr *= GAIN_RR;
  rl *= GAIN_RL;

  normalize(fl, fr, rr, rl);

  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, rl);
}

// ---------- Setup ----------
void setup() {
  S_FL.attach(PIN_FL);
  S_FR.attach(PIN_FR);
  S_RR.attach(PIN_RR);
  S_RL.attach(PIN_RL);

  stepStart = millis();
  stepDuration = A * sec_per_meter_fb * 1000;
}

// ---------- Loop ----------
void loop() {

  // Execute current segment
  switch (state) {
    case FWD:   drive(0, +speedScale); break;
    case RIGHT: drive(+speedScale, 0); break;
    case BACK:  drive(0, -speedScale); break;
    case LEFT:  drive(-speedScale, 0); break;
  }

  // Check if time elapsed
  if (millis() - stepStart < stepDuration) return;

  // Move to next segment
  stepStart = millis();

  switch (state) {
    case FWD:
      state = RIGHT;
      stepDuration = A * sec_per_meter_st * 1000;
      break;

    case RIGHT:
      state = BACK;
      stepDuration = A * sec_per_meter_fb * 1000;
      break;

    case BACK:
      state = LEFT;
      stepDuration = A * sec_per_meter_st * 1000;
      break;

    case LEFT:
      state = FWD;
      stepDuration = A * sec_per_meter_fb * 1000;
      break;
  }
}
