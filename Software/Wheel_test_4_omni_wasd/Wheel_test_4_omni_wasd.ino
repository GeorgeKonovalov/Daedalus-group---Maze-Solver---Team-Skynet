/*
  Parallax continuous-rotation servos (X-drive 4 omni)
  LATCHED serial commands: motion continues until 'x'

  Commands (send via Serial Monitor):
    w forward
    s backward
    a left
    d right
    q rotate CCW
    e rotate CW
    x stop
    0..9 speed level

  Serial Monitor:
    Baud 115200
    Line ending: No line ending (recommended)
*/

#include <Arduino.h>
#include <Servo.h>

// ----- Servo pins (EDIT) -----
const uint8_t PIN_FL = 10;   // front-left
const uint8_t PIN_FR = 11;   // front-right
const uint8_t PIN_RR = 12;   // rear-right
const uint8_t PIN_RL = 13;   // rear-left

Servo S_FL, S_FR, S_RR, S_RL;

// ----- Calibration (TUNE THESE) -----
// STOP microseconds per servo (adjust until each wheel truly stops)
int STOP_FL = 1500;
int STOP_FR = 1500;
int STOP_RR = 1500;
int STOP_RL = 1500;

// Range around STOP for full speed command (start 200–300; increase if needed)
int RANGE_FL = 250;
int RANGE_FR = 250;
int RANGE_RR = 250;
int RANGE_RL = 250;

// Invert any wheel if direction is wrong (+1 or -1)
int INV_FL = +1;
int INV_FR = +1;
int INV_RR = +1;
int INV_RL = +1;

// ----- State (latched command) -----
enum Cmd { CMD_STOP, CMD_W, CMD_A, CMD_S, CMD_D, CMD_Q, CMD_E };
Cmd currentCmd = CMD_STOP;

int speedLevel = 6; // 0..9

// ----- Helpers -----
static inline float clamp1(float v) {
  if (v > 1.0f) return 1.0f;
  if (v < -1.0f) return -1.0f;
  return v;
}

float speedScale() {
  return speedLevel / 9.0f; // 0..1
}

void writeCRServo(Servo &s, int stopUs, int rangeUs, int inv, float cmd) {
  cmd = clamp1(cmd) * inv;
  int us = (int)lround(stopUs + cmd * rangeUs);
  s.writeMicroseconds(us);
}

void stopAll() {
  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, 0);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, 0);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, 0);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, 0);
}

void normalize(float &fl, float &fr, float &rr, float &rl) {
  float m = max(max(fabs(fl), fabs(fr)), max(fabs(rr), fabs(rl)));
  if (m < 1e-6f) { fl = fr = rr = rl = 0; return; }
  if (m > 1.0f) { fl /= m; fr /= m; rr /= m; rl /= m; }
}

/*
  X-drive mixer:
    fl =  Vy + Vx + W
    fr =  Vy - Vx - W
    rr =  Vy + Vx - W
    rl =  Vy - Vx + W
  Vx right(+), Vy forward(+), W CW(+)
*/
void drive(float Vx, float Vy, float W) {
  float fl = Vy + Vx + W;
  float fr = Vy - Vx - W;
  float rr = Vy + Vx - W;
  float rl = Vy - Vx + W;

  normalize(fl, fr, rr, rl);

  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, rl);
}

// Apply the currently latched command every loop
void applyCurrentCommand() {
  float s = speedScale();
  float Vx = 0, Vy = 0, W = 0;

  switch (currentCmd) {
    case CMD_W: Vy = +1.0f * s; break;
    case CMD_S: Vy = -1.0f * s; break;
    case CMD_D: Vx = +1.0f * s; break;
    case CMD_A: Vx = -1.0f * s; break;
    case CMD_E: W  = +1.0f * s; break; // rotate CW
    case CMD_Q: W  = -1.0f * s; break; // rotate CCW
    case CMD_STOP:
    default:    stopAll(); return;
  }

  drive(Vx, Vy, W);
}

void setup() {
  Serial.begin(115200);

  S_FL.attach(PIN_FL);
  S_FR.attach(PIN_FR);
  S_RR.attach(PIN_RR);
  S_RL.attach(PIN_RL);

  stopAll();

  Serial.println("Latched X-drive ready.");
  Serial.println("Commands: w/a/s/d move, q/e rotate, x stop, 0..9 speed.");
}

void loop() {
  // Read & latch any new command
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    // speed preset
    if (c >= '0' && c <= '9') {
      speedLevel = c - '0';
      Serial.print("Speed level: "); Serial.println(speedLevel);
      if (speedLevel == 0) currentCmd = CMD_STOP;
      continue;
    }

    switch (c) {
      case 'w': currentCmd = CMD_W; Serial.println("LATCH: forward"); break;
      case 's': currentCmd = CMD_S; Serial.println("LATCH: back"); break;
      case 'a': currentCmd = CMD_A; Serial.println("LATCH: left"); break;
      case 'd': currentCmd = CMD_D; Serial.println("LATCH: right"); break;
      case 'q': currentCmd = CMD_Q; Serial.println("LATCH: rotate CCW"); break;
      case 'e': currentCmd = CMD_E; Serial.println("LATCH: rotate CW"); break;
      case 'x': currentCmd = CMD_STOP; Serial.println("STOP"); break;
      default:
        Serial.print("Unknown: "); Serial.println(c);
        break;
    }
  }

  // Keep executing the latched motion continuously
  applyCurrentCommand();

  // Servos only need ~50 Hz updates; small delay is fine
  delay(20);
}
 
