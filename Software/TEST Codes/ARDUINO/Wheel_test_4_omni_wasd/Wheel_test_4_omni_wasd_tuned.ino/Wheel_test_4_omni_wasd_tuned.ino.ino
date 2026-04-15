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

// ----- Servo pins -----
const uint8_t PIN_FL = 13; //was 10  // front-left  (measured 1.22 rot/s)
const uint8_t PIN_FR = 10; // was 11   // front-right (measured 1.27 rot/s)
const uint8_t PIN_RR = 12; //was 12   // rear-right  (measured 1.37 rot/s)
const uint8_t PIN_RL = 11; //was 13   // rear-left   (measured 1.48 rot/s)

Servo S_FL, S_FR, S_RR, S_RL;

// ----- Calibration (TUNE THESE) -----
int STOP_FL = 1500;
int STOP_FR = 1500;
int STOP_RR = 1500;
int STOP_RL = 1500;

int RANGE_FL = 250;
int RANGE_FR = 250;
int RANGE_RR = 250;
int RANGE_RL = 250;

int INV_FL = +1;
int INV_FR = +1;
int INV_RR = +1;
int INV_RL = +1;

// ----- Speed equalization gains (based on your measured rot/sec) -----
// Target = slowest wheel = 1.22 rot/sec (PIN 10 = FL)
// ----- Speed equalization gains (based on your measured rot/sec) -----
// Measured no-load: pin13=1.48, pin12=1.37, pin11=1.27, pin10=1.22 rot/s
// Target = slowest = 1.22 rot/sec (pin 10, which is RR in your current wiring)

const float GAIN_FL = -0.859f; // FL is pin 13: 1.22/1.48
const float GAIN_FR = 1.000f; // FR is pin 11: 1.22/1.27
const float GAIN_RR = 0.907f; // RR is pin 10: 1.22/1.22
const float GAIN_RL = -0.956;//f; // RL is pin 12: 1.22/1.37


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

  // 1) Normalize the raw mix so the biggest wheel cmd is ±1
  normalize(fl, fr, rr, rl);

  // 2) Apply your measured compensation gains (fast wheels get scaled down)
  fl *= GAIN_FL;
  fr *= GAIN_FR;
  rr *= GAIN_RR;
  rl *= GAIN_RL;

  // 3) Safety normalize again in case gains push something over ±1
  normalize(fl, fr, rr, rl);

  // 4) Send to servos
  writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, fl);
  writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, fr);
  writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, rr);
  writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, rl);
}

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

  Serial.println("Latched X-drive ready (with wheel speed equalization).");
  Serial.println("Commands: w/a/s/d move, q/e rotate, x stop, 0..9 speed.");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

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

  applyCurrentCommand();
  delay(20);
}
