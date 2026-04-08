/*
  Simple PID Corridor Test
  ========================
  Arduino Nano RP2040 Connect — X-drive holonomic robot

  The robot drives NORTH forever, centering itself between
  east and west walls using PID.  No turns, no events, no
  start/stop detection.  Just forward + centering.

  Use this to tune Kp, Ki, Kd on a straight corridor.

  Sensor input via UART (Serial1) from sensor Arduino:
      D1:<north>,D2:<east>,D3:<south>,D4:<west>\n
*/

#include <Arduino.h>
#include <Servo.h>

// ===== SERVO PINS =====
const uint8_t PIN_FL = 5;
const uint8_t PIN_FR = 3;
const uint8_t PIN_RR = 9;
const uint8_t PIN_RL = 4;

Servo S_FL, S_FR, S_RR, S_RL;

// ===== CALIBRATION =====
int STOP_FL = 90,  STOP_FR = 90,  STOP_RR = 90,  STOP_RL = 90;
int RANGE_FL = 90, RANGE_FR = 90, RANGE_RR = 90, RANGE_RL = 90;
int INV_FL = +1,   INV_FR = +1,   INV_RR = +1,   INV_RL = +1;

// ===== SPEED EQUALIZATION GAINS =====
const float GAIN_FL =  0.951f;
const float GAIN_FR = -0.901f;
const float GAIN_RR = -1.000f;
const float GAIN_RL =  0.938f;

// ===== SENSOR READINGS =====
float distN = 30.0f, distE = 30.0f, distS = 30.0f, distW = 30.0f;
String incomingData = "";

// ===== PID PARAMETERS =====
// Adjust these to tune corridor centering
float Kp = 0.07f;
float Ki = 0.00f;
float Kd = 0.00f;

float previousError = 0;
float integral       = 0;
unsigned long previousTime = 0;

const float INTEGRAL_LIMIT   = 20.0f;
const float CORRECTION_LIMIT = 0.5f;

// ===== MOVEMENT =====
const float FORWARD_SPEED = 0.4f;

// ===== HELPERS =====
static inline float clamp1(float v) {
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

void writeCRServo(Servo &s, int stopVal, int rangeVal, int inv, float cmd) {
    cmd = clamp1(cmd) * inv;
    int val = constrain((int)lround(stopVal + cmd * rangeVal), 0, 180);
    s.write(val);
}

void normalize(float &fl, float &fr, float &rr, float &rl) {
    float m = max(max(fabs(fl), fabs(fr)), max(fabs(rr), fabs(rl)));
    if (m < 1e-6f) { fl = fr = rr = rl = 0; return; }
    if (m > 1.0f)  { fl /= m; fr /= m; rr /= m; rl /= m; }
}

void stopAll() {
    writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, 0);
    writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, 0);
    writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, 0);
    writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, 0);
}

void drive(float Vx, float Vy, float W) {
    float fl = Vy + Vx + W;
    float fr = Vy - Vx - W;
    float rr = Vy + Vx - W;
    float rl = Vy - Vx + W;

    normalize(fl, fr, rr, rl);
    fl *= GAIN_FL;  fr *= GAIN_FR;
    rr *= GAIN_RR;  rl *= GAIN_RL;
    normalize(fl, fr, rr, rl);

    writeCRServo(S_FL, STOP_FL, RANGE_FL, INV_FL, fl);
    writeCRServo(S_FR, STOP_FR, RANGE_FR, INV_FR, fr);
    writeCRServo(S_RR, STOP_RR, RANGE_RR, INV_RR, rr);
    writeCRServo(S_RL, STOP_RL, RANGE_RL, INV_RL, rl);
}

// ===== UART PARSING =====
void parseDistances(String data) {
    int idx1 = data.indexOf("D1:") + 3;
    int idx2 = data.indexOf("D2:") + 3;
    int idx3 = data.indexOf("D3:") + 3;
    int idx4 = data.indexOf("D4:") + 3;

    int comma1 = data.indexOf(',', idx1);
    int comma2 = data.indexOf(',', idx2);
    int comma3 = data.indexOf(',', idx3);

    if (idx1 > 2 && idx2 > 2 && idx3 > 2 && idx4 > 2) {
        distN = data.substring(idx1, comma1).toFloat();
        distE = data.substring(idx2, comma2).toFloat();
        distS = data.substring(idx3, comma3).toFloat();
        distW = data.substring(idx4).toFloat();
    }
}

bool readSensorsUART() {
    bool newData = false;
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            parseDistances(incomingData);
            incomingData = "";
            newData = true;
        } else if (c != '\r') {
            incomingData += c;
        }
    }
    return newData;
}

// ===== PID =====
float calculatePID(float error) {
    unsigned long now = millis();
    float dt = (now - previousTime) / 1000.0f;
    if (dt <= 0) dt = 0.05f;

    float P = Kp * error;

    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float I = Ki * integral;

    float derivative = (error - previousError) / dt;
    float D = Kd * derivative;

    previousError = error;
    previousTime  = now;

    return constrain(P + I + D, -CORRECTION_LIMIT, CORRECTION_LIMIT);
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);

    S_FL.attach(PIN_FL);
    S_FR.attach(PIN_FR);
    S_RR.attach(PIN_RR);
    S_RL.attach(PIN_RL);

    stopAll();

    Serial.println("=========================================");
    Serial.println("  SIMPLE PID CORRIDOR TEST");
    Serial.println("  Heading: NORTH (forever)");
    Serial.println("  Centering: E/W walls");
    Serial.print("  Kp="); Serial.print(Kp, 3);
    Serial.print("  Ki="); Serial.print(Ki, 4);
    Serial.print("  Kd="); Serial.println(Kd, 3);
    Serial.print("  Speed: "); Serial.println(FORWARD_SPEED);
    Serial.println("=========================================\n");

    delay(2000);
    previousTime = millis();
}

// ===== LOOP =====
void loop() {
    if (!readSensorsUART()) return;

    // Centering error: positive = robot is too far east
    float error = distW - distE;

    // PID correction applied as lateral (Vx) movement
    float correction = calculatePID(error);

    // Stop if front wall detected
    if (distN <= 8.0f) {
        stopAll();
        Serial.print("WALL AHEAD — STOPPED  N:");
        Serial.println(distN, 1);
        delay(500);
        return;
    }

    // Drive north with lateral correction
    drive(correction, FORWARD_SPEED, 0);

    // Debug output
    Serial.print("E:"); Serial.print(distE, 1);
    Serial.print(" W:"); Serial.print(distW, 1);
    Serial.print(" | err:"); Serial.print(error, 2);
    Serial.print(" cor:"); Serial.print(correction, 3);
    Serial.print(" | P:"); Serial.print(Kp * error, 3);
    Serial.print(" I:"); Serial.print(Ki * integral, 4);
    Serial.print(" D:"); Serial.println(Kd * (error - previousError), 3);
}
