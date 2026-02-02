#include <Servo.h>

Servo FL, FR, RL, RR;

const int FL_PIN = 3;
const int FR_PIN = 5;
const int RL_PIN = 6;
const int RR_PIN = 9;

const int STOP = 1500;
const int SPEED = 200;
const unsigned long TEST_DURATION = 5000;

void setup() {
    FL.attach(FL_PIN);
    FR.attach(FR_PIN);
    RL.attach(RL_PIN);
    RR.attach(RR_PIN);
    
    Serial.begin(9600);
    stopAll();
    delay(2000);
    
    testDirection("FORWARD",      0,  1);
    testDirection("BACKWARD",     0, -1);
    testDirection("STRAFE RIGHT", 1,  0);
    testDirection("STRAFE LEFT", -1,  0);
}

void loop() {}

void testDirection(const char* name, int vx, int vy) {
    Serial.print("Testing: ");
    Serial.println(name);
    
    setMotors(vx, vy);
    delay(TEST_DURATION);
    stopAll();
    
    Serial.println("Pausing...\n");
    delay(2000);
}

void setMotors(int vx, int vy) {
    // For cardinal movement only:
    // Forward/back: all wheels same direction
    // Strafe: FL & RR together, FR & RL together (opposite)
    
    int fl = -vx + vy;
    int fr = +vx + vy;
    int rl = +vx + vy;
    int rr = -vx + vy;
    
    FL.writeMicroseconds(STOP + fl * SPEED);
    FR.writeMicroseconds(STOP + fr * SPEED);
    RL.writeMicroseconds(STOP + rl * SPEED);
    RR.writeMicroseconds(STOP + rr * SPEED);
}

void stopAll() {
    FL.writeMicroseconds(STOP);
    FR.writeMicroseconds(STOP);
    RL.writeMicroseconds(STOP);
    RR.writeMicroseconds(STOP);
}
