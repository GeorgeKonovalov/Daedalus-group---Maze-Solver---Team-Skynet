#include <Servo.h>

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Pins
const int SERVO1_PIN = 10;
const int SERVO2_PIN = 11;
const int SERVO3_PIN = 12;
const int SERVO4_PIN = 13;

// Stop values (tune these!)
int stop1 = 90;
int stop2 = 90;
int stop3 = 90;
int stop4 = 90;

void setup() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
}

void loop() {
  // Continuously send stop signal
  servo1.write(stop1);
  servo2.write(stop2);
  servo3.write(stop3);
  servo4.write(stop4);

  delay(20); // small refresh delay
}
