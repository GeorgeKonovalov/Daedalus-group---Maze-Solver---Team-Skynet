/*
  Ultrasonic Sensor Reader — UART Sender
  ----------------------------------------
  Runs on a dedicated Arduino Uno.
  Reads four HC-SR04 sensors (N/E/S/W) sequentially using a shared
  trigger pin, then sends the distances over hardware UART to the
  main robot controller.

  UART format (matches uno_sender_test protocol):
      D1:<north>,D2:<east>,D3:<south>,D4:<west>\n
  Values are floats with one decimal place (cm).

  Pin map
  -------
  Trig (shared)  → digital 2
  North echo     → digital 5
  East  echo     → digital 6
  South echo     → digital 3
  West  echo     → digital 4
*/

#include <Arduino.h>

// ===== ULTRASONIC SENSOR PINS =====
const int TRIG_ALL = 2;   // Shared trigger

const int ECHO_N = 5;
const int ECHO_E = 6;
const int ECHO_S = 3;
const int ECHO_W = 4;

// ===== SENSOR CONFIG =====
const unsigned long PULSE_TIMEOUT = 20000;  // ~340 cm max range
const float         MAX_DIST     = 30.0;    // Cap at 30 cm (maze walls)
const int           INTER_DELAY  = 15;      // ms between sequential reads

// ===== ULTRASONIC READING (same method as PID control code) =====
float readUltrasonic(int echoPin) {
    // Trigger pulse
    digitalWrite(TRIG_ALL, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_ALL, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_ALL, LOW);

    // Measure echo duration
    long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);

    // Convert to cm  (speed of sound ≈ 0.0343 cm/µs, /2 for round trip)
    float distance = duration * 0.0343 / 2.0;

    // Clamp: zero means timeout, anything beyond MAX_DIST is out of range
    if (distance == 0 || distance > MAX_DIST) {
        distance = MAX_DIST;
    }
    return distance;
}

// ===== SETUP =====
void setup() {
    Serial.begin(9600);

    // Trigger pin
    pinMode(TRIG_ALL, OUTPUT);
    digitalWrite(TRIG_ALL, LOW);

    // Echo pins
    pinMode(ECHO_N, INPUT);
    pinMode(ECHO_E, INPUT);
    pinMode(ECHO_S, INPUT);
    pinMode(ECHO_W, INPUT);

    Serial.println("Sensor Uno Ready");
    delay(500);
}

// ===== MAIN LOOP =====
void loop() {
    // Sequential reads with inter-sensor delays to prevent cross-talk
    float distN = readUltrasonic(ECHO_N);
    delay(INTER_DELAY);
    float distE = readUltrasonic(ECHO_E);
    delay(INTER_DELAY);
    float distS = readUltrasonic(ECHO_S);
    delay(INTER_DELAY);
    float distW = readUltrasonic(ECHO_W);

    // Send over UART in the agreed protocol format
    Serial.print("D1:");
    Serial.print(distN, 1);
    Serial.print(",D2:");
    Serial.print(distE, 1);
    Serial.print(",D3:");
    Serial.print(distS, 1);
    Serial.print(",D4:");
    Serial.println(distW, 1);

    // No extra delay needed — the four sensor reads + inter-delays
    // already take ~60 ms, giving roughly a 16 Hz update rate
}
