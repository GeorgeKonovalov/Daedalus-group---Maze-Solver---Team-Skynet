/*
  Ultrasonic Sensor Reader - UART Sender
  ----------------------------------------
  Runs on a dedicated Arduino Uno.
  Reads four HC-SR04 sensors (N/E/S/W) sequentially using a shared
  trigger pin, then sends the distances over hardware UART to the
  main robot controller.

  UART format:
      D1:<north>,D2:<east>,D3:<south>,D4:<west>\n
  Values are floats with one decimal place (cm).

  Pin map
  -------
  Trig (shared)  -> digital 2
  North echo     -> digital 5
  East  echo     -> digital 6
  South echo     -> digital 3
  West  echo     -> digital 4

  v3 notes
  --------
  This version keeps the original simple structure and float output,
  but updates the communication speed and restores safer shared-trigger
  timing for reliable readings.
*/

#include <Arduino.h>

// ===== ULTRASONIC SENSOR PINS =====
const int TRIG_ALL = 2;   // Shared trigger

const int ECHO_N = 5;
const int ECHO_E = 6;
const int ECHO_S = 3;
const int ECHO_W = 4;

// ===== SENSOR CONFIG =====
const unsigned long UART_BAUD        = 115200UL;
const unsigned long PULSE_TIMEOUT    = 5000UL;  // Safe timeout for short maze distances with some margin.
const float         MAX_DIST         = 30.0;    // Cap at 30 cm (maze walls).
const int           INTER_DELAY      = 5;       // ms between sequential reads to reduce cross-talk.
const int           POST_CYCLE_DELAY = 5;       // ms quiet gap before the next trigger cycle starts.

// ===== ULTRASONIC READING =====
float readUltrasonic(int echoPin) {
    // Trigger pulse
    digitalWrite(TRIG_ALL, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_ALL, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_ALL, LOW);

    // Measure echo duration
    unsigned long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);

    // Convert to cm  (speed of sound ~= 0.0343 cm/us, /2 for round trip)
    float distance = duration * 0.0343f / 2.0f;

    // Clamp: zero means timeout, anything beyond MAX_DIST is out of range
    if (distance == 0.0f || distance > MAX_DIST) {
        distance = MAX_DIST;
    }
    return distance;
}

// ===== SETUP =====
void setup() {
    Serial.begin(UART_BAUD);

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
    delay(INTER_DELAY);
    // Send over UART in the agreed protocol format
    Serial.print("D1:");
    Serial.print(distN, 1);
    Serial.print(",D2:");
    Serial.print(distE, 1);
    Serial.print(",D3:");
    Serial.print(distS, 1);
    Serial.print(",D4:");
    Serial.println(distW, 1);

    // At 115200 baud the packet finishes much faster than before, so keep
    // a short quiet gap here to avoid retriggering the shared ultrasonic
    // array too aggressively on the next loop.
    delay(POST_CYCLE_DELAY);
}
