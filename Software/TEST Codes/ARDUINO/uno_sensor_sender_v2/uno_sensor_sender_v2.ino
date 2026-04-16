/*
  Ultrasonic Sensor Reader - UART Sender v2
  -----------------------------------------
  Runs on a dedicated Arduino Uno.
  Reads four HC-SR04 sensors (N/E/S/W) sequentially using a shared
  trigger pin, then sends the distances over hardware UART to the
  main robot controller.

  UART format:
      D1:<north>,D2:<east>,D3:<south>,D4:<west>\n

  Timing goals for this version:
  - keep the sender compatible with the RP2040 receiver
  - improve throughput meaningfully without pushing into risky edge cases
  - keep enough timeout and inter-sensor spacing margin for reliable maze use

  Pin map
  -------
  Trig (shared)  -> digital 2
  North echo     -> digital 5
  East  echo     -> digital 6
  South echo     -> digital 3
  West  echo     -> digital 4
*/

#include <Arduino.h>

// ===== ULTRASONIC SENSOR PINS =====
constexpr uint8_t TRIG_ALL = 2;

constexpr uint8_t ECHO_N = 5;
constexpr uint8_t ECHO_E = 6;
constexpr uint8_t ECHO_S = 3;
constexpr uint8_t ECHO_W = 4;

// ===== SENSOR / LINK CONFIG =====
constexpr unsigned long UART_BAUD = 115200UL;
constexpr unsigned long PULSE_TIMEOUT_US = 5000UL;   // About 60 cm round-trip allowance, well above the 30 cm maze cap.
constexpr uint8_t INTER_SENSOR_DELAY_MS = 5;         // Safe crosstalk gap between sensors without falling back to the old slower timing.
constexpr uint16_t MAX_DISTANCE_TENTHS_CM = 300;     // 30.0 cm cap for the maze logic.
constexpr unsigned long STARTUP_BANNER_DELAY_MS = 100UL;

// ===== ULTRASONIC READING =====
static inline void triggerAllSensors() {
  digitalWrite(TRIG_ALL, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_ALL, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_ALL, LOW);
}

static uint16_t readUltrasonicTenthsCm(uint8_t echoPin) {
  triggerAllSensors();

  unsigned long durationUs = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (durationUs == 0UL) {
    return MAX_DISTANCE_TENTHS_CM;
  }

  // Convert echo duration to distance in 0.1 cm units.
  // distance_cm = duration_us * 0.0343 / 2
  // distance_tenths_cm = duration_us * 343 / 2000
  uint16_t distanceTenths = (uint16_t)((durationUs * 343UL + 1000UL) / 2000UL);

  if (distanceTenths > MAX_DISTANCE_TENTHS_CM) {
    distanceTenths = MAX_DISTANCE_TENTHS_CM;
  }

  return distanceTenths;
}

static void sendDistancePacket(uint16_t northTenths,
                               uint16_t eastTenths,
                               uint16_t southTenths,
                               uint16_t westTenths) {
  char line[48];
  snprintf(line, sizeof(line),
           "D1:%u.%u,D2:%u.%u,D3:%u.%u,D4:%u.%u",
           northTenths / 10U, northTenths % 10U,
           eastTenths / 10U,  eastTenths % 10U,
           southTenths / 10U, southTenths % 10U,
           westTenths / 10U,  westTenths % 10U);
  Serial.println(line);
}

// ===== SETUP =====
void setup() {
  Serial.begin(UART_BAUD);

  pinMode(TRIG_ALL, OUTPUT);
  digitalWrite(TRIG_ALL, LOW);

  pinMode(ECHO_N, INPUT);
  pinMode(ECHO_E, INPUT);
  pinMode(ECHO_S, INPUT);
  pinMode(ECHO_W, INPUT);

  Serial.println(F("Sensor Uno Ready"));
  delay(STARTUP_BANNER_DELAY_MS);
}

// ===== MAIN LOOP =====
void loop() {
  uint16_t distN = readUltrasonicTenthsCm(ECHO_N);
  delay(INTER_SENSOR_DELAY_MS);

  uint16_t distE = readUltrasonicTenthsCm(ECHO_E);
  delay(INTER_SENSOR_DELAY_MS);

  uint16_t distS = readUltrasonicTenthsCm(ECHO_S);
  delay(INTER_SENSOR_DELAY_MS);

  uint16_t distW = readUltrasonicTenthsCm(ECHO_W);
  delay(INTER_SENSOR_DELAY_MS);

  // No extra delay after the last sensor:
  // UART transmission plus the next loop's first trigger already provide spacing.
  sendDistancePacket(distN, distE, distS, distW);
}
