#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>

/*
  Nano RP2040 Connect built-in IMU yaw check
  ------------------------------------------
  What this gives you:
  - Relative yaw angle in degrees
  - Current yaw rate in deg/s
  - Gyro Z bias used for calibration

  Important:
  - This is relative yaw only, not absolute compass heading.
  - Keep the board still during startup bias calibration.

  Serial commands:
  - 'z' : zero yaw at the current heading
  - 'r' : re-run gyro bias calibration and zero yaw
  - 'h' : print help
*/

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t CALIBRATION_SAMPLES = 300;
constexpr uint16_t CALIBRATION_DELAY_MS = 5;
constexpr float GYRO_Z_DEADBAND_DPS = 1.0f;
constexpr uint32_t PRINT_PERIOD_MS = 100;

float gYawDeg = 0.0f;
float gYawRateDps = 0.0f;
float gGyroBiasZDps = 0.0f;
uint32_t gLastSampleMs = 0;
uint32_t gLastPrintMs = 0;

static float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

static float wrapAngle360(float angleDeg) {
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  return angleDeg;
}

static bool readImuSample(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    return false;
  }

  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  return true;
}

static void printHelp() {
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  z  -> zero yaw at current heading"));
  Serial.println(F("  r  -> recalibrate gyro Z bias and zero yaw"));
  Serial.println(F("  h  -> print this help"));
  Serial.println();
}

static void calibrateGyroBias() {
  Serial.println(F("Calibrating gyro Z bias. Keep the board still..."));

  float sumGz = 0.0f;
  uint16_t goodSamples = 0;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    float ax, ay, az, gx, gy, gz;
    uint32_t startMs = millis();

    while (!readImuSample(ax, ay, az, gx, gy, gz)) {
      if ((millis() - startMs) > 250) break;
      delay(2);
    }

    if ((millis() - startMs) <= 250) {
      sumGz += gz;
      goodSamples++;
    }

    delay(CALIBRATION_DELAY_MS);
  }

  if (goodSamples == 0) {
    gGyroBiasZDps = 0.0f;
    Serial.println(F("Bias calibration failed, using 0.0 dps"));
  } else {
    gGyroBiasZDps = sumGz / goodSamples;
    Serial.print(F("Gyro Z bias = "));
    Serial.print(gGyroBiasZDps, 4);
    Serial.println(F(" dps"));
  }

  gYawDeg = 0.0f;
  gLastSampleMs = millis();
}

static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;

    switch (c) {
      case 'z':
      case 'Z':
        gYawDeg = 0.0f;
        Serial.println(F("Yaw zeroed at current heading."));
        break;

      case 'r':
      case 'R':
        calibrateGyroBias();
        break;

      case 'h':
      case 'H':
      default:
        printHelp();
        break;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  if (!IMU.begin()) {
    Serial.println(F("Failed to initialize built-in IMU."));
    while (1) {
      delay(1000);
    }
  }

  Serial.println(F("Nano RP2040 Connect IMU yaw check"));
  Serial.println(F("Yaw is relative and reported in degrees."));
  printHelp();

  calibrateGyroBias();
}

void loop() {
  handleSerialCommands();

  float ax, ay, az, gx, gy, gz;
  if (!readImuSample(ax, ay, az, gx, gy, gz)) {
    return;
  }

  uint32_t now = millis();
  float dt = (gLastSampleMs == 0) ? 0.01f : (now - gLastSampleMs) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.100f) dt = 0.100f;
  gLastSampleMs = now;

  float correctedGz = gz - gGyroBiasZDps;
  if (fabsf(correctedGz) < GYRO_Z_DEADBAND_DPS) {
    correctedGz = 0.0f;
  }

  gYawRateDps = correctedGz;
  gYawDeg = wrapAngle180(gYawDeg + correctedGz * dt);

  if ((now - gLastPrintMs) >= PRINT_PERIOD_MS) {
    gLastPrintMs = now;

    float yaw360 = wrapAngle360(gYawDeg);

    Serial.print(F("yaw_deg_signed="));
    Serial.print(gYawDeg, 2);
    Serial.print(F("\tyaw_deg_360="));
    Serial.print(yaw360, 2);
    Serial.print(F("\tyaw_rate_dps="));
    Serial.print(gYawRateDps, 2);
    Serial.print(F("\tbias_z_dps="));
    Serial.println(gGyroBiasZDps, 4);
  }
}
