#include <Arduino_LSM6DSOX.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("ax\tay\taz\tgx\tgy\tgz");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);   // g
    IMU.readGyroscope(gx, gy, gz);      // deg/s

    Serial.print(ax); Serial.print('\t');
    Serial.print(ay); Serial.print('\t');
    Serial.print(az); Serial.print('\t');
    Serial.print(gx); Serial.print('\t');
    Serial.print(gy); Serial.print('\t');
    Serial.println(gz);
  }
}
