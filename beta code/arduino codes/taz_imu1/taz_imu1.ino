#include <Wire.h>
#include "LSM9DS1TR-SOLDERED.h"

LSM9DS1TR imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.begin()) {
    Serial.println("LSM9DS1 not found!");
    while (1) delay(100);
  }

  Serial.println("LSM9DS1 ready.");
}

void loop() {
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.magAvailable()) imu.readMag();

  // Šaljemo jedan redak s odvojenim vrijednostima
  Serial.print("ACC:");
  Serial.print(imu.calcAccel(imu.ax)); Serial.print(",");
  Serial.print(imu.calcAccel(imu.ay)); Serial.print(",");
  Serial.print(imu.calcAccel(imu.az)); Serial.print("; ");

  Serial.print("GYR:");
  Serial.print(imu.calcGyro(imu.gx)); Serial.print(",");
  Serial.print(imu.calcGyro(imu.gy)); Serial.print(",");
  Serial.print(imu.calcGyro(imu.gz)); Serial.print("; ");

  Serial.print("MAG:");
  Serial.print(imu.calcMag(imu.mx)); Serial.print(",");
  Serial.print(imu.calcMag(imu.my)); Serial.print(",");
  Serial.println(imu.calcMag(imu.mz));

  delay(100); // 10Hz
}
