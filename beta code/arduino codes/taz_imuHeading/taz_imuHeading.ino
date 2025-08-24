#include "LSM9DS1TR-SOLDERED.h"
#include "Wire.h"

LSM9DS1TR imu;

#define IMU_SEND_INTERVAL 50
#define DECLINATION 4.9 // stupnjeva, magnetska deklinacija za Zagreb
unsigned long lastImuSendTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.begin()) {
    Serial.println("Greska u komunikaciji s LSM9DS1. Provjerite spojeve!");
    while (1);
  }
}

void loop() {
  if (millis() - lastImuSendTime > IMU_SEND_INTERVAL) {
    lastImuSendTime = millis();
    sendImuDataCalibrated();
  }
}

void sendImuDataCalibrated() {
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.magAvailable()) imu.readMag();

  // RAW podaci 
  float ax = imu.ax, ay = imu.ay, az = imu.az;
  float mx = -imu.my, my = -imu.mx, mz = imu.mz;

  // Kalibracija akcelerometra 
  // A_acc * (raw - b_acc)
  float bx = 0.0093, by = 0.0001, bz = 0.0827;

  float aX = 1.0003 * (ax - bx) + 0.0000 * (ay - by) + 0.0051 * (az - bz);
  float aY = 0.0000 * (ax - bx) + 1.0000 * (ay - by) + 0.0012 * (az - bz);
  float aZ = 0.0051 * (ax - bx) + 0.0012 * (ay - by) + 0.9105 * (az - bz);

  // Kalibracija magnetometra 
  float mX = mx - 0.2391;
  float mY = my - (-0.1571);
  float mZ = mz - (-0.5319);

  // Roll & Pitch 
  float roll = atan2(-aY, -aZ); // rad
  float pitch = atan2(aX, sqrt(aY * aY + aZ * aZ)); // rad

  // Heading pomoću formule:
  // tan(ψ_m) = ( -mY * cos(φ) + mZ * sin(φ) ) / ( mX * cos(θ) + mY * sin(θ) * sin(φ) + mZ * cos(φ) * sin(θ) )
  float sin_phi = sin(roll);
  float cos_phi = cos(roll);
  float sin_theta = sin(pitch);
  float cos_theta = cos(pitch);

  float top = (-mY * cos_phi) + (mZ * sin_phi);
  float bottom = mX * cos_theta + mY * sin_theta * sin_phi + mZ * cos_phi * sin_theta;

  float heading = atan2(top, bottom); // rad

  // Dodaj magnetsku deklinaciju
  heading += DECLINATION * PI / 180.0;

  // Normaliziraj heading između 0 i 2*PI
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;

  // Pretvori u stupnjeve
  float pitch_deg = pitch * 180.0 / PI;
  float roll_deg = roll * 180.0 / PI;
  float heading_deg = heading * 180.0 / PI;

  // Pošalji
  Serial.print("IMU:");
  Serial.print(pitch_deg, 2); Serial.print(",");
  Serial.print(roll_deg, 2); Serial.print(",");
  Serial.println(heading_deg, 2);
}
