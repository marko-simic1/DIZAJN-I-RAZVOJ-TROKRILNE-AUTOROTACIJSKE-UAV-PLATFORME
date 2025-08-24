#include "LSM9DS1TR-SOLDERED.h"
#include "Wire.h"

// Globalne varijable
String input = "";
unsigned long lastMessageTime = 0;
const int wdTime = 300;

const int pin1 = 9;
const int pin2 = 10;
const int pin3 = 11;

int x = 127, y = 127, z = 0;
int a = 0, b = 0, c = 0;

// IMU globalne varijable
float heading = 0.0;
float roll = 0.0;
float pitch = 0.0;
const float alpha = 0.2;
bool firstPass = true;
float angular_vel = 0;

// Senzor i definirane vrijednosti
LSM9DS1TR imu;
#define IMU_SEND_INTERVAL 20
unsigned long lastImuSendTime = 0;
#define DECLINATION 4.9

void setup() {
  Serial.begin(115200);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);

  Wire.begin();

  if (!imu.begin()) {
    Serial.println("Greska u komunikaciji s LSM9DS1. Provjerite spojeve!");
    while (1);
  }
}

void loop() {
  // PRIJEM KOMANDI 
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      input.trim();
      int values[6];
      int count = 0, fromIndex = 0;

      while (count < 6 && fromIndex < input.length()) {
        int spaceIndex = input.indexOf(' ', fromIndex);
        if (spaceIndex == -1) spaceIndex = input.length();
        values[count++] = input.substring(fromIndex, spaceIndex).toInt();
        fromIndex = spaceIndex + 1;
      }

      if (count == 6) {
        x = constrain(values[0], 0, 255);
        y = constrain(values[1], 0, 255);
        z = constrain(255 - values[2], 0, 255); // invert
        a = values[3];
        b = values[4];
        c = values[5];

        analogWrite(pin1, z);
        analogWrite(pin2, z);
        analogWrite(pin3, z);

        lastMessageTime = millis();
      }
      input = "";
    } else {
      input += ch;
    }
  }

  // FILTRIRANJE OFFSETA 
  if ((x > 110) && (x < 145)) x = 127;
  if ((y > 110) && (y < 145)) y = 127;
  if (z > 249) z = 255;
  if (z < 6) z = 0;

  // WATCHDOG 
  if (millis() - lastMessageTime > wdTime) {
    if (z != 0) {
      analogWrite(pin1, 0);
      analogWrite(pin2, 0);
      analogWrite(pin3, 0);
      z = 0;
      x = 127; y = 127;
      a = b = c = 0;
    }
  }

  // OČITAJ I FILTRIRAJ IMU 
  updateImuData();

  // SLANJE PODATAKA
  if (millis() - lastImuSendTime > IMU_SEND_INTERVAL) {
    lastImuSendTime = millis();
    sendData();
  }
}

void updateImuData() {
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.magAvailable()) imu.readMag();

  // RAW podaci 
  float ax = imu.ax, ay = imu.ay, az = imu.az;
  float mx = -imu.my, my = -imu.mx, mz = imu.mz;

  // Kalibracija akcelerometra 
  float bx = 0.0093, by = 0.0001, bz = 0.0827;
  float aX = 1.0003 * (ax - bx) + 0.0051 * (az - bz);
  float aY = 1.0000 * (ay - by) + 0.0012 * (az - bz);
  float aZ = 0.0051 * (ax - bx) + 0.0012 * (ay - by) + 0.9105 * (az - bz);

  // Kalibracija magnetometra 
  float mX = mx - 0.2391;
  float mY = my - (-0.1571);
  float mZ = mz - (-0.5319);

  // Roll & Pitch 
  float roll_r = atan2(-aY, -aZ);
  float pitch_r = atan2(aX, sqrt(aY * aY + aZ * aZ));

  // Heading 
  float sin_phi = sin(roll_r), cos_phi = cos(roll_r);
  float sin_theta = sin(pitch_r), cos_theta = cos(pitch_r);

  float top = (-mY * cos_phi) + (mZ * sin_phi);
  float bottom = mX * cos_theta + mY * sin_theta * sin_phi + mZ * cos_phi * sin_theta;
  float heading_r = atan2(top, bottom);
  heading_r += DECLINATION * PI / 180.0;
  if (heading_r < 0) heading_r += 2 * PI;
  if (heading_r > 2 * PI) heading_r -= 2 * PI;

  float heading_deg = heading_r * 180.0 / PI;
  float pitch_deg = pitch_r * 180.0 / PI;
  float roll_deg = roll_r * 180.0 / PI;

  // Filtar 
  if (firstPass) {
    heading = heading_deg;
    roll = roll_deg;
    pitch = pitch_deg;
    firstPass = false;
  } else {
    heading = heading * (1 - alpha) + heading_deg * alpha;
    roll = roll * (1 - alpha) + roll_deg * alpha;
    pitch = pitch * (1 - alpha) + pitch_deg * alpha;
  }
}


void sendData() {
  Serial.print("d");
  Serial.print(roll, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.println(heading, 2);
}


