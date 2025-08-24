#include "LSM9DS1TR-SOLDERED.h"
#include "Wire.h"

String input = "";
unsigned long lastMessageTime = 0;
const int wdTime = 300;
const int pin1 = 9, pin2 = 10, pin3 = 11;
int x = 127, y = 127, z = 0;
int a = 0, b = 0, c = 0;
float heading = 0.0, heading_unfil = 0.0, roll = 0.0, pitch = 0.0;
const float alpha = 0.2;
bool firstPass = true;
float angular_vel = 0;
LSM9DS1TR imu;
#define IMU_SEND_INTERVAL 30
unsigned long lastImuSendTime = 0;
#define DECLINATION 4.9
#define LSM9DS1_M  0x1E
#define LSM9DS1_AG 0x6B

void verifyRegister(uint8_t deviceAddress, uint8_t registerAddress, const char* registerName) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddress, (uint8_t)1);
    uint8_t value = 0;
    if (Wire.available()) {
        value = Wire.read();
    }
    Serial.print("Provjera registra: ");
    Serial.print(registerName);
    Serial.print(" -> Procitano: 0x");
    Serial.println(value, HEX);
}

void forceRegisterWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value, const char* registerName) {
    Serial.print("Rucno postavljanje: ");
    Serial.print(registerName);
    Serial.print(" na 0x");
    Serial.println(value, HEX);
    
    Wire.beginTransmission(deviceAddress);
    Wire.write(registerAddress);
    Wire.write(value);
    Wire.endTransmission();
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);

  Wire.begin();

  // Postavke za IMU
  imu.settings.gyro.enabled = true;
  imu.settings.gyro.scale = 2000;
  imu.settings.gyro.sampleRate = 6;
  
  imu.settings.accel.enabled = true;
  imu.settings.accel.scale = 16;
  
  imu.settings.mag.enabled = true;
  imu.settings.mag.scale = 16;
  imu.settings.mag.sampleRate = 7;

  // Inicijalizacija
  uint16_t status = imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire);
  Serial.print("LSM9DS1 status (WHO_AM_I): 0x");
  Serial.println(status, HEX);
  if (status != 0x683D) {
    Serial.println("GRESKA: Inicijalizacija neuspjesna!");
    while (1);
  }
  Serial.println("IMU inicijaliziran. Pokrecem rucno forsiranje postavki...");
  Serial.println("------------------------------------");

  
  // Gyro & Accel: ODR=952Hz (110), Scale=2000dps/16g (11) -> 11011000 = 0xD8
  forceRegisterWrite(LSM9DS1_AG, 0x10, 0xD8, "CTRL_REG1_G");
  forceRegisterWrite(LSM9DS1_AG, 0x20, 0xD8, "CTRL_REG6_XL");

  // Mag Scale: Scale=16G(11) -> 01100000 = 0x60
  forceRegisterWrite(LSM9DS1_M, 0x21, 0x60, "CTRL_REG2_M");
  
  Serial.println("------------------------------------");
  Serial.println("Rucno postavljanje zavrseno. Ponovna provjera...");
  
  // PONOVNA VERIFIKACIJA
  Serial.println("Ocekivano za CTRL_REG1_G: 0xD8");
  verifyRegister(LSM9DS1_AG, 0x10, "CTRL_REG1_G");

  Serial.println("Ocekivano za CTRL_REG6_XL: 0xD8");
  verifyRegister(LSM9DS1_AG, 0x20, "CTRL_REG6_XL");

  Serial.println("Ocekivano za CTRL_REG2_M: 0x60");
  verifyRegister(LSM9DS1_M, 0x21, "CTRL_REG2_M");
  
  Serial.println("------------------------------------");
  Serial.println("Verifikacija zavrsena. Pokrecem loop...");
}


void loop() {
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
        z = constrain(255 - values[2], 0, 255);
        a = values[3]; b = values[4]; c = values[5];
        analogWrite(pin1, z); analogWrite(pin2, z); analogWrite(pin3, z);
        lastMessageTime = millis();
      }
      input = "";
    } else {
      input += ch;
    }
  }
  if ((x > 110) && (x < 145)) x = 127;
  if ((y > 110) && (y < 145)) y = 127;
  if (z > 249) z = 255;
  if (z < 6) z = 0;
  if (millis() - lastMessageTime > wdTime) {
    shutdown();
  }
  updateImuData();
  if (millis() - lastImuSendTime > IMU_SEND_INTERVAL) {
    lastImuSendTime = millis();
    sendData2();
  }
}

void shutdown() {
  analogWrite(pin1, 0); analogWrite(pin2, 0); analogWrite(pin3, 0);
  z = 0; x = 127; y = 127; a = b = c = 0;
}

void updateImuData() {
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.magAvailable()) imu.readMag();
  float ax = imu.ax, ay = imu.ay, az = imu.az;
  float mx = -imu.my, my = -imu.mx, mz = imu.mz;
  float bx = 0.0093, by = 0.0001, bz = 0.0827;
  float aX = 1.0003 * (ax - bx) + 0.0051 * (az - bz);
  float aY = 1.0000 * (ay - by) + 0.0012 * (az - bz);
  float aZ = 0.0051 * (ax - bx) + 0.0012 * (ay - by) + 0.9105 * (az - bz);
  float mX = mx - 0.2391;
  float mY = my - (-0.1571);
  float mZ = mz - (-0.5319);
  float roll_r = atan2(-aY, -aZ);
  float pitch_r = atan2(aX, sqrt(aY * aY + aZ * aZ));
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
  float gzPom = imu.calcGyro(imu.gz);
  heading_unfil = heading_deg;
  if (firstPass) {
    heading = heading_deg; roll = roll_deg; pitch = pitch_deg; angular_vel = gzPom;
    firstPass = false;
  } else {
    heading = heading * (1 - alpha) + heading_deg * alpha;
    roll = roll * (1 - alpha) + roll_deg * alpha;
    pitch = pitch * (1 - alpha) + pitch_deg * alpha;
    angular_vel = angular_vel * (1 - alpha) + gzPom * alpha;
  }
}

void sendData2() {
  Serial.print("d");
  Serial.print(heading, 2); Serial.print(",");
  Serial.print(heading_unfil, 2); Serial.print(",");
  Serial.println(angular_vel, 2);
}
