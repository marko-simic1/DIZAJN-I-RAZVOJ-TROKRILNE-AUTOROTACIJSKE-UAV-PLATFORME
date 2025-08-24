
#include "LSM9DS1TR-SOLDERED.h"
#include "Wire.h"


String input = "";
unsigned long lastMessageTime = 0;
const int wdTime = 300;   

const int pin1 = 9;
const int pin2 = 10;
const int pin3 = 11;

int x = 127;
int y = 127;
int z = 0;
int a = 0;
int b = 0;
int c = 0;

float heading;


LSM9DS1TR imu;

// Vremenski interval za slanje podataka s IMU-a (u milisekundama)
#define IMU_SEND_INTERVAL 50
unsigned long lastImuSendTime = 0; // Varijabla za praćenje vremena slanja

// Magnetska deklinacija
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 4.9 // Deklinacija (stupnjevi) za Zagreb Hrvatska


void setup() {
  // Inicijalizacija za XBee
  Serial.begin(115200); // Komunikacija za XBee (i slanje IMU podataka)
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);

  // Inicijalizacija za IMU 
  Wire.begin(); // Pokreni I2C komunikaciju

  // Pokreni IMU senzor
  if (imu.begin() == false) {
    // Ova poruka će se poslati preko XBee-a ako je spojen
    Serial.println("Greska u komunikaciji s LSM9DS1. Provjerite spojeve!");
    while (1); // Zaustavi program ako senzor nije pronađen
  }
}



void loop() {
  // 1. PROVJERA DOLAZNIH PODATAKA S XBEE-A
  // Ovaj dio koda ostaje isti kao vaš originalni XBee kod
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      input.trim();

      // Razdvoji primljeni string po razmacima
      int count = 0;
      int fromIndex = 0;
      int spaceIndex = 0;
      int values[6];

      while (count < 6 && fromIndex < input.length()) {
        spaceIndex = input.indexOf(' ', fromIndex);
        if (spaceIndex == -1) spaceIndex = input.length();

        String valStr = input.substring(fromIndex, spaceIndex);
        values[count] = valStr.toInt();

        fromIndex = spaceIndex + 1;
        count++;
      }

      if (count == 6) {
        // Ažuriraj vrijednosti ako je primljeno 6 brojeva
        x = constrain(values[0], 0, 255);
        y = constrain(values[1], 0, 255);
        z = constrain(255 - values[2], 0, 255); // Obrnut smjer
        a = values[3];
        b = values[4];
        c = values[5];

        analogWrite(pin1, z);
        analogWrite(pin2, z);
        analogWrite(pin3, z);

        lastMessageTime = millis(); // Resetiraj watchdog
      }
      input = ""; // Očisti ulazni string za sljedeću poruku
    } else {
      input += ch;
    }
  }

  // Zanemari mali offset
  if ( (x > 110) && (x < 145) ){
    x = 127;
  }
  if ( (y > 110) && (y < 145) ){
    y = 127;
  } 
  if ( z > 249 ) {
    z = 255;
  }
  if ( z < 6 ) {
    z = 0;
  }


  // 2. WATCHDOG ZA PRIMLJENE NAREDBE
  // Ako prođe previše vremena bez naredbe, ugasi sve
  if (millis() - lastMessageTime > wdTime) {
    if (z != 0) {
      analogWrite(pin1, 0);
      analogWrite(pin2, 0);
      analogWrite(pin3, 0);
      z = 0;
      x = 127; // Vrati na neutralne vrijednosti
      y = 127;
      a = 0;
      b = 0;
      c = 0;
    }
  }


  //heading = readHeadingImu();


  // 3. ČITANJE I SLANJE PODATAKA S IMU-A
  // Provjeri je li prošlo dovoljno vremena od zadnjeg slanja
  if (millis() - lastImuSendTime > IMU_SEND_INTERVAL) {
    lastImuSendTime = millis(); // Ažuriraj vrijeme zadnjeg slanja

    //sendImuData(); // Pozovi funkciju za čitanje i slanje
    //printState();
    sendImuDataCalibrated();
  }

}


void printState() {
  Serial.print("Heading: ");
  Serial.println(heading, 2);
  Serial.print("X, Y, Z: ");
  Serial.print(x, 2);
  Serial.print(",");
  Serial.print(y, 2);
  Serial.print(",");
  Serial.println(z, 2);

}


float readHeadingImu(){
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

  return heading * 180.0 / PI;
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