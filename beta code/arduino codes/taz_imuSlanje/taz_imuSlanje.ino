
#include "LSM9DS1TR-SOLDERED.h"
#include "Wire.h"


String input = "";
unsigned long lastMessageTime = 0;


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
  Serial.begin(115200);

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

  // 3. ČITANJE I SLANJE PODATAKA S IMU-A
  // Provjeri je li prošlo dovoljno vremena od zadnjeg slanja
  if (millis() - lastImuSendTime > IMU_SEND_INTERVAL) {
    lastImuSendTime = millis(); // Ažuriraj vrijeme zadnjeg slanja

    //sendImuData(); // Pozovi funkciju za čitanje i slanje
    sendImuData2();
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
  if (imu.magAvailable()) {
    imu.readMag();
  }

  float mx = -imu.my;
  float my = -imu.mx;
  float mz = imu.mz;

  float pom;
  if (my == 0)
    pom = (mx < 0) ? PI : 0;
  else
    pom = atan2(mx, my);

  pom -= DECLINATION * PI / 180;

  if (pom > PI) pom -= (2 * PI);
  else if (pom < -PI) pom += (2 * PI);

  // Pretvori radijane u stupnjeve
  pom *= 180.0 / PI;
  return pom;
}


void sendImuData1() {
  // Ažuriraj vrijednosti senzora ako su novi podaci dostupni
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  if (imu.accelAvailable()) {
    imu.readAccel();
  }
  if (imu.magAvailable()) {
    imu.readMag();
  }

  // Izračunaj pitch, roll i heading iz sirovih podataka
  // Preuzeto iz originalnog primjera
  float ax = imu.ax;
  float ay = imu.ay;
  float az = imu.az;
  // Osi magnetometra su suprotne akcelerometru, pa mijenjamo mx i my
  float mx = -imu.my;
  float my = -imu.mx;
  float mz = imu.mz;

  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Pretvori radijane u stupnjeve
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  // Sastavi i pošalji string preko serijske veze (XBee)
  // Format je "IMU:PITCH,ROLL,HEADING"
  Serial.print("IMU:");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.println(heading, 2); // Koristi println da pošalje i znak za novi red
}


void sendImuData2()
{
    if (imu.accelAvailable()) imu.readAccel();
    if (imu.gyroAvailable()) imu.readGyro();
    if (imu.magAvailable()) imu.readMag();

    // Izračunaj fizičke vrijednosti
    float ax = imu.calcAccel(imu.ax); // g
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    float gx = imu.calcGyro(imu.gx);  // deg/s
    float gy = imu.calcGyro(imu.gy);
    float gz = imu.calcGyro(imu.gz);

    float mx = imu.calcMag(imu.mx);   // gauss
    float my = imu.calcMag(imu.my);
    float mz = imu.calcMag(imu.mz);

    // Šalji sve podatke u jednoj liniji, CSV-style
    Serial.print("IMU9:");
    Serial.print(ax, 4); Serial.print(",");
    Serial.print(ay, 4); Serial.print(",");
    Serial.print(az, 4); Serial.print(",");
    Serial.print(gx, 4); Serial.print(",");
    Serial.print(gy, 4); Serial.print(",");
    Serial.print(gz, 4); Serial.print(",");
    Serial.print(mx, 4); Serial.print(",");
    Serial.print(my, 4); Serial.print(",");
    Serial.println(mz, 4);
}

