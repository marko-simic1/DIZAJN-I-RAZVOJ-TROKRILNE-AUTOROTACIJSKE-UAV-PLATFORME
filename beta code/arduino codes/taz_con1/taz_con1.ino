
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
#define IMU_SEND_INTERVAL 250
unsigned long lastImuSendTime = 0; // Varijabla za praćenje vremena slanja

// Magnetska deklinacija za vašu lokaciju. Izračunajte je ovdje:
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


  heading = readHeadingImu();

  


  // 3. ČITANJE I SLANJE PODATAKA S IMU-A
  // Provjeri je li prošlo dovoljno vremena od zadnjeg slanja
  if (millis() - lastImuSendTime > IMU_SEND_INTERVAL) {
    lastImuSendTime = millis(); // Ažuriraj vrijeme zadnjeg slanja

    //sendImuData(); // Pozovi funkciju za čitanje i slanje
    printState();
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


void sendImuData2() {
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.magAvailable()) imu.readMag();

  Serial.print("IMU9:");
  Serial.print(imu.ax, 3); Serial.print(",");
  Serial.print(imu.ay, 3); Serial.print(",");
  Serial.print(imu.az, 3); Serial.print(",");
  Serial.print(imu.gx, 3); Serial.print(",");
  Serial.print(imu.gy, 3); Serial.print(",");
  Serial.print(imu.gz, 3); Serial.print(",");
  Serial.print(imu.mx, 3); Serial.print(",");
  Serial.print(imu.my, 3); Serial.print(",");
  Serial.println(imu.mz, 3);
}
