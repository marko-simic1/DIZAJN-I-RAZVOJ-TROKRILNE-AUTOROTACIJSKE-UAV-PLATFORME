#include "Wire.h"

// Konstante 
const int pin1 = 9;
const int pin2 = 10;
const int pin3 = 11;
const int wdTime = 300;  // watchdog timeout (ms)

// Varijable stanja 
String input = "";
unsigned long lastMessageTime = 0;

int x = 127, y = 127, z = 0;
int a = 0, b = 0, c = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
}

void loop() {
  // Obrada serijskog unosa 
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

  // Offset korekcija 
  if ((x > 110) && (x < 145)) x = 127;
  if ((y > 110) && (y < 145)) y = 127;
  if (z > 249) z = 255;
  if (z < 6) z = 0;

  // Watchdog: isključi ako nema poruke
  if (millis() - lastMessageTime > wdTime) {
    shutdown();
    
  }
}

void shutdown() {
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
  analogWrite(pin3, 0);
  z = 0;
  x = 127; y = 127;
  a = b = c = 0;
}
