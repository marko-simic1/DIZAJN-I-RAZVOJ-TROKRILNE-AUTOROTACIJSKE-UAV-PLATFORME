String input = "";
unsigned long lastMessageTime = 0;

const int wdTime = 300;   // 300ms watchdog
const int pin1 = 9;
const int pin2 = 10;
const int pin3 = 11;

int x = 127;
int y = 127;
int z = 0;
int a = 0;
int b = 0;
int c = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
}

void loop() {
  while (Serial.available()) {  
    char ch = Serial.read();
    if (ch == '\n') {
      input.trim();

      // Split po razmacima
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
        // Sve vrijednosti primljene i parsirane
        x = constrain(values[0], 0, 255);
        y = constrain(values[1], 0, 255);
        z = constrain(255 - values[2], 0, 255); // obrnut smjer
        a = values[3];
        b = values[4];
        c = values[5];

        analogWrite(pin1, z);
        analogWrite(pin2, z);
        analogWrite(pin3, z);

        lastMessageTime = millis(); // watchdog reset
        Serial.print("x="); Serial.print(x);
        Serial.print(" y="); Serial.print(y);
        Serial.print(" z="); Serial.println(z);


      } else {
        //Serial.println("Invalid input: not 6 values, ignored.");
      }

      input = "";
    } else {
      input += ch;
    }
  }

  // Watchdog
  if (millis() - lastMessageTime > wdTime) {
    if (z != 0) {
      analogWrite(pin1, 0);
      analogWrite(pin2, 0);
      analogWrite(pin3, 0);
      z = 0;
      x = 0;
      y = 0;
      a = 0;
      b = 0;
      c = 0;
      //Serial.println("Watchdog timeout: reset all values.");
    }
  }
}
