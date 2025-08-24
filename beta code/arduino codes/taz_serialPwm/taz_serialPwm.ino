String input = "";
const int pin1 = 9;
const int pin2 = 10;
const int pin3 = 11;


void setup() {
  Serial.begin(115200);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
}

void loop() {
  // Čitaj serijski ulaz
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();

      // Parsiraj vrijednost z
      int zIndex = input.indexOf("z:");
      if (zIndex != -1) {
        int spaceIndex = input.indexOf(' ', zIndex);
        String zStr = input.substring(zIndex + 2, spaceIndex == -1 ? input.length() : spaceIndex);
        int zVal = zStr.toInt();
        zVal = constrain(255-zVal, 0, 255);
        analogWrite(pin1, zVal);
        analogWrite(pin2, zVal);
        analogWrite(pin3, zVal);
      }

      input = "";
    } else {
      input += c;
    }
  }

}
