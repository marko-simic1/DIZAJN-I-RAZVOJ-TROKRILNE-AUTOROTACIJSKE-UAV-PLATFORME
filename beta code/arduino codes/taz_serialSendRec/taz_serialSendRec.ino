String input = "";
const int pinLED = 11;


void setup() {
  Serial.begin(115200);
  pinMode(pinLED, OUTPUT);
}

void loop() {
  // Čitaj serijski ulaz
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      Serial.println("Echo: " + input);  // Vrati nazad poruku

      // Parsiraj vrijednost z
      int zIndex = input.indexOf("z:");
      if (zIndex != -1) {
        int spaceIndex = input.indexOf(' ', zIndex);
        String zStr = input.substring(zIndex + 2, spaceIndex == -1 ? input.length() : spaceIndex);
        int zVal = zStr.toInt();
        zVal = constrain(zVal, 0, 255);
        analogWrite(pinLED, zVal);
      }

      input = "";  // Resetiraj buffer za novu poruku
    } else {
      input += c;
    }
  }
}
