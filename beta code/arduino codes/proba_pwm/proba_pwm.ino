

const int pin = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int brightness = 0; brightness < 255; brightness++) {
    analogWrite(pin, brightness);
    delay(20);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(pin, brightness);
    delay(20);
  }
}
