

const int pin = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(pin, OUTPUT);
}

void loop() {
  //int brightness = 254;
  int brightness = 250;
  analogWrite(pin, brightness);

  delay(20);
}