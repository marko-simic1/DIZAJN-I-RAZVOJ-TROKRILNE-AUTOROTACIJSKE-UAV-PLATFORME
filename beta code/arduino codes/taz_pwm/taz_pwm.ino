#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

const int pin = 3;


float dutyC = 0.0;
unsigned long lastMsgTime = 0;
const unsigned long timeout = 300; // milliseconds

std_msgs::String pubMsg;
std_msgs::String stopMsg;
char msgBuffer[20];  // buffer for float-to-string conversion

ros::Publisher izlaz("izlaz", &pubMsg);

void tazCallback(const geometry_msgs::Twist& msg) {
  lastMsgTime = millis(); // reset watchdog
  dutyC = constrain(msg.linear.z, 0.0, 1.0);

  analogWrite(pin, int(dutyC*254));

  dtostrf(dutyC, 4, 2, msgBuffer); // convert float to string
  pubMsg.data = msgBuffer;
}

ros::Subscriber<geometry_msgs::Twist> sub("taz_speed", tazCallback);

void setup() {
  Serial.begin(57600);

  pinMode(pin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(izlaz);

  stopMsg.data = "Cycle stop";
}

void loop() {
  nh.spinOnce();

  if (millis() - lastMsgTime > timeout) {
    if (dutyC > 0.0) {
      dutyC = 0.0;
      analogWrite(pin, int(dutyC*254));
      izlaz.publish(&stopMsg);
    }
    //lastMsgTime = millis(); // prevent repeated publishing
  } else {
    izlaz.publish(&pubMsg);
    analogWrite(pin, int(dutyC*254));
  }

  delay(10); // stabilize loop
}
