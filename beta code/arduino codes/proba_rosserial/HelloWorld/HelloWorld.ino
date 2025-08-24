#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[] = "hello world!";
bool ros_initialized = false;

void setup() {
  Serial.begin(115200);
  delay(2000);  // Let system stabilize
}

void loop() {
  if (!ros_initialized) {
    nh.initNode();
    nh.advertise(chatter);
    ros_initialized = true;
  }

  if (nh.connected()) {
    str_msg.data = hello;
    chatter.publish(&str_msg);
  }

  nh.spinOnce();
  delay(1000);
}
