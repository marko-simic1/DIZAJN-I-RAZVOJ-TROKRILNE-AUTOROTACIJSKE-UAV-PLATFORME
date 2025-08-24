

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;


std_msgs::String str_msg;
ros::Publisher chatter("izlaz", &str_msg);

char received_data[100];  // Adjust size as needed


void messageCb( const std_msgs::String& toggle_msg){
  strncpy(received_data, toggle_msg.data, sizeof(received_data) - 1);
  received_data[sizeof(received_data) - 1] = '\0';  // Safety null-termination
  str_msg.data = received_data;
}

ros::Subscriber<std_msgs::String> sub("taz_speed", messageCb );



void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
