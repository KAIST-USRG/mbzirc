#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Bool& magnet_on_msg) {
  if (magnet_on_msg.data)
    digitalWrite(13, LOW);   // blink the led
  else
    digitalWrite(13, HIGH);   // blink the led
}

ros::Subscriber<std_msgs::Bool> sub("magnet_on", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
