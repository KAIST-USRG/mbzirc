/*
  Controlling a servo position using a potentiometer (variable resistor)
  by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

  modified on 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Knob
*/
#include <ros.h>
#include <std_msgs/Bool.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
ros::NodeHandle  nh;


int servo;    // variable to read the value from the analog pin
int val;
int flag;
bool current_status;

void messageCb( const std_msgs::Bool& magnet_on_msg) {
  if (magnet_on_msg.data) {
    val = 1023;
    servo = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
//    Serial.println(servo);
//    Serial.println("on");
    myservo.write(servo);                  // sets the servo position according to the scaled value
    delay(3000);
    flag = 1;
    current_status = 1;
    myservo.write(90);
  }
//  if (flag)
//  {
//    val = 512;
//    servo = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
////    Serial.println(servo);
//    myservo.write(servo);                  // sets the servo position according to the scaled value
//    delay(3000);
//      
//  }
  
  else if (!magnet_on_msg.data)
  {
    val = 0;
    servo = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
//    Serial.println(servo);
    myservo.write(servo);                  // sets the servo position according to the scaled value
    delay(12000);
    flag = 0;
    current_status = 0;
    myservo.write(90);
  }
  
}

ros::Subscriber<std_msgs::Bool> sub("/magnet_on", &messageCb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  flag = 0;
  current_status = 0;
}


void loop() {
  nh.spinOnce();
  delay(1);
}
