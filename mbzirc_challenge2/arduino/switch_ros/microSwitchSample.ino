#include <ros.h>
#include <std_msgs/Bool.h>

int ledPin = 13;                // choose the pin for the LED
int inputPin = 3;               // Connect sensor to input pin 3 

ros::NodeHandle  nh;

std_msgs::Bool switch_state_msg;
ros::Publisher switch_state_pub("switch_state", &switch_state_msg);

void setup() {
  nh.initNode();
  nh.advertise(switch_state_pub);
  Serial.begin(9600);           // Init the serial port
  
  pinMode(ledPin, OUTPUT);      // declare LED as output
  pinMode(inputPin, INPUT);     // declare Micro switch as input
}

void loop(){
  int val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {                // check if the input is HIGH
    digitalWrite(ledPin, LOW);      // turn LED OFF
    switch_state_msg.data = false;
    switch_state_pub.publish( &switch_state_msg );
  } else {
    digitalWrite(ledPin, HIGH);     // turn LED ON
    switch_state_msg.data = true;
    switch_state_pub.publish( &switch_state_msg );
    Serial.println("Switch Pressed!");
  }
  nh.spinOnce();
  delay(50);
}
