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

// switch
int ledPin = 13;                // choose the pin for the LED
int inputPin = 11;               // Connect sensor to input pin 3

std_msgs::Bool switch_state_msg;
ros::Publisher switch_state_pub("switch_state", &switch_state_msg); // switch



void messageCb( const std_msgs::Bool& magnet_on_msg) {
  if (magnet_on_msg.data) {
    myservo.write(175);                  // sets the servo position according to the scaled value
    delay(3000);
    flag = 1;
    current_status = 1;
    myservo.write(90);
  }
  
  else if (!magnet_on_msg.data)
  {
    val = 3;
    myservo.write(val);                  // sets the servo position according to the scaled value
    delay(12000);
    flag = 0;
    current_status = 0;
    myservo.write(90);
  }
  
}

ros::Subscriber<std_msgs::Bool> sub("/magnet_on", &messageCb ); // subscribe

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(switch_state_pub);
  Serial.begin(9600);
  
  myservo.attach(10);  // attaches the servo on pin 10 to the servo object
  flag = 0;
  current_status = 0;
  
  pinMode(inputPin, INPUT);     // declare Micro switch as input
  pinMode(ledPin, OUTPUT);      // declare LED as output
}


void loop() {
  int read_input = digitalRead(inputPin);  // read input value
  if (read_input == HIGH) {                // check if the input is HIGH
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
  delay(1);
}
