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

Servo myservo_l;  // create servo object to control a servo
Servo myservo_r;
ros::NodeHandle  nh;


int servo;    // variable to read the value from the analog pin
int val;
int flag;
bool current_status;

// switch
int ledPin = 13;                // choose the pin for the LED
int inputPin_s1 = 2;     
int inputPin_s2 = 3;  
         

std_msgs::Bool switch_state_msg;
ros::Publisher switch_state_pub("switch_state", &switch_state_msg); // switch


void messageCb( const std_msgs::Bool& magnet_on_msg) {
  if (magnet_on_msg.data) {
    myservo_l.write(175);                  // sets the servo position according to the scaled value
    delay(200);
    flag = 1;
    current_status = 1;
    myservo_l.write(90);
    delay(50);
    myservo_r.write(175);
    delay(200);
    myservo_r.write(90);
  }
  
  else if (!magnet_on_msg.data)
  {
    myservo_l.write(5);                  // sets the servo position according to the scaled value
    delay(200);
    flag = 0;
    current_status = 0;
    myservo_l.write(90);
    delay(50);
    myservo_r.write(5);
    delay(200);
    myservo_r.write(90);
  }
  
}

ros::Subscriber<std_msgs::Bool> sub("/magnet_on", &messageCb ); // subscribe

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(switch_state_pub);
  Serial.begin(9600);
  
  myservo_l.attach(10);  // attaches the servo on pin 10 to the servo object
  myservo_r.attach(11);
  flag = 0;
  current_status = 0;
  
  pinMode(inputPin_s1, INPUT);     // declare Micro switch as input
  pinMode(inputPin_s2, INPUT);     // declare Micro switch as input

  pinMode(ledPin, OUTPUT);      // declare LED as output
  for(int i=0; i < 5; i++)
  {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
}


void loop() {
  int read_input_s1 = digitalRead(inputPin_s1);  // read input value
  int read_input_s2 = digitalRead(inputPin_s2);  // read input value

  int stop_condition = read_input_s1 + read_input_s2;
  if (stop_condition >= 1) {        // at least one switch is triggered
    digitalWrite(ledPin, HIGH);      // turn LED ON
    switch_state_msg.data = true;
    switch_state_pub.publish( &switch_state_msg );
    Serial.println("Switch Pressed!");
  } else {
    digitalWrite(ledPin, LOW);     // turn LED OFF
    switch_state_msg.data = false;
    switch_state_pub.publish( &switch_state_msg );
  }
  nh.spinOnce();
  delay(1);
}
