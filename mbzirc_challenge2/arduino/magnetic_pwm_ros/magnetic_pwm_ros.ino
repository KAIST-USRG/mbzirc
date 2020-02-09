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

uint32_t cur_time = 0;
uint32_t prev_time = 0;
int time_MAG = 0;

int cur_Flag = -1;
int prev_Flag = -1;

// switch
int ledPin = 13;                // choose the pin for the LED
int inputPin_s1 = 2;     
int inputPin_s2 = 3;  
         

std_msgs::Bool switch_state_msg;
ros::Publisher switch_state_pub("switch_state", &switch_state_msg); // switch


void messageCb( const std_msgs::Bool& magnet_on_msg) {
  if (magnet_on_msg.data) {
    cur_Flag = 1;
  }
  
  else if (!magnet_on_msg.data)
  {
    cur_Flag = 0;
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
  cur_time = millis();
  prev_time = cur_time;
}


void loop() {
  
  // magnet part
  cur_time = millis();
  if(cur_Flag != prev_Flag) {
    time_MAG = millis();
    prev_Flag = cur_Flag;
  }
  if(cur_Flag == 0) { // Magnet OFF
    if(cur_time - time_MAG < 50) {
      myservo_l.write(5);
      myservo_r.write(5);
    }
    else if(cur_time - time_MAG > 200) {
      myservo_l.write(90);
      myservo_r.write(90);
    }
  }
  else if(cur_Flag == 1) { // Magnet ON
    if(cur_time - time_MAG < 50) {
      myservo_l.write(175);
      myservo_r.write(175);
    }
    else if(cur_time - time_MAG > 200) {
      myservo_l.write(90);
      myservo_r.write(90);

    }
  }

  // switch part
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
