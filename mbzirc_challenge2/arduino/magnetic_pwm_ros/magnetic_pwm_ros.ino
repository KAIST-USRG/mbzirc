#include <ros.h>
#include <std_msgs/Bool.h>
#include <Servo.h>

Servo myservo_l;  // create servo object to control a servo
Servo myservo_r;
ros::NodeHandle  nh;


int servo;    // variable to read the value from the analog pin
int val;
int cur_flag;
int prev_flag = -1;
int time_MAG = 0;

int cur_time = 0;
int prev_time = 0;

bool current_status;

// switch
int ledPin = 13;                // choose the pin for the LED
int inputPin_s1 = 2;     
int inputPin_s2 = 3;  
         

std_msgs::Bool switch_state_msg;
ros::Publisher switch_state_pub("switch_state", &switch_state_msg); // switch


void messageCb( const std_msgs::Bool& magnet_on_msg) {
  if (magnet_on_msg.data) {
    cur_flag = 1;
  }
  
  else if (!magnet_on_msg.data)
  {
    cur_flag = 0;
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
  
  pinMode(inputPin_s1, INPUT);     // declare Micro switch as input
  pinMode(inputPin_s2, INPUT);     // declare Micro switch as inputv

  pinMode(ledPin, OUTPUT);         // declare LED as output
  for(int i = 0; i < 5; i++)       // Let us know the setup is completed
  { 
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
  cur_time = millis();
  prev_time = cur_time;
}


void loop() {
  cur_time = millis();
  if (cur_flag != prev_flag)
  {
    time_MAG = millis();
    prev_flag = cur_flag;
  }
  
  if(cur_time - time_MAG >= 300){ // steady state
      myservo_l.write(90);
      myservo_r.write(90);
      
  }else // create pulse according to the flag
  {
    if(cur_flag == 0)
    {
      myservo_l.write(0);
      myservo_r.write(0);
    }else if (cur_flag == 1)
    {
      myservo_l.write(180);
      myservo_r.write(180);    
    }
  }
  
  
  // switch
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
