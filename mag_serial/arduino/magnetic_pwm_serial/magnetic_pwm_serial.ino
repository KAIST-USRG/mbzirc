/*
  Controlling a servo position using a potentiometer (variable resistor)
  by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

  modified on 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Knob
*/
//#include "header.h"
#include "structs_functions.h"
#include <Servo.h>

Servo MAG_LEFT;
Servo MAG_RIGHT;



void setup() {
  Serial.begin(921600); // USB, ROS Interface
#ifdef DEBUG_MODE
  Serial.println("Setup Start");
  Serial.println("");
#endif
  pinMode(PC13, OUTPUT); // heartbeat
  pinMode(PA0, INPUT); // TSW_1
  pinMode(PA1, INPUT); // TSW_2
//  pinMode(PA2, INPUT); // TSW_3
//  pinMode(PA3, INPUT); // TSW_4
  pinMode(PA4, OUTPUT); // LED_MAG_LEFT
  pinMode(PA5, OUTPUT); // LED_MAG_RIGHT
  pinMode(PA6, OUTPUT); // LED_TSW_1
  pinMode(PA7, OUTPUT); // LED_TSW_2
  pinMode(PB0, OUTPUT); // LED_TSW_3
  pinMode(PB1, OUTPUT); // LED_TSW_4
  MAG_LEFT.attach(PB8); // MAG_LEFT
  MAG_RIGHT.attach(PB9); // MAG_RIGHT
  delay(2);
  for(int i = 0; i < 10; i++) { // Let us know the setup is completed
    Toggle_OnOff_LED(13, 0);
    delay(200);
  }
  cur_time = millis();
  prev_time = cur_time;
}



void loop() {
  // put your main code here, to run repeatedly:
  cur_time = millis();
  if (cur_time - prev_time > 500)
  {
    Toggle_OnOff_LED(13, 0);
    prev_time = cur_time;
    flag_ROS_TX_Status = 1;
  }

  //
  // Read Tactile Switches & Control LEDs
  //
  status_TSW_1 = digitalRead(PA0);
  status_TSW_2 = digitalRead(PA1);
//  status_TSW_3 = digitalRead(PA2);
//  status_TSW_4 = digitalRead(PA3);

  Toggle_OnOff_LED(6, status_TSW_1);
  Toggle_OnOff_LED(7, status_TSW_2);
//  Toggle_OnOff_LED(0, status_TSW_3);
//  Toggle_OnOff_LED(1, status_TSW_4);
  
//  status_TSW = status_TSW_1 && status_TSW_2 && sta/tus_TSW_3 && status_TSW_4;
  status_TSW = status_TSW_1 || status_TSW_2;
  
  
  
  //
  // ROS Interface
  //
  if(flag_ROS_TX_Status == 1)
    ROS_TX(1); // [1] TX_Status
  
  ROS_RX();



  //
  // Magnet Control & Control LEDs
  //

  // for test
//  if (cur_time < 10000) {
//    cur_FlagA = 1;
//  }
//  else {
//    cur_FlagA = 0;
//  }
//
//  cur_FlagA = 0;
  
  if(cur_FlagA != prev_FlagA) {
    time_MAG = millis();
    prev_FlagA = cur_FlagA;
  }
  if(cur_FlagA == 0) { // Magnet Off
    if(cur_time - time_MAG < 50) {
      MAG_LEFT.write(5);
      MAG_RIGHT.write(5);
    }
    else if(cur_time - time_MAG > 200) {
      MAG_LEFT.write(90);
      MAG_RIGHT.write(90);
      status_MAG_LEFT = 0;
      status_MAG_RIGHT = 0;
    }
  }
  else if(cur_FlagA == 1) { // Magnet On
    if(cur_time - time_MAG < 50) {
      MAG_LEFT.write(175);
      MAG_RIGHT.write(175);
    }
    else if(cur_time - time_MAG > 200) {
      MAG_LEFT.write(90);
      MAG_RIGHT.write(90);
      status_MAG_LEFT = 1;
      status_MAG_RIGHT = 1;
    }
  }

  Toggle_OnOff_LED(4, status_MAG_LEFT);
  Toggle_OnOff_LED(5, status_MAG_RIGHT);

  status_MAG = status_MAG_LEFT && status_MAG_RIGHT;
  
}
