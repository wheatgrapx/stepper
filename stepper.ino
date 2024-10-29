/* Example sketch to control a stepper motor with TB6600 stepper motor driver, 
  AccelStepper library and Arduino: acceleration and deceleration. 
  More info: https://www.makerguides.com */

// Include the AccelStepper library:

// speed positive: move to right
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "myStepper.h"

long position = 0;


AccelStepper stepper1(1, stepPin_top, dirPin_top);
AccelStepper stepper2(1, stepPin_bottom, dirPin_bottom);

myStepper stepper_top(stepper1, limit_top_left, limit_top_right, 72);
myStepper stepper_bottom(stepper2, limit_bottom_left, limit_bottom_right, 125);

clamp_system system_top(stepper_top, motor_clamp_top, 0);
clamp_system system_bottom(stepper_bottom, stepper_top, motor_clamp_bottom, 1);

void setup() {
  Serial.begin(115200);
  pinMode(limit_top_left, INPUT_PULLUP);
  pinMode(limit_top_right, INPUT_PULLUP);
  pinMode(limit_bottom_left, INPUT_PULLUP);
  pinMode(limit_bottom_right, INPUT_PULLUP);
  
  // homing of steppers
  // delay(1000);
  // homing(stepper_top, stepper_bottom);
  // delay(1000);
  
  // position += syncMove(steppers, -step_rev, -step_rev);
  // delay(1000);
  Serial.println("begin");
}

void loop() {

  int mode = 0;
  long dist = 0;

  if(Serial.available() > 0) {
    Serial.println("data incoming...");
    for(int i = 0; i < 2; i++) {
      if(i) dist = Serial.parseInt();
      else mode = Serial.parseInt();
    }
    Serial.println(mode);
    Serial.println(dist);
  }

  if(mode == 1) {
    Serial.println("top move");
    position -= system_top.move(dist);
    Serial.print("position in mm: ");
    Serial.println(position);
    Serial.println("finished motion");
    delay(1000);
  }
  
  else if(mode == 2) {
    Serial.println("bottom move");
    position -= system_bottom.move(dist);
    Serial.print("position in mm: ");
    Serial.println(position);
    Serial.println("finished motion");
    delay(1000);
  }

  else if(mode == 3) {
    long step_bottom = dist;
    long step_top = step_bottom * 125 / 72;
    
    Serial.println("top conveyor stay still");
    position -= system_bottom.syncMove(-step_top, step_bottom);
    Serial.print("position in mm: ");
    Serial.println(position);
    Serial.println("finished motion");
    delay(1000);
  }
  
}



