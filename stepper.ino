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
myStepper stepper_top(stepper1, limit_top_left, limit_top_right, speed_set, 72);
myStepper stepper_bottom(stepper2, limit_bottom_left, limit_bottom_right, speed_set, 125);

void setup() {
  Serial.begin(115200);
  pinMode(limit_top_left, INPUT_PULLUP);
  pinMode(limit_top_right, INPUT_PULLUP);
  pinMode(limit_bottom_left, INPUT_PULLUP);
  pinMode(limit_bottom_right, INPUT_PULLUP);
  
  // Set the maximum speed and acceleration:
  stepper_top.set(speed_set, 500);
  stepper_bottom.set(speed_set, 500);


  // homing of steppers
  // delay(1000);
  homing(stepper_top, stepper_bottom);
  // delay(1000);
  // position += syncMove(steppers, -step_rev, -step_rev);
  // delay(1000);
  Serial.println("begin");
}

void loop() {
  int dist = 0;

  if(Serial.available() > 0) {
    Serial.print("data incoming...");
    dist = Serial.parseInt();
    Serial.println(dist);

    Serial.println("move motors separately");
    position -= stepper_top.move(dist);
    Serial.println(position);
    delay(1000);
    position -= stepper_bottom.move(dist);
    Serial.println(position);
    delay(1000);

    long step_bottom = dist;
    long step_top = step_bottom * 125 / 72;
    
    // Serial.println("top conveyor stay still");
    // position -= syncMove(steppers, -step_top, step_bottom, stepper_top, stepper_bottom);
    // Serial.println(position);
    // delay(1000);
    // position -= syncMove(steppers, step_top, -step_bottom, stepper_top, stepper_bottom);
    // Serial.println(position);
    // delay(1000);
    // Serial.println("finished");
  }

  // Serial.print(digitalRead(6));
  // Serial.print(digitalRead(7));
  // Serial.print(digitalRead(8));
  // Serial.println(digitalRead(9));
  

}



