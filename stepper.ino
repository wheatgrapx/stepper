/* Example sketch to control a stepper motor with TB6600 stepper motor driver, 
  AccelStepper library and Arduino: acceleration and deceleration. 
  More info: https://www.makerguides.com */

// Include the AccelStepper library:

// speed positive: move to right
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "myStepper.h"

long position = 0;

// store the clamp status, false is released, true is clamped
bool clamped_guidewire = 0;
bool clamped_catheter = 0;
bool clamped_guidewire_stepper = 0;
bool clamped_catheter_stepper = 0;

AccelStepper stepper1(1, stepPin_top, 2);
AccelStepper stepper2(1, stepPin_bottom, dirPin_bottom);
myStepper stepper_top(stepper1, limit_top_front, limit_top_back, 72);
myStepper stepper_bottom(stepper2, limit_bottom_front, limit_bottom_back, 125);

AccelStepper stepper_guidewire(1, stepPin_guidewire, dirPin_guidewire);
AccelStepper stepper_catheter(1, stepPin_catheter, dirPin_catheter);

AccelStepper stepper3(1, stepPin_clamp_guidewire, dirPin_clamp_guidewire);
AccelStepper stepper4(1, stepPin_clamp_catheter, dirPin_clamp_catheter);
myStepper clamp_guidewire(stepper3);
myStepper clamp_catheter(stepper4);

clamp_system system_top(stepper_top, clamp_guidewire, 0);
clamp_system system_bottom(stepper_bottom, stepper_top, clamp_catheter, clamp_guidewire, 1);

void setup() {
  Serial.begin(9600);

  pinMode(limit_top_front, INPUT_PULLUP);
  pinMode(limit_top_back, INPUT_PULLUP);
  pinMode(limit_bottom_front, INPUT_PULLUP);
  pinMode(limit_bottom_back, INPUT_PULLUP);

  pinMode(motor_guidewire_A, OUTPUT);
  pinMode(motor_guidewire_B, OUTPUT);
  pinMode(motor_catheter_A, OUTPUT);
  pinMode(motor_catheter_B, OUTPUT);

  stepper_guidewire.setMaxSpeed(speed_set);
  stepper_catheter.setMaxSpeed(speed_set);
  
  // homing of steppers
  // delay(1000);
  // homing(stepper_top, stepper_bottom);
  // delay(1000);

  delay(3000);
  Serial.println("begin");
  release(motor_guidewire_A, motor_guidewire_B);
  release(motor_catheter_A, motor_catheter_B);

  
}

void loop() {
  int mode = 0;
  long dist = 0;
  bool manual = 0;

  if(Serial.available() > 0) {
    Serial.println("data received");
    mode = Serial.parseInt();
    Serial.println(mode);
    dist = 500;
    if(mode < 0) dist = -500;
    mode = abs(mode);
    motion_control(mode, dist);
  }
  
  delay(10);
}

void motion_control(int mode, long dist) {
  /*
  1: top conveyor move
  2. bottom conveyor move
  3. top clamp stay still, bottom clamp move
  4: clamp / release catheter
  5: clamp / release guidewire
  6: turn catheter
  7: turn guidewire
  8: clamp / release catheter (stepper)
  9: clamp / release guidewire (stepper)
  */
  long steps_to_clamp = 1000;
  if(mode == 1) {
    Serial.println("top move");
    position -= system_top.move(dist);
    Serial.print("position in mm: ");
    Serial.println(position);
  }
  else if(mode == 2) {
    Serial.println("bottom move");
    position -= system_bottom.move(dist);
    Serial.print("position in mm: ");
    Serial.println(position);
  }
  else if(mode == 3) {
    long step_bottom = dist;
    long step_top = step_bottom * 125 / 72;
    
    Serial.println("top conveyor stay still");
    position -= system_bottom.syncMove(-step_top, step_bottom);
    Serial.print("position in mm: ");
    Serial.println(position);
  }
  else if(mode == 4) {
    if(clamped_catheter) {
      Serial.println("releasing catheter (linear motor)");
      release(motor_catheter_A, motor_catheter_B);
      delay(1000);
      clamped_catheter = false;
    }
    else {
      Serial.println("clamping catheter (linear motor)");
      clamp(motor_catheter_A, motor_catheter_B);
      delay(1000);
      clamped_catheter = true;
    }
  }
  else if(mode == 5) {
    if(clamped_guidewire) {
      Serial.println("releasing guidewire (linear motor)");
      release(motor_guidewire_A, motor_guidewire_B);
      delay(1000);
      clamped_guidewire = false;
    }
    else {
      Serial.println("clamping guidewire (linear motor)");
      clamp(motor_guidewire_A, motor_guidewire_B);
      delay(1000);
      clamped_guidewire = true;
    }
  }
  else if(mode == 6) {
    Serial.println("turning catheter");
    constSpeed(&stepper_catheter, speed_set, dist);
    delay(1000);
  }
  else if(mode == 7) {
    Serial.println("turning guidewire");
    constSpeed(&stepper_guidewire, speed_set, dist);
    delay(1000);
  }
  else if(mode == 8) {
    clamped_catheter_stepper = system_bottom.move_stepper_clamp(clamped_catheter_stepper);
    delay(1000);
  }
  else if(mode == 9) {
    clamped_guidewire_stepper = system_top.move_stepper_clamp(clamped_guidewire_stepper);
    delay(1000);
  }
  Serial.println("finished motion");
  delay(1000);
}





