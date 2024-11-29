/* Example sketch to control a stepper motor with TB6600 stepper motor driver, 
  AccelStepper library and Arduino: acceleration and deceleration. 
  More info: https://www.makerguides.com */

// Include the AccelStepper library:

// speed positive: move to right
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "myStepper.h"

long position = 0;
int order = 0;

// store the clamp status, false is released, true is clamped
bool clamped_guidewire = 0;
bool clamped_catheter = 0;
bool clamped_guidewire_stepper = 0;
bool clamped_catheter_stepper = 0;

// predefined flow
long motions[][2] = {
  {5, 0},
  {4, 0},
  {4, 0},
  // {1, -1200},
  // {9, 400},
  // {1, 500},
  // {1, -300},
  // {1, -1200},
  // {1, 1200},
  // {9, 400},
  // {1, -1200}
};


AccelStepper stepper1(1, stepPin_top, dirPin_top);
AccelStepper stepper2(1, stepPin_bottom, dirPin_bottom);

myStepper stepper_top(stepper1, limit_top_front, limit_top_back, 72);
myStepper stepper_bottom(stepper2, limit_bottom_front, limit_bottom_back, 125);

clamp_system system_top(stepper_top, motor_clamp_top, 0);
clamp_system system_bottom(stepper_bottom, stepper_top, motor_clamp_bottom, 1);

AccelStepper stepper_guidewire(1, stepPin_guidewire, dirPin_guidewire);
AccelStepper stepper_catheter(1, stepPin_catheter, dirPin_catheter);

AccelStepper clamp_guidewire(1, stepPin_clamp_guidewire, dirPin_clamp_guidewire);
AccelStepper clamp_catheter(1, stepPin_clamp_catheter, dirPin_clamp_catheter);

void setup() {
  Serial.begin(115200);
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
  clamp_guidewire.setMaxSpeed(speed_set);
  clamp_catheter.setMaxSpeed(speed_set);
  
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
  // Serial.print(digitalRead(limit_top_front));
  // Serial.print(digitalRead(limit_top_back));
  // Serial.print(digitalRead(limit_bottom_front));
  // Serial.println(digitalRead(limit_bottom_back));
  if(Serial.available() > 0) {
    Serial.println("data incoming...");
    for(int i = 0; i < 3; i++) {
      if(i == 0) mode = Serial.parseInt();
      else if(i == 1) dist = Serial.parseInt();
      else if(i == 2) manual = Serial.parseInt();
    }
    Serial.println("data received");
    if(manual) {
      for(int i = 0; i < sizeof(motions) / sizeof(motions[0]); i++) {
        int mode = motions[order][0];
        long dist = motions[order][1];
        order++;
        Serial.println(mode);
        Serial.println(dist);
      
        motion_control(mode, dist);
      }
    }
    else motion_control(mode, dist);
  }
  
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
  long stepper_clamp = 1000;
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
    if(clamped_catheter_stepper) {
      Serial.println("releasing catheter (stepper motor)");
      constSpeed(&clamp_catheter, speed_set, -stepper_clamp);
      delay(1000);
      clamped_catheter_stepper = false;
    }
    else {
      Serial.println("clamping catheter (stepper motor)");
      constSpeed(&clamp_catheter, speed_set, stepper_clamp);
      delay(1000);
      clamped_catheter_stepper = true;
    }
  }
  else if(mode == 9) {
    if(clamped_guidewire_stepper) {
      Serial.println("releasing guidewire (stepper motor)");
      constSpeed(&clamp_guidewire, speed_set, -stepper_clamp);
      delay(1000);
      clamped_guidewire_stepper = false;
    }
    else {
      Serial.println("clamping guidewire (stepper motor)");
      constSpeed(&clamp_guidewire, speed_set, stepper_clamp);
      delay(1000);
      clamped_guidewire_stepper = true;
    }
  }
  Serial.println("finished motion");
  delay(1000);
}



