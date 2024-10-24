#include "Arduino.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "myStepper.h"

int step_rev = 800;
float speed_set = 400;

myStepper::myStepper() {
  return;
}


myStepper::myStepper(AccelStepper s, int l, int r, int rev) {
  this->stepper = s;
  this->limit_left = l;
  this->limit_right = r;
  this->dist_rev = rev;
  set(speed_set, 500);
}


void myStepper::set(float max_speed, float accel) {
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(accel);
}


// return true if limit is reached
bool myStepper::limit() {
  bool reached = !digitalRead(limit_left) || !digitalRead(limit_right);
  return reached;
}


// move the stepper away when limit is reached 
bool myStepper::awayFromLimit(long prev_position) {
  long dist_to_prev = abs(getStepperPosition() - prev_position);
  long dist_predef = step_rev / 4;
  long dist_back = min(dist_to_prev, dist_predef);
  if(!digitalRead(limit_left)) {
    Serial.print(dist_back);
    Serial.println(" left_limit");
    constSpeed(&stepper, speed_set * 0.5, dist_back);
    return 0;
  }
  else if(!digitalRead(limit_right)) {
    Serial.print(dist_back);
    Serial.println(" right_limit");
    constSpeed(&stepper, speed_set * 0.5, -dist_back);
    return 1;
  }
  
}


long myStepper::getStepperPosition() {
  return stepper.currentPosition();
}


long myStepper::getTarget() {
  return stepper.targetPosition();
}


void myStepper::setSpeed(float speed) {
  stepper.setSpeed(speed);
}


void myStepper::run() {
  stepper.runSpeed();
}


void myStepper::moveRelative(long position)  {
  stepper.move(position);
}


// move the stepper relative to current position
// will not call awayFromLimit() when limit is reached
// positive is to the right
void constSpeed(AccelStepper* stepper, float speed, long position) {
  long pos = position + stepper->currentPosition();
  bool dir = position < 0;
  if(dir) speed = -speed;
  
  stepper->moveTo(pos);
  stepper->setSpeed(speed);
  int start = millis();
  while(stepper->currentPosition() != pos) {
    stepper->runSpeed();
  }
  stepper->stop();
  int total = millis() - start;
  Serial.print("time: ");
  Serial.println(total);
  stepper->setSpeed(0);
}

void withAccel(AccelStepper* stepper, long position) {
  stepper->moveTo(position);
  int start = millis();
  stepper->runToPosition();
  int total = millis() - start;
  Serial.print("time: ");
  Serial.println(total);
}


// move all stepper to the right until both limits are reached
// move the stepper a bit away from the limit after reaching the limit
void homing(myStepper top, myStepper bottom) {
  float speed_homing = speed_set * 0.75;
  long offset_dist = 20;

  Serial.println("homing...");
  top.setSpeed(speed_homing);
  bottom.setSpeed(speed_homing);

  int start = millis();
  while(true) {
    if(top.limit() && bottom.limit()) break;  // exit the while loop when both limits are reached
    if(!top.limit()) {                        // stop top motor when limit is reached
      top.run();
      Serial.print("top");
    }
    if(!bottom.limit()) {                     // stop bottom motor when limit is reached
      bottom.run();
      Serial.print("bottom");
    }
    Serial.println();
  }
  int total = millis() - start;
  Serial.print("time: ");
  Serial.println(total);
  delay(500);

  top.stepper.setCurrentPosition(0);
  bottom.stepper.setCurrentPosition(0);
  
  constSpeed(&(top.stepper), speed_homing, -offset_dist * step_rev / 72);
  Serial.println("top finished homing");
  delay(500);

  constSpeed(&(bottom.stepper), speed_homing, -offset_dist * step_rev / 125);
  Serial.println("bottom finished homing");
  delay(500);
}


clamp_system::clamp_system(myStepper stepper, int motor_clamp_pin, bool have_top) {
  this->stepper = stepper;
  this->motor_clamp_pin = motor_clamp_pin;
  this->have_top = have_top;
}


clamp_system::clamp_system(myStepper stepper, myStepper stepper_on_top, int motor_clamp_pin, bool have_top) {
  this->stepper = stepper;
  this->stepper_on_top = stepper_on_top;
  this->motor_clamp_pin = motor_clamp_pin;
  this->have_top = have_top;
}


// move the stepper relative to current position
// will call awayFromLimit() when limit is reached
// positive is to the right
long clamp_system::move(long position) {
  long temp = stepper.getStepperPosition();

  if(position == 0) return 0;
  
  stepper.moveRelative(position);
  int start = millis();
  if(position > 0) stepper.setSpeed(speed_set);
  else stepper.setSpeed(-speed_set);
  while(stepper.getStepperPosition() != stepper.getTarget()) {
    if(stepper.limit()) {
      delay(1000);
      bool dir = stepper.awayFromLimit(temp);
      delay(1000);

      long dist_moved = stepper.getStepperPosition() - temp;
      long dist_not_moved = position - dist_moved;
      Serial.println("dist");
      Serial.print(stepper.getStepperPosition());
      Serial.print("\t");
      Serial.print(temp);
      Serial.print("\t");
      Serial.print(dist_moved);
      Serial.print("\t");
      Serial.println(dist_not_moved);

      Serial.print(motor_clamp_pin);
      Serial.println(" catheter clamped");

      long step_bottom = 6000;
      long step_top = step_bottom * 125 / 72;
      if(dir) step_bottom = -step_bottom;
      else step_top = -step_top;
      Serial.println("pushback");
      syncMove(step_top, step_bottom);

      Serial.print(motor_clamp_pin);
      Serial.println(" cathether released");
      Serial.println(" guidewire clamped");

      Serial.println("stepper on top move back");
      stepper_on_top.moveRelative(step_rev);

      Serial.println(" guidewire released");

      Serial.println("continue the motion");
      move(dist_not_moved);

      break;
    }
    // Serial.println(stepper.getStepperPosition());
    stepper.run();
  }
  Serial.println("one motor finished");
  int total = millis() - start;
  Serial.print("time: ");
  Serial.println(total);
  long dist = (stepper.getStepperPosition() - temp) * stepper.dist_rev / step_rev;
  return dist;
}


long clamp_system::syncMove(long step_top, long step_bottom) {
  if(!have_top) {
    Serial.println("top");
    return move(step_top);
  }

  Serial.println("bottom");
  MultiStepper steppers;
  steppers.addStepper(stepper_on_top.stepper);
  steppers.addStepper(stepper.stepper);

  long positions[2];
  long temp_top = stepper_on_top.getStepperPosition();
  long temp_bottom = stepper.getStepperPosition();
  positions[0] = step_top + temp_top;
  positions[1] = step_bottom + temp_bottom;

  steppers.moveTo(positions);
  int start = millis();
  while(true) {
    bool limit_bottom = stepper.limit();
    bool limit_top = stepper_on_top.limit();
    if(limit_bottom) {
      stepper.awayFromLimit(temp_bottom);
      Serial.println("sync - bottom limit");
      break;
    }
    if(limit_top) {
      stepper_on_top.awayFromLimit(temp_top);
      Serial.println("sync - top limit");
      break;
    }
    if(!steppers.run()) break;
  }
  int total = millis() - start;
  Serial.print("time: ");
  Serial.print("\t");
  Serial.println(total);

  long dist = ((stepper_on_top.getStepperPosition() - temp_top) * 72 + (stepper.getStepperPosition() - temp_bottom) * 125) / step_rev;
  return dist;
}


