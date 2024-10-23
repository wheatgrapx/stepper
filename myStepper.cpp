#include "Arduino.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "myStepper.h"

int step_rev = 800;
float speed_set = 400;

myStepper::myStepper(AccelStepper s, int l, int r, float v, int rev) {
  this->stepper = s;
  this->limit_left = l;
  this->limit_right = r;
  this->speed_default = v;
  this->dist_rev = rev;
}

void myStepper::set(float max_speed, float accel) {
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(accel);
}

// move the stepper relative to current position
// will call awayFromLimit() when limit is reached
// positive is to the right
long myStepper::move(long position) {
  long temp = stepper.currentPosition();

  if(position == 0) return 0;
  
  stepper.move(position);
  int start = millis();
  if(position > 0) stepper.setSpeed(speed_default);
  else stepper.setSpeed(-speed_default);
  while(stepper.currentPosition() != stepper.targetPosition()) {
    if(limit()) {
      awayFromLimit(temp);
      break;
    }
    stepper.runSpeed();
  }
  stepper.stop();
  int total = millis() - start;
  Serial.print("time: ");
  Serial.println(total);
  long dist = (stepper.currentPosition() - temp) * dist_rev / step_rev;
  return dist;
}

void myStepper::setSpeed(float speed) {
  this->speed_default = speed;
}

// return true if limit is reached
bool myStepper::limit() {
  bool reached = !digitalRead(limit_left) || !digitalRead(limit_right);
  return reached;
}

// move the stepper away when limit is reached 
void myStepper::awayFromLimit(long prev_position) {
  long dist_to_prev = abs(stepper.currentPosition() - prev_position);
  long dist_predef = step_rev / 4;
  long dist_back = min(dist_to_prev, dist_predef);
  if(!digitalRead(limit_left)) {
    Serial.print(dist_back);
    Serial.println(" left_limit");
    constSpeed(&stepper, speed_default * 0.5, dist_back);
  }
  else if(!digitalRead(limit_right)) {
    Serial.print(dist_back);
    Serial.println(" right_limit");
    constSpeed(&stepper, speed_default * 0.5, -dist_back);
  }
  
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
  top.stepper.setSpeed(speed_homing);
  bottom.stepper.setSpeed(speed_homing);

  int start = millis();
  while(true) {
    if(top.limit() && bottom.limit()) break;  // exit the while loop when both limits are reached
    if(!top.limit()) {                        // stop top motor when limit is reached
      top.stepper.runSpeed();
      Serial.print("top");
    }
    if(!bottom.limit()) {                     // stop bottom motor when limit is reached
      bottom.stepper.runSpeed();
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

long syncMove(long step_top, long step_bottom, myStepper stepper_top, myStepper stepper_bottom) {
  MultiStepper steppers;
  steppers.addStepper(stepper_top.stepper);
  steppers.addStepper(stepper_bottom.stepper);

  long positions[2];
  long temp_top = stepper_top.stepper.currentPosition();
  long temp_bottom = stepper_bottom.stepper.currentPosition();
  positions[0] = step_top + stepper_top.stepper.currentPosition();
  positions[1] = step_bottom + stepper_bottom.stepper.currentPosition();

  steppers.moveTo(positions);
  int start = millis();
  while(true) {
    bool limit_bottom = !digitalRead(limit_bottom_left) || !digitalRead(limit_bottom_right);
    bool limit_top = !digitalRead(limit_top_left) || !digitalRead(limit_top_right);
    if(limit_bottom) {
      stepper_bottom.awayFromLimit(temp_bottom);
      Serial.println("sync - bottom limit");
      break;
    }
    if(limit_top) {
      stepper_top.awayFromLimit(temp_top);
      Serial.println("sync - top limit");
      break;
    }
    if(!steppers.run()) break;
  }
  int total = millis() - start;
  Serial.print("time: ");
  Serial.print("\t");
  Serial.println(total);

  long dist = ((stepper_top.stepper.currentPosition() - temp_top) * 72 + (stepper_bottom.stepper.currentPosition() - temp_bottom) * 125) / step_rev;
  return dist;
}


