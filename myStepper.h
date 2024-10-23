#ifndef myStepper_h
#define myStepper_h

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Arduino.h"
#include <stdlib.h>

#define dirPin_top 2
#define stepPin_top 3
#define dirPin_bottom 4
#define stepPin_bottom 5

#define limit_top_left 6
#define limit_top_right 7
#define limit_bottom_left 8
#define limit_bottom_right 9

class myStepper {
  private:
    int limit_left;   // input pin of left limit switch
    int limit_right;  // input pin of right limit switch
    float speed_default;
    int dist_rev;
  public:
    AccelStepper stepper;
    
    myStepper(AccelStepper s, int l, int r, float v, int rev);
    void set(float max_speed, float accel);
    long move(long position);                             // move with speed_default
    void setSpeed(float speed);
    bool limit();                                         // return true if any limit switch is on
    void awayFromLimit(long prev_position);
};

void constSpeed(AccelStepper* stepper, float speed, long position);
void multi(MultiStepper steppers, long positions[2], myStepper top, myStepper bottom);
void withAccel(AccelStepper* stepper, long position);
void homing(myStepper top, myStepper bottom);  // move all stepper motors to the right end and set the position as 0 
long syncMove(MultiStepper steppers, long top, long bottom, myStepper stepper_top, myStepper stepper_bottom);

extern float speed_set;
extern int step_rev;

#endif