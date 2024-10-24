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

#define motor_clamp_top 10
#define motor_clamp_bottom 11

class myStepper {
  private:
    

  public:
    AccelStepper stepper;
    int limit_left;   // input pin of left limit switch
    int limit_right;  // input pin of right limit switch
    int dist_rev;
    
    myStepper();
    myStepper(AccelStepper s, int l, int r, int rev);
    void set(float max_speed, float accel);
    void setSpeed(float speed);
    int limit();                                         // return true if any limit switch is on
    bool awayFromLimit(long prev_position);
    long getStepperPosition();
    long getTarget();
    void run();
    void moveRelative(long position);
};

class clamp_system {
  private:
    bool have_top;
    myStepper stepper;
    myStepper stepper_on_top;
    int motor_clamp_pin;

  public:
    clamp_system(myStepper stepper, int motor_clamp_pin, bool have_top);
    clamp_system(myStepper stepper, myStepper stepper_on_top, int motor_clamp_pin, bool have_top);
    long move(long position);                             // move with speed_default
    long syncMove(long top, long bottom);
    void awayFromLimitSync(MultiStepper steppers, long position[]);
};

void constSpeed(AccelStepper* stepper, float speed, long position);
void withAccel(AccelStepper* stepper, long position);
void homing(myStepper top, myStepper bottom);  // move all stepper motors to the right end and set the position as 0 


extern float speed_set;
extern int step_rev;

#endif