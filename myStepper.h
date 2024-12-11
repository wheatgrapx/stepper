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

#define limit_top_back 6
#define limit_top_front 7
#define limit_bottom_back 8
#define limit_bottom_front 9

#define dirPin_guidewire 10
#define stepPin_guidewire 11

#define dirPin_catheter 12
#define stepPin_catheter 13

#define motor_guidewire_A 14
#define motor_guidewire_B 15

#define motor_catheter_A 16
#define motor_catheter_B 17

#define dirPin_clamp_guidewire 18
#define stepPin_clamp_guidewire 19

#define dirPin_clamp_catheter 20
#define stepPin_clamp_catheter 21

class myStepper {
  private:
    
  public:
    AccelStepper stepper;
    int limit_left;   // input pin of left limit switch
    int limit_right;  // input pin of right limit switch
    int dist_rev;
    
    myStepper();
    myStepper(AccelStepper s);
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
    myStepper stepper_clamp;
    myStepper stepper_clamp_on_top;

  public:
    clamp_system(myStepper stepper, myStepper stepper_clamp, bool have_top);
    clamp_system(myStepper stepper, myStepper stepper_on_top, myStepper stepper_clamp, myStepper stepper_clamp_on_top, bool have_top);
    long move(long position);                             // move with speed_default
    long syncMove(long top, long bottom);
    bool move_stepper_clamp(bool status);
    void awayFromLimitSync(MultiStepper steppers, long position[]);
};

void constSpeed(AccelStepper* stepper, float speed, long position);
void withAccel(AccelStepper* stepper, long position);
void homing(myStepper top, myStepper bottom);  // move all stepper motors to the right end and set the position as 0 

void clamp(int A, int B);
void release(int A, int B);

extern float speed_set;
extern int step_rev;


#endif