/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       john                                                      */
/*    Created:      Sat Aug 31 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "auto_bs.h"
#include "auto_rs.h"

#include "automove.h"
#include "utils.h"

#include "math.h"
#include "vex.h"

using namespace vex;
vex::competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous() { test(); }

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int smove_vot = 2800;
double push_err = 0, push_vlc = 0, sum_err = 0, output = 0;
bool push_flag = false, push_hold = false;

/*----------------------------------------------------------------------------*/
/*   Chassis Control
/*----------------------------------------------------------------------------*/
void moving() {
  // --- moving ---
  if (btn_up)
    mmove(smove_vot, smove_vot);
  else if (btn_right)
    mmove(1.5 * smove_vot, -1.5 * smove_vot);
  else if (btn_left)
    mmove(-1.5 * smove_vot, 1.5 * smove_vot);
  else
    move(c_3, c_1);
}

/*----------------------------------------------------------------------------*/
/*   Collector Control
/*----------------------------------------------------------------------------*/
void handing() {
  // --- hand ---
  if (btn_r1) {
    push_hold = false;
    Spin(hand1, vex::directionType::fwd, 80, 1.6);
    Spin(hand2, vex::directionType::rev, 80, 1.6);
  } else if (btn_r2) {
    push_hold = false;
    Spin(hand1, vex::directionType::rev, 80, 1.6);
    Spin(hand2, vex::directionType::fwd, 80, 1.6);
  } else if (!push_hold) {
    Stop(hand1, brakeType::hold, 0.1);
    Stop(hand2, brakeType::hold, 0.1);
  }
}

/*----------------------------------------------------------------------------*/
/*   Auto Push Task
/*----------------------------------------------------------------------------*/
int auto_pushing() {
  while (push_flag) {
    push_err = 800 - fabs(push.rotation(rotationUnits::deg));
    push_vlc = fabs(push.velocity(vex::velocityUnits::pct));

    // pushing multi-layer control
    if (push_err < 10) // break
    {
      push_flag = false;
      push_hold = false;
    } 
    else if (push_err < 100) // PID control
    {
      sum_err += push_err * 0.1;
      output = 17 + push_err * 0.1 - push_vlc * 0.4 + sum_err * 0.08;
    }
    else if (push_err < 180) // PID control
    {
      output = 20 + push_err * 0.065 - push_vlc * 0.3;
    } 
    else if (push_err < 270) // 60 - 40 inertance reducing
    {
      output = 30 + push_err * 0.065 - push_vlc * 0.2;
    } 
    else // 100 fast push
    {
      output = 100;
    }
    Brain.Screen.printAt(10, 10, "output is %.2f", output);
    Spin(push, vex::directionType::fwd, output, 2.2);

    // change to coast
    if (push_err < 135) {
      Stop(hand1, brakeType::coast, 0.1);
      Stop(hand2, brakeType::coast, 0.1);
    }
    if (push_err < 520) {
      Stop(arm, brakeType::coast, 0.1);
    }
    // sampling period
    vex::task::sleep(100);
  }
  push_hold = true;
  Spin(hand1, vex::directionType::rev, 80, 1.6);
  Spin(hand2, vex::directionType::fwd, 80, 1.6);
  vex::task::sleep(100);
  Stop(hand1,brakeType::coast,0.1);
  Stop(hand2,brakeType::coast,0.1);
  push_hold = false;
  return 0;
}

/*----------------------------------------------------------------------------*/
/*   Push Control
/*----------------------------------------------------------------------------*/
void pushing() {
  // --- manual push ---
  if (btn_a) {
    // auto-push starts
    push_flag = true;
    push_hold = true;
    task AutoPush(auto_pushing);
  } else if (btn_x) {
    push_flag = false;
    push_hold = false;

    if (push_err > 700)
      Spin(push, vex::directionType::fwd, 50, 2.2);
    else if (push_err > 500)
      Spin(push, vex::directionType::fwd, 80, 2.2);
    else
      Spin(push, vex::directionType::fwd, 100, 2.2);
  } else if (btn_b) {
    push_flag = false;
    push_hold = false;

    Spin(push, vex::directionType::rev, 100, 2.2);
    push.resetRotation();
  } else if (!push_flag && !btn_l1 && !btn_l2) {
    Stop(push, brakeType::hold, 0.1);
  }
}

/*----------------------------------------------------------------------------*/
/*   Arm Control
/*----------------------------------------------------------------------------*/
void rasing() {
  if (btn_l1) {
    push_hold = false;
    Spin(arm, vex::directionType::fwd, 80, 2.2);

    if (push_err < 340) // TODO: wating for testing.
      Spin(push, vex::directionType::fwd, 60, 2.4);
  } else if (btn_l2) {
    push_hold = false;
    Spin(arm, directionType::rev, 80, 2.2);
  } else if (!push_hold) {
    Stop(arm, brakeType::hold, 0.1);
  }
}

/*----------------------------------------------------------------------------*/
/*   User control code here, inside the loop
/*----------------------------------------------------------------------------*/
void usercontrol(void) {
  push.resetRotation();

  while (true) {
    moving();
    handing();
    pushing();
    rasing();
  }
}

/*----------------------------------------------------------------------------*/
/*   Main will set up the competition utils and callbacks
/*----------------------------------------------------------------------------*/
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (1) {
    vex::task::sleep(100); // Sleep the task for a short amount of time to
                           // prevent wasted resources.
  }
}