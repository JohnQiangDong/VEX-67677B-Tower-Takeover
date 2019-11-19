/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       john                                                      */
/*    Created:      Sat Aug 31 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "bs.h"
#include "rs.h"

#include "automove.h"
#include "move.h"
#include "functions.h"
#include "status.h"

#include "math.h"
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen

// A global instance of vex::competition
vex::competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
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

void autonomous()
{

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double collector_pct = 81;
int smove_vot = 2800;
double push_err = 0, push_vlc = 0, sum_err = 0, output = 0;
bool push_flag = false, push_hold = false;
int c_1, c_3, btn_l1, btn_l2, btn_r1, btn_r2,
    btn_x, btn_a, btn_y, btn_b,
    btn_up, btn_down, btn_right, btn_left;

void reading()
{
  // --- reading ---
  c_1 = controller1.Axis1.value();
  c_3 = controller1.Axis3.value();
  btn_l1 = controller1.ButtonL1.pressing();
  btn_l2 = controller1.ButtonL2.pressing();
  btn_r1 = controller1.ButtonR1.pressing();
  btn_r2 = controller1.ButtonR2.pressing();
  btn_x = controller1.ButtonX.pressing();
  btn_a = controller1.ButtonA.pressing();
  btn_y = controller1.ButtonY.pressing();
  btn_b = controller1.ButtonB.pressing();
  btn_up = controller1.ButtonUp.pressing();
  btn_down = controller1.ButtonDown.pressing();
  btn_right = controller1.ButtonRight.pressing();
  btn_left = controller1.ButtonLeft.pressing();
}

void moving()
{
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

void handing()
{
  // --- hand ---
  if (btn_r1)
  {
    push_hold = false;
    Spin(hand1, vex::directionType::fwd, collector_pct, 1.5);
    Spin(hand2, vex::directionType::rev, collector_pct, 1.5);
  }
  else if (btn_r2)
  {
    push_hold = false;
    Spin(hand1, vex::directionType::rev, collector_pct, 1.5);
    Spin(hand2, vex::directionType::fwd, collector_pct, 1.5);
  }
  else if (!push_hold)
  {
    Stop(hand1, brakeType::hold, 0.1);
    Stop(hand2, brakeType::hold, 0.1);
  }
}

void pushing()
{
  // --- manual push ---
  if (btn_a)
  {
    // auto-push starts
    push_flag = true;
    push_hold = true;
  }
  else if (btn_x)
  {
    push_flag = false;
    push_hold = false;

    if (push_err > 700)
      Spin(push, vex::directionType::fwd, 50, 2.2);
    else if (push_err > 500)
      Spin(push, vex::directionType::fwd, 80, 2.2);
    else
      Spin(push, vex::directionType::fwd, 100, 2.2);
  }
  else if (btn_b)
  {
    push_flag = false;
    push_hold = false;

    Spin(push, vex::directionType::rev, 100, 2.2);
    push.resetRotation();
  }
  else if (!push_flag && !btn_l1 && !btn_l2)
  {
    Stop(push, brakeType::hold, 0.1);
  }
}

void rasing()
{
  // --- arm ---
  if (btn_l1)
  {
    push_hold = false;
    Spin(arm, vex::directionType::fwd, 80, 2.2);

    if (push_err < 340) // TODO: wating for testing.
      Spin(push, vex::directionType::fwd, 60, 2.4);
  }
  else if (btn_l2)
  {
    push_hold = false;
    Spin(arm, directionType::rev, 80, 2.2);
  }
  else if (!push_hold)
  {
    Stop(arm, brakeType::hold, 0.1);
  }
}

int auto_pushing()
{
  while (true)
  {
    // --- auto push ---
    if (push_flag)
    {
      // *********************** version 2 *********************** //
      push_err = 750 - fabs(push.rotation(rotationUnits::deg));
      push_vlc = fabs(push.velocity(vex::velocityUnits::pct)); // TODO: PID
      // output feedback
      if (push_err < 10) // break
      {
        push_flag = false;
        push_hold = false;
      }
      else if (push_err < 50) // PID control
      {
        sum_err += push_err * 0.1;
        output = 20 + push_err * 0.1 - push_vlc * 0.3 + sum_err * 0.03;
      }
      else if (push_err < 150) // 30 - 16 inertance off
      {
        output = 10 + push_err * 0.13;
        sum_err = 0;
      }
      else
      {
        output = 35 + push_err * 0.065; // 85 - 45 fast push
      }
      Brain.Screen.printAt(10, 10, "output is %.2f", output);
      Spin(push, vex::directionType::fwd, output, 2.2);
      // change to coast
      if (push_err < 135)
      {
        Stop(hand1, brakeType::coast, 0.1);
        Stop(hand2, brakeType::coast, 0.1);
      }
      if (push_err < 520)
      {
        Stop(arm, brakeType::coast, 0.1);
      }
      // *********************** end 2 *********************** //

      // // *********************** version 1 *********************** //
      // push_err = fabs(push.rotation(rotationUnits::deg));
      // output = 35 + (-push_err * 0.065 + 50);
      // Brain.Screen.printAt(10, 10, "output is %.2f", output);

      // if (output < 37)
      // {
      //   push_flag = false;
      //   push_hold = false;
      // }
      // else if (output < 39)
      // {
      //   Spin(push, vex::directionType::fwd, 10, 2.2);
      // }
      // else if (output < 41)
      // {
      //   Spin(push, vex::directionType::fwd, 20, 2.2);
      // }
      // else if (output < 45)
      // {
      //   Stop(hand1, brakeType::coast, 0.1);
      //   Stop(hand2, brakeType::coast, 0.1);
      //   Spin(push, vex::directionType::fwd, 30, 2.2);
      // }
      // else if (output < 70)
      // {
      //   Stop(arm, brakeType::coast, 0.1);
      //   Spin(push, vex::directionType::fwd, output, 2.2);
      // }
      // else
      //   Spin(push, vex::directionType::fwd, output, 2.2);
      // // *********************** end 1 *********************** //
    }
    vex::task::sleep(100);
  }
  return 0;
}

void usercontrol(void)
{
  push.resetRotation();
  // User control code here, inside the loop
  while (true)
  {
    reading();
    moving();
    handing();
    pushing();
    rasing();
    task AutoPush(auto_pushing);
  }
}

// Main will set up the competition functions and callbacks.
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (1)
  {
    vex::task::sleep(100); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}