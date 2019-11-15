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

void Spin(vex::motor motor, vex::directionType dt, int pct, int mt)
{
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void Stop(vex::motor motor, brakeType bt, int mt)
{
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}

void usercontrol(void)
{
  double collector_spd = 81;
  int smove_spd = 2800;
  double push_err = 0;
  bool push_flag = false;
  int c_1, c_3, btn_l1, btn_l2, btn_r1, btn_r2, btn_x,
      btn_a, btn_y, btn_b, btn_up, btn_down, btn_right,
      btn_left;

  push.resetRotation();

  // User control code here, inside the loop
  while (true)
  {
    // reading.
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

    // moving.
    if (btn_up)
    {
      mmove(smove_spd, smove_spd);
    }
    else if (btn_right)
    {
      mmove(-1.5 * smove_spd, 1.5 * smove_spd);
    }
    else if (btn_left)
    {
      mmove(1.5 * smove_spd, -1.5 * smove_spd);
    }
    else
    {
      move(c_3, c_1);
    }

    // hand
    if (btn_r1)
    {
      Spin(hand1, vex::directionType::fwd, collector_spd, 1.5);
      Spin(hand2, vex::directionType::rev, collector_spd, 1.5);

      // hand1.setMaxTorque(1.5, currentUnits::amp);
      // hand2.setMaxTorque(1.5, currentUnits::amp);
      // hand1.spin(vex::directionType::fwd, collector_spd, vex::velocityUnits::pct);
      // hand2.spin(vex::directionType::rev, collector_spd, vex::velocityUnits::pct);
    }
    else if (btn_r2)
    {
      Spin(hand1, vex::directionType::rev, collector_spd, 1.5);
      Spin(hand2, vex::directionType::fwd, collector_spd, 1.5);

      // hand1.setMaxTorque(1.5, currentUnits::amp);
      // hand2.setMaxTorque(1.5, currentUnits::amp);
      // hand1.spin(vex::directionType::rev, collector_spd, vex::velocityUnits::pct);
      // hand2.spin(vex::directionType::fwd, collector_spd, vex::velocityUnits::pct);
    }
    else
    {
      Stop(hand1, brakeType::hold, 0.1);
      Stop(hand2, brakeType::hold, 0.1);

      // hand1.setMaxTorque(0.1, currentUnits::amp);
      // hand2.setMaxTorque(0.1, currentUnits::amp);
      // hand1.stop(brakeType::hold);
      // hand2.stop(brakeType::hold);
    }

    // auto push
    Brain.Screen.printAt(10, 10, "push_err is %5f", push_err);
    push_err = abs(push.rotation(rotationUnits::deg));
    if (push_flag)
    {
      int fdbk = -push_err * 0.086 + 80;
      arm.stop(brakeType::coast);
      if (abs(fdbk) < 10)
      {
        hand1.stop(brakeType::coast);
        hand2.stop(brakeType::coast);
      }
      if (abs(fdbk) < 2)
      {
        fdbk = 0;
        push_flag = false;
      }
      push.setMaxTorque(2.4, currentUnits::amp);
      push.spin(vex::directionType::fwd, fdbk, vex::velocityUnits::pct);
    }

    // manual push
    if (btn_a)
    {
      push_flag = true;
    }
    else if (btn_x)
    {
      push_flag = false;
      push.setMaxTorque(2.4, currentUnits::amp);
      push.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
    }
    else if (btn_b)
    {
      push_flag = false;
      push.setMaxTorque(2.4, currentUnits::amp);
      push.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      push.resetRotation();
    }
    else
    {
      // arm
      if (btn_l1)
      {
        arm.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
        arm.setMaxTorque(2.4, currentUnits::amp);
        if (push_err < 340)
        {
          push.spin(directionType::fwd, 60, vex::velocityUnits::pct);
          push.setMaxTorque(2.4, currentUnits::amp);
        }
        else
        {
          push.stop(brakeType::hold);
          push.setMaxTorque(0.2, currentUnits::amp);
        }
      }
      else if (btn_l2)
      {
        arm.spin(directionType::rev, 80, vex::velocityUnits::pct);
        arm.setMaxTorque(2.4, currentUnits::amp);
        if (push_err > 0)
        {
          push.spin(directionType::rev, 45, vex::velocityUnits::pct);
          push.setMaxTorque(2.4, currentUnits::amp);
        }
        else
        {
          push.stop(brakeType::hold);
          push.setMaxTorque(0.2, currentUnits::amp);
        }
      }
      else
      {
        push.stop(coast);
        arm.stop(brakeType::hold);
        arm.setMaxTorque(2.4, currentUnits::amp);
      }
    }
  }
}

//
// Main will set up the competition functions and callbacks.
//
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