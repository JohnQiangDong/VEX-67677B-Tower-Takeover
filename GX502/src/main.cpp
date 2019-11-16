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



void usercontrol(void)
{
  double collector_spd = 81;
  int smove_spd = 2800;
  double push_err = 0;
  double fdbk = 0;
  bool push_flag = false;
  bool push_hold = false;
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
      mmove(1.5 * smove_spd, -1.5 * smove_spd);
    }
    else if (btn_left)
    {
      mmove(-1.5 * smove_spd, 1.5 * smove_spd);
    }
    else
    {
      move(c_3, c_1);
    }

    // hand
    if (btn_r1)
    {
      push_hold = false;
      Spin(hand1, vex::directionType::fwd, collector_spd, 1.5);
      Spin(hand2, vex::directionType::rev, collector_spd, 1.5);
    }
    else if (btn_r2)
    {
      push_hold = false;
      Spin(hand1, vex::directionType::rev, collector_spd, 1.5);
      Spin(hand2, vex::directionType::fwd, collector_spd, 1.5);
    }
    else
    {
      if(!push_hold)
      {
        Stop(hand1, brakeType::hold, 0.1);
        Stop(hand2, brakeType::hold, 0.1);
      }
    }

    // auto push
    if (push_flag)
    {
      push_err = fabs(push.rotation(rotationUnits::deg));
      push_hold = true;
      fdbk = 35 + (-push_err * 0.065 + 50);
      Brain.Screen.printAt(10, 10, "fdbk is %.2f", fdbk);

      if(fabs(fdbk) < 37) {
        push_flag = false;
        push_hold = false;
      }
      else if (fabs(fdbk) < 39) {
        Spin(push,vex::directionType::fwd,10,2.2);
      }
      else if (fabs(fdbk) < 41) {
        Spin(push,vex::directionType::fwd,20,2.2);
      }
      else if (fabs(fdbk) < 45) {
        Stop(hand1, brakeType::coast, 0.1);
        Stop(hand2, brakeType::coast, 0.1);
        Spin(push,vex::directionType::fwd,30,2.2);
      }
      else if (fabs(fdbk) < 70) {
        Stop(arm, brakeType::coast, 0.1);
        Spin(push,vex::directionType::fwd,fdbk,2.2);
      }
      else
        Spin(push,vex::directionType::fwd,fdbk,2.2);
    }

    // manual push
    if (btn_a)
    {
      push_flag = true;
    }
    else if (btn_x)
    {
      push_flag = false;
      push_hold = false;      
      if (push_err < 150 )
        Spin(push,vex::directionType::fwd,50,2.2);
      else if (push_err < 350 )
        Spin(push,vex::directionType::fwd,80,2.2);
      else
        Spin(push,vex::directionType::fwd,100,2.2);
    }
    else if (btn_b)
    {
      push_flag = false;
      push_hold = false;
      Spin(push, vex::directionType::rev, 100, 2.2);
      push.resetRotation();
    }
    else
    {
      if(!push_flag) Stop(push, brakeType::hold, 0.1);
      // arm
      if (btn_l1)
      {
        push_hold = false;
        Spin(arm, vex::directionType::fwd,80,2.2);

        // if (push_err < 340)
        // {
        //   Spin(push, vex::directionType::fwd,60,2.4);
        //   //push.spin(directionType::fwd, 60, vex::velocityUnits::pct);
        //   //push.setMaxTorque(2.4, currentUnits::amp);
        // }
        // else
        // {
        //   Stop(push,brakeType::hold,0.2);
        //   //push.stop(brakeType::hold);
        //   //push.setMaxTorque(0.2, currentUnits::amp);
        // }
      }
      else if (btn_l2)
      {
        push_hold = false;
        Spin(arm,directionType::rev,80,2.2);

        // if (push_err > 0)
        // {
        //   Spin(push,directionType::rev,45,2.2);
        //   //push.spin(directionType::rev, 45, vex::velocityUnits::pct);
        //   //push.setMaxTorque(2.4, currentUnits::amp);
        // }
        // else
        // {
        //   Stop(push, brakeType::hold,0.1);
        //   //push.stop(brakeType::hold);
        //   //push.setMaxTorque(0.2, currentUnits::amp);
        // }
      }
      else
      {
        if(!push_hold) Stop(arm, brakeType::hold,0.1);
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