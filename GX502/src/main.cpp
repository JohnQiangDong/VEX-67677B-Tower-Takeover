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

void autonomous() {
  
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

void usercontrol(void) {
  // User control code here, inside the loop
  double handmove = 81;
  double dangerous = 0;
  bool push_flag = false;
  int sml_move = 2800;
  // double angle = 0;
  // int PPush = 0;
  push.resetRotation();

  int c_1, c_3, btn_l1, btn_l2, btn_r1, btn_r2, btn_x, 
  btn_a, btn_y, btn_b, btn_up, btn_down, btn_right,
  btn_left;

  while (true) {
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
    Brain.Screen.printAt(10,10,"dangerous is %5f",dangerous);
    if(btn_up){
      mmove(sml_move,sml_move);
    }
    else if(btn_right){
      mmove(-1.5*sml_move,1.5*sml_move);
    }
    else if(btn_left){
      mmove(1.5*sml_move,-1.5*sml_move);
    }
    else{
      move(c_3, c_1);
    }
    if (btn_r1) {
      hand1.setMaxTorque(1.5, currentUnits::amp);
      hand2.setMaxTorque(1.5, currentUnits::amp);
      hand1.spin(vex::directionType::fwd, handmove, vex::velocityUnits::pct);
      hand2.spin(vex::directionType::rev, handmove, vex::velocityUnits::pct);
    } else if (controller1.ButtonR2.pressing()) {
      hand1.setMaxTorque(1.5, currentUnits::amp);
      hand2.setMaxTorque(1.5, currentUnits::amp);
      hand1.spin(vex::directionType::rev, handmove, vex::velocityUnits::pct);
      hand2.spin(vex::directionType::fwd, handmove, vex::velocityUnits::pct);
    } else {
      hand1.setMaxTorque(0.1, currentUnits::amp);
      hand2.setMaxTorque(0.1, currentUnits::amp);
      hand1.stop(brakeType::hold);
      hand2.stop(brakeType::hold);
    }

    dangerous = abs(push.rotation(rotationUnits::deg));
    if (push_flag) {
      int fdbk = -dangerous * 0.086 + 80;
      arm.stop(brakeType::coast);
      if(abs(fdbk) < 10){
        hand1.stop(brakeType::coast);
        hand2.stop(brakeType::coast);
      }
      if(abs(fdbk) < 2){
        fdbk = 0;
        push_flag = false;
      }
      push.setMaxTorque(2.4, currentUnits::amp);
      push.spin(vex::directionType::fwd, fdbk, vex::velocityUnits::pct);
    }

    if (controller1.ButtonA.pressing()) {
      push_flag = true;
    } else if(controller1.ButtonX.pressing()){
      push_flag = false;
      push.setMaxTorque(2.4, currentUnits::amp);
      push.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
    } else if (controller1.ButtonB.pressing()) {
      push_flag = false;
      push.setMaxTorque(2.4, currentUnits::amp);
      push.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      push.resetRotation();
    } else {
      if (controller1.ButtonL1.pressing()) {
        arm.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
        arm.setMaxTorque(2.4, currentUnits::amp);
        if (dangerous < 340) {
          push.spin(directionType::fwd, 60, vex::velocityUnits::pct);
          push.setMaxTorque(2.4, currentUnits::amp);
        } else {
          push.stop(brakeType::hold);
          push.setMaxTorque(0.2, currentUnits::amp);
        }
      } else if (controller1.ButtonL2.pressing()) {

        arm.spin(directionType::rev, 80, vex::velocityUnits::pct);
        arm.setMaxTorque(2.4, currentUnits::amp);
        if (dangerous > 0) {
          push.spin(directionType::rev, 45, vex::velocityUnits::pct);
          push.setMaxTorque(2.4, currentUnits::amp);
        } else {
          push.stop(brakeType::hold);
          push.setMaxTorque(0.2, currentUnits::amp);
        }
      } else {
        push.stop(coast);
        
        arm.stop(brakeType::hold);
        arm.setMaxTorque(2.4, currentUnits::amp);
      }
      // if (controller1.ButtonL1.pressing()){
      // arm.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
      // if (dangerous>-333){
      // push.spin(vex::directionType::fwd,66,vex::velocityUnits::pct);}
      //}
      //        else if (controller1.ButtonL2.pressing()){
      //        arm.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
      //    }
      //  else{arm.stop(hold);}
    }
  }
  // This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor + servo
  // values based on feedback from the joysticks.

  // ........................................................................
  // Insert user code here. This is where you use the joystick values to
  // update your motors, etc.
  // .......................................................................
}

//
// Main will set up the competition functions and callbacks.
//
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