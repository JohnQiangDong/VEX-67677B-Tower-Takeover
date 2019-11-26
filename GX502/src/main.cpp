/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       67677B                                                    */
/*    Created:      Sat Aug 31 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "auto_bs.h"
#include "auto_rs.h"
#include "auto_bd.h"

#include "ctrls.h"

#include "math.h"
#include "vex.h"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
int choose = 0;
void pre_auton(void) 
{
  /*double t;
  while(true){
    if (bumper_choose){
     t = Brain.timer(vex::timeUnits::msec);
     choose += 1;
       if(choose == 5) choose = 1;
    }
    switch(choose)
    {
      case 1:Brain.Screen.printAt(  10, 220, "BS");
      case 2:Brain.Screen.printAt(  10, 220, "RS");
      case 3:Brain.Screen.printAt(  10, 220, "RB");
      case 4:Brain.Screen.printAt(  10, 220, "BB");
    }
    if(fabs(t - Brain.timer(vex::timeUnits::msec)) < 500){
       if(bumper_choose)break;
    }
  }*/
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous() 
{
   /* switch(choose)
    {
      case 1:Brain.Screen.printAt(  10, 220, "BS");
      case 2:Brain.Screen.printAt(  10, 220, "RS");
      case 3:Brain.Screen.printAt(  10, 220, "RB");
      case 4:Brain.Screen.printAt(  10, 220, "BB");
    }*/
    auto_bd();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) 
{
  //secret();
  // test();
  //auto_bd();
  chsStops(brakeType::coast,2.2);
  while (true) 
  {
    moving();
    handing();
    pushing();
    rasing();
  }
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*   Main will set up the competition utils and callbacks                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

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