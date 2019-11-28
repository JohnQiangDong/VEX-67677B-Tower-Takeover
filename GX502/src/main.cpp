/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       67677B                                                    */
/*    Created:      Sat Aug 31 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "auto_bd.h"
#include "auto_bs.h"
#include "auto_rd.h"
#include "auto_rs.h"

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

int choose = 0, bk_count = 10;
bool bk_flag = true;

void pre_auton(void)
{
/*  while(true)
  {
    if(bumper_choose) // 按下
    {
      bk_count = 10;
      while(bumper_choose) // 按着未松开
      {
        if(--bk_count <= 0) 
        {
          // 按下超过 1 秒
          Brain.Screen.printAt(10, 220, "Break             ");
          return; 
        }
        task::sleep(100);
      }
      
      // 松开按钮
      choose = choose % 4 + 1;
      switch (choose)
      {
      case 1:
        Brain.Screen.printAt(10, 200, "Blue-Single       ");
        break;
      case 2:
        Brain.Screen.printAt(10, 200, "Red-Single        ");
        break;
      case 3:
        Brain.Screen.printAt(10, 200, "Red-Double        ");
        break;
      case 4:
        Brain.Screen.printAt(10, 200, "Blue-Double       ");
        break;
      }
    }
    task::sleep(50);
  }*/


  while (bk_flag)
  {
    bk_count = 10;
    bk_flag = true;

    if (bumper_choose)
    {
      choose = choose % 4 + 1;
      switch (choose)
      {
      case 1:
        Brain.Screen.printAt(10, 200, "Blue-Single       ");
        break;
      case 2:
        Brain.Screen.printAt(10, 200, "Red-Single        ");
        break;
      case 3:
        Brain.Screen.printAt(10, 200, "Red-Double        ");
        break;
      case 4:
        Brain.Screen.printAt(10, 200, "Blue-Double       ");
        break;
      }
    }

    while (bumper_choose)
    {
      if (bk_count-- <= 0)
      {
        switch (--choose)
        {
        case 1:
          Brain.Screen.printAt(10, 200, "Blue-Single     ");
          break;
        case 2:
          Brain.Screen.printAt(10, 200, "Red-Single      ");
          break;
        case 3:
          Brain.Screen.printAt(10, 200, "Red-Double      ");
          break;
        case 4:
          Brain.Screen.printAt(10, 200, "Blue-Double     ");
          break;
        }
        Brain.Screen.printAt(10, 220, "Break             ");
        bk_flag = false;
        break;
      }
      task::sleep(100);
    }
    task::sleep(100);
  }

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
  switch (choose)
  {
  case 1:
    bs_six();
    break;
  case 2:
    rs_six();
    break;
  case 3:
    auto_rd();
    break;
  case 4:
    auto_bd();
    break;
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
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
    vex::task::sleep(100); // prevent wasted resources.
  }
}