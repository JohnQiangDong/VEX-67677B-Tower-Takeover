/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef VEX_H
#define VEX_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

/*----------------------------------------------------------------------------*/
/*   Motors and Controller Definition
/*----------------------------------------------------------------------------*/

vex::brain Brain;
vex::motor right_1(vex::PORT17, vex::gearSetting::ratio18_1, false);
vex::motor right_2(vex::PORT16, vex::gearSetting::ratio18_1, true);
vex::motor left_1(vex::PORT15, vex::gearSetting::ratio18_1, true);
vex::motor left_2(vex::PORT13, vex::gearSetting::ratio18_1, false);
vex::motor arm(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor hand1(vex::PORT19, vex::gearSetting::ratio36_1, true);
vex::motor hand2(vex::PORT3, vex::gearSetting::ratio36_1, true);
vex::motor push(vex::PORT1, vex::gearSetting::ratio36_1, true);
vex::bumper cubeplacerpush = vex::bumper(Brain.ThreeWirePort.A);
vex::bumper cubeplacerpull = vex::bumper(Brain.ThreeWirePort.C);
vex::bumper stopper_1 = vex::bumper(Brain.ThreeWirePort.E);
vex::bumper stopper_2 = vex::bumper(Brain.ThreeWirePort.F);
vex::controller controller1 = vex::controller();

/*----------------------------------------------------------------------------*/
/*   Controller Definition
/*----------------------------------------------------------------------------*/

#define c_1          controller1.Axis1.value()
#define c_2          controller1.Axis2.value()
#define c_3          controller1.Axis3.value()
#define c_4          controller1.Axis4.value()
#define btn_l1          controller1.ButtonL1.pressing()
#define btn_l2          controller1.ButtonL2.pressing()
#define btn_r1          controller1.ButtonR1.pressing()
#define btn_r2          controller1.ButtonR2.pressing()
#define btn_x          controller1.ButtonX.pressing()
#define btn_y          controller1.ButtonY.pressing()
#define btn_a          controller1.ButtonA.pressing()
#define btn_b          controller1.ButtonB.pressing()
#define btn_left        controller1.ButtonLeft.pressing()
#define btn_right       controller1.ButtonRight.pressing()
#define btn_up          controller1.ButtonUp.pressing()
#define btn_down        controller1.ButtonDown.pressing()

#endif