/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#ifndef VEX_H
#define VEX_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
using namespace vex;

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

/*
vex::brain Brain;
vex::motor right_1(vex::PORT17,vex::gearSetting::ratio18_1,true);
vex::motor right_2(vex::PORT13,vex::gearSetting::ratio18_1,false);
vex::motor left_1(vex::PORT18,vex::gearSetting::ratio18_1,false);
vex::motor left_2(vex::PORT14,vex::gearSetting::ratio18_1,true);
vex::motor arm(vex::PORT20,vex::gearSetting::ratio36_1,false);
vex::motor hand1(vex::PORT2,vex::gearSetting::ratio36_1,false);
vex::motor hand2(vex::PORT10,vex::gearSetting::ratio36_1,false);
vex::motor push(vex::PORT16,vex::gearSetting::ratio36_1,true);
vex::bumper cubeplacerpush=vex::bumper(Brain.ThreeWirePort.A);
vex::bumper cubeplacerpull=vex::bumper(Brain.ThreeWirePort.C);
vex::bumper stopper_1=vex::bumper(Brain.ThreeWirePort.E);
vex::bumper stopper_2=vex::bumper(Brain.ThreeWirePort.F);
vex::controller controller1 =vex::controller();*/

// define your global instances of motors and other devices here
#endif