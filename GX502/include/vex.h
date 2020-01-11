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
/*
vex::brain Brain;
vex::competition Competition;

vex::motor right_1(vex::PORT14, vex::gearSetting::ratio18_1, false);
vex::motor right_2(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor left_1(vex::PORT13, vex::gearSetting::ratio18_1, true);
vex::motor left_2(vex::PORT11, vex::gearSetting::ratio18_1, false);
vex::motor arm(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor hand1(vex::PORT17, vex::gearSetting::ratio36_1, true);
vex::motor hand2(vex::PORT5, vex::gearSetting::ratio36_1, false);
vex::motor push(vex::PORT2, vex::gearSetting::ratio36_1, true);
vex::bumper auto_choose(vex::bumper(Brain.ThreeWirePort.C));
vex::gyro gyro_1 = vex::gyro(Brain.ThreeWirePort.B);
vex::controller ctrler = vex::controller();*/

/*----------------------------------------------------------------------------*/
/*   Motors and Controller Definition (new robot)
/*----------------------------------------------------------------------------*/

vex::brain Brain;
vex::competition Competition;

vex::motor right_1(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::motor right_2(vex::PORT13, vex::gearSetting::ratio18_1, true);
vex::motor left_1(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor left_2(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::motor arm(vex::PORT15, vex::gearSetting::ratio36_1, false);
vex::motor hand1(vex::PORT12, vex::gearSetting::ratio36_1, false);
vex::motor hand2(vex::PORT7, vex::gearSetting::ratio36_1, true);
vex::motor push(vex::PORT9, vex::gearSetting::ratio36_1, true);

vex::bumper armbumper = vex::bumper(Brain.ThreeWirePort.F);
vex::limit pushlimit = vex::limit(Brain.ThreeWirePort.H);
vex::bumper stopper_1 = vex::bumper(Brain.ThreeWirePort.E);
vex::bumper stopper_2 = vex::bumper(Brain.ThreeWirePort.F);
vex::bumper auto_choose(vex::bumper(Brain.ThreeWirePort.A));
vex::gyro gyro_1 = vex::gyro(Brain.ThreeWirePort.A);
vex::controller ctrler = vex::controller();

/*----------------------------------------------------------------------------*/
/*   Controller Definition
/*----------------------------------------------------------------------------*/

#define c_tur ctrler.Axis1.value()
#define c_2 ctrler.Axis2.value()
#define c_fwd ctrler.Axis3.value()
#define c_4 ctrler.Axis4.value()

#define btn_arm_up ctrler.ButtonL1.pressing()
#define btn_arm_dw ctrler.ButtonL2.pressing()
#define btn_hand_in ctrler.ButtonR1.pressing()
#define btn_hand_ot ctrler.ButtonR2.pressing()

#define btn_score_push ctrler.ButtonX.pressing()
#define btn_y ctrler.ButtonY.pressing()
#define btn_score_auto ctrler.ButtonA.pressing()
#define btn_score_pull ctrler.ButtonB.pressing()

#define btn_left ctrler.ButtonLeft.pressing()
#define btn_right ctrler.ButtonRight.pressing()
#define btn_fwd ctrler.ButtonUp.pressing()
#define btn_bck ctrler.ButtonDown.pressing()

#define gyro_deg gyro_1.heading(vex::rotationUnits::deg)

#define bumper_choose auto_choose.pressing()
#define armstop armbumper.pressing()
#define pushstop pushlimit.pressing()

#endif