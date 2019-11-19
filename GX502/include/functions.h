#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
#include "automove.h"
#include "vex.h"

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

void Spin(vex::motor motor, vex::directionType dt, int pct, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void Stop(vex::motor motor, brakeType bt, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}
#endif