#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
#include "automove.h"
#include "vex.h"

#define c_1          Controller.Axis1.value()
#define c_2          Controller.Axis2.value()
#define c_3          Controller.Axis3.value()
#define c_4          Controller.Axis4.value()
#define btn_l1          Controller.ButtonL1.pressing()
#define btn_l2          Controller.ButtonL2.pressing()
#define btn_r1          Controller.ButtonR1.pressing()
#define btn_r2          Controller.ButtonR2.pressing()
#define btn_x          Controller.ButtonX.pressing()
#define btn_y          Controller.ButtonY.pressing()
#define btn_a          Controller.ButtonA.pressing()
#define btn_b          Controller.ButtonB.pressing()
#define btn_left        Controller.ButtonLeft.pressing()
#define btn_right       Controller.ButtonRight.pressing()
#define btn_up          Controller.ButtonUp.pressing()
#define btn_down        Controller.ButtonDown.pressing()

void Spin(vex::motor motor, vex::directionType dt, int pct, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void Stop(vex::motor motor, brakeType bt, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}
#endif