#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
#include "automove.h"
#include "vex.h"

void MotorSpin(vex::motor motor, vex::directionType dt, int pct, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void MotorStop(vex::motor motor, brakeType bt, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}