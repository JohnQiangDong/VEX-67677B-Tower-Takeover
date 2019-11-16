#include "automove.h"
#include "vex.h"

void Spin(vex::motor motor, vex::directionType dt, int pct, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void Stop(vex::motor motor, brakeType bt, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}