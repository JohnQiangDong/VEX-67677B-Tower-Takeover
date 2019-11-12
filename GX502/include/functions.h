#include "automove.h"
#include "vex.h"
// positive to collect cubes
void hand_move(int velocity) {
  hand1.spin(vex::directionType::fwd, velocity, velocityUnits::pct);
  hand2.spin(vex::directionType::rev, velocity, velocityUnits::pct);
}
// positive to raise
void arm_move(int velocity) {
  arm.spin(vex::directionType::fwd, velocity, velocityUnits::pct);
}
// positive to push cubes
void push_move(int velocity) {
  push.spin(vex::directionType::fwd,velocity,velocityUnits::pct);
}
// true for hold
// false for coast
void hand_stop(bool stoptype) {
  if (stoptype == false) {
    hand1.stop(brakeType::coast);
    hand2.stop(brakeType::coast);
  } else if (stoptype == true) {
    hand1.stop(brakeType::hold);
    hand2.stop(brakeType::hold);
  }
}
void arm_stop(bool stoptype){
  
}