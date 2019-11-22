#ifndef BS_H
#define BS_H

#include "vex.h"
#include "automove.h"
#include "utils.h"
#include "ctrls.h"

int start_hand()
{
  vex::task::sleep(300);
  handsSpin(vex::directionType::fwd, 80, 1.6);

  return 0;
}

int start_arm_push()
{
  motorSpin(push, vex::directionType::rev, 60, 2.2);
  motorSpin(arm, vex::directionType::rev, 60, 2.2);
  vex::task::sleep(300);
  motorStop(arm, brakeType::hold, 0.3);
  vex::task::sleep(300);
  motorStop(push, brakeType::hold, 0.3);

  return 0;
}

void bs_new(){
  task ArmPushStart(start_arm_push);
  moveTarget(450, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); // tar, max_pct, fwd_tur, bt, kp, kd, ki
  task HandStart(start_hand);
  moveTarget(-400,100, true, vex::brakeType::brake, 0.3, 0.01, 0.3);                                                                                                                                                                                                                                               0, 100, 0.3, 0.01, 0.3, vex::brakeType::brake); // tar, max_pct, kp, kd, ki
}

///////////////////////////////////////////////////////////////////////////////////

void auto_bs()
{
  /*
  push.spin(directionType ::fwd,50,velocityUnits::pct);
  push.setMaxTorque(1.0, currentUnits:: amp);
  
  arm.spin (directionType::fwd,100,vex::velocityUnits::pct);
  chsSpin(3000,3000);
  vexDelay(700);
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  push.spin(directionType ::fwd,-50,velocityUnits::pct);
  arm.spin(directionType::rev,100,vex::velocityUnits::pct);
  chsSpin(-4000,-4000);
  vexDelay(750);
  push.stop();
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  arm.stop(brakeType::hold);

  arm.setMaxTorque(0.2, currentUnits::amp);hand1.spin(directionType::fwd,100,vex::velocityUnits::pct);
  */
  vexDelay(2000);
  arm.spin(directionType::rev, 5, vex::velocityUnits::pct);
  arm.setMaxTorque(0.4, currentUnits::amp);
  push.stop(brakeType::hold);
  push.setMaxTorque(0.2, currentUnits::amp);
  hand1.spin(directionType::fwd, 100, vex::velocityUnits::pct);
  hand2.spin(directionType::rev, 100, vex::velocityUnits::pct);
  //collect frist 3 cubes
  automove(830, 830, 6000, 4000);

  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);

  hand1.setMaxTorque(0.1, currentUnits::amp);
  hand2.setMaxTorque(0.1, currentUnits::amp);
  //stop hand
  automove(190, -190, 2000, 14000);
  autoturn(-865, -865, 3000, 14000);
  //go back
  automove(-180, 180, 2000, 14000);
  //automove(-200,-200,2000,10000);

  //start to collect another four cubes
  hand1.setMaxTorque(2.4, currentUnits::amp);
  hand2.setMaxTorque(2.4, currentUnits::amp);
  hand1.spin(directionType::fwd, 100, vex::velocityUnits::pct);
  hand2.spin(directionType::rev, 100, vex::velocityUnits::pct);
  automove(1200, 1200, 7000, 3500);
  vexDelay(70);
  //dalay for get cube at right position
  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);
  hand1.setMaxTorque(0.1, currentUnits::amp);
  hand2.setMaxTorque(0.1, currentUnits::amp);
  automove(-705, -705, 2000, 14000);
  automove(-420, 420, 1000, 9900);
  automove(400, 400, 4000, 6000);

  push.setMaxTorque(2.4, currentUnits::amp);
  push.spin(vex::directionType::fwd, 43, vex::velocityUnits::pct);
  vexDelay(1740);
  push.stop(brakeType::coast);
  hand1.spin(directionType::rev, 100, vex::velocityUnits::pct);
  hand2.spin(directionType::fwd, 100, vex::velocityUnits::pct);
  vexDelay(100); //prevent hand stucking cubes

  automove(-200, -200, 1000, 14000);
  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);
  hand1.setMaxTorque(0.1, currentUnits::amp);
  hand2.setMaxTorque(0.1, currentUnits::amp);
}
#endif