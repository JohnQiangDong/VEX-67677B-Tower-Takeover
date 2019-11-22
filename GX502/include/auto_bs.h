#ifndef BS_H
#define BS_H

#include "vex.h"
#include "automove.h"
#include "utils.h"

void auto_bs()
{
  /*
  push.spin(directionType ::fwd,50,velocityUnits::pct);
  push.setMaxTorque(1.0, currentUnits:: amp);
  
  arm.spin (directionType::fwd,100,vex::velocityUnits::pct);
  mmove(3000,3000);
  vexDelay(700);
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  push.spin(directionType ::fwd,-50,velocityUnits::pct);
  arm.spin(directionType::rev,100,vex::velocityUnits::pct);
  mmove(-4000,-4000);
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

void test()
{
  moveTarget(700, 50, 0.15, 0.01, 0.3, vex::brakeType::brake); // tar, max_pct, kp, kd, ki
  // turnTarget(500, 80, 0.1, 0.01, 0.3, vex::brakeType::brake); // tar, max_pct, kp, kd, ki
}

int hand_start()
{
  vex::task::sleep(300);
  Spin(hand1, vex::directionType::fwd, 80, 1.6);
  Spin(hand2, vex::directionType::rev, 80, 1.6);

  return 0;
}

int start_arm_push()
{
  Spin(push, vex::directionType::rev, 60, 2.2);
  Spin(arm, vex::directionType::rev, 60, 2.2);
  vex::task::sleep(300);
  Stop(arm, brakeType::hold, 0.3);
  vex::task::sleep(300);
  Stop(push, brakeType::hold, 0.3);

  return 0;
}

void n_bs(){
  task ArmPushStart(start_arm_push);
  moveTarget(450, 100, 0.3, 0.01,0.3, vex::brakeType::brake); // tar, max_pct, kp, kd, ki
  task HandStart(hand_start);
  moveTarget(-400,100,0.3,0.01,0.3,vex::brakeType::brake);                                                                                                                                                                                                                                               0, 100, 0.3, 0.01, 0.3, vex::brakeType::brake); // tar, max_pct, kp, kd, ki
  
  // moveTarget(-500,100,0.15,0.1,0.3,vex::brakeType::coast);
  // moveTarget(830,60,0.15,0.1,0.3,vex::brakeType::brake);

}

#endif