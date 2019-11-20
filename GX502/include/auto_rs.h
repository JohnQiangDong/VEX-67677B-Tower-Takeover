#ifndef RS_H
#define RS_H
#include "utils.h"
#include "vex.h"
#include "automove.h"

void auto_rs(){
  /*push.stop(brakeType::hold);
  push.setMaxTorque(0.2, currentUnits:: amp);
  arm.spin (directionType::fwd,100,vex::velocityUnits::pct);
  mmove(3000,3000);
  vexDelay(500);
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  arm.spin(directionType::rev,100,vex::velocityUnits::pct);
  mmove(-4000,-4000);
  vexDelay(500);
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  arm.stop(brakeType::hold);*/

  arm.setMaxTorque(0.2, currentUnits::amp);
  //automove(-100, -100, 1000, 5000);
  push.setMaxTorque(2.4, vex::currentUnits::amp);
  hand1.spin(directionType::fwd,100,vex::velocityUnits::pct);
  hand2.spin(directionType::rev,100,vex::velocityUnits::pct);
  //collect frist 3 cubes
  automove(1350,1350,3000,7800);


  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);

  hand1.setMaxTorque(0.1,currentUnits::amp);
  hand2.setMaxTorque(0.1,currentUnits::amp);
  //stop hand
  //automove(-114,114,2000,14000);
  autoturn(-1290,-1290,2000,9900);
  //go back
  //automove(144, -144, 2000, 14000);
  
  //start to collect another four cubes
  hand1.setMaxTorque(2.4,currentUnits::amp);
  hand2.setMaxTorque(2.4,currentUnits::amp);
  hand1.spin(directionType::fwd,100,vex::velocityUnits::pct);
  hand2.spin(directionType::rev,100,vex::velocityUnits::pct);
  automove(1150,1150,3000,4500);  
  vexDelay(70);
  //dalay for get cube at right position
  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);
  hand1.setMaxTorque(0.1,currentUnits::amp);
  hand2.setMaxTorque(0.1,currentUnits::amp);
  automove(-655, -655, 2000, 9900);
  automove(400, -400, 1000, 9900);
  automove(450, 450, 4000, 6000);
  
  push.setMaxTorque(2.4, currentUnits::amp);  
  push.spin(vex::directionType::fwd,43,vex::velocityUnits::pct);
  vexDelay(1740);
  push.stop(brakeType::hold);
  hand1.spin(directionType::rev,100,vex::velocityUnits::pct);
  hand2.spin(directionType::fwd,100,vex::velocityUnits::pct);
  vexDelay(280);//prevent hand stucking cubes
  
  automove(-200, -200, 1000, 14000);
  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);
  hand1.setMaxTorque(0.1,currentUnits::amp);
  hand2.setMaxTorque(0.1,currentUnits::amp);  
}

#endif