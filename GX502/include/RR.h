#ifndef RR_H
#define RR_H
#include "vex.h"
#include "automove.h"
#include "move.h"

void RR(){
  double time;
  RotateMotor.stop();
  RotateMotor.setStopping(brakeType ::coast);
  automove(500,500,300,10000);
  automove(-500,-500,400,8000);
  //将方块顶到前面
  LiftMotor.setMaxTorque(0.2,vex::currentUnits::amp);
  LiftMotor.spin(vex::directionType::fwd,20,vex::percentUnits::pct);
  LeftRollerMotor.spin(vex::directionType::rev,-100,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,-100,vex::percentUnits::pct);
  //往后顶墙壁

  vex::task::sleep(350);
  RotateMotor.stop();
  RotateMotor.setStopping(brakeType ::coast);
  RotateMotor.spin(vex::directionType::rev,50,vex::percentUnits::pct);
  LeftRollerMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);
  LiftMotor.setMaxTorque(1.0,vex::currentUnits::amp);
  LiftMotor.startRotateFor(10,vex::rotationUnits::deg,5,vex::velocityUnits::pct);
  automove(1100,1100,3000,5000);

  RotateMotor.setStopping(brakeType ::coast);
  LiftMotor.setMaxTorque(0.2,vex::currentUnits::amp);
  LiftMotor.spin(vex::directionType::fwd,20,vex::percentUnits::pct);
  
  automove(-400,-400,400,10000);

  vex::task::sleep(150);
  gyroturn(-60,6000,400);
  vex::task::sleep(50);
  automove(-800,-800,800,10000);
  vex::task::sleep(50);
  gyroturn(60,6000,400);
  
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 400){
    mmove(-12000,-12000);
  }
  vex::task::sleep(50);
  LeftRollerMotor.spin(vex::directionType::rev,-50,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,-50,vex::percentUnits::pct);
  automove(400,400,400,10000);
  
  LeftRollerMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);  
  automove(900,900,3000,4000);
  
  automove(-500,-500,450,12000);
  RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
  RotateMotor.resetRotation();
  RotateMotor.startRotateFor(1400,vex::rotationUnits::deg,25+0.022*(1500-RotateMotor.rotation(vex::rotationUnits::deg)),vex::velocityUnits::pct);
  automove(-550,-550,550,10000);
  LeftRollerMotor.stop(vex::brakeType::brake);
  RightRollerMotor.stop(vex::brakeType::brake);

  vex::task::sleep(50);
  gyroturn(135,4000,2500);
  vex::task::sleep(50);
  automove(350,350,500,4000);

  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 1400){
    RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,15,vex::percentUnits::pct);
  } 
  RotateMotor.stop();
  RotateMotor.setStopping(brakeType ::hold);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 600){
    mmove(-4000,-4000);
  }
  autostop(1,50);
  RotateMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
}

#endif