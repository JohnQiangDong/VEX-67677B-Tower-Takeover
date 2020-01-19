#ifndef BL_H
#define BL_H
#include "vex.h"
#include "automove.h"
#include "move.h"

void BL(){
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

  vex::task::sleep(100);
  RotateMotor.stop();
  RotateMotor.setStopping(brakeType ::coast);
  RotateMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);
  vex::task::sleep(350);
  RotateMotor.spin(vex::directionType::rev,50,vex::percentUnits::pct);
  LeftRollerMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);
  LiftMotor.setMaxTorque(1.0,vex::currentUnits::amp);
  LiftMotor.startRotateFor(10,vex::rotationUnits::deg,5,vex::velocityUnits::pct);
  automove(1100,1100,3000,5000);
  RotateMotor.setStopping(brakeType ::coast);
  
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 680){
    mmove(-12000,-12000);
  }
  gyroturn(50,6000,400);
  //陀螺仪控制旋转
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 730){
    mmove(-10000,-10000);
  }
  vex::task::sleep(50);
  gyroturn(-50,6000,400);


  LeftRollerMotor.spin(vex::directionType::rev,-50,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,-50,vex::percentUnits::pct);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 200){
    mmove(-10000,-10000);
  }
  //vex::task::sleep(50);
  LeftRollerMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
  RightRollerMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);  
  automove(800,800,3000,4000);

  
  gyroturn(30,8000,400);
  //陀螺仪控制旋转
  vex::task::sleep(50);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 600){
    mmove(4000,4000);
  }
  gyroturn(-30,8000,200);


  automove(-750,-750,750,10000);
  LeftRollerMotor.stop(vex::brakeType::brake);
  RightRollerMotor.stop(vex::brakeType::brake);
  RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
  RotateMotor.spin(vex::directionType::fwd,40,vex::percentUnits::pct);
  gyroturn(-300,7000,1800);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 200){
    mmove(10000,10000);
  }
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 200){
    mmove(3500,3500);
  }

  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 1850){
    RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,45,vex::percentUnits::pct);
  } 
  RotateMotor.setMaxTorque(0.2,vex::currentUnits::amp);
  RotateMotor.stop();
  RotateMotor.setStopping(brakeType ::hold);
  vex::task::sleep(150);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 600){
    mmove(-4000,-4000);
  }
  autostop(1,50);
}

#endif