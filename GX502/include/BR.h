#ifndef BR_H
#define BR_H
#include "vex.h"
#include "automove.h"
#include "move.h"

void BR(){
double time;
LiftMotor.setMaxTorque(0.2,vex::currentUnits::amp);
LiftMotor.stop(brakeType::hold);
LeftMotor1.spin(vex::directionType::fwd,-1000,vex::voltageUnits::mV);
LeftMotor2.spin(vex::directionType::fwd,-1000,vex::voltageUnits::mV);
RightMotor1.spin(vex::directionType::fwd,-1000,vex::voltageUnits::mV);
RightMotor2.spin(vex::directionType::fwd,-1000,vex::voltageUnits::mV);
LeftRollerMotor.spin(vex::directionType::rev,-100,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,-100,vex::percentUnits::pct);
RotateMotor.spin(vex::directionType::rev,40,vex::percentUnits::pct);
vex::task::sleep(500);
RotateMotor.stop();
RotateMotor.setStopping(brakeType ::coast);
vex::task::sleep(400);
RotateMotor.spin(vex::directionType::rev,40,vex::percentUnits::pct);
vex::task::sleep(500);
RotateMotor.stop();
RotateMotor.setStopping(brakeType ::hold);

LeftRollerMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);
automove(640,640,1500,6000);
gyroturn(40,6000,800);
time = Brain.timer(vex::timeUnits::msec);
while (Brain.timer(vex::timeUnits::msec) - time < 350){
  mmove(10000,10000);
}
autostop(1,200);
automove(320,320,1500,2500);
 
time = Brain.timer(vex::timeUnits::msec);
while (Brain.timer(vex::timeUnits::msec) - time < 600){
  mmove(0,-7000);
}
  
gyroturn(289,6000,1800);
automove(600,600,3000,4000);
gyroturn(-190,6000,1800);

automove(570,570,3000,4000);
LeftRollerMotor.spin(vex::directionType::rev,-60,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,-60,vex::percentUnits::pct);
vex::task::sleep(200);
LeftRollerMotor.stop(vex::brakeType::brake);
RightRollerMotor.stop(vex::brakeType::brake);  
RotateMotor.resetRotation();
  RotateMotor.startRotateFor(1350,vex::rotationUnits::deg,30+0.022*(1400-RotateMotor.rotation(vex::rotationUnits::deg)),vex::velocityUnits::pct);
  
  
time = Brain.timer(vex::timeUnits::msec);
while (Brain.timer(vex::timeUnits::msec) - time < 200){
  mmove(-6000,-6000);
}
gyroturn(45,6000,1800);
time = Brain.timer(vex::timeUnits::msec);
while (Brain.timer(vex::timeUnits::msec) - time < 1400){
  mmove(3500,3500);
}
LeftRollerMotor.stop(vex::brakeType::coast);
  RightRollerMotor.stop(vex::brakeType::coast);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 500){
    RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,30,vex::percentUnits::pct);
    LeftMotor1.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
    LeftMotor2.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
    RightMotor1.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
    RightMotor2.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
  } 
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 700){
    RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,30,vex::percentUnits::pct);
    LeftMotor1.spin(vex::directionType::fwd,4,vex::velocityUnits::pct);
    LeftMotor2.spin(vex::directionType::fwd,4,vex::velocityUnits::pct);
    RightMotor1.spin(vex::directionType::fwd,4,vex::velocityUnits::pct);
    RightMotor2.spin(vex::directionType::fwd,4,vex::velocityUnits::pct);
  } 
  /*while (Brain.timer(vex::timeUnits::msec) - time < 200){
    RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,35,vex::percentUnits::pct);
    LeftMotor1.stop(vex::brakeType::hold);
    LeftMotor2.stop(vex::brakeType::hold);
    RightMotor1.stop(vex::brakeType::hold);
    RightMotor2.stop(vex::brakeType::hold);
  } */
  RotateMotor.setMaxTorque(0.2,vex::currentUnits::amp);
  RotateMotor.stop();
  RotateMotor.setStopping(brakeType ::hold);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 600){
     RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,20,vex::percentUnits::pct);
    mmove(-4000,-4000);
  }
  autostop(1,50);
  RotateMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
}

#endif