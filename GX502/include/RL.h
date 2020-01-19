#ifndef RL_H
#define RL_H
#include "vex.h"
#include "automove.h"
#include "move.h"

void RL(){
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
LeftRollerMotor.spin(vex::directionType::rev,0,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,0,vex::percentUnits::pct);
automove(630,630,1500,6000);
gyroturn(-45,6000,800);
LeftRollerMotor.spin(vex::directionType::rev,90,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,90,vex::percentUnits::pct);
time = Brain.timer(vex::timeUnits::msec);
while (Brain.timer(vex::timeUnits::msec) - time < 340){
  mmove(10000,10000);
}
LeftRollerMotor.spin(vex::directionType::rev,100,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,100,vex::percentUnits::pct);
autostop(1,200);
automove(350,350,1600,2500);

time = Brain.timer(vex::timeUnits::msec);
while (Brain.timer(vex::timeUnits::msec) - time < 540){
  mmove(-8000,0);
}
  
  gyroturn(-290,6000,1800);
  automove(600,600,3000,4500);
  // automove(-250,-250,1500,6000);
  gyroturn(180,6000,1800);
  automove(520,520,3500,4000);
vex::task::sleep(800);
  LeftRollerMotor.spin(vex::directionType::rev,-10,vex::percentUnits::pct);
RightRollerMotor.spin(vex::directionType::fwd,-10,vex::percentUnits::pct);
vex::task::sleep(200);
  LeftRollerMotor.stop(vex::brakeType::coast);
  RightRollerMotor.stop(vex::brakeType::coast);  
 RotateMotor.resetRotation();
  RotateMotor.startRotateFor(950,vex::rotationUnits::deg,30+0.022*(1400-RotateMotor.rotation(vex::rotationUnits::deg)),vex::velocityUnits::pct);
  
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 250){
    mmove(-5500,-5500);
  }
  gyroturn(-80,6000,1800);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 1300){
    mmove(3400,3400);
  }
  LeftRollerMotor.stop(vex::brakeType::coast);
  RightRollerMotor.stop(vex::brakeType::coast);
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 400){
    RotateMotor.setMaxTorque(2.4,vex::currentUnits::amp);
    RotateMotor.spin(vex::directionType::fwd,35,vex::percentUnits::pct);
    LeftMotor1.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
    LeftMotor2.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
    RightMotor1.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
    RightMotor2.spin(vex::directionType::fwd,-1,vex::velocityUnits::pct);
  } 
  time = Brain.timer(vex::timeUnits::msec);
  while (Brain.timer(vex::timeUnits::msec) - time < 800){
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