#ifndef MOVE_H
#define MOVE_H
#include "vex.h"
void mmove(double leftvelocity,double rightvelocity){//定值前进
  //leftvelocity *= 1.10;
  LeftMotor1.spin(vex::directionType::fwd,leftvelocity,vex::voltageUnits::mV);
  LeftMotor2.spin(vex::directionType::fwd,leftvelocity,vex::voltageUnits::mV);
  RightMotor1.spin(vex::directionType::fwd,rightvelocity,vex::voltageUnits::mV);
  RightMotor2.spin(vex::directionType::fwd,rightvelocity,vex::voltageUnits::mV);
}
void move(double lv, double rv){   //前进（可转弯）
  if(lv > 100) lv = 100;
  if(lv < -100) lv = -100;
  if(rv > 100) rv = 100;
  if(rv < -100) rv = -100;
  double leftvelocity, rightvelocity, maxv = 100;
  double k = 1;
  double olv, orv;
  olv = (LeftMotor1.velocity(vex::velocityUnits::pct)+LeftMotor2.velocity(vex::velocityUnits::pct))/2;
  orv = (RightMotor1.velocity(vex::velocityUnits::pct)+RightMotor2.velocity(vex::velocityUnits::pct))/2;
  if(abs(lv) < 5) lv = 0;
  if(abs(rv) < 5) rv = 0;
  //Brain.Screen.printAt(  300,  40, "lv %3.0f", lv );
  //Brain.Screen.printAt(  300,  60, "rv %3.0f", rv );
  
  if(!lv && !rv && abs(olv) < 7 && abs(orv) < 7){//？
      LeftMotor1.stop(vex::brakeType::coast);
      LeftMotor2.stop(vex::brakeType::coast);
      RightMotor1.stop(vex::brakeType::coast);
      RightMotor2.stop(vex::brakeType::coast);
      return;
  }
  else if(lv*olv < 0 && rv*orv < 0){//both change direction
      if(abs(lv-olv) > 50 && abs(rv-orv) > 50) k = 0.18;
      else k = 0.7;
  }
  else if(lv*olv < 0 || rv*orv < 0){//either change direction
      if(abs(lv-olv) > 50 || abs(rv-orv) > 50) k = 0.5;
      else k = 1;
  }
  else {
      if(abs(lv-olv) > 75 && abs(rv-orv) > 75) k = 0.8;
      else k = 1;
  }
  
  leftvelocity = 130*(k*lv+(1-k)*olv);
  rightvelocity = 130*(k*rv+(1-k)*orv);
  mmove(leftvelocity,rightvelocity);
}
#endif