#ifndef MOVE_H
#define MOVE_H
#include "vex.h"
#include "math.h"
#include "status.h"

double lv, rv, ftr, k, olv, orv;  

void mmove(double leftvelocity,double rightvelocity){
  //leftvelocity *= 0.93;
  left_1.spin(vex::directionType::fwd ,leftvelocity,vex::voltageUnits::mV);
  left_2.spin(vex::directionType::fwd,leftvelocity,vex::voltageUnits::mV);
  right_1.spin(vex::directionType::fwd,rightvelocity,vex::voltageUnits::mV);
  right_2.spin(vex::directionType::fwd,rightvelocity,vex::voltageUnits::mV);
}
void move(int c_3, int c_1){
  // channel mis-touch preventing
  if(abs(c_1) < 10) c_1 = 0;
  if(abs(c_3) < 10) c_3 = 0;
  // rotate speed protecting
  if(abs(c_1) > 60) ftr = 0.8;
  else ftr = 0.65;
  lv = c_3 + c_1 * ftr;
  rv = c_3 - c_1 * ftr;

  if(lv > 100) lv = 100;
  if(lv < -100) lv = -100;
  if(rv > 100) rv = 100;
  if(rv < -100) rv = -100;

  olv = (left_1.velocity(vex::velocityUnits::pct)+left_2.velocity(vex::velocityUnits::pct))/2;
  orv = (right_1.velocity(vex::velocityUnits::pct)+right_2.velocity(vex::velocityUnits::pct))/2;
  if(fabs(lv) < 5) lv = 0;
  if(fabs(rv) < 5) rv = 0;
  //Brain.Screen.printAt(  300,  40, "lv %3.0f", lv );
  //Brain.Screen.printAt(  300,  60, "rv %3.0f", rv );
  
  if(!lv && !rv && fabs(olv) < 7 && fabs(orv) < 7){
      left_1.stop(vex::brakeType::coast);
      left_2.stop(vex::brakeType::coast);
      right_1.stop(vex::brakeType::coast);
      right_2.stop(vex::brakeType::coast);
      return;
  }
  else if(lv*olv < 0 && rv*orv < 0){//both change direction
      if(fabs(lv-olv) > 50 && fabs(rv-orv) > 50) k = 0.18;
      else k = 0.7;
  }
  else if(lv*olv < 0 || rv*orv < 0){//either change direction
      if(fabs(lv-olv) > 50 || fabs(rv-orv) > 50) k = 0.5;
      else k = 1;
  }
  else {
      if(fabs(lv-olv) > 75 && fabs(rv-orv) > 75) k = 0.8;
      else k = 1;
  }

  mmove(130 * (k * lv + (1 - k) * olv),
        130 * (k * rv + (1 - k) * orv));
}
#endif