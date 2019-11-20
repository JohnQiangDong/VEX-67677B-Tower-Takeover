#ifndef _UTILS_H_
#define _UTILS_H_

#include "automove.h"
#include "vex.h"
#include "math.h"

/*----------------------------------------------------------------------------*/
/*   Motor Control
/*----------------------------------------------------------------------------*/

void Spin(vex::motor motor, vex::directionType dt, int pct, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void Stop(vex::motor motor, brakeType bt, double mt) {
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}

/*----------------------------------------------------------------------------*/
/*   Chassis Moving Control
/*----------------------------------------------------------------------------*/

double lv, rv, ftr, k, olv, orv;  

void mmove(double left_vot,double right_vot)
{
  left_1.spin(vex::directionType::fwd ,left_vot,vex::voltageUnits::mV);
  left_2.spin(vex::directionType::fwd,left_vot,vex::voltageUnits::mV);
  right_1.spin(vex::directionType::fwd,right_vot,vex::voltageUnits::mV);
  right_2.spin(vex::directionType::fwd,right_vot,vex::voltageUnits::mV);
}

void move(int c3, int c1)
{
  // channel mis-touch preventing
  if(abs(c1) < 10) c1 = 0;
  if(abs(c3) < 10) c3 = 0;
  // rotate speed protecting
  if(abs(c1) > 60) ftr = 0.8;
  else ftr = 0.65;
  lv = c3 + c1 * ftr;
  rv = c3 - c1 * ftr;

  if(lv > 100) lv = 100;
  if(lv < -100) lv = -100;
  if(rv > 100) rv = 100;
  if(rv < -100) rv = -100;
  if(fabs(lv) < 5) lv = 0;
  if(fabs(rv) < 5) rv = 0;

  olv = (left_1.velocity(vex::velocityUnits::pct) + left_2.velocity(vex::velocityUnits::pct)) / 2;
  orv = (right_1.velocity(vex::velocityUnits::pct) + right_2.velocity(vex::velocityUnits::pct)) / 2;
  
  if(!lv && !rv && fabs(olv) < 7 && fabs(orv) < 7)
  {
      left_1.stop(vex::brakeType::coast);
      left_2.stop(vex::brakeType::coast);
      right_1.stop(vex::brakeType::coast);
      right_2.stop(vex::brakeType::coast);
      return;
  }
  else if(lv * olv < 0 && rv * orv < 0) //both change direction
  {
      if(fabs(lv - olv) > 50 && fabs(rv - orv) > 50) k = 0.18;
      else k = 0.5;
  }
  else if(lv * olv < 0 || rv * orv < 0) //either change direction
  {
      if(fabs(lv - olv) > 50 || fabs(rv - orv) > 50) k = 0.5;
      else k = 0.8;
  }
  else
  {
      if(fabs(lv - olv) > 50 && fabs(rv - orv) > 50) k = 0.8;
      else k = 1;
  }

  mmove(130 * (k * lv + (1 - k) * olv),
        130 * (k * rv + (1 - k) * orv));
}

#endif