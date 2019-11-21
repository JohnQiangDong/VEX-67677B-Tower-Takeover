#ifndef _UTILS_SOURCE_
#define _UTILS_SOURCE_

#include "automove.h"
#include "vex.h"
#include "math.h"
#include "pid.h"

/*----------------------------------------------------------------------------*/
/*   Motor Control
/*----------------------------------------------------------------------------*/

void Spin(vex::motor motor, vex::directionType dt, int pct, double mt)
{
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void Stop(vex::motor motor, brakeType bt, double mt)
{
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}

/*----------------------------------------------------------------------------*/
/*   Chassis Moving Control
/*----------------------------------------------------------------------------*/

double lv, rv, ftr, k, olv, orv;

void mmove(double left_vot, double right_vot)
{
  left_1.spin(vex::directionType::fwd, left_vot, vex::voltageUnits::mV);
  left_2.spin(vex::directionType::fwd, left_vot, vex::voltageUnits::mV);
  right_1.spin(vex::directionType::fwd, right_vot, vex::voltageUnits::mV);
  right_2.spin(vex::directionType::fwd, right_vot, vex::voltageUnits::mV);
}

/*----------------------------------------------------------------------------*/
/*   Chassis Controller Move
/*----------------------------------------------------------------------------*/

void move(int c3, int c1)
{
  // channel mis-touch preventing
  if (abs(c1) < 10)
    c1 = 0;
  if (abs(c3) < 10)
    c3 = 0;
  // rotate speed protecting
  if (abs(c1) > 60)
    ftr = 0.8;
  else
    ftr = 0.65;
  lv = c3 + c1 * ftr;
  rv = c3 - c1 * ftr;

  if (lv > 100)
    lv = 100;
  if (lv < -100)
    lv = -100;
  if (rv > 100)
    rv = 100;
  if (rv < -100)
    rv = -100;
  if (fabs(lv) < 5)
    lv = 0;
  if (fabs(rv) < 5)
    rv = 0;

  olv = (left_1.velocity(vex::velocityUnits::pct) + left_2.velocity(vex::velocityUnits::pct)) / 2;
  orv = (right_1.velocity(vex::velocityUnits::pct) + right_2.velocity(vex::velocityUnits::pct)) / 2;

  if (!lv && !rv && fabs(olv) < 7 && fabs(orv) < 7)
  {
    stopChassis(vex::brakeType::coast);
    return;
  }
  else if (lv * olv < 0 && rv * orv < 0) //both change direction
  {
    if (fabs(lv - olv) > 50 && fabs(rv - orv) > 50)
      k = 0.18;
    else
      k = 0.5;
  }
  else if (lv * olv < 0 || rv * orv < 0) //either change direction
  {
    if (fabs(lv - olv) > 50 || fabs(rv - orv) > 50)
      k = 0.5;
    else
      k = 0.8;
  }
  else
  {
    if (fabs(lv - olv) > 50 && fabs(rv - orv) > 50)
      k = 0.8;
    else
      k = 1;
  }

  mmove(130 * (k * lv + (1 - k) * olv),
        130 * (k * rv + (1 - k) * orv));
}

/*----------------------------------------------------------------------------*/
/*   Reset Chassis Rotation
/*----------------------------------------------------------------------------*/

void resetChsRot()
{
  left_1.resetRotation();
  left_2.resetRotation();
  right_1.resetRotation();
  right_2.resetRotation();
}

/*----------------------------------------------------------------------------*/
/*   Get Chassis Rotation ( 1 for foward / 0 for turn)
/*----------------------------------------------------------------------------*/

double getChsRot(int fow_tur)
{
  if (fow_tur)
    return fow_tur(left_1.rotation(rotationUnits::deg) + left_2.rotation(rotationUnits::deg) +
                   right_1.rotation(rotationUnits::deg) + right_2.rotation(rotationUnits::deg)) / 4;
  else
    return fow_tur(left_1.rotation(rotationUnits::deg) + left_2.rotation(rotationUnits::deg) -
                   right_1.rotation(rotationUnits::deg) + right_2.rotation(rotationUnits::deg)) / 4;
}

/*----------------------------------------------------------------------------*/
/*   Chassis Stop
/*----------------------------------------------------------------------------*/

void stopChs(vex::brakeType bt)
{
  left_1.stop(bt);
  left_2.stop(bt);
  right_1.stop(bt);
  right_2.stop(bt);
}

/*----------------------------------------------------------------------------*/
/*   PID Move Forward
/*----------------------------------------------------------------------------*/

void moveTarget(int tar, int max_pct, int kp, int kd, int ki)
{
  int ma = max_pct, mi = 0;
  if (tar < 0)
  {
    ma = 0;
    mi = -max_pct;
  }

  stopChassis(vex::brakeType::coast);
  vex::task::sleep(100); // waiting for stability

  PID pid = PID(0.1, ma, mi, kp, kd, ki);
  resetChsRot();

  while (err > 20)
  {
    double output = pid.calculate(tar, getChsRot(1));
    move(output, 0);
    vex::task::sleep(100);
  }
}

/*----------------------------------------------------------------------------*/
/*   PID Turn
/*----------------------------------------------------------------------------*/

void turnTarget(int tar, int max_pct, int kp, int kd, int ki)
{
  int ma = max_pct, mi = 0;
  if (tar < 0)
  {
    ma = 0;
    mi = -max_pct;
  }

  stopChassis(vex::brakeType::coast);
  vex::task::sleep(100); // waiting for stability

  PID pid = PID(0.1, ma, mi, kp, kd, ki);
  resetChsRot();

  while (err > 20)
  {
    double output = pid.calculate(tar, getChsRot(0));
    move(0, output);
    vex::task::sleep(100);
  }
}

#endif