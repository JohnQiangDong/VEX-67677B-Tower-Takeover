#ifndef _UTILS_H_
#define _UTILS_H_

#include "vex.h"
#include "math.h"

/*----------------------------------------------------------------------------*/
/*   Motor Control
/*----------------------------------------------------------------------------*/

void motorSpin(vex::motor motor, vex::directionType dt, int pct, double mt)
{
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.spin(dt, pct, vex::velocityUnits::pct);
}

void motorStop(vex::motor motor, vex::brakeType bt, double mt)
{
  motor.setMaxTorque(mt, currentUnits::amp);
  motor.stop(bt);
}

/*----------------------------------------------------------------------------*/
/*   Chassis Control
/*----------------------------------------------------------------------------*/

void chsStop(vex::brakeType bt)
{
  left_1.stop(bt);
  left_2.stop(bt);
  right_1.stop(bt);
  right_2.stop(bt);
}

void chsSpin(double left_vot, double right_vot)
{
  left_1.spin(vex::directionType::fwd, left_vot, vex::voltageUnits::mV);
  left_2.spin(vex::directionType::fwd, left_vot, vex::voltageUnits::mV);
  right_1.spin(vex::directionType::fwd, right_vot, vex::voltageUnits::mV);
  right_2.spin(vex::directionType::fwd, right_vot, vex::voltageUnits::mV);
}

/*----------------------------------------------------------------------------*/
/*   Hands Control
/*----------------------------------------------------------------------------*/

void handsStop(vex::brakeType bt, double mt)
{
  motorStop(hand1, bt, mt);
  motorStop(hand2, bt, mt);
}

void handsSpin(vex::directionType dt, int pct, double mt)
{
  motorSpin(hand1, dt, pct, mt);
  motorSpin(hand2, dt, pct, mt);
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

double getChsRot_Left()
{
  return (left_1.rotation(rotationUnits::deg) + left_2.rotation(rotationUnits::deg)) / 2;
}

double getChsRot_Right()
{
  return (right_1.rotation(rotationUnits::deg) + right_2.rotation(rotationUnits::deg)) / 2;
}


double getChsRot(bool fow_tur)
{
  return fow_tur ? (getChsRot_Left() + getChsRot_Right()) / 2 : (getChsRot_Left() - getChsRot_Right()) / 2;
}

/*----------------------------------------------------------------------------*/
/*   Get Chassis Velocity
/*----------------------------------------------------------------------------*/

double getChsVlc_Left()
{
  return (left_1.velocity(vex::velocityUnits::pct) + left_2.velocity(vex::velocityUnits::pct)) / 2;
}

double getChsVlc_Right()
{
  return (right_1.velocity(vex::velocityUnits::pct) + right_2.velocity(vex::velocityUnits::pct)) / 2;
}


/*----------------------------------------------------------------------------*/
/*   Gyro calculate task
/*----------------------------------------------------------------------------*/
float gyro_value = 0;
bool gyro_cal = false;

void resetGyro()
{
  gyro_cal = true;
}

int calcGyro()
{
  float _ori_gyro = 0;
  float _last_ori_gyro = 0;
  float _smart_gyro = 0;
  float count = 0;
  float gyroOffset = 0;

  while(true)
  {
    _ori_gyro = GYRO;
    if(_last_ori_gyro - _ori_gyro > 300)
    {
      count += 1;
    }
    if(_last_ori_gyro - _ori_gyro < -300)
    {
      count += -1;
    }
    _smart_gyro = count * 360 + _ori_gyro;
    if(gyro_cal)
    {
      gyroOffset += gyro_value;
      gyro_cal = false;
    }
    gyro_value = _smart_gyro - gyroOffset;
    _last_ori_gyro = _ori_gyro;
    printScreen(10, 100, "Real Gyro: %f", -gyro_value);
    delay(1);
  }
}

#endif