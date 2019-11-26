#ifndef _CTRLS_H_
#define _CTRLS_H_

#include "math.h"
#include "pid.h"
#include "utils.h"
#include "vex.h"

/*----------------------------------------------------------------------------*/
/*   Chassis Controller Move
/*----------------------------------------------------------------------------*/

double lv, rv, ftr, k, olv, orv;

void moveCtrl(int c3, int c1) {
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

  lv = fmin(100, lv);
  lv = fmax(-100, lv);
  rv = fmin(100, rv);
  rv = fmax(-100, rv);

  if (fabs(lv) < 5)
    lv = 0;
  if (fabs(rv) < 5)
    rv = 0;

  olv = getChsVlc_Left();
  orv = getChsVlc_Right();

  if (!lv && !rv && fabs(olv) < 7 && fabs(orv) < 7) {
    chsStop(vex::brakeType::coast);
    return;
  } else if (lv * olv < 0 && rv * orv < 0) // both change direction
  {
    if (fabs(lv - olv) > 50 && fabs(rv - orv) > 50)
      k = 0.18;
    else
      k = 0.5;
  } else if (lv * olv < 0 || rv * orv < 0) // either change direction
  {
    if (fabs(lv - olv) > 50 || fabs(rv - orv) > 50)
      k = 0.5;
    else
      k = 0.8;
  } else {
    if (fabs(lv - olv) > 50 && fabs(rv - orv) > 50)
      k = 0.8;
    else
      k = 1;
  }

  chsSpin(130 * (k * lv + (1 - k) * olv), 130 * (k * rv + (1 - k) * orv));
}

/*----------------------------------------------------------------------------*/
/*   Chassis Control
/*----------------------------------------------------------------------------*/

int smove_vot = 3000;

void moving() {
  if (btn_bck)
    chsSpin(-smove_vot, -smove_vot);
  else if (btn_fwd)
    chsSpin(smove_vot, smove_vot);
  else if (btn_right)
    chsSpin(1.5 * smove_vot, -1.5 * smove_vot);
  else if (btn_left)
    chsSpin(-1.5 * smove_vot, 1.5 * smove_vot);
  else
    moveCtrl(c_fwd, c_tur);
}

/*----------------------------------------------------------------------------*/
/*   Collector Control
/*----------------------------------------------------------------------------*/
bool push_flag = false, push_hold = false;
double push_err = 0, push_vlc = 0, sum_err = 0, output = 0;

void handing() {
  if (btn_hand_in) {
    push_hold = false;
    handsSpin(vex::directionType::fwd, 80, 1.6);
  } else if (btn_hand_ot) {
    push_hold = false;
    handsSpin(vex::directionType::rev, 80, 1.6);
  } else if (!push_hold) {
    handsStop(vex::brakeType::hold, 0.1);
  }
}

/*----------------------------------------------------------------------------*/
/*   Arm Control
/*----------------------------------------------------------------------------*/
void rasing() {
  if (btn_arm_up) {
    push_hold = false;
    motorSpin(arm, vex::directionType::fwd, 80, 2.2);

    if (fabs(push.rotation(rotationUnits::deg)) <
        300) // TODO: wating for testing.
      motorSpin(push, vex::directionType::fwd, 60, 2.4);
    else {
      motorStop(push, vex::brakeType::hold, 0.2);
    }
  } else if (btn_arm_dw) {
    push_hold = false;
    motorSpin(arm, directionType::rev, 80, 2.2);
  } else if (!push_hold) {
    motorStop(arm, brakeType::hold, 0.1);
  }
}

/*----------------------------------------------------------------------------*/
/*   Auto Push Task
/*----------------------------------------------------------------------------*/

int autoPush() {
  while (push_flag) {
    push_err =
        800 -
        fabs(push.rotation(rotationUnits::deg)); // target is 800 for 67677b
    push_vlc = fabs(push.velocity(vex::velocityUnits::pct));

    // pushing multi-layer control
    if (push_err < 10) // break
    {
      push_flag = false;
      push_hold = false;
    } else if (push_err < 100) // PID control
    {
      sum_err += push_err * 0.1;
      output = 17 + push_err * 0.1 - push_vlc * 0.4 + sum_err * 0.08;
    } else if (push_err < 180) // PID control
    {
      output = 20 + push_err * 0.065 - push_vlc * 0.3;
    } else if (push_err < 370) // 60 - 40 inertance reducing
    {
      output = 30 + push_err * 0.065 - push_vlc * 0.2;
    } else // 100 fast push
    {
      output = 100;
    }
    Brain.Screen.printAt(10, 10, "output is %.2f", output);
    motorSpin(push, vex::directionType::fwd, output, 2.2);

    // change to coast
    if (push_err < 135) {
      handsStop(vex::brakeType::coast, 0.1);
    }
    if (push_err < 520) {
      motorStop(arm, brakeType::coast, 0.1);
    }
    // sampling period
    vex::task::sleep(100);
  }

  push_hold = true;
  handsSpin(vex::directionType::rev, 80, 1.6);
  vex::task::sleep(100);
  handsStop(vex::brakeType::coast, 0.1);
  push_hold = false;

  return 0;
}

/*----------------------------------------------------------------------------*/
/*   Push Control
/*----------------------------------------------------------------------------*/

void pushing() {
  // --- manual push ---
  if (btn_score_auto) {
    // auto-push starts
    if (push_flag)
      return;
    push_flag = true;
    push_hold = true;
    task AutoPush(autoPush);
  } else if (btn_score_push) {
    push_flag = false;
    push_hold = false;

    motorSpin(push, vex::directionType::fwd, 35, 2.2);
  } else if (btn_score_pull) {
    push_flag = false;
    push_hold = false;

    motorSpin(push, vex::directionType::rev, 100, 2.2);
    push.resetRotation();
  } else if (!push_flag && !btn_arm_up && !btn_arm_dw) {
    motorStop(push, brakeType::hold, 0.1);
  }
}

/*----------------------------------------------------------------------------*/
/*   PID Move Forward
/*----------------------------------------------------------------------------*/

void moveTarget(int tar, int max_pct, bool fwd_tur, vex::brakeType bt,
                double kp, double kd, double ki) {
  int ma = max_pct, mi = 0, cof = 1, count = 20;
  if (tar < 0) {
    ma = 0;
    mi = -max_pct;
    cof = -1;
  }

  if (bt == brakeType::coast) chsSetBT(brakeType::coast);

  PID pid = PID(0.05, ma, mi, kp, kd, ki);
  resetChsRot();

  while (cof * (tar - getChsRot(fwd_tur)) > 20) {
    if (isChsStop()) {
      if (count-- <= 0)
        break;
    } else
      count = 20;

    double output = pid.calculate(tar, getChsRot(fwd_tur));
    Brain.Screen.printAt(10, 20, "err is %.2f", tar - getChsRot(fwd_tur));
    Brain.Screen.printAt(10, 40, "opt is %.2f", output);

    if (fwd_tur)
      moveCtrl(output, 0);
    else
      moveCtrl(0, output);

    vex::task::sleep(50);
  }

  if (bt == brakeType::coast) return;

  chsStop(bt);
  while (!isChsStop())
    task::sleep(10);

  // while (true)
  // {
  //   Brain.Screen.printAt(10, 20, "err is %.2f", tar - getChsRot(fwd_tur));
  //   vex::task::sleep(100);
  // }
}

void moveTarget_LR(int tar_l, int tar_r, int max_pct, vex::brakeType bt,
                   double kp, double kd, double ki) {
  if (tar_l * tar_r < 0)
    return; // tar_l and tar_r must be in same driection.

  int ma = max_pct, mi = 0, cof = 1, count = 20;
  if (tar_l < 0 && tar_r < 0) {
    ma = 0;
    mi = -max_pct;
    cof = -1;
  }

  if (bt == brakeType::coast) chsSetBT(brakeType::coast);

  PID pid_l = PID(0.05, ma, mi, kp, kd, ki);
  PID pid_r = PID(0.05, ma, mi, kp, kd, ki);
  resetChsRot();

  while (cof * (tar_l - getChsRot_Left()) > 20 ||
         cof * (tar_r - getChsRot_Right()) > 20) {
    if (isChsStop()) {
      if (count-- <= 0)
        break;
    } else
      count = 20;

    double output_l = 130 * pid_l.calculate(tar_l, getChsRot_Left());
    double output_r = 130 * pid_r.calculate(tar_r, getChsRot_Right());
    Brain.Screen.printAt(10, 20, "err is %.2f", tar_l - getChsRot(1));
    Brain.Screen.printAt(10, 50, "opt is %.2f", output);

    chsSpin(output_l, output_r);

    vex::task::sleep(50);
  }

  if (bt == brakeType::coast)
    return;
  chsStop(bt);
  while (!isChsStop())
    task::sleep(10);
}

void moveTarget_LR_PCT(int tar_l, int tar_r, int max_pct, vex::brakeType bt, double kp, double kd, double ki) 
{
  if (tar_l * tar_r < 0) return; // tar_l and tar_r must be in same driection.
  int ma = max_pct, mi = 0, cof = 1, count = 20, tar = fmax(tar_l, tar_r);
  if (tar_l < 0 && tar_r < 0) {
    ma = 0;
    mi = -max_pct;
    cof = -1;
    tar = fmin(tar_l, tar_r);
  }

  if (bt == brakeType::coast) chsSetBT(brakeType::coast);

  PID pid = PID(0.05, ma, mi, kp, kd, ki);
  resetChsRot();

  if (tar == tar_l) 
  {
    while (cof * (tar - getChsRot_Left()) > 20) 
    {
      if (isChsStop()) {
        if (count-- <= 0) break;
      } else count = 20;

      double output = 130 * pid.calculate(tar, getChsRot_Left());

      chsSpin(output, output * (tar_r / tar_l));

      vex::task::sleep(50);
    }
  } 
  else
  {
    while (cof * (tar - getChsRot_Right()) > 20) 
    {
      if (isChsStop()) {
        if (count-- <= 0) break;
      } else count = 20;

      double output = 130 * pid.calculate(tar, getChsRot_Right());

      chsSpin(output * (tar_l / tar_r), output);

      vex::task::sleep(50);
    }
  }

  if (bt == brakeType::coast) return;
  chsStop(bt);
  while (!isChsStop()) task::sleep(10);
}

/*----------------------------------------------------------------------------*/
/*   PID Move Turn by Gyro
/*----------------------------------------------------------------------------*/

void turnTarget(int tar, int max_pct, vex::brakeType bt, double kp, double kd,
                double ki) {
  int ma = max_pct, mi = 0, cof = 1, count = 20;

  if (tar < 0) {
    ma = 0;
    mi = -max_pct;
    cof = -1;
  }

  PID pid = PID(0.05, ma, mi, kp, kd, ki);
  gyro_1.setHeading(0, vex::rotationUnits::deg);

  while (cof * (tar - gyro_deg) > 5) {
    if (isChsStop()) {
      if (count-- <= 0)
        break;
    } else
      count = 20;

    double output = pid.calculate(tar, gyro_deg);
    Brain.Screen.printAt(10, 20, "err is %.2f", cof * (tar - gyro_deg));
    Brain.Screen.printAt(10, 50, "opt is %.2f", output);

    moveCtrl(0, output);

    vex::task::sleep(50);
  }

  if (bt == brakeType::coast)
    return;
  chsStops(bt, 2.2);
  while (!isChsStop())
    task::sleep(10);

  // while (true)
  // {
  //   Brain.Screen.printAt(10, 10, "err is %.2f", tar - gyro_deg);
  //   vex::task::sleep(100);
  // }
}

/*----------------------------------------------------------------------------*/
/*   Secret
/*----------------------------------------------------------------------------*/

void secret() {
  while (!(btn_left && btn_bck && btn_score_auto && btn_score_pull &&
           btn_arm_up && btn_arm_dw && btn_hand_in && btn_hand_ot)) {
    vex::task::sleep(1000);
  }
}
#endif
