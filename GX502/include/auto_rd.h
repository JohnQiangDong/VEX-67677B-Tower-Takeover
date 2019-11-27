#ifndef RD_H
#define RD_H

#include "vex.h"
#include "utils.h"
#include "ctrls.h"

int cube_position_rd()
{
  vex::task::sleep(2800);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(250);
  handsStop(vex::brakeType::brake, 0.15);
  return 0;
}
int cube_prevent_stuck_rd(){
  vex::task::sleep(1200);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(200);
  handsSpin(vex::directionType::fwd, 70, 1.6);

  return 0;
}
int push_in_auto_rd() // > 3s
{
  motorSpin(push,vex::directionType::rev,40,2.2);
  vex::task::sleep(300);
  motorStop(push,brakeType::hold,0.2);
  vex::task::sleep(3000);
  push.resetRotation();
  double t = Brain.timer(vex::timeUnits::msec);
  
  while (push_flag && (Brain.timer(vex::timeUnits::msec) - t) < 3500) { // 2-3s
    push_err = 800 - fabs(push.rotation(rotationUnits::deg));//target is 800 for 67677b
    push_vlc = fabs(push.velocity(vex::velocityUnits::pct));
    // pushing multi-layer control
    if (push_err < 10) // break
    {
      push_flag = false;
    } 
    else if (push_err < 100) // PID control
    {
      sum_err += push_err * 0.1;
      output = 17 + push_err * 0.1 - push_vlc * 0.4 + sum_err * 0.2;
    }
    else if (push_err < 150) // PID control
    {
      output = 20 + push_err * 0.065 - push_vlc * 0.3;
    } 
    else if (push_err < 300) // 60 - 40 inertance reducing
    {
      output = 30 + push_err * 0.065 - push_vlc * 0.2;
    } 
    else // 100 fast push
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
  motorStop(push, vex::brakeType ::hold, 0.2);
  handsSpin(vex::directionType::rev, 80, 1.6);
  vex::task::sleep(100);
  handsStop(vex::brakeType::coast, 0.1);
  push_hold = false;
  vex::task::sleep(700);
  return 0;
}

int start_hand_rd()
{
  handsSpin(vex::directionType::fwd,55, 1.6);

  return 0;
}

int start_arm_push_rd()
{
  motorSpin(push, vex::directionType::rev, 60, 2.2);
  motorSpin(arm, vex::directionType::rev, 60, 2.2);
  vex::task::sleep(300);
  motorStop(arm, brakeType::hold, 0.1);
  vex::task::sleep(300);
  motorStop(push, brakeType::hold, 0.1);

  return 0;
}

int push_up_rd()
{
  motorSpin(push, vex::directionType::fwd, 100, 2.2);
  vex::task::sleep(300);
  motorSpin(push, vex::directionType::rev, 100, 2.2);
  vex::task::sleep(600);
  motorStop(push, brakeType::hold, 0.1);
  return 0;
}

void auto_rd(){
  //sqrawl
  task StartArmPushBD(start_arm_push_rd);
  //moveTarget(250, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); // tar, max_pct, fwd_tur, bt, kp, kd, ki
  // start the robot and sprawl
  task HandStartBS(start_hand_rd);
  moveTarget(-20,100, true, vex::brakeType::brake, 0.3, 0.01, 0.3);
  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());
  task::sleep(200);//200
  //move forward 800
  task PushUpBD(push_up_rd);
  chsSpin(6000, 6000);
  task::sleep(200);//200
  handsStop(brakeType::coast,0.1);
  moveTarget(485, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3);//467
  //turn right 35
  turnTarget(-35,63,brake,4,0.1,0.5);//37
  //collecter start turn
  task StartHandBD(start_hand_rd);
  //slow accelerate forward
  moveTarget(200,60, true, brakeType::coast, 0.1, 0.03, 0.1);
  handsStop(brakeType::coast,0.1);
  task PushUpBD2(push_up_rd); 
  vexDelay(300);
  handsSpin(vex::directionType::fwd, 100,2.2);
  //moveTarget(100, 60, true, brakeType::brake, 0.3,0.01, 0.3);
  handsSpin(vex::directionType::fwd, 100,2.2);
  //moveTarget(160, 70, true, brakeType::brake, 3,0.01, 0.3);
  moveTarget_LR(255,190, 60, brakeType::brake, 0.2, 0.01, 0.3);//355,255
  task::sleep(500);
  task CubePrevenStuck (cube_prevent_stuck_rd);
  //turn right 135
  turnTarget(-95, 100, vex::brakeType::brake, 3, 0.03,0.1);//150
  push_flag = true;
  push_hold = true;
  task CubePositionBD(cube_position_rd);
 // task PushInAuToBD(push_in_auto_bd);
  moveTarget(550,40,true, brakeType::brake, 0.3, 0.01,0.3); 


  moveTarget_LR(330, 130, 100, brakeType::coast, 0.5, 0.1, 0.3);//355,255  
  chsSpin(2000, 4000);
  task::sleep(700);
  chsStops(brakeType::coast, 0.2);
  task::sleep(300);
  chsStops(brakeType::hold, 0.2);

  //turnTarget(-20, 100, vex::brakeType::brake, 5, 0.01, 0.3);  

  //moveTarget(100,70,true, brakeType::coast, 0.3, 0.01,0.3);  
  //moveTarget_LR(400, 200, 40, brakeType::brake, 0.3, 0.01, 0.3);//355,295
  motorSpin(push, vex::directionType::fwd, 100,2.2);
  vexDelay(1300);
  motorStop(push, brakeType::brake, 0.1);
  vexDelay(300);
  moveTarget(-300,70,true, brakeType::brake, 0.5, 0.01,0.3);
  
}

#endif