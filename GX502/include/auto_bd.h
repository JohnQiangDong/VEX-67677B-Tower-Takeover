#ifndef BD_H
#define BD_H

#include "vex.h"
#include "utils.h"
#include "ctrls.h"

int cube_position_bd()
{
  vex::task::sleep(2800);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(250);
  handsStop(vex::brakeType::brake, 0.15);
  return 0;
}
int cube_prevent_stuck(){
  vex::task::sleep(1200);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(200);
  handsSpin(vex::directionType::fwd, 70, 1.6);

  return 0;
}
int push_in_auto_bd() // > 3s
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

int start_hand_bd()
{
  handsSpin(vex::directionType::fwd,55, 1.6);

  return 0;
}

int start_arm_push_bd()
{
  motorSpin(push, vex::directionType::rev, 60, 2.2);
  motorSpin(arm, vex::directionType::rev, 60, 2.2);
  vex::task::sleep(300);
  motorStop(arm, brakeType::hold, 0.1);
  vex::task::sleep(300);
  motorStop(push, brakeType::hold, 0.1);

  return 0;
}

int push_up_bd()
{
  task::sleep(1550);
  motorSpin(push, vex::directionType::fwd, 100, 2.2);
  vex::task::sleep(400);
  motorSpin(push, vex::directionType::rev, 100, 2.2);
  vex::task::sleep(650);
  motorStop(push, brakeType::hold, 0.1);
  return 0;
}

void auto_bd(){
  //sqrawl
  task StartArmPushBD(start_arm_push_bd);
  //moveTarget(250, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); // tar, max_pct, fwd_tur, bt, kp, kd, ki
  // start the robot and sprawl
  task HandStartBS(start_hand_bd);
  task PushUpBD(push_up_bd);
 // //moveTarget(-20,100, true, vex::brakeType::brake, 0.3, 0.01, 0.3);
  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());
  task::sleep(2000);//200
  //move forward 800
  chsSpin(6000, 6000);
  task::sleep(200);//200
  handsStop(brakeType::coast,0.1);
  moveTarget(465, 80, true, vex::brakeType::brake, 0.3, 0.01, 0.3);//467
  //turn right 35
  turnTarget(34,63,brake,4,0.3,0.5);//37
  //collecter start turn
  task StartHandBD(start_hand_bd);
  //slow accelerate forward
  moveTarget(200,60, true, brakeType::coast, 0.2, 0.03, 0.1);//0.1
  handsStop(brakeType::coast,0.1);
  task PushUpBD2(push_up_bd); 
  vexDelay(300);
  handsSpin(vex::directionType::fwd, 100,2.2);

  handsSpin(vex::directionType::fwd, 100,2.2);

  moveTarget_LR(190, 255, 60, brakeType::brake, 0.2, 0.01, 0.3);//355,255
  task CubePrevenStuck (cube_prevent_stuck);
  //turn right 135 
  turnTarget(103, 100, vex::brakeType::brake, 3, 0.3,0.1);//150
  push_flag = true;
  push_hold = true;
  task CubePositionBD(cube_position_bd);
 // task PushInAuToBD(push_in_auto_bd);
  moveTarget(550,40,true, brakeType::brake, 0.3, 0.01,0.3); 


  moveTarget_LR(130, 330, 100, brakeType::coast, 0.5, 0.1, 0.3);//355,255  
  chsSpin(3500, 1700);
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