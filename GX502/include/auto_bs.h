#ifndef BS_H
#define BS_H

#include "vex.h"
#include "utils.h"
#include "ctrls.h"

int cube_position_bs()
{
  handsSpin(vex::directionType::fwd, 80, 1.6);
  vex::task::sleep(1000);
  handsStop(vex::brakeType::hold, 0.1);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(350);
  handsStop(vex::brakeType::hold, 0.1);
  return 0;
}

int push_in_auto_bs() // > 3s
{
  motorSpin(push,vex::directionType::rev,40,2.2);
  vex::task::sleep(300);
  motorStop(push,brakeType::hold,0.2);
  vex::task::sleep(700);
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
    else if (push_err < 180) // PID control
    {
      output = 20 + push_err * 0.065 - push_vlc * 0.3;
    } 
    else if (push_err < 360) // 60 - 40 inertance reducing
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

int start_hand_bs()
{
  vex::task::sleep(100);
  handsSpin(vex::directionType::fwd, 80, 1.6);

  return 0;
}

int start_arm_push_bs()
{
  motorSpin(push, vex::directionType::rev, 60, 2.2);
  motorSpin(arm, vex::directionType::rev, 60, 2.2);
  vex::task::sleep(300);
  motorStop(arm, brakeType::hold, 0.1);
  vex::task::sleep(225);
  motorStop(push, brakeType::coast, 0.1);

  return 0;
}

int push_up_bs()
{
  vex::task::sleep(600);
  motorSpin(push, vex::directionType::fwd, 80, 2.2);
  vex::task::sleep(400);
  motorSpin(push, vex::directionType::rev, 100, 2.2);
  vex::task::sleep(1000);
  motorStop(push, brakeType::hold, 0.1);
  return 0;
}

void auto_bs(){

  // hold the position of arm and push
  task ArmPushStart(start_arm_push_bs);
  moveTarget(220, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); // tar, max_pct, fwd_tur, bt, kp, kd, ki
  // start the robot and sprawl
  task HandStartBS(start_hand_bs);
  moveTarget(-240,100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); //back to reduce error
  // move forward get pre-loaded cube and 3 other cubes
  
  task PushUpBS(push_up_bs);
  moveTarget(870,35,true,vex::brakeType::brake,0.3, 0.01, 0.3);
  vex::task::sleep(500);  
  handsStop(brakeType::hold, 0.1);
  //turn right
  moveTarget(50,100,false, vex::brakeType::brake,2,0.01,0.1);
  //move back
  moveTarget(-600,100,true,vex::brakeType::brake,0.3, 0.01, 0.3); 
  //turn right
  
  //move back to reduce error
  
  //move forward and collect 4 cubes
  task ArmPushStartBS(start_arm_push_bs);
  handsSpin(fwd, 100, 2.2);  
  chsSpin(6000, 6000);
  task::sleep(200);//200
  moveTarget(170, 90, true, vex::brakeType::coast, 0.3, 0.01, 0.3);
  moveTarget(555, 25, true, vex::brakeType::brake, 0.3, 0.01, 0.3);
  //turn left and collect 1 cube (optional)
  moveTarget(45,90,false, brakeType::brake,3,0.01,0.2);
  moveTarget(160,90,true, brakeType::hold,0.3,0.01,0.3);
  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());

  handsStop(brakeType::hold,0.2);
  turnTarget(-140, 90, brakeType::brake, 5, 0.1, 0.1); // 200 deg
  task CubePosition(cube_position_bs);
  task::sleep(500);
  //start pushing during moving towards scoring area   
  push_flag = true;
  push_hold = true;
  task PushInAuto(push_in_auto_bs);
  // slow start moving
  chsSpin(6000, 6000);
  task::sleep(200);
  moveTarget_LR(390, 280, 100, brakeType::coast, 0.3, 0.01, 0.3);//355,295
  moveTarget_LR(330, 240, 60, brakeType::coast, 0.2, 0.01, 0.3);//355,255
  chsSpin(4000, 2000);
  task::sleep(500);
  chsSpin(2000, 4000);
  task::sleep(500);
  chsStops(brakeType::coast, 0.2);
  task::sleep(500);
  chsStops(brakeType::hold, 0.2);

  while(push_hold)
  {
    Brain.Screen.printAt(10, 50, "not break");

    task::sleep(50);
  }
  vexDelay(1000);
  moveTarget(-300, 60, true, brakeType::brake, 0.1, 0.01, 0.7);
}

void test()
{
  // moveTarget(300,80,false, vex::brakeType::brake,0.33,0.01,0.1); // +-1 error.
  // moveTarget(340,100,false, vex::brakeType::brake,0.3,0.01,0.1); // +-1 error. 90 deg
  // moveTarget(100,100,false, vex::brakeType::brake,2,0.01,0.1); // +-10 error. 40 deg
  // moveTarget(-100,100,false, vex::brakeType::brake,2,0.01,0.1); // +-10 error. 40 deg
  // moveTarget(-300,65,false,hold,0.3,0.01,1.2); //large angle pid

  // gyro_1.startCalibration();
  // while(gyro_1.isCalibrating());
  // turnTarget(-105, 100, vex::brakeType::brake, 5, 0.1, 0.1); // 200 deg

  // moveTarget_LR(600, 500, 100, brakeType::coast, 0.3, 0.01, 0.1);

  chsSpin(-6000, -6000);
  task::sleep(200);
  moveTarget_LR_PCT(-235,-140,100,brakeType::coast,2, 0.01, 0.1);
  moveTarget_LR_PCT(-120,-50,100,brakeType::coast,2, 0.01, 0.1);
  moveTarget(-45,80,true,brakeType::coast,5,0.01,0.3);
  moveTarget_LR_PCT(-50,-260,100,brakeType::coast,3, 0.01, 0.1);
  moveTarget_LR_PCT(-5,-300,100,brakeType::coast,3, 0.01, 0.1);
  moveTarget_LR_PCT(-1,-150,100,brakeType::brake,3, 0.01, 0.1);


  // moveTarget_LR(-223,-136,100,brakeType::coast,3, 0.3, 0.01, 0.1);
  // moveTarget_LR(-95,-45,100,brakeType::coast,3, 3, 0.01, 0.1);
  // moveTarget(-220,100,true,brakeType::coast,3,0.01,0.3);
  // moveTarget_LR(-100,-380,100,brakeType::coast,0.3, 3, 0.01, 0.1);
  // moveTarget_LR(-80,-150,100,brakeType::coast,0.5, 0.5, 0.01, 0.1);
  //chsSpin(-3000, -3000);
  task::sleep(300);
}


void bs_six(){
  // hold the position of arm and push
  task ArmPushStart(start_arm_push_bs);
  moveTarget(190, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); // tar, max_pct, fwd_tur, bt, kp, kd, ki
  // start the robot and sprawl
  task HandStart(start_hand_bs);
  moveTarget(-210,60, true, vex::brakeType::brake, 0.3, 0.01, 0.3); //back to reduce error
  // move forward get pre-loaded cube and 3 other cubes
  task PushUp(push_up_bs);
  vexDelay(1500);
  handsSpin(fwd, 100, 2.2);  
  chsSpin(6000, 6000);
  task::sleep(200);//200
  moveTarget(165, 100, true, vex::brakeType::coast, 0.3, 0.01, 0.3);
  moveTarget(555, 26, true, vex::brakeType::brake, 0.3, 0.01, 0.3);
  //turn left and collect 1 cube (optional)
  moveTarget(44,100,false, brakeType::brake,3,0.01,0.2);
  moveTarget(182,90,true, brakeType::hold,0.3,0.01,0.3);

  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());

  handsStop(brakeType::hold,0.2);
  //moveTarget(-310,70,false,hold,0.2,0.01,3);
  turnTarget(-150, 100, brakeType::brake, 5, 0.1, 0.1); // 200 deg
  task CubePosition(cube_position_bs);
  task::sleep(500);
  //start pushing during moving towards scoring area   
  push_flag = true;
  push_hold = true;
  task PushInAuto(push_in_auto_bs);
  // slow start moving
  chsSpin(6000, 6000);
  task::sleep(300);
  moveTarget_LR(570, 280, 90, brakeType::coast, 0.3, 0.01, 0.3);//355,295
  moveTarget_LR(340, 270, 60, brakeType::coast, 0.2, 0.01, 0.3);//355,255
  chsSpin(4000, 2000);
  task::sleep(500);
  chsSpin(2000, 4000);
  task::sleep(500);
  chsStops(brakeType::coast, 0.2);
  task::sleep(500);
  chsStops(brakeType::hold, 0.2);

  while(push_hold)
  {
    Brain.Screen.printAt(10, 50, "not break");

    task::sleep(50);
  }
  moveTarget(-300, 60, true, brakeType::brake, 0.05, 0.01, 0.7);
}

#endif