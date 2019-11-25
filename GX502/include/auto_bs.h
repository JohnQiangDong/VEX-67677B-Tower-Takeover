#ifndef BS_H
#define BS_H

#include "vex.h"
#include "utils.h"
#include "ctrls.h"

int cube_position()
{
  handsSpin(vex::directionType::fwd, 80, 1.6);
  vex::task::sleep(1000);
  handsStop(vex::brakeType::hold, 0.1);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(280);
  handsStop(vex::brakeType::hold, 0.1);
  return 0;
}

int push_in_auto() // > 3s
{
  motorSpin(push,vex::directionType::rev,40,2.2);
  vex::task::sleep(300);
  motorStop(push,brakeType::hold,0.2);
  vex::task::sleep(600);
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

int start_hand()
{
  vex::task::sleep(200);
  handsSpin(vex::directionType::fwd, 80, 1.6);

  return 0;
}

int start_arm_push()
{
  motorSpin(push, vex::directionType::rev, 60, 2.2);
  motorSpin(arm, vex::directionType::rev, 60, 2.2);
  vex::task::sleep(300);
  motorStop(arm, brakeType::hold, 0.1);
  vex::task::sleep(225);
  motorStop(push, brakeType::coast, 0.1);

  return 0;
}

int push_up()
{
  motorSpin(push, vex::directionType::fwd, 80, 2.2);
  vex::task::sleep(400);
  motorSpin(push, vex::directionType::rev, 100, 2.2);
  vex::task::sleep(1000);
  motorStop(push, brakeType::hold, 0.1);
  return 0;
}

void auto_bs(){

  // hold the position of arm and push
  /*task ArmPushStart(start_arm_push);
  moveTarget(250, 100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); // tar, max_pct, fwd_tur, bt, kp, kd, ki
  // start the robot and sprawl
  task HandStart(start_hand);
  moveTarget(-290,100, true, vex::brakeType::brake, 0.3, 0.01, 0.3); //back to reduce error
  // move forward get pre-loaded cube and 3 other cubes
  task PushUp(push_up);
  moveTarget(870,35,true,vex::brakeType::brake,0.3, 0.01, 0.3);
  vex::task::sleep(500);  
  handsStop(brakeType::hold, 0.1);
  //turn right
  moveTarget(50,100,false, vex::brakeType::brake,2,0.01,0.1);
  //move back
  moveTarget(-600,100,true,vex::brakeType::brake,0.3, 0.01, 0.3); */ 
  //turn right
  
  //move back to reduce error
  
  //move forward and collect 4 cubes
  task ArmPushStart(start_arm_push);
  handsSpin(fwd, 100, 2.2);  
  chsSpin(6000, 6000);
  task::sleep(200);//200
  moveTarget(170, 100, true, vex::brakeType::coast, 0.3, 0.01, 0.3);
  moveTarget(555, 28, true, vex::brakeType::brake, 0.3, 0.01, 0.3);
  //turn left and collect 1 cube (optional)
  moveTarget(45,100,false, brakeType::brake,3,0.01,0.2);
  moveTarget(160,100,true, brakeType::hold,0.3,0.01,0.3);

  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());

  handsStop(brakeType::hold,0.2);
  //moveTarget(-310,70,false,hold,0.2,0.01,3);
  turnTarget(-140, 100, brakeType::brake, 5, 0.1, 0.1); // 200 deg
  task CubePosition(cube_position);
  task::sleep(500);
  //start pushing during moving towards scoring area   
  push_flag = true;
  push_hold = true;
  task PushInAuto(push_in_auto);
  // slow start moving
  chsSpin(6000, 6000);
  task::sleep(200);

  //moveTarget(250,100,true, vex::brakeType::coast, 0.3, 0.01, 0.3);
  //moveTarget(250,60,true, vex::brakeType::coast, 0.2, 0, 0.3);
  moveTarget_LR(390, 280, 100, brakeType::coast, 0.3, 0.01, 0.3);//355,295
  moveTarget_LR(350, 240, 60, brakeType::coast, 0.2, 0.01, 0.3);//355,255

  //moveTarget(90,30,true, vex::brakeType::coast, 0.4, 0, 0.3);
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
  moveTarget_LR_PCT(-223,-136,100,brakeType::coast,2, 0.01, 0.1);
  moveTarget_LR_PCT(-95,-45,100,brakeType::coast,2, 0.01, 0.1);
  moveTarget(-220,100,true,brakeType::coast,3,0.01,0.3);
  moveTarget_LR_PCT(-100,-380,100,brakeType::coast,2, 0.01, 0.1);
  moveTarget_LR_PCT(-80,-150,100,brakeType::coast,2, 0.01, 0.1);

  // moveTarget_LR(-223,-136,100,brakeType::coast,3, 0.3, 0.01, 0.1);
  // moveTarget_LR(-95,-45,100,brakeType::coast,3, 3, 0.01, 0.1);
  // moveTarget(-220,100,true,brakeType::coast,3,0.01,0.3);
  // moveTarget_LR(-100,-380,100,brakeType::coast,0.3, 3, 0.01, 0.1);
  // moveTarget_LR(-80,-150,100,brakeType::coast,0.5, 0.5, 0.01, 0.1);
  chsSpin(-3000, -3000);
  task::sleep(300);
}

#endif