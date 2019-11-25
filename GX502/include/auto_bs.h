#ifndef BS_H
#define BS_H

#include "vex.h"
#include "automove.h"
#include "utils.h"
#include "ctrls.h"
int cubeposition()
{
  handsSpin(vex::directionType::fwd, 80, 1.6);
  vex::task::sleep(700);
  handsStop(vex::brakeType::hold, 0.1);
  handsSpin(vex::directionType::rev, 60, 1.6);
  vex::task::sleep(300);
  handsStop(vex::brakeType::hold, 0.1);
  return 0;
}

int PushInAuto() // > 3s
{
  motorSpin(push,vex::directionType::rev,40,2.2);
  vex::task::sleep(300);
  motorStop(push,brakeType::hold,0.2);
  vex::task::sleep(1000);
  push.resetRotation();
  double t = Brain.timer(vex::timeUnits::msec);
  
  while (push_flag && (Brain.timer(vex::timeUnits::msec) - t) < 3500) { // 2-3s
    push_err = 790 - fabs(push.rotation(rotationUnits::deg));//target is 800 for 67677b
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
  vex::task::sleep(230);
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
void test()
{
  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());

  // moveTarget(300,80,false, vex::brakeType::brake,0.33,0.01,0.1); // +-1 error.
  // moveTarget(340,100,false, vex::brakeType::brake,0.3,0.01,0.1); // +-1 error. 90 deg
  // moveTarget(100,100,false, vex::brakeType::brake,2,0.01,0.1); // +-10 error. 40 deg
  // moveTarget(-100,100,false, vex::brakeType::brake,2,0.01,0.1); // +-10 error. 40 deg
  // moveTarget(-300,65,false,hold,0.3,0.01,1.2); //large angle pid

  // turnTarget(-105, 100, vex::brakeType::brake, 5, 0.1, 0.1); // 200 deg
  moveTarget_LR(600, 500, 100, brakeType::coast, 0.3, 0.01, 0.1);
}
void bs_new(){

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
  task::sleep(200);
  moveTarget(170, 100, true, vex::brakeType::coast, 0.3, 0.01, 0.3);
  moveTarget(540, 30, true, vex::brakeType::brake, 0.3, 0.01, 0.3);
  //turn left and collect 1 cube (optional)
  moveTarget(45,100,false,brake,3,0.01,0.2);
  moveTarget(140,100,true,hold,0.3,0.01,0.3);

  gyro_1.startCalibration();
  while(gyro_1.isCalibrating());

  handsStop(brakeType::hold,0.2);
  //moveTarget(-310,70,false,hold,0.2,0.01,3);
  turnTarget(-140, 100, vex::brakeType::brake, 5, 0.1, 0.1); // 200 deg
  task cube_position(cubeposition);
  vexDelay(500);
  //start pushing during moving towards scoring area   
  push_flag = true;
  push_hold = true;
  task Push_In_Auto(PushInAuto);
  chsSpin(6000, 6000);
  task::sleep(200);
  //moveTarget(250,100,true, vex::brakeType::coast, 0.3, 0.01, 0.3);
  //moveTarget(250,60,true, vex::brakeType::coast, 0.2, 0, 0.3);
  moveTarget_LR(335, 285, 100, brakeType::coast, 0.3, 0.01, 0.3);
  moveTarget_LR(325, 225, 60, brakeType::coast, 0.2, 0.01, 0.3);
  Brain.Screen.printAt(10, 50, "break");
  
  moveTarget(90,30,true, vex::brakeType::coast, 0.4, 0, 0.3);
  task::sleep(500);
  chsStops(brakeType::hold, 0.2);
  while(push_hold)
  {
    Brain.Screen.printAt(10, 50, "not break");

    task::sleep(50);
  }
  moveTarget(-300,60,true,brakeType::brake,0.01,0.01,0.4);
}

///////////////////////////////////////////////////////////////////////////////////

void auto_bs()
{
  /*
  push.spin(directionType ::fwd,50,velocityUnits::pct);
  push.setMaxTorque(1.0, currentUnits:: amp);
  
  arm.spin (directionType::fwd,100,vex::velocityUnits::pct);
  chsSpin(3000,3000);
  vexDelay(700);
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  push.spin(directionType ::fwd,-50,velocityUnits::pct);
  arm.spin(directionType::rev,100,vex::velocityUnits::pct);
  chsSpin(-4000,-4000);
  vexDelay(750);
  push.stop();
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  arm.stop(brakeType::hold);

  arm.setMaxTorque(0.2, currentUnits::amp);hand1.spin(directionType::fwd,100,vex::velocityUnits::pct);
  */
  vexDelay(2000);
  arm.spin(directionType::rev, 5, vex::velocityUnits::pct);
  arm.setMaxTorque(0.4, currentUnits::amp);
  push.stop(brakeType::hold);
  push.setMaxTorque(0.2, currentUnits::amp);
  hand1.spin(directionType::fwd, 100, vex::velocityUnits::pct);
  hand2.spin(directionType::rev, 100, vex::velocityUnits::pct);
  //collect frist 3 cubes
  automove(830, 830, 6000, 4000);

  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);

  hand1.setMaxTorque(0.1, currentUnits::amp);
  hand2.setMaxTorque(0.1, currentUnits::amp);
  //stop hand
  automove(190, -190, 2000, 14000);
  autoturn(-865, -865, 3000, 14000);
  //go back
  automove(-180, 180, 2000, 14000);
  //automove(-200,-200,2000,10000);

  //start to collect another four cubes
  hand1.setMaxTorque(2.4, currentUnits::amp);
  hand2.setMaxTorque(2.4, currentUnits::amp);
  hand1.spin(directionType::fwd, 100, vex::velocityUnits::pct);
  hand2.spin(directionType::rev, 100, vex::velocityUnits::pct);
  automove(1200, 1200, 7000, 3500);
  vexDelay(70);
  //dalay for get cube at right position
  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);
  hand1.setMaxTorque(0.1, currentUnits::amp);
  hand2.setMaxTorque(0.1, currentUnits::amp);
  automove(-705, -705, 2000, 14000);
  automove(-420, 420, 1000, 9900);
  automove(400, 400, 4000, 6000);

  push.setMaxTorque(2.4, currentUnits::amp);
  push.spin(vex::directionType::fwd, 43, vex::velocityUnits::pct);
  vexDelay(1740);
  push.stop(brakeType::coast);
  hand1.spin(directionType::rev, 100, vex::velocityUnits::pct);
  hand2.spin(directionType::fwd, 100, vex::velocityUnits::pct);
  vexDelay(100); //prevent hand stucking cubes

  automove(-200, -200, 1000, 14000);
  hand1.stop(vex::brakeType::hold);
  hand2.stop(vex::brakeType::hold);
  hand1.setMaxTorque(0.1, currentUnits::amp);
  hand2.setMaxTorque(0.1, currentUnits::amp);
}
#endif