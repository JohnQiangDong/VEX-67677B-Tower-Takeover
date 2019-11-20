#ifndef AUTOMOVE_H
#define AUTOMOVE_H
#include "vex.h"
#include "utils.h"
#include "math.h"

void autostop(int i,int t){
  Brain.Screen.printAt(1,40,"autostopping");
  left_1.stop(vex::brakeType::hold);
  left_2.stop(vex::brakeType::hold);
  right_1.stop(vex::brakeType::hold);
  right_2.stop(vex::brakeType::hold);
  if(i==0){
      left_1.resetRotation();
      left_2.resetRotation();
      right_1.resetRotation();
      right_2.resetRotation();
  }
  vex::task::sleep(t);
}
void autostart(double left, double right,double speed){//left and right must smaller than 200
  double minV=1000,maxV=speed,left_error,right_error,olv,orv,startV=2500,K=40;
  double left_vot,right_vot,errorvelocity=0;
  double k,k1=0.02,instant_error;
  static double left_rotation,right_rotation;
  double l=0,r=0;
  //read the initial value,set the target
  //reset the timer
  double left_target,right_target;
  left_target=left_1.rotation(vex::rotationUnits::deg)+left;
  right_target=right_1.rotation(vex::rotationUnits::deg)+right;
  double t=Brain.timer(vex::timeUnits::msec);
  //speed up
  while(1){
      if(Brain.timer(vex::timeUnits::msec)-t>2000)break;
      left_error=left_target-left_1.rotation(vex::rotationUnits::deg);
      right_error=right_target-right_1.rotation(vex::rotationUnits::deg);  
      left_vot=(fabs(left-left_error)*K+startV)*left/fabs(left);
      right_vot=(fabs(right-right_error)*K+startV)*right/fabs(right);
      instant_error=fabs(left_error)-fabs(right_error);
      errorvelocity=1-fabs(instant_error)*0.02;
      if(errorvelocity<0.7)errorvelocity=0.7;
      if(instant_error>0)right_vot*=errorvelocity;
      else left_vot*=errorvelocity;

      if(left_vot>maxV)left_vot=maxV;
      if(left_vot<-maxV)left_vot=-maxV;
      if(right_vot>maxV)right_vot=maxV;
      if(right_vot<-maxV)right_vot=-maxV;
      mmove(left_vot,right_vot);
      if(left>=0&&left_error<=0)l=1;
      if(left<=0&&left_error>=0)l=1;
      if(right>=0&&right_error<=0)r=1;
      if(right<=0&&right_error>=0)r=1;
      if(l&&r)break;
  }
}

void autobrake(double left_target,double right_target,double timelimit,double speed){
  double left_Kp=30,right_Kp=30,Ki=0.0002,Kd=10,left_constantV=1000,right_constantV=1000,maxV=speed;
  double left_integral=0,left_error,left_last_error=0,left_D;
  double right_integral=0,right_error,right_last_error=0,right_D;
  double left_vot,right_vot,instant_error,k,k1=0.1;
  double t=Brain.timer(vex::timeUnits::msec),time=Brain.timer(vex::timeUnits::msec),frequency=0;
  left_error=left_target-left_1.rotation(vex::rotationUnits::deg);
  right_error=right_target-right_1.rotation(vex::rotationUnits::deg);

  while(1){
      if(Brain.timer(timeUnits::msec)-frequency>1){
        frequency=Brain.timer(timeUnits::msec);
        left_error=left_target-left_1.rotation(vex::rotationUnits::deg);
        right_error=right_target-right_1.rotation(vex::rotationUnits::deg);
      }
      if(Brain.timer(vex::timeUnits::msec)-time>timelimit)break;//2000
      Brain.Screen.drawRectangle(1,1,400,400,vex::color::yellow);        
      left_last_error=left_error;
      right_last_error=right_error;
      if(Brain.timer(timeUnits::msec)-frequency>1){
        frequency=Brain.timer(timeUnits::msec);
        left_error=left_target-left_1.rotation(vex::rotationUnits::deg);
        right_error=right_target-right_1.rotation(vex::rotationUnits::deg);
      }
      left_D=left_error-left_last_error;
      right_D=right_error-right_last_error;
      left_integral=left_integral+left_error;
      right_integral=right_integral+right_error;
      left_vot=left_Kp*left_error + Ki*left_integral+Kd*left_D;
      right_vot=right_Kp*right_error + Ki*right_integral+Kd*right_D;
      if(left_error>0)left_vot=left_vot+left_constantV;
      if(left_error<0)left_vot=left_vot-left_constantV;
      if(right_error>0)right_vot=right_vot+right_constantV;
      if(right_error<0)right_vot=right_vot-right_constantV;
      if(left_vot>maxV)left_vot=maxV;
      if(left_vot<-maxV)left_vot=-maxV;
      if(right_vot>maxV)right_vot=maxV;
      if(right_vot<-maxV)right_vot=-maxV;
      mmove(left_vot,right_vot);
      /*if(abs(left_error)<3&&abs(right_error)<3)Brain.Screen.drawRectangle(1,1,400,400,vex::color::red);//LED signal
      else Brain.Screen.drawRectangle(1,1,400,400,vex::color::green);*/
      //if(!left_constantV&&!right_constantV)break;
      if(abs(left_error)>1||abs(right_error)>1)t=Brain.timer(vex::timeUnits::msec);
      if(Brain.timer(vex::timeUnits::msec)-t>=250)break;
  }
  if(abs(left_error)<2&&abs(right_error)<2)Brain.Screen.drawRectangle(1,1,400,400,vex::color::red);
  
}
void autorun(double left_target,double right_target,double time,double speed){
  double left_error,right_error,left_vot,right_vot,instant_error,errorvelocity;
  double t=Brain.timer(vex::timeUnits::msec),distance=speed/35;
  while(1){
      if(Brain.timer(vex::timeUnits::msec)-t>time)break;
      left_error=left_target-left_1.rotation(vex::rotationUnits::deg);
      right_error=right_target-right_1.rotation(vex::rotationUnits::deg);
      if(fabs(left_error)<=distance||fabs(right_error)<=distance)break;
      //instant_error=abs(LeftMotor1.velocity(vex::velocityUnits::pct))-abs(RightMotor1.velocity(vex::velocityUnits::pct));
      instant_error=fabs(left_error)-fabs(right_error);
      errorvelocity=1-fabs(instant_error)*0.02;
      if(errorvelocity<0.7)errorvelocity=0.7;
      if(instant_error>0)right_vot*=errorvelocity;
      else left_vot*=errorvelocity;
      if(left_error>distance)left_vot=speed;
      if(left_error<-distance)left_vot=-speed;
      if(right_error>distance)right_vot=speed;
      if(right_error<-distance)right_vot=-speed;
      mmove(left_vot,right_vot);
  }
}
void automove(double left,double right,double time,double speed){
  double start_left,start_right,start_distance=speed/35,left_target,right_target;
  start_left=left/2;
  start_right=right/2;
  if(start_left>start_distance)start_left=start_distance;
  if(start_left<-start_distance)start_left=-start_distance;
  if(start_right>start_distance)start_right=start_distance;
  if(start_right<-start_distance)start_right=-start_distance;
  
  left_target=left_1.rotation(vex::rotationUnits::deg)+left;
  right_target=right_1.rotation(vex::rotationUnits::deg)+right;
  //autostart(start_left,start_right,speed);
  autorun(left_target,right_target,time,speed);
  autobrake(left_target,right_target,500,speed);
  //autostop(1,25);
}
void autoturn(double left,double right,double time,double speed){
  double start_left,start_right,start_distance=speed/35,left_target,right_target;
  start_left=left/2;
  start_right=right/2;
  if(start_left>start_distance)start_left=start_distance;
  if(start_left<-start_distance)start_left=-start_distance;
  if(start_right>start_distance)start_right=start_distance;
  if(start_right<-start_distance)start_right=-start_distance;
  
  left_target=left_1.rotation(vex::rotationUnits::deg)+left;
  right_target=right_1.rotation(vex::rotationUnits::deg)+right;
  //autostart(start_left,start_right,speed);
  
  autorun(left_target,right_target,time,speed);
  
  autobrake(left_target,right_target,500,speed);
  autostop(1,25);
}
#endif