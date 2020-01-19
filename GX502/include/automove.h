#ifndef AUTOMOVE_H
#define AUTOMOVE_H
#include "vex.h"
#include "move.h"
void autostop(int i,int t){//停止
  Brain.Screen.printAt(1,40,"autostopping");
  LeftMotor1.stop(vex::brakeType::coast);
  LeftMotor2.stop(vex::brakeType::coast);
  RightMotor1.stop(vex::brakeType::coast);
  RightMotor2.stop(vex::brakeType::coast);
  if(i==0){
      LeftMotor1.resetRotation();
      LeftMotor2.resetRotation();
      RightMotor1.resetRotation();
      RightMotor2.resetRotation();
  }
  vex::task::sleep(t);
}
void autostart(double left, double right,double speed){//left and right must smaller than 200
//自动开始
  double minV=1000,maxV=speed,left_error,right_error,olv,orv,startV=1000,K=40;
  double leftvelocity,rightvelocity,errorvelocity=0;
  double k,k1=0.02,instant_error;
  static double left_rotation,right_rotation;
  double l=0,r=0;
  //read the initial value,set the target
  //reset the timer
  double left_target,right_target;
  left_target=LeftMotor1.rotation(vex::rotationUnits::deg)+left;
  right_target=RightMotor1.rotation(vex::rotationUnits::deg)+right;
  double t=Brain.timer(vex::timeUnits::msec);
  //speed up
  while(1){
    //PID
      if(Brain.timer(vex::timeUnits::msec)-t>150)break;
      left_error=left_target-LeftMotor1.rotation(vex::rotationUnits::deg);
      right_error=right_target-RightMotor1.rotation(vex::rotationUnits::deg);  
      leftvelocity=(abs(left-left_error)*K+startV)*left/abs(left);
      rightvelocity=(abs(right-right_error)*K+startV)*right/abs(right);
      instant_error=abs(left_error)-abs(right_error);
      errorvelocity=1-abs(instant_error)*0.02;
      if(errorvelocity<0.7)errorvelocity=0.7;
      if(instant_error>0)rightvelocity*=errorvelocity;
      else leftvelocity*=errorvelocity;

      if(leftvelocity>maxV)leftvelocity=maxV;
      if(leftvelocity<-maxV)leftvelocity=-maxV;
      if(rightvelocity>maxV)rightvelocity=maxV;
      if(rightvelocity<-maxV)rightvelocity=-maxV;
      mmove(leftvelocity,rightvelocity);
      if(left>=0&&left_error<=0)l=1;
      if(left<=0&&left_error>=0)l=1;
      if(right>=0&&right_error<=0)r=1;
      if(right<=0&&right_error>=0)r=1;
      if(l&&r)break;
  }
}

void autobrake(double left_target,double right_target,double timelimit,double speed){
  //停止
  double left_Kp=30,right_Kp=30,Ki=0.0002,Kd=10,left_constantV=1000,right_constantV=500,maxV=speed;
  double left_integral=0,left_error,left_last_error=0,left_D;
  double right_integral=0,right_error,right_last_error=0,right_D;
  double leftvelocity,rightvelocity,instant_error,k,k1=0.1;
  double t=Brain.timer(vex::timeUnits::msec),time=Brain.timer(vex::timeUnits::msec),frequency=0;
  left_error=left_target-LeftMotor1.rotation(vex::rotationUnits::deg);
  right_error=right_target-RightMotor1.rotation(vex::rotationUnits::deg);

  while(1){
    //PID
      if(Brain.timer(timeUnits::msec)-frequency>1){
        frequency=Brain.timer(timeUnits::msec);
        left_error=left_target-LeftMotor1.rotation(vex::rotationUnits::deg);
        right_error=right_target-RightMotor1.rotation(vex::rotationUnits::deg);
      }
      if(Brain.timer(vex::timeUnits::msec)-time>timelimit)break;//2000
      Brain.Screen.drawRectangle(1,1,400,400,vex::color::yellow);        
      left_last_error=left_error;
      right_last_error=right_error;
      if(Brain.timer(timeUnits::msec)-frequency>1){
        frequency=Brain.timer(timeUnits::msec);
        left_error=left_target-LeftMotor1.rotation(vex::rotationUnits::deg);
        right_error=right_target-RightMotor1.rotation(vex::rotationUnits::deg);
      }
      left_D=left_error-left_last_error;
      right_D=right_error-right_last_error;
      left_integral=left_integral+left_error;
      right_integral=right_integral+right_error;
      leftvelocity=left_Kp*left_error + Ki*left_integral+Kd*left_D;
      rightvelocity=right_Kp*right_error + Ki*right_integral+Kd*right_D;
      if(left_error>0)leftvelocity=leftvelocity+left_constantV;
      if(left_error<0)leftvelocity=leftvelocity-left_constantV;
      if(right_error>0)rightvelocity=rightvelocity+right_constantV;
      if(right_error<0)rightvelocity=rightvelocity-right_constantV;
      if(leftvelocity>maxV)leftvelocity=maxV;
      if(leftvelocity<-maxV)leftvelocity=-maxV;
      if(rightvelocity>maxV)rightvelocity=maxV;
      if(rightvelocity<-maxV)rightvelocity=-maxV;
      mmove(leftvelocity,rightvelocity);
      /*if(abs(left_error)<3&&abs(right_error)<3)Brain.Screen.drawRectangle(1,1,400,400,vex::color::red);//LED signal
      else Brain.Screen.drawRectangle(1,1,400,400,vex::color::green);*/
      //if(!left_constantV&&!right_constantV)break;
      if(abs(left_error)>1||abs(right_error)>1)t=Brain.timer(vex::timeUnits::msec);
      if(Brain.timer(vex::timeUnits::msec)-t>=250)break;
  }
  if(abs(left_error)<2&&abs(right_error)<2)Brain.Screen.drawRectangle(1,1,400,400,vex::color::red);
  
}
void autorun(double left_target,double right_target,double time,double speed){
  double left_error,right_error,leftvelocity,rightvelocity,instant_error,errorvelocity;
  double t=Brain.timer(vex::timeUnits::msec),distance=speed/35;
  while(1){
      if(Brain.timer(vex::timeUnits::msec)-t>time)break;
      left_error=left_target-LeftMotor1.rotation(vex::rotationUnits::deg);
      right_error=right_target-RightMotor1.rotation(vex::rotationUnits::deg);
      if(abs(left_error)<=distance||abs(right_error)<=distance)break;
      //instant_error=abs(LeftMotor1.velocity(vex::velocityUnits::pct))-abs(RightMotor1.velocity(vex::velocityUnits::pct));
      instant_error=abs(left_error)-abs(right_error);
      errorvelocity=1-abs(instant_error)*0.02;
      if(errorvelocity<0.7)errorvelocity=0.7;
      if(instant_error>0)rightvelocity*=errorvelocity;
      else leftvelocity*=errorvelocity;
      if(left_error>distance)leftvelocity=speed;
      if(left_error<-distance)leftvelocity=-speed;
      if(right_error>distance)rightvelocity=speed;
      if(right_error<-distance)rightvelocity=-speed;
      mmove(leftvelocity,rightvelocity);
  }
}
void automove(double left,double right,double time,double speed){
  double start_left,start_right,start_distance=speed/35,left_target,right_target;
  start_left=left/3;
  start_right=right/3;
  if(start_left>start_distance)start_left=start_distance;
  if(start_left<-start_distance)start_left=-start_distance;
  if(start_right>start_distance)start_right=start_distance;
  if(start_right<-start_distance)start_right=-start_distance;
  
  left_target=(LeftMotor1.rotation(vex::rotationUnits::deg)+LeftMotor2.rotation(vex::rotationUnits::deg))/2+left;
  right_target=(RightMotor1.rotation(vex::rotationUnits::deg)+RightMotor2.rotation(vex::rotationUnits::deg))/2+right;
  autostart(start_left*2,start_right*2,speed);
  autorun(left_target,right_target,time,speed);
  autobrake(left_target,right_target,50,speed);
  //autostop(1,25);
}
void gyroturnleft(int fordeg,int v,double time){
 	int error_now = 0, error_last = 0, error_sum = 0, pidout = 0, value_now = 0;
	float kp = 0.8, ki = 0.00001, kd = 0.01;
  int aim;
  double t = Brain.timer(vex::timeUnits::msec);
  //int deg_now = Gyro1.heading(vex::rotationUnits::deg);
  //aim = deg_now - fordeg*1;
	//value_now = deg_now;
  aim = -fordeg*1;
	value_now = Gyro1.heading(vex::rotationUnits::deg);
  while(value_now>aim){
    if(Brain.timer(vex::timeUnits::msec)-t>time) break;
    for (int i = 0; i < 5; i++){
			value_now += Gyro1.heading(vex::rotationUnits::deg);
		  value_now /= 5;
    }
		value_now = Gyro1.heading(vex::rotationUnits::deg);
		Brain.Screen.printAt(1, 140, "the value_now is %d\n ", value_now);
		error_now = aim - value_now;
		error_sum += error_now;
    pidout = 100*(kp * error_now + ki * error_sum + kd * (error_now - error_last));
    if (pidout > v) 
		  pidout = v;
		else if (pidout < -v)
			pidout = -v;
    mmove(-v, v);
}

autostop(1, 20);

}
void gyroturnrigt(int fordeg,int v,double time){
 	int error_now = 0, error_last = 0, error_sum = 0, pidout = 0, value_now = 0;
	float kp = 1, ki = 0.00001, kd = 0.01;
  int aim;
  Gyro1.setHeading(0,rotationUnits::deg);
   double t=Brain.timer(vex::timeUnits::msec);
  	int deg_now = Gyro1.heading(vex::rotationUnits::deg);
    aim = deg_now+fordeg;
	value_now = deg_now;
while(value_now<aim){
  if(Brain.timer(vex::timeUnits::msec)-t>time)break;
  	value_now = 0;
    for (int i = 0; i < 5; i++){
			value_now += Gyro1.heading(vex::rotationUnits::deg);
		  value_now /= 5;
      }
		Brain.Screen.printAt(1, 140, "the value_now is %d\n ", value_now);
		error_now = value_now - aim;
		error_sum += error_now;
pidout=100*(kp * error_now + ki * error_sum + kd * (error_now - error_last));
    
     
		if (pidout > v) 
		pidout = v;
		else if (pidout < -v)
			pidout = -v;
       mmove(v, -v);
}

autostop(1, 20);
}






void gyroturn(int aim,int v,double time){
 	double error_now = 0, pidout = 0, value_now = 0;
	float kp = 900;
  double t = Brain.timer(vex::timeUnits::msec);
  Gyro1.setHeading(0,rotationUnits::deg);
	value_now = Gyro1.heading(vex::rotationUnits::deg);
  while(value_now - aim > 2.3 || value_now - aim < -2.3){
    if (abs(value_now) > abs(aim)) break;
    if (Brain.timer(vex::timeUnits::msec) - t > time) break;
    //--------------------------------------------------------
    value_now = 0;
    for (int i = 0; i < 5; i++){
			value_now += Gyro1.heading(vex::rotationUnits::deg);
    }
    value_now /= 5;
    //-------------------------------------------------------
		error_now = aim - value_now;
    pidout = kp * error_now;
    if (pidout > v) 
		  pidout = v;
		else if (pidout < -v)
			pidout = -v;
    mmove(pidout, -pidout);
  }
  LeftMotor1.stop(vex::brakeType::hold);
  LeftMotor2.stop(vex::brakeType::hold);
  RightMotor1.stop(vex::brakeType::hold);
  RightMotor2.stop(vex::brakeType::hold);
}


#endif