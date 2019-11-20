#ifndef STATUS_H
#define STATUS_H
#include "vex.h"


int status() {
  double lv,rv,lr,rr,k=0.99;
  
  while(1) 
  {
    lv=left_1.velocity(vex::velocityUnits::pct);
    rv=right_1.velocity(vex::velocityUnits::pct);
    lr=left_1.rotation(vex::rotationUnits::deg);
    rr=right_1.rotation(vex::rotationUnits::deg);             
    //Brain.Screen.printAt(  10,  40, "leftVelocity %3.2f", push.rotation(vex::rotationUnits::deg) );
    //Brain.Screen.printAt(  10,  60, "rightVelocity %3.2f", rv );
    Brain.Screen.printAt(  10,  80, "leftrotation %5f", lr);
    Brain.Screen.printAt(  10, 100, "rightrotation %5f", rr);
    //Brain.Screen.printAt(  40, 40, "L-LIMIT %5f",Limit_left.value());
  }
}
#endif