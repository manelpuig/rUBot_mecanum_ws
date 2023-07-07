#ifndef _KINEMATICS_HPP_
#define _KINEMATICS_HPP_
#include"config.h"
int rpm2pwm(float rpm){
  return int(rpm/max_rmp*255);
}

//w=(rmp/60)*360*(PI/180)
//v=w*r
//rpm=30*v/(r*PI)
int speed2rpm(float spd){
  return 30.0*spd/(r*PI);
}

int speed2pwm(float spd){
  return(rpm2pwm(speed2rpm(spd)));
}


//wABCD m/s
void InverseKinematic(float vx,float vy,float omega, float &pwmA,float &pwmB,float &pwmC,float &pwmD){
  pwmA=speed2pwm((vx+vy-K*omega)); 
  pwmB=speed2pwm(vx-vy+K*omega); 
  pwmC=speed2pwm(vx-vy-K*omega);
  pwmD=speed2pwm(vx+vy+K*omega); 
//  SERIAL.print(pwmA);
//  SERIAL.print("  :   ");
//  SERIAL.print(pwmB);
//  SERIAL.print("  :   ");
//  SERIAL.print(pwmC);
//  SERIAL.print("  :   ");
//  SERIAL.println(pwmD);
}

void ForwardKinematic(float wA,float wB,float wC,float wD,float &vx,float &vy,float &w){
  vx=r/4.0*(wA+wB+wC+wD);
  vy=r/4.0*(+wA-wB-wC+wD);
  w=r/(4.0*K)*(-wA+wB-wC+wD);
}

#endif
