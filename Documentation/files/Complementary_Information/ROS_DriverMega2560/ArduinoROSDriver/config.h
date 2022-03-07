#ifndef _CONFIG_H_
#define _CONFIG_H_
//电机引脚
#define PWMD 12    //D电机转速
#define DIRD1 34 
#define DIRD2 35  //D电机方向
#define PWMC 8    //C电机转速
#define DIRC1 37 
#define DIRC2 36  //C电机方向
#define PWMB 9   //B电机转速
#define DIRB1 43 
#define DIRB2 42  //B电机方向
#define PWMA 5    //A电机转速
#define DIRA1 A4   //26  
#define DIRA2 A5    //27  //A电机方向

//in our configuration
//   (A)//   x+     \\(B)
//   
//   y+
//
//   (C)\\          //(D)

float xn=0.08725, yn=0.105, r=0.03, K=abs(xn)+abs(yn); //m
int max_rmp=330;
float resolution=2*PI*r/1440.0;

#define constrain(amt,low,high) \
  ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))



#endif
