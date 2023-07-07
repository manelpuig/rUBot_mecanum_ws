#ifndef _CONFIG_H_
#define _CONFIG_H_
//configuracion motores
//#define PWMD 12    //  F_RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA
#define PWMD 5    //  F_RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA
#define PWMC 9    //  F_RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA
#define PWMB 8    //  F_RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA
#define PWMA 12    //  F_RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA

#define DIRD1 A4  //Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define DIRD2 A5   //Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)

#define DIRC1 43  //Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define DIRC2 42   //Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)

#define DIRB1 37  //Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define DIRB2 36   //Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)

#define DIRA1 34  //Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define DIRA2 35   //Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)



//#define DIRD1 34  //Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
//#define DIRD2 35   //Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)
//#define PWMC 8    // F_Left WHEEL PWM pin D7 connect front MODEL-X ENB
//#define DIRC1 37  //Left front Motor direction pin 1 to Front MODEL-X IN3 (  K3)
//#define DIRC2 36  //Left front Motor direction pin 2 to Front MODEL-X IN4 (  K3)
//#define PWMB 9   //RIGHT WHEEL PWM pin connect Back MODEL-X ENA
//#define DIRB1 43 //Rear Right Motor direction pin 1 to Back MODEL-X IN1 (  K1)
//#define DIRB2 42   //Rear Right Motor direction pin 2 to Back MODEL-X IN2 (  K1)
//#define PWMA 5     //LEFT WHEEL  PWM pin D8 connect Rear MODEL-X ENB
//#define DIRA1 A4   //Rear left Motor direction pin 1 to Back MODEL-X IN3  K3
//#define DIRA2 A5   //Rear left Motor direction pin 2 to Back MODEL-X IN4  k3

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
