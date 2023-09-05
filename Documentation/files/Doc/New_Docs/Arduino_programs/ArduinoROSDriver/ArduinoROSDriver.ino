
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include<std_msgs/Bool.h>


#include"encoder.h"
#include"kinematics.hpp"
#include"motor.h"
#include"pid.hpp"
#include"imu.hpp"

//#define HDW_DEBUG

#if !defined(HDW_DEBUG)
ros::NodeHandle_<ArduinoHardware, 5, 5, 512, 1024> nh;
//tf::TransformBroadcaster broadcaster;


#endif

float ctrlrate=1.0;
unsigned long lastctrl;
//geometry_msgs::TransformStamped t;
geometry_msgs::Twist twist;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

double x=0,y=0,theta=0;


float KP=0.3,KI=0.2,KD=0.2;
MPID PIDA(encA,KP,KI,KD,true);
MPID PIDB(encB,KP,KI,KD,false);
MPID PIDC(encC,KP,KI,KD,true);
MPID PIDD(encD,KP,KI,KD,false);

float wA,wB,wC,wD;

BMX055 Imu;

#if !defined(HDW_DEBUG)
void cmdVelCb( const geometry_msgs::Twist& twist_msg){
  float vx=twist_msg.linear.x;
  float vy=twist_msg.linear.y;
  float w=twist_msg.angular.z;

  float pwma=0,pwmb=0,pwmc=0,pwmd=0;
  InverseKinematic(vx,vy,w,pwma,pwmb,pwmc,pwmd);
  PIDA.tic();
  MotorA(PIDA.getPWM(pwma));
  wA=PIDA.getWheelRotatialSpeed();
  PIDA.toc();

  PIDB.tic();
  MotorB(PIDB.getPWM(pwmb));
  wB=PIDB.getWheelRotatialSpeed();
  PIDB.toc();

  PIDC.tic();
  MotorC(PIDC.getPWM(pwmc));
  wC=PIDC.getWheelRotatialSpeed();
  PIDC.toc();

  PIDD.tic();
  MotorD(PIDD.getPWM(pwmd));
  wD=PIDD.getWheelRotatialSpeed();
  PIDD.toc();

  lastctrl=millis();
}

void resetCb(const std_msgs::Bool& reset){
  if(reset.data){
    x=0.0;y=0.0;theta=0.0;
  }else{
    
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCb );
ros::Subscriber<std_msgs::Bool> resub("rest_odom", resetCb );
#endif

void setup()
{
  #if defined(HDW_DEBUG)
	Serial.begin(9600);
 #endif
	IO_init();
  PIDA.init();
  PIDB.init();
  PIDC.init();
  PIDD.init();
  Imu.SetupDevice();
  #if !defined(HDW_DEBUG)
  nh.initNode();
  //broadcaster.init(nh);
  nh.subscribe(sub);
  nh.subscribe(resub);
  nh.advertise(odom_pub);
  lastctrl=millis();
  #endif
}

void loop(){
  float ax,ay,az,gx,gy,gz;
  delay(10);

  float vxi=0,vyi=0,omegai=0;
  ForwardKinematic(wA,wB,wC,wD,vxi,vyi,omegai);

  float dt=PIDA.getDeltaT();
  x+=vxi*cos(theta)*dt-vyi*sin(theta)*dt;
  y+=vxi*sin(theta)*dt+vyi*cos(theta)*dt;
  theta+=omegai*dt;
  if(theta > 3.14)
    theta=-3.14;
//TODO IMU击穿了，货到后补上滤波器
  Imu.getGyro(gx,gy,gz);
  Imu.getAcc(ax,ay,az);
  #if defined(HDW_DEBUG)
  Serial.print("ACC:  x: ");
  Serial.print(ax);
  Serial.print(" y: ");
  Serial.print(ay);
  Serial.print(" z: ");
  Serial.println(az);

  Serial.print("GRYO:  x: ");
  Serial.print(gx);
  Serial.print(" y: ");
  Serial.print(gy);
  Serial.print(" z: ");
  Serial.println(gz);

  #endif

  #if !defined(HDW_DEBUG)

  //t.header.frame_id = "odom";
  //t.child_frame_id = "base_link";
  
  //t.transform.translation.x = x;
  //t.transform.translation.y = y;
  
  //t.transform.rotation = tf::createQuaternionFromYaw(theta);
  //t.header.stamp = nh.now();
  
  //broadcaster.sendTransform(t);

  
  odom.header.stamp = nh.now();;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation =tf::createQuaternionFromYaw(theta);;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vxi;
  odom.twist.twist.linear.y = vyi;
  odom.twist.twist.angular.z = omegai;

  odom_pub.publish(&odom);


  if((millis()-lastctrl)>1000*ctrlrate){
    STOP();
  }
  nh.spinOnce();
  #endif
 
}
