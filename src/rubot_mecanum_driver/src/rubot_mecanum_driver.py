#!/usr/bin/env python3


import pigpio
import time as t
import rospy
import math
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
import threading
import tf2_ros
from library2 import Encoder, MPID, DCMotorController 

class rubot_drive():
    def __init__(self):
        # Constant Parameter definition
        self.xn = 0.082 
        self.yn = 0.105 
        self.r = 0.03 # Otro robot 0.04
        self.K = abs(self.xn) + abs(self.yn)
        self.max_rmp = 330
        self.resolution = 2 * math.pi * self.r / 1440 # 1028.6
        
        self.vx = 0
        self.vy = 0
        self.w = 0
        self.theta = 0
        self.num = 0
        self.x = 0
        self.y = 0
        self.vxi = 0
        self.vyi = 0
        self.wi = 0
        self.sample = 4
        
        # Motor Variables and Config
        self.kp = 0.1 #0.1
        self.kd = 0.1 #0.1
        self.ki = 0.4 #0.45
        
        #Encoder position variables 
        self.position1 = 0
        self.position2 = 0
        self.position3 = 0
        self.position4 = 0

        # Initiate pigpio
        self.pi = pigpio.pi()
        
        # Motor1 -----FRONT-LEFT Wheel--------
        self.EnA = Encoder(self.pi,21, 26,self.callback1) #21,20
        self.PIDA = MPID(self.kp, self.ki, self.kd, False,self.r,self.resolution,self.max_rmp)
        self.MotorA = DCMotorController(self.pi,18, 11, 9, 10)#18, 5, 11, 10
     
        # Motor2 -----FRONT-RIGHT Wheel--------
        self.EnB = Encoder(self.pi,20, 16,self.callback2) #8 i 16
        self.PIDB = MPID(self.kp, self.ki, self.kd, False,self.r,self.resolution,self.max_rmp)
        self.MotorB = DCMotorController(self.pi,12,5,6, 10) #12,6,26, 10
        
        # Motor3 -----BACK-LEFT Wheel--------
        self.EnC = Encoder(self.pi,23,15,self.callback3)
        self.PIDC = MPID(self.kp, self.ki, self.kd, False,self.r,self.resolution,self.max_rmp)
        self.MotorC = DCMotorController(self.pi,19, 4, 17, 10)
        
  
        # Motor4 -----BACK-RIGHT Wheel--------
        self.EnD = Encoder(self.pi,25, 24,self.callback4)
        self.PIDD = MPID(self.kp, self.ki, self.kd, False,self.r,self.resolution,self.max_rmp)
        self.MotorD = DCMotorController(self.pi,13, 27, 22, 10)
        
       
        # ROS-related variables,msg and node creation
        rospy.init_node('rubot_mecanum')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.speed_callback)
        #rospy.Subscriber('reset_odom', Bool, self.reset_odom_callback)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        
        #Time control variables
        self.ctrlrate = 1
        self.lastctrl = rospy.Time.now()
    

    def callback1(self,pos):
        self.position1 += pos
    def callback2(self,pos):
        self.position2 += pos
    def callback3(self,pos):
        self.position3 += pos
    def callback4(self,pos):
        self.position4 += pos
        
    def rpm2pwm(self, rpm):
        return int(rpm / self.max_rmp * 255)

    def speed2rpm(self, spd):
        return int( 30.0 * spd / (self.r * math.pi))

    def speed2pwm(self, spd):
        return self.rpm2pwm(self.speed2rpm(spd))

    def InverseKinematic(self, vx, vy, omega):
        pwmA = self.speed2pwm((vx - vy - self.K * omega))
        pwmB = self.speed2pwm((vx + vy + self.K * omega))
        pwmC = self.speed2pwm((vx + vy - self.K * omega))
        pwmD = self.speed2pwm((vx - vy + self.K * omega))
        return pwmA, pwmB, pwmC, pwmD

    def ForwardKinematic(self, wA, wB, wC, wD):
        vx = self.r / 4 * (wA + wB + wC + wD)
        vy = self.r / 4 * (-wA + wB + wC - wD)
        omega = self.r / (4 * self.K) * (-wA + wB - wC + wD)
        return vx, vy, omega

    def speed_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.w = msg.angular.z
        self.lastctrl = rospy.Time.now()
    def shutdown(self):
        self.MotorA.speed(self.pi,0)
        self.MotorB.speed(self.pi,0)
        self.MotorC.speed(self.pi,0)
        self.MotorD.speed(self.pi,0)

    def pid_thread(self):
        rate = rospy.Rate(25)  # Frecuencia de actualización de 10 Hz
        self.duty1 = 0        
        self.duty2 = 0
        self.duty3 = 0 
        self.duty4 = 0 
        self.constant = 0    
        wa = 0
        wb = 0
        wc = 0
        wd = 0    
        while not rospy.is_shutdown():
        	
            self.pwma, self.pwmb, self.pwmc, self.pwmd = self.InverseKinematic(self.vx, self.vy, self.w)
            

            self.PIDA.tic(self.position1)
            self.duty1 = self.PIDA.set_pwm(self.pwma)
            self.MotorA.speed(self.pi,self.duty1)
            wa += self.PIDA.getWheelRotatialSpeed()
            self.PIDA.toc()
            
            
            
            self.PIDB.tic(self.position2)
            self.duty2 = self.PIDB.set_pwm(self.pwmb)
            self.MotorB.speed(self.pi,self.duty2)
            wb += self.PIDB.getWheelRotatialSpeed()
            self.PIDB.toc()
     
            
            
            self.PIDC.tic(self.position3)
            self.duty3 = self.PIDC.set_pwm(self.pwmc)
            self.MotorC.speed(self.pi,self.duty3)
            wc += (self.PIDC.getWheelRotatialSpeed())  
            self.PIDC.toc()
            
           
          
            self.PIDD.tic(self.position4)
            self.duty4 = self.PIDD.set_pwm(self.pwmd)
            self.MotorD.speed(self.pi,self.duty4)
            wd += self.PIDD.getWheelRotatialSpeed()
            self.PIDD.toc()
   

            if self.constant == self.sample:
                self.constant = 0
                self.vxi, self.vyi, self.wi = self.ForwardKinematic(wa/self.sample, wb/self.sample, wc/self.sample, wd/self.sample)
                """
                print("MotorA: " + str(wa/2))
                print("MotorB: " + str(wb/2))
                print("MotorC: " + str(wc/2))
                print("MotorD: " + str(wd/2))
                
                
                print("MotorA: " + str(wa/2))
                print("MotorB: " + str(wb/2))
                print("MotorC: " + str(wc/2))
                print("MotorD: " + str(wd/2))
                """

                # Update odometry and publish here
                dt = self.PIDA.get_deltaT()
                self.theta += self.wi * dt * self.sample
                if self.theta > 2 * math.pi:
                    self.theta -= 2 * math.pi
                elif self.theta < 0:
                    self.theta += 2 * math.pi
                self.x += self.vxi * math.cos(self.theta) * dt - self.vyi * math.sin(self.theta) * dt * self.sample
                self.y += self.vxi * math.sin(self.theta) * dt + self.vyi * math.cos(self.theta) * dt * self.sample
                if abs(self.wi) > 0.001:
                    self.num += 1
                wa = 0
                wb = 0
                wc = 0
                wd = 0
            self.constant += 1
            
            rate.sleep()
 
    def open_thread(self):
        rate = rospy.Rate(14)  # Frecuencia de actualización de 10 Hz
        self.duty1 = 0        
        self.duty2 = 0
        self.duty3 = 0 
        self.duty4 = 0  
        self.constant = 0  
        wa = 0
        wb = 0
        wc = 0
        wd = 0    
        while not rospy.is_shutdown():
        	
            self.pwma, self.pwmb, self.pwmc, self.pwmd = self.InverseKinematic(self.vx, self.vy, self.w)
    

            self.PIDA.tic(self.position1)
            self.duty1 = self.PIDA.set_pwm(self.pwma)
            self.MotorA.speed(self.pi,self.pwma)
            wa += self.PIDA.getWheelRotatialSpeed()
            self.PIDA.toc()

            
            self.PIDB.tic(self.position2)
            self.duty2 = self.PIDB.set_pwm(self.pwmb)
            self.MotorB.speed(self.pi,self.pwmb)
            wb += self.PIDB.getWheelRotatialSpeed()
            self.PIDB.toc()
            
            self.PIDC.tic(self.position3)
            self.duty3 = self.PIDC.set_pwm(self.pwmc)
            self.MotorC.speed(self.pi,self.pwmc)
            wc += self.PIDC.getWheelRotatialSpeed() 
            self.PIDC.toc()
          
           
          
            self.PIDD.tic(self.position4)
            self.duty4 = self.PIDD.set_pwm(self.pwmd)
            self.MotorD.speed(self.pi,self.pwmd)
            wd += self.PIDD.getWheelRotatialSpeed()
            self.PIDD.toc()

            if self.constant == 3:
                self.constant = 0
                self.vxi, self.vyi, self.wi = self.ForwardKinematic(wa/3, wb/3, wc/3, wd/3)
                print("MotorA dut: " + str(self.duty1))
                print("MotorB dut: " + str(self.duty2))
                print("MotorC dut: " + str(self.duty3))
                print("MotorD dut: " + str(self.duty4))
                
                print("WA: " + str(wa/3))
                print("WB: " + str(wb/3))
                print("WC: " + str(wc/3))
                print("WD: " + str(wd/3))
                
                print("PWMA: " + str(self.position1))
                print("PWMB: " + str(self.position2))
                print("PWMC: " + str(self.position3))
                print("PWMD: " + str(self.position4))

                # Update odometry and publish here
                dt = self.PIDA.get_deltaT()
                self.theta += self.wi * dt * 3
                if self.theta > 2 * math.pi:
                    self.theta -= 2 * math.pi
                elif self.theta < 0:
                    self.theta += 2 * math.pi
                self.x += self.vxi * math.cos(self.theta) * dt - self.vyi * math.sin(self.theta) * dt 
                self.y += self.vxi * math.sin(self.theta) * dt + self.vyi * math.cos(self.theta) * dt 
                if abs(self.wi) > 0.001:
                    self.num += 1
                wa = 0
                wb = 0
                wc = 0
                wd = 0
                
            self.constant += 1
            
            rate.sleep()
        
    def publishOdom_thread(self):
        rate = rospy.Rate(50)  # Frecuencia de actualización de 10 Hz

        while not rospy.is_shutdown():
            quaternion = quaternion_from_euler(0, 0, self.theta)
            #print("PWM applied: " + str(self.pwma))
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x 
            t.transform.translation.y = self.y 
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            #t.transform.rotation = quaternion_from_euler(0, 0, 0)
            self.broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = self.x * self.sample
            odom.pose.pose.position.y = self.y * self.sample
            #odom.pose.pose.orientation = quaternion_from_euler(0, 0, 0)
            odom.twist.twist.linear.x = self.vxi
            odom.twist.twist.linear.y = self.vyi
            odom.twist.twist.linear.z = self.num
            odom.twist.twist.angular.x = self.wi
            odom.twist.twist.angular.y = self.theta * 180 / math.pi
            odom.twist.twist.angular.z = self.w
            self.odom_pub.publish(odom)

            if (rospy.Time.now() - self.lastctrl).to_sec() > 5.0 / self.ctrlrate:
                self.shutdown()
                self.vx, self.vy, self.w = 0, 0, 0
                self.vxi, self.vyi, self.wi = 0, 0, 0    

            rate.sleep()

    def start(self):
        pid_thread = threading.Thread(target=self.pid_thread)
        publishOdom_thread = threading.Thread(target=self.publishOdom_thread)
        #open_thread = threading.Thread(target=self.open_thread)  
 
        pid_thread.start()
        publishOdom_thread.start()
        #open_thread.start()

        pid_thread.join()
        publishOdom_thread.join()
        #open_thread.join()

if __name__ == '__main__':
    try:
        rUBot1 = rubot_drive()
        rUBot1.start()
        rUBot1.shutdown()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rUBot1.shutdown()
        #rospy.spin()
        pass
