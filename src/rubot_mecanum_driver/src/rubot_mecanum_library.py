import pigpio
import time as t
import rospy
from std_msgs.msg import Float32
import math


class Encoder:

   """Class to decode mechanical rotary encoder pulses."""

   def __init__(self, pi, gpioA, gpioB, callback):

      self.pi = pi
      self.gpioA = gpioA
      self.gpioB = gpioB
      self.callback = callback

      self.levA = 0
      self.levB = 0

      self.lastGpio = None

      self.pi.set_mode(gpioA, pigpio.INPUT)
      self.pi.set_mode(gpioB, pigpio.INPUT)

      self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
      self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level

      if gpio != self.lastGpio: # debounce
        self.lastGpio = gpio

        if   gpio == self.gpioA:
            if level == 1:
                if self.levB == 1:  self.callback(1)
                else:               self.callback(-1)
            else: # level == 0:
                if self.levB == 0:  self.callback(1)
                else:               self.callback(-1)
        else: # gpio == self.gpioB
            if level == 1:
                if self.levA == 1:  self.callback(-1)
                else:               self.callback(1)
            else: # level == 0:
                if self.levA == 0:  self.callback(-1)
                else:               self.callback(1)

   def cancel(self):

      self.cbA.cancel()
      self.cbB.cancel()


class MPID(object):
    def __init__(self,kp,ki,kd,revs,r,resolution,max_rpm):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.revs = revs
        self.r = r
        self.resolution = resolution
        self.max_rpm = max_rpm

        self.error = 0
        self.currentError = 0
        self.lastError = 0
        self.lastEnc = 0
        self.currentEnc = 0
        self.deltaEnc = 0
        self.deltaT = 0.1
        self.lastTime = 0
        self.lastEnc = 0
        
    def tic(self,position):
        self.currentTime = t.time()
        self.currentEnc = position
        self.currentEnc = -self.currentEnc if self.revs else self.currentEnc 

    def toc(self):
        self.lastTime = self.currentTime
        self.lastEnc = self.currentEnc

    def getWheelRotatialSpeed(self):
        return self.deltaEnc * self.resolution / self.deltaT / self.r
    
    def rpm2pwm(self,rpm):
        return int(rpm / self.max_rpm * 255)

    def speed2rpm(self,spd):
        return int(30.0 * spd / (self.r * math.pi))

    def speed2pwm(self,spd):
        return self.rpm2pwm(self.speed2rpm(spd))
    
    def set_pwm(self, given):
        self.deltaEnc = self.currentEnc - self.lastEnc
        #print(self.deltaEnc)
        self.deltaT = (self.currentTime - self.lastTime)
        #print(self.deltaT)
        mesPWM = self.speed2pwm(self.deltaEnc * self.resolution / self.deltaT)
        #print("PWM mesurat:" + str(mesPWM))
        self.error = given - mesPWM
        #print("Error: " + str(self.error))
        self.currentError += self.Ki * self.error
        if (self.currentError > 100):
            self.currentError = 100
        if given == 0 and self.error == 0:
            self.currentError = 0    
        s = self.Kp * self.error + self.currentError + self.Kd * (self.error - self.lastError)
        self.lastError = self.error
        return max(min(s, 200), -200)
    
    def get_deltaT(self):
        return self.deltaT



class DCMotorController():
    def __init__(self,pi,pwm,dir1,dir2,stby):
        # GPIO pin configuration
        self.enable_pin = pwm
        self.input1_pin = dir1
        self.input2_pin = dir2
        self.standby_pin = stby
        # Speed configuration
        self.frequency = 1000  # PWM frequency in Hz
        self.duty_cycle = 0  # Initial duty cycle (0-100)
       

        # Raspberry Pi setup

        pi.set_mode( self.enable_pin,pigpio.OUTPUT)
        pi.set_mode( self.input1_pin,pigpio.OUTPUT)
        pi.set_mode( self.input2_pin,pigpio.OUTPUT)
        pi.set_mode( self.standby_pin,pigpio.OUTPUT)

        pi.set_PWM_frequency( self.enable_pin,self.frequency)
        pi.set_PWM_range( self.enable_pin, 255)
        pi.set_PWM_dutycycle( self.enable_pin, 0)

    def speed(self,pi,speed):
        #If the speed given is > 1 the motor goes forward
        if(speed >= 1):
            pi.write(self.input1_pin, 0)
            pi.write(self.input2_pin, 1)
            pi.write(self.standby_pin, 1)
            pi.set_PWM_dutycycle( self.enable_pin,abs(speed))
            #rospy.loginfo('Motor Forward')
        #If the speed given is > 1 the motor goes backwards    
        elif(speed <= -1):
            pi.write(self.input1_pin, 1)
            pi.write(self.input2_pin, 0)
            pi.write(self.standby_pin, 1)
            pi.set_PWM_dutycycle( self.enable_pin,abs(speed))
            #rospy.loginfo('Motor Backwards')
        #If the speed given is 0 the motor stops 
        else:
            pi.write(self.input1_pin, 0)
            pi.write(self.input2_pin, 0)
            pi.write(self.standby_pin, 0)
            pi.set_PWM_dutycycle( self.enable_pin,0)
            #rospy.loginfo('Motor Stopped')


