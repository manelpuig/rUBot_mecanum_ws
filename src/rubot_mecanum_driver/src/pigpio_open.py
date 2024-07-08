import pigpio
import time as t

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
        self.max_speed = 6.28  # Maximum speed in rad/s (adjust as necessary)

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
            pi.write(self.input1_pin, 1)
            pi.write(self.input2_pin, 0)
            pi.write(self.standby_pin, 1)
            pi.set_PWM_dutycycle( self.enable_pin, speed)
            #rospy.loginfo('Motor Forward')
        #If the speed given is > 1 the motor goes backwards    
        elif(speed <= -1):
            pi.write(self.input1_pin, 0)
            pi.write(self.input2_pin, 1)
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

pi = pigpio.pi()

MotorA = DCMotorController(pi,19, 4, 17, 10)

MotorB = DCMotorController(pi,13, 27, 22, 10)

MotorA.speed(pi,50)
MotorB.speed(pi,50)
t.sleep(5)

MotorA.speed(pi,70)
MotorB.speed(pi,70)

t.sleep(5)

MotorA.speed(pi,0)
MotorB.speed(pi,0)

