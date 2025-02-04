## **Launch rUBot serial-node**

To bringup we need to run the driver designed for rubot_mecanum robot. The driver is in fact an arduino program that controls:

- The kinematics of the 4 mecanum wheels to apply the twist message in /cmd_vel topic
- The encoders to obtain the odometry
- Read the LIDAR and USB-Camera
- interface with all the other sensors/actuators connected to arduino-mega board

The "rUBot_drive_mpuig6.ino" arduino program is located on /Documentation/files/arduino/ folder

>Carefull!:
>
>You need to install Encoder.h lib: https://www.arduino.cc/reference/en/libraries/encoder/

This final code contains:
- Subscriber to /cmd_vel topic 
- Publisher to /odom topic

>Take care about:
>- Motor connections

![](./Images/02_rubot_rock/02b_motor.png)

>- Shield schematics

![](./Images/02_rubot_rock/03b_shield.png)

>- Pin number of encoders, PWM and DIR in config.h and encoder.h files

![](./Images/02_rubot_rock/04b_pinout.png)

>- Inverse Kinematics expressions in kinematics.hpp library according to:
![](./Images/02_rubot_rock/05b_mecanum_kine4.png)
The kinematics.hpp has to include the correct expressions:
```python
//wABCD m/s
void InverseKinematic(float vx,float vy,float omega, float &pwmA,float &pwmB,float &pwmC,float &pwmD){
  pwmA=speed2pwm((vx-vy-K*omega)); 
  pwmB=speed2pwm(vx+vy+K*omega); 
  pwmC=speed2pwm(vx+vy-K*omega);
  pwmD=speed2pwm(vx-vy+K*omega); 
```
>- The Odometry expression is calculated according to:
![](./Images/02_rubot_rock/06b_odom_mecanum.png)
> This is implemented in the main loop of arduino program:
```python
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
```

> Increase the /odom publisher buffer size:
> - We need to increase the buffer size of /odom publisher because the Arduino MEGA Buffer size for messages is 512bits (not enough for Odometry messages). To perform this modification, in **ROS.h** file from the Arduino library you have to add (at the end in else case section):
  ```python
  #else
    //typedef NodeHandle_<ArduinoHardware> NodeHandle; // default 25, 25, 512, 512
    typedef NodeHandle_<ArduinoHardware, 5, 5, 1024, 1024> NodeHandle;
  #endif  
  ```

> Increase the communication baudrate:
> - The default baudrate to communicate with Arduino board is 57600. I suggest to maintain the Baudrate to 57600!
>
>In some cases is necessary to increase it. To increase this baudrate you need in **ArduinoHardware.h** file from the Arduino >library to change this default baudrate:
```python
class ArduinoHardware {
  public:
    //ArduinoHardware(SERIAL_CLASS* io , long baud= 57600){
    ArduinoHardware(SERIAL_CLASS* io , long baud= 115200){
      iostream = io;
      baud_ = baud;
    }
    ArduinoHardware()
    {
#if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
      iostream = &Serial1;
#elif defined(USE_TEENSY_HW_SERIAL) or defined(USE_STM32_HW_SERIAL)
      iostream = &Serial1;
#else
      iostream = &Serial;
#endif
      //baud_ = 57600;
      baud_ = 115200;
    }
```
> Important!: This changes have to be made in the library files where Arduino is installed (/home/arduino/libraries). This can be found when in arduino IDLE we go to settings to see the Exemples folder.

To **test your rubot_mecanum arduino program** you need to:
- open arduino IDE
- upload the rubot_mecanum.ino file
- Open 3 new terminals and type:
```shell
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '[0.5, 0, 0]' '[0, 0, 0]'
```
> the port to which the Arduino is connected,is usually /dev/ttyACM0. Change it if you have another one.

> The last command sends a Twist message to the robot. The wheels should be moving forward. You can try different movements by modifying the numbers inside the brackets: '[vx, vy, vz]' '[wx, wy, wz]'

