### **Create a service to Bringup the robot on boot**

There are different methods:
- systemd
- rc.local
- others

Se documentation: 
- https://www.luisllamas.es/como-ejecutar-aplicacion-al-arranque-de-una-raspberry-pi/

## systemd method

You need to create a service rubot_bringup.service in the /etc/systemd/system/ directory.
````shell
sudo nano /etc/systemd/system/rubot_bringup.service
````
- Add the Service Configuration:
````shell
[Unit]
Description=ROS Bringup Service for Rubot Mecanum
After=network.target
Requires=network-online.target

[Service]
Type=simple
User=ubuntu  # Replace with your actual username
WorkingDirectory=/home/ubuntu/rUBot_mecanum_ws  # Replace with your catkin workspace path
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
````
- Enable and Start the Service
````shell
sudo systemctl enable rubot_bringup.service
sudo systemctl start rubot_bringup.service
````
- Check the Service Status:
````shell
sudo systemctl status rubot_bringup.service
````
- To delete the service:
````shell
sudo systemctl stop rubot_bringup
sudo systemctl disable rubot_bringup
sudo rm /etc/systemd/system/rubot_bringup.service
sudo systemctl daemon-reload
sudo systemctl is-enabled rubot_bringup
````

## rc.local method

- Create the file rc.local
````shell
sudo nano /etc/rc.local
````
- update the file contents:
````shell
#!/bin/bash
/bin/bash /home/ubuntu/rubot_bringup.sh &
exit 0

````
- make file executable:
````shell
sudo chmod +x /etc/rc.local
````
- Create the rubot_bringup.sh execution file:
````shell
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu/rUBot_mecanum_ws/devel/setup.bash
roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch
````
- Create the rc-local.service
````shell
sudo nano /etc/systemd/system/rc-local.service
````
- with the contents:
````shell
[Unit]
Description=/etc/rc.local Compatibility
After=network.target

[Service]
Type=forking
ExecStart=/etc/rc.local start
TimeoutSec=0
StandardOutput=tty
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
````
- you need to enable it and start it to ensure it runs on boot (verify the service status):
````shell
sudo systemctl enable rc-local
sudo systemctl start rc-local
sudo systemctl status rc-local
````
- Verify the ros processes
````shell
ps aux | grep ros
journalctl -u rc-local
````
- Kill process if necessary:
````shell
sudo lsof -i :11311
ps aux | grep ros
sudo kill -9 12977
````
