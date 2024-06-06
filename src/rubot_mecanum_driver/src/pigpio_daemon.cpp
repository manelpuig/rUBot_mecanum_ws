#include <cstdlib>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pigpiod");

    int ret = system("~/rUBot_mecanum_ws/src/rubot_mecanum_driver/launch/pigpio_launch.sh");

    if (ret == 0) {
        ROS_INFO("El script se ha ejecutado exitosamente.");
    } else {
        ROS_ERROR("Ha ocurrido un error al ejecutar el script.");
        std::cerr << "Error: " << ret << std::endl;
    }

    return 0;
}

