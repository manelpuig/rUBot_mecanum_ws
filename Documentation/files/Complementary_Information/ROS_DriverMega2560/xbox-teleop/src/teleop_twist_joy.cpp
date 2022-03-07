
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>


namespace teleop_twist_joy {

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
    struct TeleopTwistJoy::Impl {
        void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

        void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg);

        ros::Subscriber joy_sub;
        ros::Publisher cmd_vel_pub;
    };

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
    TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
        pimpl_ = new Impl;

        pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
    }

    void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg) {
        // Initializes with zeros by default.
        geometry_msgs::Twist cmd_vel_msg;

            cmd_vel_msg.linear.x = joy_msg->axes[1];
            cmd_vel_msg.linear.y = joy_msg->axes[3];
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.z = joy_msg->axes[0] * 2;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
       

    }

    void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
        sendCmdVelMsg(joy_msg);
    }  // namespace teleop_twist_joy
}
