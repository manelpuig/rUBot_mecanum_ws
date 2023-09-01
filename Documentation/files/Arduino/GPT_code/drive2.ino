#include <ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Odometry message
nav_msgs::Odometry odom_msg;

// Wheelbase of the robot (distance between the left and right wheels)
const float wheelbase = 0.2; // Replace with the actual wheelbase of your robot

// Encoder pulses per revolution of the wheel
const int pulses_per_revolution = 360; // Replace with the actual value for your encoder

// Variables to store encoder counts
long left_encoder_count = 0;
long right_encoder_count = 0;

// Time variables for calculating velocity
unsigned long prev_time = 0;
unsigned long current_time = 0;

// Callback function for cmd_vel topic
void cmd_velCallback(const geometry_msgs::Twist &cmd_vel_msg) {
  // Read the linear and angular velocities from the cmd_vel message
  float linear_velocity = cmd_vel_msg.linear.x;
  float angular_velocity = cmd_vel_msg.angular.z;

  // Implement your odometry calculation here
  // Update left_encoder_count and right_encoder_count with the encoder values

  // Calculate time elapsed since the last update
  current_time = millis();
  float dt = (current_time - prev_time) / 1000.0; // Convert to seconds

  // Calculate linear and angular displacements
  float linear_distance = linear_velocity * dt;
  float angular_distance = angular_velocity * dt;

  // Calculate velocities
  float linear_velocity_avg = linear_velocity; // Replace with actual calculation based on encoder values
  float angular_velocity_avg = angular_velocity; // Replace with actual calculation based on encoder values

  // Update previous time
  prev_time = current_time;

  // Calculate the velocities in x and y direction of the robot (assumes only rotation and translation along the x-axis)
  float vx = linear_velocity_avg;
  float vy = 0.0;
  float vth = angular_velocity_avg;

  // Calculate the change in position and orientation using the velocities
  float delta_x = (vx * cos(odom_msg.pose.pose.orientation.z) - vy * sin(odom_msg.pose.pose.orientation.z)) * dt;
  float delta_y = (vx * sin(odom_msg.pose.pose.orientation.z) + vy * cos(odom_msg.pose.pose.orientation.z)) * dt;
  float delta_th = vth * dt;

  // Update the pose of the robot
  odom_msg.pose.pose.position.x += delta_x;
  odom_msg.pose.pose.position.y += delta_y;
  odom_msg.pose.pose.orientation.z += delta_th;

  // Update the twist (velocity) of the robot
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = vth;

  // Publish the odometry message
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.covariance[0] = 0.1; // Replace with actual covariance values
  odom_msg.pose.covariance[7] = 0.1; // Replace with actual covariance values
  odom_msg.pose.covariance[35] = 0.2; // Replace with actual covariance values
  odom_msg.twist.covariance[0] = 0.1; // Replace with actual covariance values
  odom_msg.twist.covariance[7] = 0.1; // Replace with actual covariance values
  odom_msg.twist.covariance[35] = 0.2; // Replace with actual covariance values

  odom_pub.publish(&odom_msg);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_velCallback);
ros::Publisher odom_pub("/odom", &odom_msg);

void setup() {
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
}

void loop() {
  nh.spinOnce();
}
