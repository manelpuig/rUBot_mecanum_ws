
#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H

namespace ros { class NodeHandle; }

namespace teleop_twist_joy
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class TeleopTwistJoy
{
public:
  TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace teleop_twist_joy

#endif  // TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
