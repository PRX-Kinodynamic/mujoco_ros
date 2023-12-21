#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/defs.h>
#include <ml4kp_bridge/plan_stepper.hpp>
#include <prx_models/mj_mushr.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace interface
{
struct copy_ackermann_t
{
  void operator()(ackermann_msgs::AckermannDriveStamped& msg, const ml4kp_bridge::SpacePoint& pt)
  {
    msg.drive.steering_angle = pt.point[0].data;
    msg.drive.speed = pt.point[1].data;
  }
};

using AckermannDrivePlanStepper = ml4kp_bridge::plan_stepper_t<ackermann_msgs::AckermannDriveStamped, copy_ackermann_t>;
}  // namespace interface
PLUGINLIB_EXPORT_CLASS(interface::AckermannDrivePlanStepper, nodelet::Nodelet);