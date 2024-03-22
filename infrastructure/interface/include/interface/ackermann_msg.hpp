#pragma once

#include <prx_models/mj_mushr.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace interface
{

inline void translate_msg(ackermann_msgs::AckermannDriveStamped& ctrl_msg, const ml4kp_bridge::SpacePoint& point_msg)
{
  using prx_models::mushr_t::control::steering_idx;
  using prx_models::mushr_t::control::velocity_idx;
  ctrl_msg.drive.steering_angle = point_msg.point[steering_idx];
  ctrl_msg.drive.speed = point_msg.point[velocity_idx];
}
}  // namespace interface