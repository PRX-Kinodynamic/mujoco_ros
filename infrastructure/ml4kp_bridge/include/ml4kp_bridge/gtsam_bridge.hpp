#pragma once

#include <ros/assert.h>

#include <geometry_msgs/TransformStamped.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>

#include <ml4kp_bridge/msgs_utils.hpp>

namespace ml4kp_bridge
{
inline void copy(geometry_msgs::Transform& msg, const gtsam::Pose3& pose)
{
  msg.translation.x = pose.x();
  msg.translation.y = pose.y();
  msg.translation.z = pose.z();

  const gtsam::Quaternion quat{ pose.rotation().toQuaternion() };
  msg.rotation.w = quat.w();
  msg.rotation.x = quat.x();
  msg.rotation.y = quat.y();
  msg.rotation.z = quat.z();
}

inline void copy(geometry_msgs::TransformStamped& msg, const gtsam::Pose3& pose)
{
  msg.header.stamp = ros::Time::now();
  copy(msg.transform, pose);
}

}  // namespace ml4kp_bridge