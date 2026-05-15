#pragma once

#include <ros/assert.h>

#include <geometry_msgs/TransformStamped.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>

#include <ml4kp_bridge/SpacePoint.h>
#include <ml4kp_bridge/msgs_utils.hpp>
#include "ml4kp_bridge/SpacePointStamped.h"
#include "ml4kp_bridge/product_lie_group.hpp"

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

template <typename Type>
inline void copy(Type& type, const ml4kp_bridge::SpacePointStamped& msg)
{
  copy(type, msg.space_point);
}

template <int Dim>
inline void copy(Eigen::Vector<double, Dim>& type, const ml4kp_bridge::SpacePoint& vector)
{
  for (int i = 0; i < type.size(); ++i)
  {
    type[i] = vector.point[i];
  }
}

template <typename G, typename H>
inline void copy(gtsam::ProductLieGroupV43<G, H>& type, const ml4kp_bridge::SpacePoint& vector)
{
  static constexpr Eigen::Index DimG{ gtsam::traits<G>::dimension };
  static constexpr Eigen::Index DimH{ gtsam::traits<H>::dimension };
  ml4kp_bridge::SpacePoint vG;
  vG.point = std::vector<double>(vector.point.begin(), vector.point.begin() + DimG);
  ml4kp_bridge::SpacePoint vH;
  vH.point = std::vector<double>(vector.point.begin() + DimG, vector.point.end());
  copy(type.first, vG);
  copy(type.second, vH);
}

inline void copy(gtsam::Rot2& type, const ml4kp_bridge::SpacePoint& vector)
{
  type = gtsam::Rot2(vector.point[0]);
}

inline void copy(gtsam::Pose2& type, const ml4kp_bridge::SpacePoint& vector)
{
  type = gtsam::Pose2(vector.point[0], vector.point[1], vector.point[2]);
}

inline void copy(double& type, const ml4kp_bridge::SpacePoint& vector)
{
  type = vector.point[0];
}

}  // namespace ml4kp_bridge