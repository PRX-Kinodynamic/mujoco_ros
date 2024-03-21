#pragma once

#include <ros/assert.h>
#include <prx/utilities/spaces/space.hpp>
#include <ml4kp_bridge/SpacePointStamped.h>

namespace ml4kp_bridge
{
inline void copy(ml4kp_bridge::SpacePoint& msg, const prx::space_snapshot_t& state)
{
  msg.point.resize(state.size());
  for (std::size_t i = 0; i < msg.point.size(); ++i)
  {
    msg.point[i].data = state[i];
  }
}

inline void copy(ml4kp_bridge::SpacePointStamped& msg, const prx::space_snapshot_t& state)
{
  msg.header.seq++;
  msg.header.stamp = ros::Time::now();
  copy(msg.space_point, state);
}

inline void copy(prx::space_snapshot_t& state, const ml4kp_bridge::SpacePoint& msg)
{
  ROS_ASSERT(state.size() == msg.point.size());
  for (std::size_t i = 0; i < msg.point.size(); ++i)
  {
    state[i] = msg.point[i].data;
  }
}

inline void copy(prx::space_snapshot_t& state, const ml4kp_bridge::SpacePointStamped& msg)
{
  copy(state, msg.space_point);
}

inline void copy(ml4kp_bridge::SpacePoint& msg, const Eigen::VectorXd& state)
{
  msg.point.resize(state.size());
  for (std::size_t i = 0; i < msg.point.size(); ++i)
  {
    msg.point[i].data = state[i];
  }
}
}  // namespace ml4kp_bridge