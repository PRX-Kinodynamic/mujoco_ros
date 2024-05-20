#pragma once

#include <filesystem>

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
    msg.point[i] = state[i];
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
    state[i] = msg.point[i];
  }
}

inline void copy(prx::space_snapshot_t& state, const ml4kp_bridge::SpacePointStamped& msg)
{
  copy(state, msg.space_point);
}

inline void to_file(const ml4kp_bridge::SpacePoint& msg, std::ofstream& ofs)
{
  for (auto value : msg.point)
  {
    ofs << value << " ";
  }
  ofs << "\n";
}

inline void to_file(const ml4kp_bridge::SpacePointStamped& msg, std::ofstream& ofs)
{
  to_file(msg.header, ofs);
  to_file(msg.space_point, ofs);
}

inline void copy(ml4kp_bridge::SpacePoint& state_out, const ml4kp_bridge::SpacePoint& state_in)
{
  state_out.point.resize(state_in.point.size());
  for (std::size_t i = 0; i < state_in.point.size(); ++i)
  {
    state_out.point[i] = state_in.point[i];
  }
}
}  // namespace ml4kp_bridge