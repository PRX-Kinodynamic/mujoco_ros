#pragma once

#include <ros/assert.h>
#include <prx/utilities/spaces/space.hpp>
#include <ml4kp_bridge/SpacePointStamped.h>
#include <visualization_msgs/Marker.h>

namespace ml4kp_bridge
{

inline prx::param_loader create(const ml4kp_bridge::SpacePoint msg)
{
  prx::param_loader params;
  params.set(msg.point);
  return params;
}
inline prx::param_loader create(const ml4kp_bridge::SpacePointStamped msg)
{
  return create(msg.space_point);
}

inline void copy(ml4kp_bridge::SpacePoint& msg, const prx::param_loader params)
{
  const std::vector<double> values{ params.as<std::vector<double>>() };
  msg.point.resize(values.size());
  for (std::size_t i = 0; i < msg.point.size(); ++i)
  {
    msg.point[i] = values[i];
  }
}

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
  prx_assert(state.size() == msg.point.size(),
             "[space_bridge::copy] mismatch sizes:\nTo " << state << "\nFrom:" << msg);
  for (std::size_t i = 0; i < msg.point.size(); ++i)
  {
    state[i] = msg.point[i];
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
    msg.point[i] = state[i];
  }
}

inline void to_file(const ml4kp_bridge::SpacePoint& msg, std::ofstream& ofs)
{
  for (auto value : msg.point)
  {
    ofs << value << " ";
  }
  // ofs << "\n";
}

inline void to_file(const ml4kp_bridge::SpacePointStamped& msg, std::ofstream& ofs)
{
  to_file(msg.header, ofs);
  to_file(msg.space_point, ofs);
}

}  // namespace ml4kp_bridge