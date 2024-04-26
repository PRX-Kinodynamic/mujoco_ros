#pragma once

#include <ros/assert.h>
#include <prx/simulation/playback/trajectory.hpp>

#include <ml4kp_bridge/msgs_utils.hpp>

// Msgs
#include <ml4kp_bridge/SpacePointStamped.h>
#include <ml4kp_bridge/TrajectoryStamped.h>

namespace ml4kp_bridge
{
inline void copy(ml4kp_bridge::Trajectory& msg, const prx::trajectory_t& trajectory)
{
  msg.data.resize(trajectory.size());

  for (unsigned i = 0; i < trajectory.size(); ++i)
  {
    copy(msg.data[i], trajectory[i]);
  }
}

inline void copy(ml4kp_bridge::TrajectoryStamped& msg, const prx::trajectory_t& trajectory)
{
  msg.header.seq++;
  msg.header.stamp = ros::Time::now();
  copy(msg.trajectory, trajectory);
}

inline void copy(prx::trajectory_t& trajectory, const ml4kp_bridge::Trajectory& msg)
{
  trajectory.resize(msg.data.size());
  for (unsigned i = 0; i < msg.data.size(); ++i)
  {
    copy(trajectory[i], msg.data[i]);
  }
}

inline void copy(prx::trajectory_t& trajectory, const ml4kp_bridge::TrajectoryStamped& msg)
{
  copy(trajectory, msg.trajectory);
}

inline void to_file(const ml4kp_bridge::Trajectory& msg, std::ofstream& ofs)
{
  for (auto state : msg.data)
  {
    to_file(state, ofs);
  }
  ofs << "\n";
}

}  // namespace ml4kp_bridge