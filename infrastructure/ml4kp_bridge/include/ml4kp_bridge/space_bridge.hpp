#include <prx/utilities/spaces/space.hpp>
#include <ml4kp_bridge/SpacePoint.h>

namespace ml4kp_bridge
{
inline void copy(ml4kp_bridge::SpacePoint& msg, const prx::space_snapshot_t& state)
{
  msg.state.resize(state.size());
  for (std::size_t i = 0; i < msg.state.size(); ++i)
  {
    msg.state[i].data = state[i];
  }
}

inline void copy(prx::space_snapshot_t& state, const ml4kp_bridge::SpacePoint& msg)
{
  ROS_ASSERT(state.size() == msg.state.size());
  for (std::size_t i = 0; i < msg.state.size(); ++i)
  {
    state[i] = msg.state[i].data;
  }
}

inline ml4kp_bridge::SpacePoint create(const prx::space_snapshot_t& state)
{
  ml4kp_bridge::SpacePoint msg;
  msg.state.resize(state.size());
  for (std::size_t i = 0; i < msg.state.size(); ++i)
  {
    msg.state[i].data = state[i];
  }
  return msg;
}

}  // namespace ml4kp_bridge