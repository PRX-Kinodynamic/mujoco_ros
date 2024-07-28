#pragma once

#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_copy.hpp>

namespace prx_models
{

template <typename Ctrl>
inline void copy(Ctrl ctrl_out, const ml4kp_bridge::SpacePoint& msg)
{
  for (std::size_t i = 0; i < msg.point.size(); ++i)
  {
    ctrl_out[i] = msg.point[i];
  }
}

template <typename Ctrl>
inline void copy(Ctrl ctrl_out, const ml4kp_bridge::SpacePointStamped& msg)
{
  copy(ctrl_out, msg.space_point);
}

}  // namespace prx_models