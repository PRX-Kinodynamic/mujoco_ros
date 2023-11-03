#pragma once

#include "prx_models/MushrControl.h"
#include "prx_models/MushrPlan.h"
#include "prx_models/mj_copy.hpp"

namespace prx_models
{

struct mushr_t
{
  static constexpr std::size_t u_dim{ 2 };
};

template <typename Ctrl>
inline void copy(Ctrl ctrl_out, const prx_models::MushrControl& msg)
{
  ctrl_out[0] = msg.steering_angle.data;
  ctrl_out[1] = msg.velocity.data;
}

}  // namespace prx_models