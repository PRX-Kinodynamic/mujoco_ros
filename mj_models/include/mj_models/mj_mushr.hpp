#pragma once

#include "mj_models/MushrControl.h"
#include "mj_models/MushrPlan.h"
#include "mj_models/mj_copy.hpp"

namespace mj_models
{

struct mushr_t
{
  static constexpr std::size_t u_dim{ 2 };
};

// inline void copy(Ctrl ctrl_out, const mj_models::MushrControl::ConstPtr msg)
template <typename Ctrl>
inline void copy(Ctrl ctrl_out, const mj_models::MushrControl& msg)
{
  ctrl_out[0] = msg.steering_angle.data;
  ctrl_out[1] = msg.velocity.data;
}

}  // namespace mj_models