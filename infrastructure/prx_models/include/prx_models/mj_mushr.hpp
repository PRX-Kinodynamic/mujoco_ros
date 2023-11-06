#pragma once

#include "prx_models/mj_copy.hpp"

#include <prx_models/MushrControl.h>
#include <prx_models/MushrPlan.h>
#include <prx_models/MushrObservation.h>
#include <prx_models/MushrFeedback.h>

namespace prx_models
{

struct mushr_t
{
  static constexpr std::size_t u_dim{ 2 };
  struct sensors_t
  {
    static constexpr std::size_t PosX{ 0 };
    static constexpr std::size_t PosY{ 1 };
    static constexpr std::size_t PosZ{ 2 };
    static constexpr std::size_t QuatW{ 3 };
    static constexpr std::size_t QuatX{ 4 };
    static constexpr std::size_t QuatY{ 5 };
    static constexpr std::size_t QuatZ{ 6 };
  };
};

template <typename Ctrl>
inline void copy(Ctrl ctrl_out, const prx_models::MushrControl& msg)
{
  ctrl_out[0] = msg.steering_angle.data;
  ctrl_out[1] = msg.velocity.data;
}

template <typename Sensor>
inline void get_observation(prx_models::MushrObservation& msg, const Sensor& sensordata)
{
  msg.pose.position.x = sensordata[mushr_t::sensors_t::PosX];
  msg.pose.position.y = sensordata[mushr_t::sensors_t::PosY];
  msg.pose.position.z = sensordata[mushr_t::sensors_t::PosZ];
  msg.pose.orientation.x = sensordata[mushr_t::sensors_t::QuatX];
  msg.pose.orientation.y = sensordata[mushr_t::sensors_t::QuatY];
  msg.pose.orientation.z = sensordata[mushr_t::sensors_t::QuatZ];
  msg.pose.orientation.w = sensordata[mushr_t::sensors_t::QuatW];
}

}  // namespace prx_models