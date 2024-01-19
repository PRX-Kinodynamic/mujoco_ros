#pragma once

#include <prx_models/mj_mushr.hpp>

namespace interface
{

inline void translate_msg(prx_models::MushrControl& ctrl_msg, const ml4kp_bridge::SpacePoint& point_msg)
{
  using prx_models::mushr_t::control::steering_idx;
  using prx_models::mushr_t::control::velocity_idx;
  ctrl_msg.steering_angle.data = point_msg.point[steering_idx].data;
  ctrl_msg.velocity.data = point_msg.point[velocity_idx].data;
}

inline void translate_msg(ackermann_msgs::AckermannDriveStamped& ctrl_msg, const ml4kp_bridge::SpacePoint& point_msg)
{
  using prx_models::mushr_t::control::steering_idx;
  using prx_models::mushr_t::control::velocity_idx;
  ctrl_msg.drive.steering_angle = point_msg.point[steering_idx].data;
  ctrl_msg.drive.speed = point_msg.point[velocity_idx].data;
}

inline void translate_msg(prx_models::MushrObservation& mushr_observation,
                          const interface::SensorDataStamped& observation)
{
  // TODO: This is a bit weird since we want it to be the same for real and sim.
  get_observation(mushr_observation, observation.raw_sensor_data);
}
}  // namespace interface