#pragma once

#include <prx_models/mj_mushr.hpp>

namespace interface
{

inline void translate_msg(prx_models::MushrControl& ctrl_msg, const ml4kp_bridge::SpacePoint& point_msg)
{
  using prx_models::mushr_t::control::steering_idx;
  using prx_models::mushr_t::control::velocity_idx;
  ctrl_msg.steering_angle.data = point_msg.point[steering_idx];
  ctrl_msg.velocity.data = point_msg.point[velocity_idx];
}

inline void translate_msg(ackermann_msgs::AckermannDriveStamped& ctrl_msg, const ml4kp_bridge::SpacePoint& point_msg)
{
  using prx_models::mushr_t::control::steering_idx;
  using prx_models::mushr_t::control::velocity_idx;
  ctrl_msg.drive.steering_angle = point_msg.point[steering_idx];
  ctrl_msg.drive.speed = point_msg.point[velocity_idx];
}

inline void translate_msg(prx_models::MushrPlan& mushr_plan, const ml4kp_bridge::PlanStamped& stamped_plan)
{
  const unsigned plan_size(stamped_plan.plan.steps.size());
  const std::size_t u_dim{ prx_models::mushr_t::u_dim };
  mushr_plan.controls.resize(plan_size);
  mushr_plan.durations.resize(plan_size);
  for (int i = 0; i < plan_size; ++i)
  {
    const ml4kp_bridge::PlanStep plan_step{ stamped_plan.plan.steps[i] };
    // copy(mushr_plan.controls[i], plan_step.control.state);
    mushr_plan.controls[i].velocity.data = plan_step.control.point[0];
    mushr_plan.controls[i].steering_angle.data = plan_step.control.point[1];
    mushr_plan.durations[i] = stamped_plan.plan.steps[i].duration;
  }
}

inline void translate_msg(prx_models::MushrObservation& mushr_observation,
                          const interface::SensorDataStamped& observation)
{
  // TODO: This is a bit weird since we want it to be the same for real and sim.
  get_observation(mushr_observation, observation.raw_sensor_data);
}
}  // namespace interface