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

inline void translate_msg(prx_models::MushrPlan& mushr_plan, const ml4kp_bridge::Plan& plan)
{
  const unsigned plan_size(plan.steps.size());
  const std::size_t u_dim{ prx_models::mushr_t::u_dim };
  mushr_plan.controls.resize(plan_size);
  mushr_plan.durations.resize(plan_size);
  for (int i = 0; i < plan_size; ++i)
  {
    const ml4kp_bridge::PlanStep plan_step{ plan.steps[i] };
    // copy(mushr_plan.controls[i], plan_step.control.state);
    mushr_plan.controls[i].velocity.data = plan_step.control.point[0].data;
    mushr_plan.controls[i].steering_angle.data = plan_step.control.point[1].data;
    mushr_plan.durations[i] = plan.steps[i].duration;
  }
}

inline void translate_msg(prx_models::MushrObservation& mushr_observation,
                          const interface::SensorDataStamped& observation)
{
  get_observation(mushr_observation, observation.raw_sensor_data);
}
}  // namespace interface