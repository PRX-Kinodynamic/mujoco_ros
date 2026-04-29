#pragma once

#include <ros/assert.h>

#include <prx/simulation/playback/plan.hpp>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/plan_step_bridge.hpp>
#include <prx/simulation/system_group.hpp>

#include <ml4kp_bridge/PlanStamped.h>

namespace ml4kp_bridge
{
using SystemGroupPtr = std::shared_ptr<prx::system_group_t>;

inline void propagate(const PlanStep& plan_step, SystemGroupPtr system_group)
{
  const double duration{ plan_step.duration.data.toSec() };
  const int steps{ static_cast<int>((duration / prx::simulation_step) + .1) };
  if (steps > 0)
  {
    system_group->propagate(steps, plan_step.control.point);
  }
}

inline void propagate(const SpacePoint x0, const Plan& plan, SpacePoint& result, SystemGroupPtr system_group)
{
  system_group->get_state_space()->copy_from(x0.point);

  for (auto&& step : plan.steps)
  {
    propagate(step, system_group);
  }
  system_group->get_state_space()->copy_to(result.point);
}

inline void propagate(const SpacePoint x0, const std::vector<ml4kp_bridge::PlanStepStamped>& plan, SpacePoint& result,
                      SystemGroupPtr system_group)
{
  system_group->get_state_space()->copy_from(x0.point);

  for (auto&& step : plan)
  {
    propagate(step.plan_step, system_group);
  }
  system_group->get_state_space()->copy_to(result.point);
}

inline void propagate(const SpacePointStamped x0, const std::vector<ml4kp_bridge::PlanStepStamped>& plan,
                      std::vector<SpacePointStamped>& result, SystemGroupPtr system_group)
{
  result.clear();
  const std::size_t state_dim{ x0.space_point.point.size() };

  system_group->get_state_space()->copy_from(x0.space_point.point);

  result.push_back(x0);
  for (auto&& step : plan)
  {
    const ros::Time t0{ result.back().header.stamp };
    propagate(step.plan_step, system_group);
    result.emplace_back();
    result.back().header.stamp = t0 + step.plan_step.duration.data;
    result.back().space_point.point.resize(state_dim);
    system_group->get_state_space()->copy_to(result.back().space_point.point);
  }
}

}  // namespace ml4kp_bridge