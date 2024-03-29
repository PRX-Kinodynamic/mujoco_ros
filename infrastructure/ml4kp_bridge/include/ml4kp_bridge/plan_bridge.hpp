#pragma once

#include <ros/assert.h>

#include <prx/simulation/playback/plan.hpp>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/plan_step_bridge.hpp>

#include <ml4kp_bridge/PlanStamped.h>

namespace ml4kp_bridge
{
inline void copy(ml4kp_bridge::Plan& msg, const prx::plan_t& plan)
{
  msg.steps.resize(plan.size());
  for (unsigned i = 0; i < plan.size(); ++i)
  {
    copy(msg.steps[i], plan[i]);
  }
}

inline void copy(ml4kp_bridge::PlanStamped& msg, const prx::plan_t& plan)
{
  msg.header.seq++;
  msg.header.stamp = ros::Time::now();
  copy(msg.plan, plan);
}

inline void copy(prx::plan_t& plan, const ml4kp_bridge::Plan& msg)
{
  // plan.resize(msg.steps.size());
  for (unsigned i = 0; i < msg.steps.size(); ++i)
  {
    plan.append_onto_back(0.0);
    copy(plan.back(), msg.steps[i]);
  }
}

inline void copy(prx::plan_t& plan, const ml4kp_bridge::PlanStamped& msg)
{
  copy(plan, msg.plan);
}

inline void add_zero_control(prx::plan_t& plan, double duration = 0.0)
{
  plan.append_onto_back(duration);
  Vec(plan.back().control) = Eigen::VectorXd::Zero(plan.back().control->get_dim());
}

inline void add_zero_plan(ml4kp_bridge::PlanStamped& plan_stamped, double duration = 0.0, int dim = 2)
{
  plan_stamped.plan.steps.push_back(ml4kp_bridge::PlanStep());
  plan_stamped.plan.steps.back().duration.data = ros::Duration(duration);
  plan_stamped.plan.steps.back().control.point.resize(dim);
  for (int i = 0; i < dim; ++i)
  {
    plan_stamped.plan.steps.back().control.point[i] = 0.0;
  }
}
}  // namespace ml4kp_bridge