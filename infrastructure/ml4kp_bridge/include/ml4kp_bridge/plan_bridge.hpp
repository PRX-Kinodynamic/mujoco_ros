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

inline void append(ml4kp_bridge::Plan& plan_to_append, const ml4kp_bridge::Plan& prev_plan)
{
  for (auto prev_step : prev_plan.steps)
  {
    plan_to_append.steps.emplace_back();
    copy(plan_to_append.steps.back(), prev_step);
  }
}

inline void copy(ml4kp_bridge::Plan& plan_out, const ml4kp_bridge::Plan& plan_in)
{
  plan_out.steps.clear();
  append(plan_out, plan_in);
}

inline ros::Duration duration(const ml4kp_bridge::Plan& plan_in)
{
  ros::Duration duration;
  for (auto step : plan_in.steps)
  {
    duration += step.duration.data;
  }
  return duration;
}

inline void add_zero_control(prx::plan_t& plan, double duration = 0.0)
{
  plan.append_onto_back(duration);
  Vec(plan.back().control) = Eigen::VectorXd::Zero(plan.back().control->get_dim());
}

inline void to_file(const ml4kp_bridge::Plan& msg, std::ofstream& ofs)
{
  for (auto step : msg.steps)
  {
    to_file(step, ofs);
  }
  ofs << "\n";
}

inline void to_file(const ml4kp_bridge::PlanStamped& msg, std::ofstream& ofs)
{
  to_file(msg.plan, ofs);
}

}  // namespace ml4kp_bridge