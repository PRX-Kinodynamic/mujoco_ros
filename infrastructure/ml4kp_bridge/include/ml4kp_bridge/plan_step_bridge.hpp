#pragma once

#include <ros/assert.h>

#include <prx/simulation/playback/plan.hpp>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/space_bridge.hpp>

#include <ml4kp_bridge/PlanStepStamped.h>
namespace ml4kp_bridge
{
inline void copy(ml4kp_bridge::PlanStep& msg, const prx::plan_step_t& plan_step)
{
  copy(msg.control, plan_step.control);
  msg.duration.data = ros::Duration(plan_step.duration);
}

inline void copy(ml4kp_bridge::PlanStepStamped& msg, const prx::plan_step_t& plan_step)
{
  msg.header.seq++;
  msg.header.stamp = ros::Time::now();
  copy(msg.plan_step, plan_step);
}

inline void copy(prx::plan_step_t& plan_step, const ml4kp_bridge::PlanStep& msg)
{
  copy(plan_step.control, msg.control);
  plan_step.duration = msg.duration.data.toSec();
}

inline void copy(prx::plan_step_t& plan_step, const ml4kp_bridge::PlanStepStamped& msg)
{
  copy(plan_step, msg.plan_step);
}

inline void copy(ml4kp_bridge::PlanStep& to, const ml4kp_bridge::PlanStep& from)
{
  copy(to.control, from.control);
  to.duration.data = from.duration.data;
}

inline void to_file(const ml4kp_bridge::PlanStep& msg, std::ofstream& ofs)
{
  ofs << msg.duration.data.toSec() << " ";
  to_file(msg.control, ofs);
}

}  // namespace ml4kp_bridge