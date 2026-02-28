#pragma once

#include <ml4kp_bridge/defs.h>

#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/playback/trajectory.hpp>
#include <prx_models/PlannerStats.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace prx_models
{

void copy(PlannerStats& msg, const prx::planner_t::statistics_t& stats)
{
  msg.planned_duration = stats.planned_duration;
  msg.iteration_count = stats.iteration_count;
  msg.total_nodes = stats.total_nodes;
  msg.cost_current_solution = stats.cost_current_solution;
  msg.time_current_solution = stats.time_current_solution;
  msg.iters_current_solution = stats.iters_current_solution;
}

}  // namespace prx_models