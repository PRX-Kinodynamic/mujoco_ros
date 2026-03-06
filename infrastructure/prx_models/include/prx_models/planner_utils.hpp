#pragma once

#include <ml4kp_bridge/defs.h>

#include <prx/planning/planners/dirt_replanning.hpp>
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

void copy(PlannerStats& msg, const prx::dirt_replan_t::statistics_t& stats)
{
  copy(msg, static_cast<prx::planner_t::statistics_t>(stats));
  msg.random_edges_bnb = stats.random_edges_counter.bnb;
  msg.random_edges_prunning = stats.random_edges_counter.prunning;
  msg.random_edges_collision_check = stats.random_edges_counter.collision_check;
  msg.random_edges_final = stats.random_edges_counter.final;

  msg.blossom_edges_bnb = stats.blossom_edges_counter.bnb;
  msg.blossom_edges_prunning = stats.blossom_edges_counter.prunning;
  msg.blossom_edges_collision_check = stats.blossom_edges_counter.collision_check;
  msg.blossom_edges_final = stats.blossom_edges_counter.final;

  msg.solution_type = stats.solution_type;
}

}  // namespace prx_models