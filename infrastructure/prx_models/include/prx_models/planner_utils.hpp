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

  msg.random_f_rejected = stats.random_edges_counter.f_rejected;
  msg.random_prunning = stats.random_edges_counter.pruning;
  msg.random_collision_check = stats.random_edges_counter.collision_check;
  msg.random_accepted = stats.random_edges_counter.accepted;
  msg.random_bnb = stats.random_edges_counter.bnb;

  msg.blossom_f_rejected = stats.blossom_edges_counter.f_rejected;
  msg.blossom_prunning = stats.blossom_edges_counter.pruning;
  msg.blossom_collision_check = stats.blossom_edges_counter.collision_check;
  msg.blossom_accepted = stats.blossom_edges_counter.accepted;
  msg.blossom_bnb = stats.blossom_edges_counter.bnb;

  msg.solution_type = stats.solution_type;
}

inline void to_stream(std::ofstream& ofs, const prx_models::PlannerStats& stats)
{
  ofs << stats.planned_duration << " ";
  ofs << stats.iteration_count << " ";
  ofs << stats.total_nodes << " ";
  ofs << stats.cost_current_solution << " ";
  ofs << stats.time_current_solution << " ";
  ofs << stats.iters_current_solution << " ";

  ofs << stats.random_f_rejected << " ";
  ofs << stats.random_prunning << " ";
  ofs << stats.random_collision_check << " ";
  ofs << stats.random_accepted << " ";
  ofs << stats.random_bnb << " ";

  ofs << stats.blossom_f_rejected << " ";
  ofs << stats.blossom_prunning << " ";
  ofs << stats.blossom_collision_check << " ";
  ofs << stats.blossom_accepted << " ";
  ofs << stats.blossom_bnb << " ";
}

}  // namespace prx_models