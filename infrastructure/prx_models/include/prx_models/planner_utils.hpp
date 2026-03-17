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
  msg.total_planning_time = stats.total_planning_time;
  msg.total_iterations = stats.total_iterations;
  msg.total_nodes = stats.total_nodes;

  // # Solution-specific stats
  msg.solution_found = stats.solution_found;

  // # Current solution stats
  msg.current_solution_cost = stats.current_solution_cost;
  msg.current_solution_time = stats.current_solution_time;
  msg.current_solution_iterations = stats.current_solution_iterations;

  // # First solution stats
  msg.first_solution_cost = stats.first_solution_cost;
  msg.first_solution_time = stats.first_solution_time;
  msg.first_solution_iterations = stats.first_solution_iterations;
}

void copy(PlannerStats& msg, const prx::dirt_replan_t::statistics_t& stats)
{
  copy(msg, static_cast<prx::planner_t::statistics_t>(stats));

  msg.best_f_value = stats.best_f_value;
  msg.random_g_rejected = stats.random_edges_counter.g_rejected;
  msg.random_prunning = stats.random_edges_counter.pruning;
  msg.random_collision_check = stats.random_edges_counter.collision_check;
  msg.random_accepted = stats.random_edges_counter.accepted;
  msg.random_bnb = stats.random_edges_counter.bnb;

  msg.blossom_g_rejected = stats.blossom_edges_counter.g_rejected;
  msg.blossom_prunning = stats.blossom_edges_counter.pruning;
  msg.blossom_collision_check = stats.blossom_edges_counter.collision_check;
  msg.blossom_accepted = stats.blossom_edges_counter.accepted;
  msg.blossom_bnb = stats.blossom_edges_counter.bnb;
}

inline std::string header(const prx_models::PlannerStats stats)
{
  std::stringstream strstr;

  strstr << "total_planning_time ";
  strstr << "total_iterations ";
  strstr << "total_nodes ";

  strstr << "solution_found ";

  strstr << "current_solution_cost ";
  strstr << "current_solution_time ";
  strstr << "current_solution_iterations ";

  strstr << "first_solution_cost ";
  strstr << "first_solution_time ";
  strstr << "first_solution_iterations ";

  strstr << "best_f_value ";

  strstr << "random_g_rejected ";
  strstr << "random_prunning ";
  strstr << "random_collision_check ";
  strstr << "random_accepted ";
  strstr << "random_bnb ";

  strstr << "blossom_g_rejected ";
  strstr << "blossom_prunning ";
  strstr << "blossom_collision_check ";
  strstr << "blossom_accepted ";
  strstr << "blossom_bnb ";
  return strstr.str();
}

inline void to_stream(std::ofstream& ofs, const prx_models::PlannerStats& stats)
{
  ofs << stats.total_planning_time << " ";
  ofs << stats.total_iterations << " ";
  ofs << stats.total_nodes << " ";

  ofs << (stats.solution_found ? "true" : "false") << " ";

  ofs << stats.current_solution_cost << " ";
  ofs << stats.current_solution_time << " ";
  ofs << stats.current_solution_iterations << " ";

  ofs << stats.first_solution_cost << " ";
  ofs << stats.first_solution_time << " ";
  ofs << stats.first_solution_iterations << " ";

  ofs << stats.best_f_value << " ";

  ofs << stats.random_g_rejected << " ";
  ofs << stats.random_prunning << " ";
  ofs << stats.random_collision_check << " ";
  ofs << stats.random_accepted << " ";
  ofs << stats.random_bnb << " ";

  ofs << stats.blossom_g_rejected << " ";
  ofs << stats.blossom_prunning << " ";
  ofs << stats.blossom_collision_check << " ";
  ofs << stats.blossom_accepted << " ";
  ofs << stats.blossom_bnb << " ";
}

}  // namespace prx_models