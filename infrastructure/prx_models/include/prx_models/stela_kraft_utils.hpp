#pragma once

#include <ml4kp_bridge/defs.h>

#include <fstream>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/playback/trajectory.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx_models/PlannerStats.h>
#include <prx_models/StelaKraft.h>

#include <prx_models/tree_utils.hpp>
#include <type_traits>
#include "tree_utils.hpp"
// #include <utils/rosp>
// #include <utils/dbg_utils.hpp>

namespace prx_models
{

inline std::string header(const prx_models::PlannerStats stats)
{
  std::stringstream strstr;

  strstr << "planned_duration ";
  strstr << "iteration_count ";
  strstr << "total_nodes ";
  strstr << "cost_current_solution ";
  strstr << "time_current_solution ";
  strstr << "iters_current_solution ";

  strstr << "random_edges_bnb ";
  strstr << "random_edges_prunning ";
  strstr << "random_edges_collision_check ";
  strstr << "random_edges_final ";

  strstr << "blossom_edges_bnb ";
  strstr << "blossom_edges_prunning ";
  strstr << "blossom_edges_collision_check ";
  strstr << "blossom_edges_final ";
  return strstr.str();
}

inline void to_stream(std::ofstream& ofs, const prx_models::PlannerStats& stats)
{
  ofs << stats.planned_duration << " ";
  ofs << stats.iteration_count << " ";
  ofs << stats.total_nodes << " ";
  ofs << stats.cost_current_solution << " ";
  ofs << stats.time_current_solution << " ";
  ofs << stats.iters_current_solution << " ";

  ofs << stats.random_edges_bnb << " ";
  ofs << stats.random_edges_prunning << " ";
  ofs << stats.random_edges_collision_check << " ";
  ofs << stats.random_edges_final << " ";

  ofs << stats.blossom_edges_bnb << " ";
  ofs << stats.blossom_edges_prunning << " ";
  ofs << stats.blossom_edges_collision_check << " ";
  ofs << stats.blossom_edges_final << " ";
}

inline prx::param_loader create(const prx_models::StelaKraft::Request req)
{
  prx::param_loader params;
  params["use_contingency"].set(req.use_contingency);
  params["solution_duration"].set(req.solution_duration.toSec());
  params["iterations"].set(req.iterations);
  params["condition"].set("ITERATIONS | TIME");
  params["radius"].set(req.radius);
  params["root"] = prx_models::create(req.root);
  params["goal"] = ml4kp_bridge::create(req.goal);

  return params;
}

inline void copy(prx_models::StelaKraft::Request& req, const prx::param_loader& params)
{
  req.use_contingency = params["use_contingency"].as<bool>();
  req.solution_duration = ros::Duration(params["solution_duration"].as<double>());
  req.iterations = params["iterations"].as<int>();
  req.deadline = ros::Time::ZERO;  // This needs to be changed before sending the req.
  req.radius = params["radius"].as<double>();
  ml4kp_bridge::copy(req.goal, params["goal"]);
  if (params.exists("root"))
  {
    auto root_params = params["root"];
    prx_models::copy(req.root, root_params);
  }
  const std::string condition{ params["condition"].as<>() };
  if (condition == "ITERATIONS")
  {
    req.condition = prx_models::StelaKraft::Request::CONDITION_ITERATIONS;
  }
  else if (condition == "TIME")
  {
    req.condition = prx_models::StelaKraft::Request::CONDITION_TIME;
  }
}

}  // namespace prx_models