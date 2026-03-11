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

inline prx::param_loader create(const prx_models::StelaKraft::Request req)
{
  prx::param_loader params;
  // params["use_contingency"].set(req.use_contingency);
  params["solution_duration"].set(req.solution_duration.toSec());
  params["iterations"].set(req.iterations);
  params["condition"].set("ITERATIONS | TIME");
  // params["radius"].set(req.radius);
  params["root"] = prx_models::create(req.root);
  // params["goal/state"] = ml4kp_bridge::create(req.goal);
  // params["goal/region_radius"] = ml4kp_bridge::create(req.goal);

  return params;
}

inline void copy(prx_models::StelaKraft::Request& req, const prx::param_loader& params)
{
  req.solution_duration = ros::Duration(params["solution_duration"].as<double>());
  req.iterations = params["iterations"].as<int>();
  req.deadline = ros::Time::ZERO;  // This needs to be changed before sending the req.
  // if (params.exists("goal"))
  // {
  //   req.radius = params["goal/radius"].as<double>();
  //   ml4kp_bridge::copy(req.goal, params["goal/state"]);
  // }
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
  else
  {
    const std::string unknown_condition{ params["condition"].as<>() };
    DEBUG_VARS(unknown_condition);
  }
}

}  // namespace prx_models