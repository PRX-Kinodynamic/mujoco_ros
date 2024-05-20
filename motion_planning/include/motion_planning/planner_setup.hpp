#pragma once

#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/dirt.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>

#include <prx_models/Graph.h>
#include <prx_models/NodeEdge.h>
#include <prx_models/Tree.h>

namespace motion_planning
{

prx::PairNameObstacles setup_world(const prx::param_loader& params, std::shared_ptr<prx::system_t>& plant,
                                   std::shared_ptr<prx::world_model_t>& world_model,
                                   std::shared_ptr<prx::system_group_t>& system_group,
                                   std::shared_ptr<prx::collision_group_t>& collision_group)
{
  const prx::PairNameObstacles obstacles{ prx::load_obstacles(params["planner/environment"].as<>()) };
  const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  const std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");
  plant->init(params["plant"]);

  world_model.reset(new prx::world_model_t({ plant }, { obstacle_list }));
  world_model->create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model->get_context("context");
  system_group = context.first;
  collision_group = context.second;

  system_group->get_state_space()->init(params["/plant/state_space/"]);
  system_group->get_control_space()->init(params["/plant/control_space/"]);
  system_group->get_parameter_space()->init(params["/plant/parameter_space/"]);
  return obstacles;
}

void setup_spec(const prx::param_loader& params, std::shared_ptr<prx::rrt_specification_t>& spec)
{
  spec->init(params["plant"]);
  spec->init(params["planner"]);
}

void setup_spec(const prx::param_loader& params, std::shared_ptr<prx::dirt_specification_t>& spec)
{
  std::shared_ptr<prx::rrt_specification_t> rrt_spec{ std::dynamic_pointer_cast<prx::rrt_specification_t>(spec) };
  setup_spec(params, rrt_spec);
  spec->h = [&](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    return (Vec(s1).head(2) - Vec(s2).head(2)).norm();
  };
}

void setup_spec(const prx::param_loader& params, std::shared_ptr<prx::aorrt_specification_t>& spec)
{
  std::shared_ptr<prx::rrt_specification_t> rrt_spec{ std::dynamic_pointer_cast<prx::rrt_specification_t>(spec) };
  setup_spec(params, rrt_spec);
}

template <typename PlannerQueryPtr>
void setup_query(const prx::param_loader& params, PlannerQueryPtr& query,
                 std::shared_ptr<prx::condition_check_t>& checker)
{
  query->init(params["planner"]);
  query->init(params["plant"]);

  const std::string checker_type{ params["/planner/checker_type"].as<>() };
  const int checker_value{ params["/planner/checker_value"].as<int>() };

  checker = std::make_shared<prx::condition_check_t>(checker_type, checker_value);
}

}  // namespace motion_planning