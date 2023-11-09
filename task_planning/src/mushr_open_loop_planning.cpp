#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  prx::simulation_step = 0.1;
  ROS_WARN("Using simulation step %f", prx::simulation_step);

  std::string plant_name = "Ackermann_FO";
  std::string plant_path = "Ackermann_FO";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");

  prx::world_model_t planning_model({ plant }, {});
  planning_model.create_context("planner_context", { plant_name }, {});
  auto planning_context = planning_model.get_context("planner_context");
  auto ss = planning_context.first->get_state_space();
  auto cs = planning_context.first->get_control_space();

  std::shared_ptr<prx::dirt_t> dirt = std::make_shared<prx::dirt_t>("dirt");

  prx::dirt_specification_t dirt_spec(planning_context.first, planning_context.second);
  ROS_WARN("Using defaults for distance function and heuristic");
  dirt_spec.min_control_steps = 1;
  dirt_spec.max_control_steps = 10;
  dirt_spec.blossom_number = 5;
  dirt_spec.use_pruning = true;

  prx::dirt_query_t* dirt_query = new prx::dirt_query_t(ss, cs);
  dirt_query->start_state = ss->make_point();
  dirt_query->goal_state = ss->make_point();
  dirt_query->goal_region_radius = 0.5;
  ROS_WARN("Using default goal check");

  dirt->link_and_setup_spec(&dirt_spec);
  dirt->preprocess();
  dirt->link_and_setup_query(dirt_query);
}