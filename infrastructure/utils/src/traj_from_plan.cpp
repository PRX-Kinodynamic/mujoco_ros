#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <tf2_ros/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.cpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <prx_models/mushr.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TrajFromPlan");
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };

  std::string plan_file;
  std::string ML4KP_plant_params;
  std::string ML4KP_simulation_params;

  ROS_PARAM_SETUP(nh, plan_file);
  ROS_PARAM_SETUP(nh, ML4KP_plant_params);
  ROS_PARAM_SETUP(nh, ML4KP_simulation_params);

  prx::param_loader params(ML4KP_simulation_params, "");
  prx::param_loader plant_params(ML4KP_plant_params, "");
  params["plant"] = plant_params;

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["plant/name"].as<>() };

  auto plant = prx::system_factory_t::create_system(plant_name, plant_name);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> system_group{ context.first };

  prx::space_t* ss{ system_group->get_state_space() };
  prx::space_t* cs{ system_group->get_control_space() };

  plant->init(plant_params);

  prx::space_point_t x0{ ss->make_point() };
  ss->copy(x0, params["/plant/start_state"].as<std::vector<double>>());

  prx::plan_t plan(cs);
  plan.from_file(plan_file);
  prx::trajectory_t solution_traj(ss);

  DEBUG_VARS(x0);
  DEBUG_VARS(plan);

  system_group->propagate(x0, plan, solution_traj);
  DEBUG_VARS(solution_traj);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, solution_traj, body_name, ss);
  vis_group->add_animation(solution_traj, ss, x0);
  vis_group->output_html("mjros_traj_from_plan.html");

  delete vis_group;

  return 0;
}