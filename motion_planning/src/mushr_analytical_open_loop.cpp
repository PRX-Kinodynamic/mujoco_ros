#include <thread>
// #include "mujoco_ros/control_listener.hpp"
// #include "mujoco_ros/sensordata_publisher.hpp"
// #include "mujoco_ros/Collision.h"
#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
// #include "control/MushrControlPropagation.h"
// #include "motion_planning/replanner_service.hpp"
// #include "motion_planning/planner_client.hpp"
// #include "motion_planning/PlanningResult.h"
// #include "mujoco_ros/Collision.h"
// #include "std_msgs/Empty.h"
#include <utils/std_utils.cpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHRSimulation" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string params_file;
  std::string file_out;
  PARAM_SETUP(nh, params_file);
  PARAM_SETUP(nh, file_out);

  prx::param_loader params{ prx::param_loader(params_file, "") };

  prx::simulation_step = 0.1;
  const std::string plant_name{ params["/name"].as<std::string>() };
  const std::string plant_path{ params["/path"].as<std::string>() };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");

  // auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
  // std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  // std::vector<std::string> obstacle_names{ obstacles.first };

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("planner_context", { plant_name }, {});
  auto context = world_model.get_context("planner_context");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  auto ps = context.first->get_parameter_space();
  std::vector<double> min_control_limits = params["/control_space/lower_bound"].as<std::vector<double>>();
  std::vector<double> max_control_limits = params["/control_space/upper_bound"].as<std::vector<double>>();
  std::vector<double> min_state_limits = params["/state_space/lower_bound"].as<std::vector<double>>();
  std::vector<double> max_state_limits = params["/state_space/upper_bound"].as<std::vector<double>>();
  ss->set_bounds(min_state_limits, max_state_limits);
  cs->set_bounds(min_control_limits, max_control_limits);
  std::vector<double> param_values = params["/parameter_space/values"].as<std::vector<double>>();
  ps->copy_from(param_values);
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  // std::vector<double> x0{ params["start_state"].as<std::vector<double>>() };
  std::vector<double> steering{ -1.0, -0.75, -0.5, -0.25, -0.1, 0.1, 0.25, 0.5, 0.75, 1.0 };

  prx::space_point_t x0(ss->make_point());
  x0->init(params["start_state"]);

  prx::plan_t plan(cs);

  Eigen::Vector2d ctrl{ Eigen::Vector2d::Zero() };
  ctrl[prx_models::mushr_t::control::velocity_idx] = 0.5;
  plan.copy_onto_back(ctrl, 10);

  auto file_mode = std::ofstream::trunc;
  for (int i = 0; i < steering.size(); ++i)
  {
    prx::trajectory_t traj(ss);
    ss->copy_from(x0);
    ctrl[prx_models::mushr_t::control::steering_idx] = steering[i];
    cs->copy(plan.begin()->control, ctrl);
    sg->propagate(x0, plan, traj);
    traj.to_file(file_out, file_mode);
    file_mode = std::ofstream::app;
  }

  return 0;
}