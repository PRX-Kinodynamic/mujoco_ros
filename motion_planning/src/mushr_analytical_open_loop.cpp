#include <prx/simulation/system.hpp>
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
#include <utils/std_utils.hpp>

#include <prx_models/defs.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

void read_plan(const std::string filename, prx::plan_t& plan)
{
  using prx::utilities::convert_to;
  using CsvReader = prx::utilities::csv_reader_t;
  CsvReader reader(filename);

  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;
    if (line[0][0] == '#')  // #steering velocity_desired duration
      continue;
    const double steering{ convert_to<double>(line[0]) };
    const double velocity_desired{ convert_to<double>(line[1]) };
    const double plan_duration{ convert_to<double>(line[2]) };

    const Eigen::Vector2d ctrl{ Eigen::Vector2d(velocity_desired, steering) };
    plan.copy_onto_back(ctrl, plan_duration);
  }
}
// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHROpenLoop" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string params_file;
  std::string file_out;
  std::string plan_file;
  using prx::simulation_step;
  // prx::simulation_step = 0.1;
  // double simulation_step
  PARAM_SETUP(nh, params_file);
  PARAM_SETUP(nh, file_out);
  PARAM_SETUP(nh, plan_file);
  PARAM_SETUP(nh, simulation_step);

  prx::param_loader params{ prx::param_loader(params_file, "") };

  const std::string plant_name{ params["/name"].as<std::string>() };
  const std::string plant_path{ params["/path"].as<std::string>() };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");
  plant->init(params);

  // auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
  // std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  // std::vector<std::string> obstacle_names{ obstacles.first };

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("planner_context", { plant_name }, {});
  auto context = world_model.get_context("planner_context");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  auto ps = context.first->get_parameter_space();
  // std::vector<double> min_control_limits = params["/control_space/lower_bound"].as<std::vector<double>>();
  // std::vector<double> max_control_limits = params["/control_space/upper_bound"].as<std::vector<double>>();
  // std::vector<double> min_state_limits = params["/state_space/lower_bound"].as<std::vector<double>>();
  // std::vector<double> max_state_limits = params["/state_space/upper_bound"].as<std::vector<double>>();
  // ss->set_bounds(min_state_limits, max_state_limits);
  // cs->set_bounds(min_control_limits, max_control_limits);
  // std::vector<double> param_values = params["/parameter_space/values"].as<std::vector<double>>();
  // ps->copy_from(param_values);
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  // std::vector<double> x0{ params["start_state"].as<std::vector<double>>() };
  // std::vector<double> steering{ -1.0, -0.75, -0.5, -0.25, -0.1, 0.1, 0.25, 0.5, 0.75, 1.0 };

  prx::space_point_t x0(ss->make_point());
  if (params.exists("start_state"))
  {
    x0->init(params["start_state"]);
  }
  else
  {
    prx::plan_t init_plan(cs);
    init_plan.copy_onto_back(Eigen::Vector2d::Zero(), 5.0);

    ss->copy_to(x0);
    sg->propagate(x0, init_plan, x0);
  }
  DEBUG_VARS(x0)
  DEBUG_VARS(plant);
  prx::plan_t plan(cs);

  std::ofstream ofs(file_out.c_str());

  read_plan(plan_file, plan);

  // auto file_mode = std::ofstream::trunc;
  prx::trajectory_t traj(ss);
  sg->propagate(x0, plan, traj);

  DEBUG_VARS(plan)
  const double traj_duration{ traj.duration() };
  for (double ti = 0.0; ti < traj_duration; ti += 0.1)
  {
    ofs << ti << " ";
    ofs << traj.at(ti, false);
    ofs << "\n";
  }
  // traj.to_file(file_out, file_mode);
  // file_mode = std::ofstream::app;

  return 0;
}