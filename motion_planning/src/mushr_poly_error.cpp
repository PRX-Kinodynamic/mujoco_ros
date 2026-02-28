#include <cstddef>
#include <fstream>
#include <iterator>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/random.hpp>
#include <thread>
// #include "mujoco_ros/control_listener.hpp"
// #include "mujoco_ros/sensordata_publisher.hpp"
// #include "mujoco_ros/Collision.h"
#include <Eigen/src/Core/Matrix.h>
#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "utils/dbg_utils.hpp"
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
    // const double plan_duration{ 1.0 };

    const Eigen::Vector2d ctrl{ Eigen::Vector2d(velocity_desired, steering) };
    plan.copy_onto_back(ctrl, plan_duration);
  }
}

void read_traj(const std::string filename, prx::trajectory_t& traj)
{
  using prx::utilities::convert_to;
  using CsvReader = prx::utilities::csv_reader_t;
  CsvReader reader(filename);

  Eigen::Vector<double, 6> xin;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;
    if (line[0][0] == '#')  // #steering velocity_desired duration
      continue;

    // # x y theta xDot yDot thetaDot
    for (int i = 0; i < 6; ++i)
    {
      xin[i] = convert_to<double>(line[i + 1]);
    }
    traj.push_back(xin);
    // states.push_back(xin);
    // const Eigen::Vector2d ctrl{ Eigen::Vector2d(velocity_desired, steering) };
    // plan.copy_onto_back(ctrl, plan_duration);
  }
  // const State x0_inv{ states[0].inverse() };

  // for (auto state : states)
  // {
  //   const State xi{ x0_inv * state };
  //   trajectory.emplace_back(xi[0], xi[1]);
  // }
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
  std::string traj_file;
  std::string errors_filename;
  prx::simulation_step = 0.1;

  int traj_step;
  // Eigen::Vector3d vt;
  // DEBUG_VARS(std::numeric_limits<float>::lowest());
  // DEBUG_VARS(std::numeric_limits<float>::max());
  // prx::simulation_step = 0.1;
  // double simulation_step

  PARAM_SETUP(nh, params_file);
  PARAM_SETUP(nh, plan_file);
  PARAM_SETUP(nh, traj_file);
  PARAM_SETUP(nh, file_out);
  PARAM_SETUP(nh, errors_filename);
  PARAM_SETUP(nh, traj_step);

  prx::param_loader params{ prx::param_loader(params_file, "") };

  std::string model_type{ "" };
  PARAM_SETUP_WITH_DEFAULT(nh, model_type, model_type);
  // params.print();
  if (model_type != "")
  {
    // PARAM_SETUP(nh, simulation_step);
    // DEBUG_VARS(model_type)

    params["/torch/type"].set(model_type);
  }
  // params.print();

  std::ofstream ofs(errors_filename.c_str());

  const std::string plant_name{ params["/name"].as<std::string>() };
  const std::string plant_path{ params["/path"].as<std::string>() };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");
  plant->init(params);

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("planner_context", { plant_name }, {});
  auto context = world_model.get_context("planner_context");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  auto ps = context.first->get_parameter_space();

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  // std::vector<double> x0{ params["start_state"].as<std::vector<double>>() };
  // std::vector<double> steering{ -1.0, -0.75, -0.5, -0.25, -0.1, 0.1, 0.25, 0.5, 0.75, 1.0 };

  prx::space_point_t x0(ss->make_point());

  // prx::plan_t init_plan(cs);
  // prx::trajectory_t init_traj(ss);
  // init_plan.copy_onto_back(Eigen::Vector2d::Zero(), 5.0);

  // ss->copy_to(x0);
  // sg->propagate(x0, init_plan, x0);

  // DEBUG_VARS(plant);
  prx::plan_t plan(cs);
  prx::plan_t partial_plan(cs);

  prx::trajectory_t traj(ss);
  prx::trajectory_t traj_in(ss);

  read_plan(plan_file, plan);
  read_traj(traj_file, traj_in);

  // DEBUG_VARS(plan)
  plan.expand();
  // DEBUG_VARS(plan)
  DEBUG_VARS(plan.size())
  traj_step = std::min(plan.size(), static_cast<std::size_t>(traj_step));
  int plan_idx{ 0 };
  for (; plan_idx < traj_step - 1; ++plan_idx)
  {
    // DEBUG_VARS(plan_idx)
    const auto plan_step = plan[plan_idx];
    partial_plan.copy_onto_back(plan_step.control, plan_step.duration);
  }

  auto traj_opt = std::ofstream::trunc;
  std::ofstream ofs_traj(file_out.c_str());

  // DEBUG_VARS(plan.duration(), traj_in.duration())
  for (int i = 0; i < traj_in.size() - (traj_step + 1); ++i, ++plan_idx)
  {
    // DEBUG_PRINT
    traj.clear();
    if (plan_idx >= plan.size())
      break;
    ss->copy(x0, traj_in[i]);
    sg->propagate(x0, partial_plan, traj);
    // DEBUG_VARS(traj)

    // DEBUG_VARS(plan.size(), plan_idx, i, traj_step, i + traj_step, traj_in.size())
    partial_plan.pop_front();
    const auto plan_step = plan[plan_idx];
    partial_plan.copy_onto_back(plan_step.control, plan_step.duration);
    std::size_t traj_idx{ std::min(traj_in.size() - 1, static_cast<std::size_t>(i + traj_step)) };
    const Eigen::Vector3d xy_gt{ Vec(traj_in[traj_idx]).head(3) };
    // DEBUG_PRINT
    const Eigen::Vector3d xy_pred{ Vec(traj.back()).head(3) };
    const double xy_err{ (xy_gt - xy_pred).head(2).norm() };
    const double th_diff{ xy_gt[2] - xy_pred[2] };
    const double th_err{ std::fabs(std::atan2(std::sin(th_diff), std::cos(th_diff))) };

    ofs << xy_err << " ";
    ofs << th_err << "\n";

    ofs_traj << xy_gt.transpose() << " ";
    ofs_traj << xy_pred.transpose() << "\n";
    // traj.to_file(file_out, traj_opt);
    // traj_opt = std::ofstream::app;
  }
  ofs.close();

  //   return res;
  // DEBUG_VARS(x0)
  // auto file_mode = std::ofstream::trunc;

  // traj_in.to_file();
  // traj.to_file(file_out);
  // DEBUG_VARS(plan)
  // const double traj_duration{ traj.duration() };
  // for (double ti = 0.0; ti < traj_duration; ti += 0.1)
  // {
  //   ofs << ti << " ";
  //   ofs << traj.at(ti, false);
  //   ofs << "\n";
  // }
  // traj.to_file(file_out, file_mode);
  // file_mode = std::ofstream::app;

  return 0;
}