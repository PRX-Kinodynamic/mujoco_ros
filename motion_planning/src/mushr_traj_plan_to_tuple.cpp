#include <cstddef>
#include <fstream>
#include <filesystem>
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
namespace fs = std::filesystem;
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
}

int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHRTrajsPlansToTuples" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string params_file;
  std::string file_out;

  std::string data_directory;
  prx::simulation_step = 0.01;

  int traj_step;
  // Eigen::Vector3d vt;
  // DEBUG_VARS(std::numeric_limits<float>::lowest());
  // DEBUG_VARS(std::numeric_limits<float>::max());
  // prx::simulation_step = 0.1;
  // double simulation_step

  PARAM_SETUP(nh, params_file);
  PARAM_SETUP(nh, data_directory);
  PARAM_SETUP(nh, file_out);

  std::ofstream ofs(file_out.c_str());

  ofs << "# xd1 = xd0 + xdd * dt = xd0 + f(xd0, u01) * dt\n";
  ofs << "# xd1_x xd1_y xd1_th xd0_x xd0_y xd0_th xdd_x xdd_y xdd_th u01_{steering_angle} u01_{velocity_desired}";
  // fs::path dir{ data_directory };

  prx::param_loader params{ prx::param_loader(params_file, "") };

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

  prx::space_point_t x0(ss->make_point());

  //   # plan="${ML4KP_ROS}/data/mj_mushr/sysid_v2/mj_plan_e${i}.txt"
  // # traj="${ML4KP_ROS}/data/mj_mushr/sysid_v2/mj_traj_vels_e${i}.txt"
  int idx{ 0 };
  for (; idx < 2000; ++idx)
  {
    prx::plan_t plan_in(cs);
    prx::trajectory_t traj_in(ss);

    std::stringstream strstr;
    strstr << std::setw(4) << std::setfill('0') << idx << ".txt";

    const std::string plan_file{ data_directory + "/mj_plan_e" + strstr.str() };
    const std::string traj_file{ data_directory + "/mj_traj_vels_e" + strstr.str() };

    read_plan(plan_file, plan_in);
    read_traj(traj_file, traj_in);

    const double dt{ 0.1 };
    const double plan_duration(plan_in.duration());
    const double traj_duration(traj_in.duration());
    for (int i = 0; i < traj_in.size() - 1; ++i)
    {
      const double ti{ i * 0.1 };
      if (ti > plan_duration)
        break;

      auto xd0 = Vec(traj_in[i]).tail(3).transpose();
      auto u01 = Vec(plan_in.at(ti)).transpose();
      auto xd1 = Vec(traj_in[i + 1]).tail(3).transpose();

      auto xdd = (xd1 - xd0) / dt;

      // xd1 = xd0 + xdd * dt = xd0 + f(xd0, u01) * dt
      // ofs << << " ";
      ofs << xd1 << " ";
      ofs << xd0 << " ";
      ofs << xdd << " ";
      ofs << u01 << " ";
      ofs << "\n";
    }
  }

  ofs.close();
  // auto traj_opt = std::ofstream::trunc;
  // std::ofstream ofs_traj(file_out.c_str());

  return 0;
}