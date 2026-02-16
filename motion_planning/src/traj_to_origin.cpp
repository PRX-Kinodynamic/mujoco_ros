#include <iterator>
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
#include <vector>

void read_traj(const std::string filename, prx::trajectory_t& traj)
{
  using prx::utilities::convert_to;
  using CsvReader = prx::utilities::csv_reader_t;
  using StateVec = Eigen::Vector<double, 6>;
  using State = gtsam::Pose2;
  CsvReader reader(filename);

  StateVec xin;
  std::vector<StateVec> full_traj;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;
    if (line[0][0] == '#')  // #steering velocity_desired duration
      continue;

    // # dt x y theta xDot yDot thetaDot
    for (int i = 0; i < 6; ++i)
    {
      xin[i] = convert_to<double>(line[i + 1]);
    }
    full_traj.push_back(xin);
    // traj.push_back(xin);
    // states.push_back(xin);
    // const Eigen::Vector2d ctrl{ Eigen::Vector2d(velocity_desired, steering) };
    // plan.copy_onto_back(ctrl, plan_duration);
  }
  const State x0{ State(full_traj[0][0], full_traj[0][1], full_traj[0][2]) };
  const State x0_inv{ x0.inverse() };

  for (auto xi_vec : full_traj)
  {
    const State xi{ State(xi_vec[0], xi_vec[1], xi_vec[2]) };

    const State xi_0{ x0_inv * xi };
    const StateVec xivec_0{ xi_0.x(), xi_0.y(), xi_0.theta(), xi_vec[3], xi_vec[4], xi_vec[5] };
    traj.push_back(xivec_0);
  }
}

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHRTrajToOrigin" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string params_file;
  std::string file_out;
  std::string plan_file;
  std::string traj_file;
  prx::simulation_step = 0.1;

  // Eigen::Vector3d vt;
  // DEBUG_VARS(std::numeric_limits<float>::lowest());
  // DEBUG_VARS(std::numeric_limits<float>::max());
  // prx::simulation_step = 0.1;
  // double simulation_step
  PARAM_SETUP(nh, params_file);
  PARAM_SETUP(nh, traj_file);
  PARAM_SETUP(nh, file_out);
  // PARAM_SETUP(nh, simulation_step);

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

  // DEBUG_VARS(plant);
  prx::trajectory_t traj(ss);

  // std::ofstream ofs(file_out.c_str());

  read_traj(traj_file, traj);

  // ss->copy(x0, traj_in.front());
  // DEBUG_VARS(x0)
  // auto file_mode = std::ofstream::trunc;
  // sg->propagate(x0, plan, traj);

  // traj_in.to_file();
  traj.to_file(file_out);

  return 0;
}