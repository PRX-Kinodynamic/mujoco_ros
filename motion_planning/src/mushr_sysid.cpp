#include <thread>
// #include "mujoco_ros/control_listener.hpp"
// #include "mujoco_ros/sensordata_publisher.hpp"
// #include "mujoco_ros/Collision.h"
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

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>
#include <prx_models/mushr_factors.hpp>

#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using State = prx_models::mushr_types::State::type;
using StateDot = prx_models::mushr_types::StateDot::type;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using Params = prx_models::mushr_types::Control::params;
using Polynomial = prx_models::mushr_types::Control::Poly;

std::vector<std::tuple<double, State, double>> read_file(const std::string file, State& x0, bool calculate_dt)
{
  using prx::utilities::convert_to;
  using CsvReader = prx::utilities::csv_reader_t;
  CsvReader reader(file);
  std::vector<std::tuple<double, State, double>> states;

  double t_prev{ -1.0 };
  CsvReader::Line<std::string> prev_line;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;
    if (line[0][0] == '#')
      continue;

    if (prev_line.size() == line.size())
    {
      bool equal_line{ true };
      for (int i = 1; i <= 7; ++i)
      {
        equal_line = equal_line and prev_line[i] == line[i];
      }
      if (equal_line)
        continue;
    }
    prev_line = line;

    const double t_now{ convert_to<double>(line[0]) };

    const double x{ convert_to<double>(line[1]) };
    const double y{ convert_to<double>(line[2]) };
    const double z{ convert_to<double>(line[3]) };

    const double qw{ convert_to<double>(line[4]) };
    const double qx{ convert_to<double>(line[5]) };
    const double qy{ convert_to<double>(line[6]) };
    const double qz{ convert_to<double>(line[7]) };

    const Eigen::Quaterniond q{ Eigen::Quaterniond(qw, qx, qy, qz) };
    const double angle{ prx::quaternion_to_euler(q)[2] };

    double dt{ t_now };
    if (calculate_dt)
    {
      if (t_prev < 0)
      {
        dt = 0;
      }
      else
      {
        dt = t_now - t_prev;
      }
    }

    states.emplace_back(std::make_tuple(dt, State(x, y, angle), t_now));
    t_prev = t_now;
  }

  x0 = std::get<1>(states[0]);
  const State x0_inv{ x0.inverse() };

  for (auto& state : states)
  {
    std::get<1>(state) = x0_inv * std::get<1>(state);
    // const State xi{ x0_inv * state };
    // trajectory.emplace_back(xi[0], xi[1]);
  }
  return states;
}

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHRSimulation" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // If the input file has timestamps, we need to calculate the dt between each observation
  bool calculate_dt{ false };
  // whether to output the traj with x0 = Origin or in original coordinates
  bool output_in_origin{ false };

  std::string filename;
  std::string file_out;

  PARAM_SETUP(nh, filename);
  PARAM_SETUP(nh, file_out);
  PARAM_SETUP(nh, output_in_origin)
  PARAM_SETUP_WITH_DEFAULT(nh, calculate_dt, calculate_dt);

  DEBUG_VARS(calculate_dt)
  // prx::param_loader params{ prx::param_loader(params_file, "") };

  prx::simulation_step = 0.1;
  // const std::string plant_name{ params["/name"].as<std::string>() };
  // const std::string plant_path{ params["/path"].as<std::string>() };
  // auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  // prx_assert(plant != nullptr, "Failed to create plant");

  // // auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
  // // std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  // // std::vector<std::string> obstacle_names{ obstacles.first };

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  State z0;
  std::vector<std::tuple<double, State, double>> z_traj{ read_file(filename, z0, calculate_dt) };

  const StateDot xdot_init{ StateDot::Zero() };
  gtsam::Key key_xt0{ gtsam::Symbol('X', 0) };
  // lie_integration_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
  // const NoiseModel& cost_model, const double h, const std::string label = "LieOdeIntegration")
  using MushrStateIntegrator = prx::fg::lie_integration_factor_t<State, StateDot>;
  NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-3) };
  for (int i = 0; i < z_traj.size() - 1; ++i)
  {
    gtsam::Key key_xt1{ gtsam::Symbol('X', i + 1) };
    gtsam::Key key_xdot{ gtsam::Symbol('D', i) };

    const double& dt{ std::get<0>(z_traj[i]) };
    const State& x0{ std::get<1>(z_traj[i]) };

    values.insert(key_xt0, x0);
    values.insert(key_xdot, xdot_init);
    graph.addPrior(key_xt0, x0, prior_noise);
    graph.emplace_shared<MushrStateIntegrator>(key_xt1, key_xt0, key_xdot, nullptr, dt);

    key_xt0 = key_xt1;
  }
  values.insert(key_xt0, std::get<1>(z_traj.back()));

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  const gtsam::Values result{ optimizer.optimize() };

  const double error{ graph.error(result) };
  std::ofstream ofs(file_out.c_str());
  ofs << "# Final Error: " << error << "\n";
  ofs << "# dt x y theta xDot yDot thetaDot stamp\n";

  // Adding the initial state;
  ofs << "0.0 ";
  if (output_in_origin)
  {
    ofs << "0.0 0.0 0.0 ";
  }
  else
  {
    ofs << z0.x() << " " << z0.y() << " " << z0.angle() << " ";
  }
  ofs << "0.0 0.0 0.0 0.0";
  ofs << "\n";

  for (int i = 0; i < z_traj.size() - 2; ++i)
  {
    const double& dt{ std::get<0>(z_traj[i]) };
    const double& stamp{ std::get<2>(z_traj[i]) };
    const gtsam::Key key_xt0{ gtsam::Symbol('X', i + 1) };
    const gtsam::Key key_xdot0{ gtsam::Symbol('D', i) };
    const State xi{ output_in_origin ? result.at<State>(key_xt0) : z0 * result.at<State>(key_xt0) };
    const StateDot xdot_i{ result.at<StateDot>(key_xdot0) };
    ofs << dt << " ";
    ofs << xi.x() << " " << xi.y() << " " << xi.angle() << " ";
    ofs << xdot_i[0] << " " << xdot_i[1] << " " << xdot_i[2] << " ";
    ofs << prx::utilities::convert_to<std::string>(stamp) << " ";
    ofs << "\n";
  }
  ofs.close();

  return 0;
}