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
#include <prx_models/mushr_factors.hpp>

#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using State = prx_models::mushr_types::State::type;
using StateDot = prx_models::mushr_types::StateDot::type;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using Params = prx_models::mushr_types::Control::params;
using Polynomial = prx_models::mushr_types::Control::Poly;

std::vector<std::pair<double, State>> read_file(const std::string file, State& x0)
{
  using prx::utilities::convert_to;
  using CsvReader = prx::utilities::csv_reader_t;
  CsvReader reader(file);
  std::vector<std::pair<double, State>> states;

  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;

    const double dt{ convert_to<double>(line[0]) };

    const double x{ convert_to<double>(line[1]) };
    const double y{ convert_to<double>(line[2]) };
    const double z{ convert_to<double>(line[3]) };

    const double qw{ convert_to<double>(line[4]) };
    const double qx{ convert_to<double>(line[5]) };
    const double qy{ convert_to<double>(line[6]) };
    const double qz{ convert_to<double>(line[7]) };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(qw, qx, qy, qz) };

    const double angle{ prx::quaternion_to_euler(q)[2] };
    states.emplace_back(std::make_pair(dt, State(x, y, angle)));
  }

  x0 = states[0].second;
  const State x0_inv{ x0.inverse() };

  for (auto& state : states)
  {
    state.second = x0_inv * state.second;
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

  std::string filename;
  std::string file_out;

  PARAM_SETUP(nh, filename);
  PARAM_SETUP(nh, file_out);

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
  std::vector<std::pair<double, State>> z_traj{ read_file(filename, z0) };

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

    const State& x0{ z_traj[i].second };
    const double& dt{ z_traj[i].first };

    values.insert(key_xt0, x0);
    values.insert(key_xdot, xdot_init);
    graph.addPrior(key_xt0, x0, prior_noise);
    graph.emplace_shared<MushrStateIntegrator>(key_xt1, key_xt0, key_xdot, nullptr, dt);

    key_xt0 = key_xt1;
  }
  values.insert(key_xt0, z_traj.back().second);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  const gtsam::Values result{ optimizer.optimize() };

  const double error{ graph.error(result) };
  std::ofstream ofs(file_out.c_str());
  ofs << "# Final Error: " << error << "\n";
  ofs << "# dt x y theta xDot yDot thetaDot\n";

  // Adding the initial state;
  ofs << "0.0 ";
  ofs << z0.x() << " " << z0.y() << " " << z0.angle() << " ";
  ofs << "0.0 0.0 0.0 ";
  ofs << "\n";

  for (int i = 0; i < z_traj.size() - 2; ++i)
  {
    const double& dt{ z_traj[i].first };
    const gtsam::Key key_xt0{ gtsam::Symbol('X', i + 1) };
    const gtsam::Key key_xdot0{ gtsam::Symbol('D', i) };
    const State xi{ z0 * result.at<State>(key_xt0) };
    const StateDot xdot_i{ result.at<StateDot>(key_xdot0) };
    ofs << dt << " ";
    ofs << xi.x() << " " << xi.y() << " " << xi.angle() << " ";
    ofs << xdot_i[0] << " " << xdot_i[1] << " " << xdot_i[2] << " ";
    ofs << "\n";
  }
  ofs.close();
  // result.print("result");

  // gtsam::Values values_sysid;
  // gtsam::NonlinearFactorGraph graph_sysid;

  // gtsam::Key key_params{ gtsam::Symbol('P', 0) };

  // Params params_init{ 1.50000, 0.20000, 0.90000, 0.90000, 1.05000 };
  // values_sysid.insert(key_params, params_init);

  // Eigen::Vector2d ctrl{ Eigen::Vector2d::Zero() };
  // ctrl[prx_models::mushr_t::control::velocity_idx] = 0.5;
  // ctrl[prx_models::mushr_t::control::steering_idx] = -1.0;

  // const Polynomial poly{ -0.4397, 3.773e-5, 0.8677, 5.8e-6 };
  // for (int i = 0; i < z_traj.size() - 2; ++i)
  // {
  //   // gtsam::Key key_xt0{ gtsam::Symbol('X', i) };
  //   // gtsam::Key key_xt1{ gtsam::Symbol('X', i + 1) };
  //   const gtsam::Key key_xdot0{ gtsam::Symbol('D', i) };
  //   const gtsam::Key key_xdot1{ gtsam::Symbol('D', i + 1) };

  //   const double& dt{ z_traj[i].first };
  //   const StateDot xdot0{ result.at<StateDot>(key_xdot0) };
  //   const StateDot xdot1{ result.at<StateDot>(key_xdot1) };
  //   // mushr_CtrlAccel_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const gtsam::Key dt,
  //   // const NoiseModel& cost_model, const Params params, const Polynomial& steering_poly)

  //   graph_sysid.emplace_shared<prx_models::mushr_params_sysid_t>(key_params, xdot1, xdot0, ctrl, dt, poly, nullptr);
  // }

  // gtsam::LevenbergMarquardtOptimizer optimizer_sysid(graph_sysid, values_sysid, lm_params);
  // gtsam::Values result_sysid{ optimizer_sysid.optimize() };

  // result_sysid.print("sysid");

  return 0;
}