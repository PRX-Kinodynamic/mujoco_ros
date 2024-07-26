#include <thread>

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <estimation/TrajectoryEstimation.h>
#include <estimation/StateEstimation.h>
#include <estimation/fg_trajectory_estimation.hpp>
#include <analytical/fg_ltv_sde.hpp>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/mushr.hpp>

// ML4KP
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include <prx/factor_graphs/utilities/default_parameters.hpp>

// GTSAM
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using State = prx_models::mushr_types::State::type;
using StateDot = prx_models::mushr_types::StateDot::type;
using Control = prx_models::mushr_types::Control::type;
using Ubar = prx_models::mushr_types::Ubar::type;
using Params = prx_models::mushr_types::Ubar::params;

using Trajectory = std::vector<std::pair<State, double>>;
using Plan = std::vector<Control>;

using csv_reader_t = prx::utilities::csv_reader_t;
using Line = csv_reader_t::Line<std::string>;
using Block = csv_reader_t::Block<std::string>;
using prx::utilities::convert_to;

// using Formatter = prx::fg::symbol_factory_t::formatter;

Trajectory read_files(const std::string path)
{
  csv_reader_t reader(path, ' ');

  Trajectory traj;
  Block block{ reader.next_block() };

  bool first{ true };
  ros::Time t0{ 0 };
  for (auto line : block)
  {
    if (line.size() == 0 or line[0] == "#")
      continue;

    const ros::Time t{ convert_to<double>(line[2]) };

    const double x{ convert_to<double>(line[3]) };
    const double y{ convert_to<double>(line[4]) };

    const double qw{ convert_to<double>(line[6]) };
    const double qx{ convert_to<double>(line[7]) };
    const double qy{ convert_to<double>(line[8]) };
    const double qz{ convert_to<double>(line[9]) };

    if (first)
    {
      t0 = t;
    }

    const double dt{ (ros::Time(t) - t0).toSec() };

    const Eigen::Quaterniond q(qw, qx, qy, qz);
    const Eigen::Vector3d euler{ prx::quaternion_to_euler(q) };
    const double theta{ euler[2] };
    // const double theta{ euler[2] - prx::constants::pi / 4.0 };
    const State xi{ State(x, y, theta) };
    // if (first or (xi - traj.back().first).norm() > 0.01)
    // {
    //   traj.push_back(std::make_pair(xi, dt));
    // }

    first = false;
  }
  for (int i = 0; i < traj.size() - 1; ++i)
  {
    traj[i].second = traj[i + 1].second;
  }
  return traj;
}

int main(int argc, char** argv)
{
  const std::string node_name{ "mushrTunning" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  prx::simulation_step = 0.01;

  std::vector<std::string> files;
  std::vector<std::string> control_files;
  std::vector<double> params_init;
  std::string output_file;
  std::string initial_trajs_file;
  std::string observations_out;

  ROS_PARAM_SETUP(nh, files);
  ROS_PARAM_SETUP(nh, control_files);
  ROS_PARAM_SETUP(nh, output_file);
  ROS_PARAM_SETUP(nh, params_init);
  ROS_PARAM_SETUP(nh, initial_trajs_file);
  ROS_PARAM_SETUP(nh, observations_out);

  std::vector<Eigen::Vector4d> start_states;
  start_states.emplace_back(3.10625, -1.10301, -3.1415, 0.0);
  start_states.emplace_back(2.94545, -0.318187, 3.14159, 0.0);
  start_states.emplace_back(2.91369, -1.69273, 3.14159, 0.0);
  prx_assert(files.size() == control_files.size(), "trajs and plans different number of files");
  // std::vector<Control> controls;
  // controls.push_back(Control(1, 0));
  // controls.push_back(Control(0.5, -1));
  // controls.push_back(Control(0.5, 1));

  prx::system_ptr_t mushr{ prx::system_factory_t::create_system("mushrFG", "mushrFG") };
  prx::world_model_t world({ mushr }, {});
  world.create_context("context", { "mushrFG" }, {});
  auto context = world.get_context("context");
  std::shared_ptr<prx::system_group_t> system_group{ context.first };

  prx::space_t* ss{ system_group->get_state_space() };
  prx::space_t* cs{ system_group->get_control_space() };
  prx::space_t* ps{ system_group->get_parameter_space() };

  prx::space_point_t start_state{ ss->make_point() };
  prx::space_point_t current_state{ ss->make_point() };
  prx::plan_t prx_plan{ cs };
  prx::trajectory_t prx_traj{ ss };

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  int traj_idx{ 0 };
  std::vector<Trajectory> trajectories;

  const gtsam::Key paramskey{ prx_models::mushr_utils_t::keyParams(0, 0) };
  Params params(params_init[0], params_init[1], params_init[2]);
  values.insert(paramskey, params);

  ps->copy_from(params);
  std::ofstream ofs_z(observations_out);
  for (auto file : files)
  {
    Trajectory traj{ read_files(file) };
    // Plan plan(traj.size(), controls[traj_idx]);

    prx_plan.clear();
    prx_plan.from_file(control_files[traj_idx]);

    Vec(start_state) = start_states[traj_idx];
    system_group->propagate(start_state, prx_plan, prx_traj);
    prx_traj.to_file(initial_trajs_file, traj_idx == 0 ? std::ofstream::trunc : std::ofstream::app);

    gtsam::SharedNoiseModel cm_x_z{ gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector3d(1e-0, 1e-0, 1e1)) };

    const State zF{ traj.back().first };
    traj.pop_back();

    StateDot state_dot{ StateDot::Zero() };
    Ubar ubar{ Ubar::Zero() };

    // const gtsam::Key x0key{ prx_models::mushr_utils_t::keyX(0, traj_idx) };
    // graph.addPrior(x0key, traj[0].first);
    // const double plan_duration{ prx_plan.duration() };
    // const std::size_t total_observations{ traj.size() };
    // for (int i = 0; i < traj.size(); ++i)
    // {
    //   const double ti{ i / static_cast<double>(total_observations) };
    //   const std::pair<State, double> state_dt{ traj[i] };
    //   const Control ctrl{ Vec(prx_plan.at(plan_duration * ti)) };

    //   const State observation{ state_dt.first };
    //   const double dt{ state_dt.second };
    //   ofs_z << observation.transpose() << "\n";

    //   const gtsam::Key x0key{ prx_models::mushr_utils_t::keyX(i, traj_idx) };
    //   const gtsam::Key x1key{ prx_models::mushr_utils_t::keyX(i + 1, traj_idx) };
    //   const gtsam::Key xdot0key{ prx_models::mushr_utils_t::keyXdot(i, traj_idx) };
    //   //   const gtsam::Key xdot1key{ prx_models::mushr_utils_t::keyXdot(t + 1, traj_idx) };
    //   //   const gtsam::Key ukey{ prx_models::mushr_utils_t::keyU(t, traj_idx) };
    //   //   const gtsam::Key ubar0key{ prx_models::mushr_utils_t::keyUbar(t, traj_idx) };
    //   //   const gtsam::Key ubar1key{ prx_models::mushr_utils_t::keyUbar(t + 1, traj_idx) };

    //   //   // DEBUG_VARS(dt);
    //   gtsam::SharedNoiseModel cm_x_xdot{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-10) };
    //   // gtsam::SharedNoiseModel cm_x_xdot_ub{ gtsam::noiseModel::Isotropic::Sigma(3, dt) };
    //   // gtsam::SharedNoiseModel cm_ub_u{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    //   //   // gtsam::SharedNoiseModel cm_ub_u{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };

    //   graph.emplace_shared<prx_models::mushr_x_xdot_t>(x0key, xdot0key, x1key, dt, cm_x_xdot);
    //   // graph.emplace_shared<prx_models::mushr_x_xdot_ub_t>(x0key, xdot1key, ubar0key, cm_x_xdot_ub);
    //   // graph.emplace_shared<prx_models::mushr_ub_ufix_xdot_param_t>(ubar1key, ubar0key, paramskey, ctrl, cm_ub_u);
    //   graph.emplace_shared<prx_models::mushr_x_observation_t>(x0key, observation, cm_x_z);

    //   //   // const State state{ prx_models::mushr_x_xdot_t::predict(_state, _state_dot, simulation_step) };
    //   current_state = prx_traj.at(ti, false);
    //   const State xi{ Vec(current_state).head(3) };
    //   ubar = prx_models::mushr_ub_u_xdot_param_t::dynamics(ctrl, ubar, params);
    //   const State xdoti{ prx_models::mushr_x_xdot_ub_t::dynamics(xi, ubar) };
    //   values.insert(x0key, xi);
    //   values.insert(xdot0key, xdoti);
    //   //   values.insert(ukey, ctrl);
    //   //   values.insert(ubar0key, ubar);
    // }
    // ofs_z << zF.transpose() << "\n\n";

    // const gtsam::Key xFkey{ prx_models::mushr_utils_t::keyX(traj.size(), traj_idx) };
    // const gtsam::Key xdotFkey{ prx_models::mushr_utils_t::keyXdot(traj.size(), traj_idx) };
    // const Control uF{ Vec(prx_plan.back().control) };
    // graph.emplace_shared<prx_models::mushr_x_observation_t>(xFkey, zF, gtsam::noiseModel::Isotropic::Sigma(3,
    // 1e-10));
    // // const gtsam::Key ubarFkey{ prx_models::mushr_utils_t::keyUbar(traj.size(), traj_idx) };

    // current_state = prx_traj.back();
    // const State xF{ Vec(current_state).head(3) };
    // ubar = prx_models::mushr_ub_u_xdot_param_t::dynamics(uF, ubar, params);
    // const State xdoti{ prx_models::mushr_x_xdot_ub_t::dynamics(xF, ubar) };
    // values.insert(xFkey, xF);
    // values.insert(xdotFkey, xdoti);
    // // values.insert(ubarFkey, ubar);

    // trajectories.push_back(traj);
    traj_idx++;
  }
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(10);

  // graph.printErrors(values, "Graph", prx::fg::symbol_factory_t::formatter);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

  gtsam::Values result{ optimizer.optimize() };

  std::ofstream ofs(output_file);

  traj_idx = 0;
  for (auto traj : trajectories)
  {
    for (int i = 0; i <= trajectories[0].size(); ++i)
    {
      const gtsam::Key x0key{ prx_models::mushr_utils_t::keyX(i, traj_idx) };
      const State x{ result.at<State>(x0key) };

      ofs << i << " " << prx::fg::symbol_factory_t::formatter(x0key) << " " << x << "\n";
    }
    traj_idx++;
    ofs << "\n";
  }
  ofs.close();

  // const Params params_estimated{ result.at<Params>(paramskey) };

  // DEBUG_VARS(params_estimated.transpose());
  return 0;
}