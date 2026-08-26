#include <memory>
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/defs.hpp>
#include <prx_models/unicycle_fg_tracking.hpp>

#include <motion_planning/stela_sliding_window.hpp>
#include <motion_planning/tree_validation.hpp>

#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>
#include <interface/levenberg_marquardt_interface.hpp>
#include <interface/StelaStatus.h>
#include <std_msgs/Int32.h>
#include <control/mushr_contingency_controllers.hpp>
#include <motion_planning/goal_checker.hpp>
#include <motion_planning/safety_checker.hpp>
#include <motion_planning/randup.hpp>
#include <prx_models/mushr.hpp>

// #include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <prx_models/StelaKraft.h>
#include <motion_planning/morse_graph_reachability.hpp>
#include <motion_planning/reachability_gt.hpp>
#include <prx_models/unicycle_model.hpp>
#include <motion_planning/gotube.hpp>
// #include <motion_planning/scene_optimization.hpp>
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using State = typename prx::unicycle_model_t::State;
using Control = typename prx::unicycle_model_t::Control;
using Trajectory = std::vector<State>;
using Plan = std::vector<prx::piecewise_step_t<Control, double>>;
using StateSampler = prx::lie_group_gaussian_noise_t<State>;
using StateDotSampler = prx::multivariate_gaussian_t<3>;

using LessThanFn = prx::fg::VectorLessThanCmp<Control>;
using GreaterThanFn = prx::fg::VectorGreaterThanCmp<Control>;
using LessThanFactor = prx::fg::constraint_factor_t<Control, LessThanFn>;
using GreaterThanFactor = prx::fg::constraint_factor_t<Control, GreaterThanFn>;

using UnicyclePieceWiseStep = prx::piecewise_step_t<prx::unicycle_model_t::Control, double>;
using UnicycleOpenLoopController = std::vector<UnicyclePieceWiseStep>;
// using UnicycleFGController = prx::fg_trajectory_tracking_controller_t<prx::unicycle_model_t>;

using FwdPropOpenLoop = prx::forward_propagation_t<prx::unicycle_model_t, Trajectory, UnicycleOpenLoopController>;
using FwdPropFG = prx::forward_propagation_t<prx::unicycle_model_t, Trajectory,
                                             prx::fg_trajectory_tracking_controller_t<prx::unicycle_model_t>>;

// template <typename State, typename Control>

// struct UnicycleFGController
//   : public prx::fg_trajectory_tracking_controller_t<prx::unicycle_model_t::State, prx::unicycle_model_t::Control>
// {
//   using Base = fg_trajectory_tracking_controller_t<prx::unicycle_model_t::State, prx::unicycle_model_t::Control>;
//   using State = prx::unicycle_model_t::State;
//   using Control = prx::unicycle_model_t::Control;

//   std::shared_ptr<prx::unicycle_model_t> plant;
//   Control u_min;
//   Control u_max;
//   // Control fg_control(std::shared_ptr<prx::unicycle_model_t> plant, const Trajectory traj_gt, const Plan plan,
//   //                      const State& xt, const Control u_min, const Control u_max,
//   //                      gtsam::LevenbergMarquardtParams& lm_params)

//   virtual Control fg_control(const State& xt, const Base::Trajectory& traj_gt, const Base::Plan& plan) const override
//   {
//     gtsam::Values values;
//     gtsam::NonlinearFactorGraph graph;

//     // PRINT_MSG("Building FG")
//     const NoiseModel f_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
//     const NoiseModel x0_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-3) };

//     // DEBUG_VARS(traj_gt.size())
//     for (int i = 0; i < traj_gt.size() - 1; ++i)
//     {
//       const gtsam::Key xk0{ gtsam::Symbol('X', i) };
//       const gtsam::Key xk1{ gtsam::Symbol('X', i + 1) };
//       const gtsam::Key uk01{ gtsam::Symbol('U', i) };

//       // const State xi_w{ traj_w[i] };
//       const State xi_gt{ traj_gt[i] };

//       values.insert(xk0, xi_gt);
//       values.insert(uk01, plan[i].control);

//       if (i != 0)
//       {
//         graph.addPrior(xk0, xi_gt);
//       }
//       graph.emplace_shared<LessThanFactor>(uk01, u_min);
//       graph.emplace_shared<GreaterThanFactor>(uk01, u_max);
//       graph.emplace_shared<prx::unicycle_factor_t>(xk1, xk0, uk01, plant, f_nm);
//     }

//     const gtsam::Key xk0{ gtsam::Symbol('X', 0) };
//     const gtsam::Key xkT{ gtsam::Symbol('X', traj_gt.size() - 1) };
//     // values.insert(xk0, traj_w.front());
//     graph.addPrior(xk0, xt, x0_nm);
//     values.insert(xkT, traj_gt.back());

//     // values.print();

//     std::vector<State> traj;
//     std::vector<Control> ctrls;

//     gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

//     gtsam::Values result{ optimizer.optimize() };
//     // for (int i = 0; i < traj_gt.size(); ++i)
//     // {
//     //   const gtsam::Key xk0{ gtsam::Symbol('X', i) };
//     //   traj.push_back(result.at<State>(xk0));
//     // }
//     // for (int i = 0; i < traj_gt.size() - 1; ++i)
//     // {
//     // const gtsam::Key uk{ gtsam::Symbol('U', i) };
//     // ctrls.push_back(result.at<Control>(uk));
//     // }
//     const gtsam::Key ku0{ gtsam::Symbol('U', 0) };
//     const Control u0{ result.at<Control>(ku0) };

//     return u0;
//     // return { traj, ctrls };
//   }
// };

int main(int argc, char** argv)
{
  const std::string node_name{ "unicycle_tracking_fg" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  gtsam::LevenbergMarquardtParams lm_params;
  interface::initialize(lm_params, ros::NodeHandle(nh, "lm"));
  // std::vector<Eigen::Vector2d> plan(5, Eigen::Vector2d(1., -0.99));
  // plan.push_back(Eigen::Vector2d::Zero());

  ros::Publisher w_publisher{ nh.advertise<visualization_msgs::Marker>("/OpenLoop", 1, true) };
  ros::Publisher gt_publisher{ nh.advertise<visualization_msgs::Marker>("/GT", 1, true) };
  ros::Publisher fg_publisher{ nh.advertise<visualization_msgs::Marker>("/FG", 1, true) };

  std::string plant_parameters;
  GLOBAL_PARAM_BLOCKER(plant_parameters);

  std::shared_ptr<prx::unicycle_model_t> plant;
  plant = prx::unicycle_model_t::create(plant_parameters);

  std::vector<State> traj_gt;

  State x0_gt{ State(0., -0.5, 1.57) };
  traj_gt.push_back(x0_gt);

  prx::simulation_step = 0.1;
  std::vector<prx::piecewise_step_t<prx::unicycle_model_t::Control, double>> plan;
  for (int i = 0; i < 5; ++i)
  {
    plan.emplace_back(Eigen::Vector2d(1.0, -0.99), prx::simulation_step);
  }
  for (int i = 0; i < 5; ++i)
  {
    plan.emplace_back(Eigen::Vector2d(1.0, 0.99), prx::simulation_step);
  }
  // plan.emplace_back(Eigen::Vector2d(1.0, -0.99), prx::simulation_step);

  FwdPropOpenLoop::propagate(traj_gt, plan, plant);

  StateSampler x0_sampler;
  StateDotSampler w_sampler;
  Eigen::Matrix3d cov_w{ Eigen::Matrix3d::Identity() * 0.01 };
  Eigen::Matrix3d cov_x0{ Eigen::Matrix3d::Identity() * 0.001 };
  cov_x0(2, 2) *= 4;
  w_sampler.set(cov_w);
  x0_sampler.set(cov_x0);

  // prx::unicycle_fg_controller_t fg_controller;
  prx::fg_trajectory_tracking_controller_t<prx::unicycle_model_t> fg_controller;
  fg_controller.plant = plant;
  fg_controller.u_max = Control(1.1, 1.1);
  fg_controller.u_min = Control(-1.1, -1.1);

  fg_controller.plan = plan;
  fg_controller.traj_nominal = traj_gt;
  fg_controller.lm_params = lm_params;
  // fg_controller.fg_control = [&](const State& x0, const Trajectory& traj, const Plan& plan) {
  //   return plan_from_fg(plant, traj, plan, x0, u_min, u_max, lm_params);
  // };

  while (true)
  {
    std::vector<State> traj_w, traj_fg;
    const State x0_noise{ x0_sampler(x0_gt) };
    DEBUG_VARS(x0_noise)

    traj_w.push_back(x0_noise);
    FwdPropOpenLoop::propagate(traj_w, plan, plant);

    // traj_fg.push_back(x0_noise);
    FwdPropFG::propagate(traj_fg, x0_noise, fg_controller, plant, w_sampler);

    // auto [traj_fg, plan_fg] = plan_from_fg(plant, traj_gt, plan, traj_w[0], u_min, u_max, lm_params);

    // Markers
    // nh.advertise()

    visualization_msgs::Marker marker_w{ ml4kp_bridge::create_marker(0.01, /*color*/ { 1, 1, 0, 0 }) };
    visualization_msgs::Marker marker_gt{ ml4kp_bridge::create_marker(0.01, /*color*/ { 1, 0, 1, 0 }) };
    visualization_msgs::Marker marker_fg{ ml4kp_bridge::create_marker(0.01, /*color*/ { 1, 0, 0, 1 }) };
    marker_w.type = marker_gt.type = marker_fg.type = visualization_msgs::Marker::LINE_LIST;

    ml4kp_bridge::update_marker(marker_w, traj_w, 0, 1, 2, visualization_msgs::Marker::LINE_LIST);
    ml4kp_bridge::update_marker(marker_gt, traj_gt, 0, 1, 2, visualization_msgs::Marker::LINE_LIST);
    ml4kp_bridge::update_marker(marker_fg, traj_fg, 0, 1, 2, visualization_msgs::Marker::LINE_LIST);

    w_publisher.publish(marker_w);
    gt_publisher.publish(marker_gt);
    fg_publisher.publish(marker_fg);
    // DEBUG_VARS(traj_gt);
    // DEBUG_VARS(marker_gt);

    ros::spinOnce();
    int n;
    try
    {
      std::cin >> n;
    }
    catch (...)
    {
      PRINT_MSG("Catch")
      return 0;
    }
    if (n == 0)
    {
      PRINT_MSG("Bye!")
      return 0;
    }
  }

  return 0;
}