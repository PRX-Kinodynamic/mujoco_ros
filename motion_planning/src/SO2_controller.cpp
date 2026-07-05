#include <memory>
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/SO2_system.hpp>

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
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <prx/factor_graphs/factors/fxu_factor.hpp>
#include <motion_planning/error_covariance_estimation.hpp>

struct SO2_close_loop_t
{
  using This = SO2_close_loop_t;
  using DynamicalSystem = prx::SO2_system_t;
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using StateCovariance = Eigen::Matrix2d;

  using Trajectory = std::vector<State>;
  using PlanMsg = ml4kp_bridge::PlanStepStampedArray;

  using SO2PieceWiseStep = prx::piecewise_step_t<prx::SO2_system_t::Control, double>;
  using SO2Controller = std::vector<SO2PieceWiseStep>;
  using CovarianceEstimator = motion_planning::error_covariance_estimation_t<DynamicalSystem>;

  // using Randup = motion_planning::randup_t<DynamicalSystem, Controller>;
  // using ReachabilityGT = motion_planning::reachability_gt_t<DynamicalSystem, Controller>;
  // using RandupCovariance = typename Randup::Covariance;
  // using MGReachability = motion_planning::morse_graph_reachability_t<DynamicalSystem, Controller>;

  ros::Timer _timer;
  ros::Subscriber _sensor_subscriber;
  // ros::Subscriber _state_subscriber, _plan_subscriber, _cov_x0_subscriber, _cov_w_subscriber,
  // _total_trajs_subscriber;
  ros::Publisher _trajectory_publisher, _nominal_trajectory_publisher, _controller_topic;
  std::shared_ptr<motion_planning::safety_checker_t> safety_checker;

  ml4kp_bridge::SpacePointStamped _state_estimate;
  // ml4kp_bridge::PlanStepStampedArray _plan;

  // randup
  bool _randup_time;
  // Randup _randup;
  int _total_trajs;

  // MG
  // MGReachability _mg_reach;

  // Both
  // RandupCovariance _cov_x0, _cov_w;
  ros::Duration _check_duration;

  // GT
  // ReachabilityGT _gt;
  std::shared_ptr<CovarianceEstimator> _cov_estimator;

  bool valid_state, valid_plan;
  std::string algorithm;
  int repetitions;

  SO2_close_loop_t(ros::NodeHandle& nh)
    : _lm_params(prx::fg::default_levenberg_marquardt_parameters())
    , _x0_covariance(Eigen::Matrix2d::Identity() * 0.001)
    , _x_current(0., 0.)
    , _t_curr(0.0)
    , _dt(0.1)
    , _trajectory_type("u0p6")
    , _controller("ZERO")
  {
    std::string plant_parameters, control_topic, sensor_topic_name;

    std::string& trajectory_type{ _trajectory_type };
    std::string& controller{ _controller };
    int& replan_horizon{ _replan_horizon };

    PARAM_SETUP(nh, control_topic)
    PARAM_SETUP(nh, sensor_topic_name)
    PARAM_SETUP_WITH_DEFAULT(nh, controller, "ZERO")
    PARAM_SETUP_WITH_DEFAULT(nh, replan_horizon, 0)
    PARAM_SETUP_WITH_DEFAULT(nh, trajectory_type, trajectory_type)

    GLOBAL_PARAM_BLOCKER(plant_parameters);

    _cov_estimator = std::make_shared<CovarianceEstimator>(nh);

    // prx_assert(algorithm == "randup" or algorithm == "mg" or algorithm == "gt",
    //            "[safety_checker_t] Parameter 'algorithm' needs to be 'randup' or 'mg' ");
    // if (algorithm == "randup")
    // {
    //   bool& randup_time{ _randup_time };
    //   PARAM_SETUP(nh, randup_time)
    // }
    // // safety_checker = std::make_shared<motion_planning::safety_checker_t>(ros::NodeHandle(nh, "safety"));

    _plant = DynamicalSystem::create(plant_parameters);
    _trajectory_publisher = nh.advertise<visualization_msgs::Marker>("/SO2/trajectory/marker", 1);
    _nominal_trajectory_publisher = nh.advertise<visualization_msgs::Marker>("/SO2/trajectory/nominal/marker", 1);

    _controller_topic = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1);

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
    _timer = nh.createTimer(ros::Duration(_dt), &This::timer_callback, this);

    // _state_subscriber = nh.subscribe(state_topic, 1, &safety_helper_t::state_callback, this);
    // _plan_subscriber = nh.subscribe("/safety/input/plan", 1, &safety_helper_t::plan_callback, this);
    // _cov_x0_subscriber = nh.subscribe<ml4kp_bridge::SpacePoint>(
    //     "/safety/input/cov_x0", 1,
    //     boost::bind(&safety_helper_t::covariance_callback, this, boost::placeholders::_1, boost::ref(_cov_x0)));
    // _cov_w_subscriber = nh.subscribe<ml4kp_bridge::SpacePoint>(
    //     "/safety/input/cov_w", 1,
    //     boost::bind(&safety_helper_t::covariance_callback, this, boost::placeholders::_1, boost::ref(_cov_w)));

    // _total_trajs_subscriber =
    //     nh.subscribe("/safety/input/randup/total_trajs", 1, &safety_helper_t::total_randup_trajs_callback, this);

    // _total_time_subscriber =
    //     nh.subscribe("/safety/input/randup/total_time", 1, &safety_helper_t::total_time_callback, this);
    // publish_trajectory();
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    if (_plan.data.size() == 0 or ros::Time::now() > _plan.data[_replan_horizon].header.stamp)
    {
      if (_controller == "FG")
      {
        _plan = compute_controls(_x_current, _x0_covariance);
      }
      else if (_controller == "ZERO")
      {
        _plan = zero_plan();
      }
      _replan_idx = 0;
    }
    // const Control u0{ _plan.data[0].plan_step.control.point[0] };
    // msg.point.push_back(u0);
    // DEBUG_VARS(u0)
    ml4kp_bridge::SpacePoint msg{ _plan.data[_replan_idx].plan_step.control };
    _u0 = msg.point[0];
    _controller_topic.publish(msg);
    _replan_idx++;
    _w_covariance = _cov_estimator->covariance();
    DEBUG_VARS(_w_covariance)
    // _plan.data.erase(_plan.data.begin());
  }

  std::pair<std::vector<State>, std::vector<Control>> nominal_trajectory(const State x0, const double horizon) const
  {
    DEBUG_VARS(_trajectory_type)
    std::vector<State> traj;
    std::vector<Control> plan;
    if (_trajectory_type == "u0p6")
    {
      using SO2PieceWiseStep = std::vector<prx::piecewise_step_t<prx::SO2_system_t::Control, double>>;
      using FwdProp = prx::forward_propagation_t<DynamicalSystem, Trajectory, SO2PieceWiseStep>;

      SO2PieceWiseStep controller;
      for (double i = 0; i < horizon; i += _dt)
      {
        // piecewise_step_t(ControlType control, const DurationType duration) : control(control), duration(duration)
        controller.emplace_back(0.6, 0.1);
        plan.emplace_back(0.6);
      }

      FwdProp::propagate(traj, x0, controller, _plant);
    }

    return { traj, plan };
  }

  // State state_from_trajectory(const gtsam::Rot2 R) const
  State state_from_trajectory(const double t) const
  {
    // if (_trajectory_type == "trajectory")
    // {
    // const double rx{ 3. };
    // const double ry{ 1. };
    // // const double t{ R.theta() };
    // const double cx{ prx::constants::pi };
    const double cy{ 1.5 };
    const double tp{ 3. * t };
    // return State(-(cx + rx * std::cos(t)), cy + ry * std::sin(t));
    return State(t, cy + std::sin(tp));
    // }
    // else
    // {
    //   return State(0., 0.);
    // }
    // return State(t, std::sin(t));
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr& msg)
  {
    const double& th{ msg->raw_sensor_data[0] };
    const double& thdot{ msg->raw_sensor_data[1] };

    const double dt{ (msg->header.stamp - _t_prev).toSec() };
    _t_prev = ros::Time::now();
    const State x1hat(th, thdot);
    _cov_estimator->add_estimate(x1hat, _x_current, _u0, dt);
    _x_current = x1hat;
  }

  void publish_trajectory()
  {
    visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.1, /*color*/ { 0.5, 1, 0, 1 }) };
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    // PRINT_MSG("Creating trajectory")
    for (double ti = 0; ti < 2 * prx::constants::pi; ti += 0.01)
    {
      const State xi{ state_from_trajectory(ti) };
      marker.points.emplace_back();
      marker.points.back().x = xi.first.theta();
      marker.points.back().y = xi.second;
      marker.points.back().z = 0.;
    }
    _trajectory_publisher.publish(marker);
  }

  PlanMsg values_to_plan(const gtsam::Values values, const int T) const
  {
    PlanMsg msg;

    ros::Time now{ ros::Time::now() };
    const ros::Duration r_dt{ ros::Duration(_dt) };

    for (int i = 0; i < T; ++i)
    {
      const gtsam::Key ku{ gtsam::Symbol('U', i) };
      const Control ui{ values.at<Control>(ku) };

      msg.data.emplace_back();
      msg.data.back().header.stamp = now;
      msg.data.back().plan_step.control.point.push_back(ui);
      msg.data.back().plan_step.duration.data = r_dt;

      now += r_dt;
    }
    return msg;
  }

  PlanMsg zero_plan()
  {
    gtsam::Values values;
    values.insert(gtsam::Symbol('U', 0), 0.);
    values.insert(gtsam::Symbol('U', 1), 0.);
    return values_to_plan(values, 2);
  }

  PlanMsg compute_controls(const State x0, const StateCovariance x0_covariance) const
  {
    using TransitionFactor = prx::fg::fxu_factor_t<State, Control, DynamicalSystemPtr>;
    using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

    using FgCtrlr = std::tuple<gtsam::NonlinearFactorGraph, gtsam::Values, gtsam::LevenbergMarquardtParams, int>;
    using FgFwdProp = prx::forward_propagation_t<DynamicalSystem, Trajectory, FgCtrlr>;

    double horizon{ 1.0 };
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const NoiseModel x0_nm{ gtsam::noiseModel::Gaussian::Covariance(x0_covariance) };
    const NoiseModel transition_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    const NoiseModel prior_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };

    const auto [traj, plan] = nominal_trajectory(x0, horizon);

    int i{ 0 };

    gtsam::Key kx0{ gtsam::Symbol('X', i) };
    gtsam::Key kx1{ gtsam::Symbol('X', i + 1) };
    gtsam::Key ku0{ gtsam::Symbol('U', i) };

    graph.addPrior(kx0, x0, x0_nm);
    gtsam::Rot2 q0(x0.first);

    for (auto&& ui : plan)
    {
      const State& xi{ traj[i] };

      kx0 = gtsam::Symbol('X', i);
      kx1 = gtsam::Symbol('X', i + 1);
      ku0 = gtsam::Symbol('U', i);

      graph.emplace_shared<TransitionFactor>(kx0, ku0, kx1, transition_nm, _dt, _plant);
      values.insert(kx0, xi);
      values.insert(ku0, ui);
      graph.addPrior(kx0, xi, prior_nm);

      i++;
    }
    kx0 = gtsam::Symbol('X', i);
    values.insert(kx0, traj[i]);
    graph.addPrior(kx1, traj[i], prior_nm);

    Trajectory fg_traj;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, _lm_params);
    const gtsam::Values result{ optimizer.optimize() };

    values_to_marker(result, i);
    PlanMsg msg{ values_to_plan(result, i) };

    // graph.printErrors(result);
    return msg;
  }

  PlanMsg compute_controls_p(const State x0, const StateCovariance x0_covariance) const
  {
    using TransitionFactor = prx::fg::fxu_factor_t<State, Control, DynamicalSystemPtr>;
    using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

    double horizon{ 1.0 };
    // double dt{ 0.1 };

    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const NoiseModel x0_nm{ gtsam::noiseModel::Gaussian::Covariance(x0_covariance) };
    const NoiseModel transition_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    const NoiseModel prior_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    int i{ 0 };
    double ti{ 0 };

    gtsam::Key kx0{ gtsam::Symbol('X', 0) };
    gtsam::Key kx1{ gtsam::Symbol('X', 1) };
    gtsam::Key ku0{ gtsam::Symbol('U', 0) };

    graph.addPrior(kx0, x0, x0_nm);
    gtsam::Rot2 q0(x0.first);
    // DEBUG_VARS(x0, q0)
    for (; ti < horizon; ti += _dt, i++)
    {
      kx0 = gtsam::Symbol('X', i);
      kx1 = gtsam::Symbol('X', i + 1);
      ku0 = gtsam::Symbol('U', i);

      graph.emplace_shared<TransitionFactor>(kx0, ku0, kx1, transition_nm, _dt, _plant);

      const State xi{ state_from_trajectory(q0.theta()) };
      values.insert(kx0, xi);
      values.insert(ku0, 0.0);
      graph.addPrior(kx0, xi, prior_nm);
      q0 = q0 * gtsam::traits<gtsam::Rot2>::Expmap(Eigen::Vector<double, 1>(_dt));
      // DEBUG_VARS(q0, xi)
    }
    const State xT{ state_from_trajectory(q0.theta()) };
    values.insert(kx1, xT);
    graph.addPrior(kx1, xT, prior_nm);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, _lm_params);
    const gtsam::Values result{ optimizer.optimize() };

    values_to_marker(result, i);
    PlanMsg msg{ values_to_plan(result, i) };

    // graph.printErrors(result);
    return msg;
  }

  void values_to_marker(const gtsam::Values& result, const int T) const
  {
    visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.1, /*color*/ { 0.8, 0, 0.8, 0.1 }) };
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::DELETEALL;
    _nominal_trajectory_publisher.publish(marker);

    marker.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < T; ++i)
    {
      const gtsam::Key kx{ gtsam::Symbol('X', i) };
      const State x{ result.at<State>(kx) };

      marker.points.emplace_back();
      ml4kp_bridge::update_point(marker.points.back(), x, 0, 1, 0.1);
    }

    _nominal_trajectory_publisher.publish(marker);
  }

  std::string _controller;
  double _u0;
  ros::Time _t_prev;

  CovarianceEstimator::Covariance _w_covariance;

  std::string _trajectory_type;
  int _replan_horizon, _replan_idx;
  double _t_curr, _dt;
  State _x_current;
  StateCovariance _x0_covariance;
  DynamicalSystemPtr _plant;
  gtsam::LevenbergMarquardtParams _lm_params;
  PlanMsg _plan;
};

int main(int argc, char** argv)
{
  const std::string node_name{ "SO2Controller" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // std::shared_ptr<SO2HelperPiecewise> SO2_helper;
  // std::shared_ptr<MushrHelperPiecewise> mushr_helper;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  SO2_close_loop_t controller(nh);

  // controller.run();
  // SO2_helper = std::make_shared<SO2HelperPiecewise>(nh);

  while (ros::ok())
  {
    controller.publish_trajectory();
    // controller.publish_trajectory();
    // controller.compute_controls();
    ros::Duration(1.0).sleep();
  }
  spinner.stop();

  return 0;
}