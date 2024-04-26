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

// ML4KP
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"

// GTSAM
#include <gtsam/nonlinear/ISAM2.h>

template <typename Model, typename IntegrationFactor, typename DynamicsFactor, typename ObservationFactor>
class trajectory_estimator
{
  using NoiseModel = typename gtsam::noiseModel::Base::shared_ptr;
  using Observation = typename Model::Observation;
  using State = typename Model::State;
  using StateDot = typename Model::StateDot;
  using Control = typename Model::Control;

  using TrajectoryEstimation = estimation::fg::trajectory_estimation;

  using SF = prx::fg::symbol_factory_t;

public:
  trajectory_estimator(ros::NodeHandle& nh)
    : _tf_listener(_tf_buffer)
    , _isam(gtsam::ISAM2Params())
    , _prev_header()
    , _x(State::Zero())
    , _xdot(State::Zero())
    , _u(Control::Zero())
    , _trajectory_estimation_service_name("/trajectory")
    , _state_estimation_service_name("/current_state")
    , idx(0)
  {
    _trajectory_estimation_service_name = ros::this_node::getNamespace() + _trajectory_estimation_service_name;
    _state_estimation_service_name = ros::this_node::getNamespace() + _state_estimation_service_name;

    std::string control_topic;
    std::vector<double> start_state;
    std::vector<double> start_statedot;
    std::vector<double> sigmas_dynamics;
    std::vector<double> sigmas_integration;
    std::vector<double> sigmas_observation;

    PARAM_SETUP(nh, start_state);
    PARAM_SETUP(nh, start_statedot);
    PARAM_SETUP(nh, control_topic);
    PARAM_SETUP(nh, sigmas_integration);
    PARAM_SETUP(nh, sigmas_observation);
    PARAM_SETUP(nh, sigmas_dynamics);
    utils::get_param_and_check(nh, "world_frame", _world_frame);
    utils::get_param_and_check(nh, "robot_frame", _robot_frame);

    Eigen::VectorXd diagonal_dynamics(sigmas_dynamics.size());
    Eigen::VectorXd diagonal_integration(sigmas_integration.size());
    Eigen::VectorXd diagonal_observation(sigmas_observation.size());
    for (int i = 0; i < sigmas_integration.size(); ++i)
    {
      diagonal_integration[i] = sigmas_integration[i];
    }
    for (int i = 0; i < sigmas_observation.size(); ++i)
    {
      diagonal_observation[i] = sigmas_observation[i];
    }
    for (int i = 0; i < sigmas_dynamics.size(); ++i)
    {
      diagonal_dynamics[i] = sigmas_dynamics[i];
    }
    for (int i = 0; i < start_state.size(); ++i)
    {
      _x[i] = start_state[i];
    }
    for (int i = 0; i < start_statedot.size(); ++i)
    {
      _xdot[i] = start_statedot[i];
    }

    _dynamics_noise = gtsam::noiseModel::Diagonal::Sigmas(diagonal_dynamics);
    _integration_noise = gtsam::noiseModel::Diagonal::Sigmas(diagonal_integration);
    _observation_noise = gtsam::noiseModel::Diagonal::Sigmas(diagonal_observation);

    k_u = [](const std::size_t& idx) { return SF::create_hashed_symbol("U_{", idx, "}"); };
    k_x = [](const std::size_t& idx) { return SF::create_hashed_symbol("X_{", idx, "}"); };
    k_xdot = [](const std::size_t& idx) { return SF::create_hashed_symbol("\\dot{X}_{", idx, "}"); };

    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    graph.addPrior(k_x(idx), _x);
    graph.addPrior(k_xdot(idx), _xdot);
    values.insert(k_x(idx), _x);
    values.insert(k_xdot(idx), _xdot);
    _isam2_result = _isam.update(graph, values);

    _prev_header.stamp = ros::Time::now();

    _control_subscriber = nh.subscribe(control_topic, 1, &trajectory_estimator::control_callback, this);

    _trajectory_estimation_service =
        nh.advertiseService(_trajectory_estimation_service_name, &trajectory_estimator::estimate_trajectory, this);
    _state_estimation_service =
        nh.advertiseService(_state_estimation_service_name, &trajectory_estimator::estimate_current_state, this);

    _estimator_timer = nh.createTimer(ros::Duration(0.1), &trajectory_estimator::add_observations, this);
  }

  ~trajectory_estimator(){};

  void control_callback(const ml4kp_bridge::SpacePointConstPtr msg)
  {
    Model::copy(_u, msg);
  }

  void add_observations(const ros::TimerEvent& event)
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));

      if (_tf.header.stamp > _prev_header.stamp)
      {
        const double dt{ (_tf.header.stamp - _prev_header.stamp).toSec() };
        Model::copy(_z, _tf);

        _xdot = DynamicsFactor::predict(_xdot, _u, dt);  // \dot{x} = f(\dot{x}, u)
        _x = IntegrationFactor::predict(_x, _xdot, dt);  // x_{t+1} = x_t + \dot{x}_t * dt

        gtsam::NonlinearFactorGraph graph{
          TrajectoryEstimation::get_dyn_factor<IntegrationFactor, DynamicsFactor, ObservationFactor>(
              idx, dt, _z, k_x, k_xdot, k_u, _integration_noise, _observation_noise)
        };
        gtsam::Values values{ TrajectoryEstimation::get_dyn_values(idx, _x, _xdot, _u, k_x, k_xdot, k_u) };
        graph.addPrior(k_u(idx), _u);

        // graph.print("graph:", SF::formatter);
        // values.print("values:", SF::formatter);
        // SF::symbols_to_file("/Users/Gary/pracsys/catkin_ws/factor_graphs_symbols.txt");

        _isam2_result = _isam.update(graph, values);

        _prev_header = _tf.header;
        idx++;
      }
    }
    catch (tf2::TransformException& ex)
    {
      // ROS_WARN("%s", ex.what());
      // ros::Duration(1.0).sleep();
      // continue;
    }
  }

  bool estimate_trajectory(estimation::TrajectoryEstimation::Request& req,
                           estimation::TrajectoryEstimation::Response& res)
  {
    const gtsam::Values estimate{ _isam.calculateEstimate() };

    for (std::size_t i = 0; i < idx; ++i)
    {
      res.trajectory.data.emplace_back();
      const State x{ estimate.at<State>(k_x(i)) };
      const StateDot xdot{ estimate.at<StateDot>(k_xdot(i)) };
      Model::copy(res.trajectory.data.back(), x, xdot);
    }

    const std::filesystem::path path(req.command.data);
    if (std::filesystem::exists(path.parent_path()))
    {
      ml4kp_bridge::to_file(res.trajectory, req.command.data, std::ofstream::trunc);
    }
    return true;
  }

  bool estimate_current_state(estimation::StateEstimation::Request& req, estimation::StateEstimation::Response& res)
  {
    if (idx == 0)  // no observation yet
      return false;
    const std::size_t t_now{ idx - 1 };
    const State xt{ _isam.calculateEstimate<State>(k_x(t_now)) };
    const State xtdot{ _isam.calculateEstimate<StateDot>(k_xdot(t_now)) };

    Model::copy(res.state, xt, xtdot);

    return true;
  }

private:
  gtsam::ISAM2 _isam;
  gtsam::Values _values;
  gtsam::NonlinearFactorGraph _graph;
  gtsam::ISAM2Result _isam2_result;

  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;

  ros::Subscriber _control_subscriber;

  ros::ServiceServer _trajectory_estimation_service;
  ros::ServiceServer _state_estimation_service;

  std::string _world_frame;
  std::string _robot_frame;
  std::string _trajectory_estimation_service_name;
  std::string _state_estimation_service_name;
  std_msgs::Header _prev_header;

  std::size_t idx;
  State _x;
  StateDot _xdot;
  Observation _z;
  Control _u;
  NoiseModel _integration_noise;
  NoiseModel _observation_noise;
  NoiseModel _dynamics_noise;
  std::function<gtsam::Key(const std::size_t&)> k_u;
  std::function<gtsam::Key(const std::size_t&)> k_x;
  std::function<gtsam::Key(const std::size_t&)> k_xdot;

  ros::Timer _estimator_timer;
};

struct ltv_sde
{
  using Observation = Eigen::Vector2d;
  using State = Eigen::Vector2d;
  using StateDot = Eigen::Vector2d;
  using Control = Eigen::Vector2d;
};

int main(int argc, char** argv)
{
  const std::string node_name{ "TrajectoryEstimationFromTf" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  using EulerFactor = prx::fg::euler_integration_factor_t<ltv_sde::State, ltv_sde::StateDot>;
  using DynamicsFactor = prx::fg::euler_integration_factor_t<ltv_sde::StateDot, ltv_sde::Control>;
  using ObservationFactor = gtsam::PriorFactor<ltv_sde::State>;
  trajectory_estimator<prx::fg::ltv_sde_utils_t, EulerFactor, DynamicsFactor, ObservationFactor> estimator(nh);

  ros::spin();

  return 0;
}