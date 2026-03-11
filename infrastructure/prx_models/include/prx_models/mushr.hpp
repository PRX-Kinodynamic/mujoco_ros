#pragma once

#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <string>

// Ros
#include <Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/tree_msg_wrapper.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/stela_robot_interface.hpp>
#include <prx_models/Edge.h>
#include <prx_models/tree_utils.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <interface/SensorDataStamped.h>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>

namespace prx_models
{
class mushrFG_t;

struct mushr_types_t
{
  static constexpr std::string_view plant_name = "mushrFG";
  using State = prx_models::mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using StateDotDot = mushr_types::StateDot::type;

  using Control = mushr_types::Control::type;
  // using Noise = Eigen::Vector<double, 2>;
  using Observation = State;

  using StateKeys = std::array<gtsam::Key, 2>;
  using ControlKeys = std::array<gtsam::Key, 1>;
  using TimeKeys = std::array<gtsam::Key, 1>;

  using StateEstimates = std::tuple<State, StateDot>;
  using ControlEstimates = std::tuple<Control>;
};

class mushr_stela_t : public stela_robot_interface_t<mushr_stela_t, mushr_types_t>
{
  using This = mushr_stela_t;
  using Base = stela_robot_interface_t<mushr_stela_t, mushr_types_t>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

  using XVelFactor = prx_models::mushr_x_xdot_t;
  using VelUbarFactor = prx_models::mushr_xdot_ub_t;
  using CtrlUbarFactor = prx_models::mushr_ub_u_xdot_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = typename mushr_types_t::State;
  using StateDot = typename mushr_types_t::StateDot;
  using StateDotDot = typename mushr_types_t::StateDotDot;

  using Control = typename mushr_types_t::Control;
  // using Noise = typename mushr_types_t::Noise;
  using Observation = typename mushr_types_t::Observation;

  using StateKeys = typename mushr_types_t::StateKeys;
  using ControlKeys = typename mushr_types_t::ControlKeys;
  using TimeKeys = typename mushr_types_t::TimeKeys;

  using StateEstimates = typename mushr_types_t::StateEstimates;
  using ControlEstimates = typename mushr_types_t::ControlEstimates;

  using Poly = mushr_types::Control::Poly;
  using Parameters = mushr_types::Control::params;
  using PrxPlant = mushrFG_t;

  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };

  mushr_stela_t() : Base() {};

  mushr_stela_t(ros::NodeHandle& nh)
    : Base(nh, State::Zero(), StateDot::Zero(), Control::Zero(), 0.1, Control::Zero(), Control::Zero())
    , _params(This::default_params)
    , _poly(This::default_poly)
  {
    std::string sensor_topic_name, current_state_topic;

    PARAM_SETUP(nh, sensor_topic_name)
    PARAM_SETUP(nh, current_state_topic)

    DEBUG_VARS(sensor_topic_name)
    ros::NodeHandle nh_ctrl(nh, "control_space");

    std::vector<double> lower_bound;
    std::vector<double> upper_bound;

    PARAM_SETUP(nh_ctrl, lower_bound)
    PARAM_SETUP(nh_ctrl, upper_bound)

    _ctrl_lower_bound = Control(lower_bound.data());
    _ctrl_upper_bound = Control(upper_bound.data());

    _current_state_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(current_state_topic, 1, true);
    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<double>& zi{ msg->raw_sensor_data };
    _last_observation.first[0] = zi[0];
    _last_observation.first[1] = zi[1];
    const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3], zi[4], zi[5], zi[6]) };
    _last_observation.first[2] = prx::quaternion_to_euler(q)[2];
    _last_observation.second = msg->header.stamp;
    _new_observation = true;
    // DEBUG_VARS(_last_observation.first);
    // DEBUG_VARS(_new_observation, _last_observation.first[0], _last_observation.first[1], _last_observation.first[2])
  }

  // Factor graph for "Idle" state (i.e. before starting execution or after reaching the goal)
  virtual GraphValues idle_state_to_fg(const std::size_t parent, const std::size_t child,
                                       const bool time_as_variable = true) override
  {
    // return node_edge_to_fg(parent, child, _idle_state, _idle_state_dot, _idle_control, _idle_dt, time_as_variable);
    using StateStateDotTimeFactor = prx_models::mushr_x_xdot_t;
    using StateStateDotNoTimeFactor = prx_models::mushr_x_xdot_nodT_t;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
    using XdotIntegrationTimeFactor = prx_models::mushr_CtrlAccel_t<double>;
    using XdotIntegrationNoTimeFactor = prx_models::mushr_CtrlAccel_t<>;
    using NHCFactor = prx_models::mushr_NHC_t;

    GraphValues graph_values;

    const gtsam::Key k_x0{ keyX(1, parent) };
    const gtsam::Key k_x1{ keyX(1, child) };

    const gtsam::Key k_xdot0{ keyXdot(1, parent) };
    const gtsam::Key k_xdot1{ keyXdot(1, child) };

    const gtsam::Key k_u01{ keyU(parent, child) };
    const gtsam::Key k_t01{ keyT(parent, child) };

    NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e0) };
    NoiseModel dt_limit_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-1) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };
    NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };

    if (time_as_variable)
    {
      graph_values.first.emplace_shared<XdotIntegrationTimeFactor>(k_xdot1, k_xdot0, k_u01, k_t01, xd_integration_noise,
                                                                   _params, _poly);
      graph_values.first.emplace_shared<StateStateDotTimeFactor>(k_x1, k_x0, k_xdot0, k_t01, integration_noise);
      graph_values.first.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_limit_noise);
      graph_values.first.addPrior(k_t01, _idle_dt, dt_noise);
    }
    else
    {
      graph_values.first.emplace_shared<XdotIntegrationNoTimeFactor>(k_xdot1, k_xdot0, k_u01, _idle_dt,
                                                                     xd_integration_noise, _params, _poly);
      graph_values.first.emplace_shared<StateStateDotNoTimeFactor>(k_x1, k_x0, k_xdot0, integration_noise, _idle_dt);
      PRINT_MSG_ONCE("Using fix time!")
    }

    // NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    // NoiseModel x_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    // NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };

    graph_values.first.addPrior(k_u01, _idle_control, u_prior_noise);
    // graph_values.first.addPrior(k_x1, _idle_state, prior_noise);
    graph_values.first.addPrior(k_xdot1, _idle_state_dot, xdot_prior_noise);

    graph_values.second.insert(k_t01, _idle_dt);
    graph_values.second.insert(k_x1, _idle_state);

    graph_values.second.insert(k_xdot1, _idle_state_dot);
    graph_values.second.insert(k_u01, _idle_control);

    return graph_values;
  }

  virtual void publish_current_state(const StateEstimates& estimates) override
  {
    ml4kp_bridge::SpacePointStamped msg;
    msg.header.stamp = ros::Time::now();

    msg.space_point.point.push_back(std::get<0>(estimates)[0]);
    msg.space_point.point.push_back(std::get<0>(estimates)[1]);
    msg.space_point.point.push_back(std::get<0>(estimates)[2]);

    msg.space_point.point.push_back(std::get<1>(estimates)[0]);
    msg.space_point.point.push_back(std::get<1>(estimates)[1]);
    msg.space_point.point.push_back(std::get<1>(estimates)[2]);

    _current_state_publisher.publish(msg);
  }

  virtual void copy_estimates(ml4kp_bridge::SpacePoint& pt, const StateEstimates& estimates) override
  {
    const State& x{ std::get<0>(estimates) };
    const StateDot& xdot{ std::get<1>(estimates) };

    pt.point.resize(6);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = x[2];
    pt.point[3] = xdot[0];
    pt.point[4] = xdot[1];
    pt.point[5] = xdot[2];
  }

  virtual void copy_control(Control& u, const ml4kp_bridge::SpacePoint& msg) override
  {
    u[mushr_types::Control::vel_desired] = msg.point[mushr_types::Control::vel_desired];
    u[mushr_types::Control::steering] = msg.point[mushr_types::Control::steering];
  }

  virtual void copy_control(ml4kp_bridge::SpacePoint& msg, const Control& u) override
  {
    msg.point[mushr_types::Control::vel_desired] = u[mushr_types::Control::vel_desired];
    msg.point[mushr_types::Control::steering] = u[mushr_types::Control::steering];
  }

  virtual void copy_state(State& x, const ml4kp_bridge::SpacePoint& pt) override
  {
    x[0] = pt.point[0];
    x[1] = pt.point[1];
    x[2] = pt.point[2];
  }

  virtual void copy_stateDot(StateDot& xd, const ml4kp_bridge::SpacePoint& pt) override
  {
    xd[0] = pt.point[3];
    xd[1] = pt.point[4];
    xd[2] = pt.point[5];
  }

  using Base::distance;

  virtual double distance(const State& x0, const State& x1) override
  {
    const State between{ x0.between(x1) };
    const Eigen::VectorXd error{ State::Logmap(between) };
    return error.norm();
  }

  struct ConfigFromState
  {
    void operator()(Eigen::Matrix3d& rotation, Eigen::Vector3d& translation, const State& state)
    {
      const Eigen::Vector<double, 1> vec{ state.angle() };
      rotation = prx::euler_to_rotation<Eigen::Matrix3d>(vec, "Z");
      translation[0] = state[0];
      translation[1] = state[1];
      translation[2] = 0;
    }

    // void operator()(const Eigen::Vector3d& translation, const State& state, Eigen::MatrixXd& H)
    // void operator()(const State& state, const Eigen::Vector3d& translation, Eigen::MatrixXd& H)
    // {
    //   const Eigen::Vector2d normalized{ translation.head(2).normalized() };
    //   H = Eigen::Matrix<double, 1, 3>::Zero();
    //   H(0, 0) = normalized[0];
    //   H(0, 1) = normalized[1];
    // }

    void operator()(const bool collision, const State& state, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                    Eigen::MatrixXd& H)
    {
      H = Eigen::Matrix<double, 1, 3>::Zero();
      Eigen::Vector2d vec{ (p1 - p2).head(2) };
      if (collision)
      {
        vec = p1.head(2);
      }

      // vT =  HT * R
      // V = (HT * R)T = RT * H
      const Eigen::Matrix2d R{ state.rotation<Eigen::Matrix2d>() };
      vec = -R.transpose() * vec;

      const double Sth{ std::sin(state[2]) };
      const double Cth{ std::cos(state[2]) };
      const double ax{ collision ? p1[0] : (p1[0] - state[0]) };
      const double ay{ collision ? p1[1] : (p1[1] - state[1]) };
      const double fx{ vec[0] };
      const double fy{ vec[1] };
      H(0, 0) = vec[0];
      H(0, 1) = vec[1];
      H(0, 2) = -fx * (ax * Sth + ay * Cth) + fy * (ax * Cth - ay * Sth);
    }

    void configuration(Eigen::Vector2d& pt, const State& x)
    {
      pt[0] = x[0];
      pt[1] = x[1];
    }

    void jacobian(const State& x0, const Eigen::Matrix<double, 1, 2>& Hconfig, Eigen::MatrixXd& H0) const
    {
      // const double Jth{ Hconfig[0] * rad * () + Hconfig[1] };
      // if (Hconfig[0] * Hconfig[1] > 0)
      // {
      //   H0 = Eigen::Matrix<double, 1, 3>(-Hconfig[0], -Hconfig[1], 0.0);
      // }
      // else
      // {
      H0 = Eigen::Matrix<double, 1, 3>::Zero();
      const Eigen::Matrix2d R{ x0.rotation<Eigen::Matrix2d>() };
      H0.block<1, 2>(0, 0) = Hconfig * R;
      // H0.block<1, 2>(0, 0) = (R * Hconfig.transpose()).transpose();
      // H0.block<1, 2>(0, 0) = Hconfig * R.transpose();
      // H0 = Eigen::Matrix<double, 1, 3>(Hconfig[0], Hconfig[1], 0.0);
      // }
      // H0 = Eigen::Matrix<double, 1, 3>(0.0, 0.0, 0.0);
      // H0 = Eigen::Matrix<double, 1, 3>(Hconfig[1], Hconfig[0], 0.0);
      // H0 = Eigen::Matrix<double, 1, 3>(0.0, Hconfig[0], Hconfig[1]);
      // H0 = H0 / H0.norm();
      // H0 = H0 * 0.1;
      // LOG_VARS(x0, H0)
      // H0 = Eigen::Matrix<double, 1, 3>(-Hconfig[0], -Hconfig[1], 0.0);
      // H0 = Hconfig;
    }
  };

  // using Base::node_edge_to_fg;
  // Create a FG that goes from N0 to N1 with plan P01
  // virtual GraphValues node_edge_to_fg(const std::size_t parent, const std::size_t child, const State& x1,
  //                                     const StateDot& xdot1, const Control& u01, const double& dt,
  //                                     const bool time_as_variable = true) override
  virtual GraphValues node_edge_to_fg(const prx_models::Node& node, const prx_models::Edge& edge) override
  {
    using StateStateDotTimeFactor = prx_models::mushr_x_xdot_t;
    using StateStateDotNoTimeFactor = prx_models::mushr_x_xdot_nodT_t;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
    using XdotIntegrationTimeFactor = prx_models::mushr_CtrlAccel_t<double>;
    using XdotIntegrationNoTimeFactor = prx_models::mushr_CtrlAccel_t<>;
    using NHCFactor = prx_models::mushr_NHC_t;

    const ml4kp_bridge::SpacePoint& edge_control{ edge.plan.steps[0].control };
    const double dt{ edge.plan.steps[0].duration.data.toSec() };
    State x1;
    StateDot xdot1;
    Control u01;

    copy_state(x1, node.point);
    copy_stateDot(xdot1, node.point);
    u01[0] = edge_control.point[0];
    u01[1] = edge_control.point[1];

    GraphValues graph_values{ aux_graph };
    aux_graph.first.erase(aux_graph.first.begin(), aux_graph.first.end());
    aux_graph.second.clear();
    const std::size_t& parent{ edge.source };
    const std::size_t& child{ edge.target };

    const gtsam::Key k_x0{ keyX(1, parent) };
    const gtsam::Key k_x1{ keyX(1, child) };

    const gtsam::Key k_xdot0{ keyXdot(1, parent) };
    const gtsam::Key k_xdot1{ keyXdot(1, child) };

    const gtsam::Key k_u01{ keyU(parent, child) };
    const gtsam::Key k_t01{ keyT(parent, child) };

    NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    // NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 5e0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e0) };
    NoiseModel dt_limit_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-1) };
    // NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    // NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };

    // graph_values.first.emplace_shared<XdotIntegrationTimeFactor>(k_xdot1, k_xdot0, k_u01, k_t01,
    // xd_integration_noise, default_params, default_poly);
    graph_values.first.emplace_shared<XdotIntegrationTimeFactor>(k_xdot1, k_xdot0, k_u01, k_t01, xd_integration_noise,
                                                                 _params, _poly);
    graph_values.first.emplace_shared<StateStateDotTimeFactor>(k_x1, k_x0, k_xdot0, k_t01, integration_noise);
    // graph_values.first.emplace_shared<StateStateDotTimeFactor>(k_x1, k_x0, k_xdot1, k_t01, integration_noise);
    graph_values.first.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_limit_noise);
    graph_values.first.addPrior(k_t01, dt, dt_noise);

    graph_values.second.insert(k_t01, dt);

    graph_values.first.addPrior(k_x1, x1);
    graph_values.first.addPrior(k_xdot1, xdot1);

    // if (std::fabs(u01[prx_models::mushr_t::control::velocity_idx]) < 0.1)
    // {
    //   LOG_VARS(u01[0], u01[1])
    //   const double u_vel{ u01[prx_models::mushr_t::control::velocity_idx] };

    NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1.0) };
    graph_values.first.addPrior(k_u01, u01, u_prior_noise);

    // graph_values.first.addPrior(k_u01, u01);
    // graph_values.first.addPrior(k_xdot1, xdot1, prior_noise);
    graph_values.second.insert(k_x1, x1);

    // aux_graph.first.addPrior(k_u01, u01, u_prior_noise);
    graph_values.second.insert(k_xdot1, xdot1);
    graph_values.second.insert(k_u01, u01);

    return graph_values;
  };

  bool propagate_plan(const StateEstimates& estimates, prx_models::tree_msg_wrapper_t& new_tree)
  {
    _plan->clear();
    _traj->clear();

    prx_models::tree_msg_wrapper_t::NodeIdx curr_node_idx{ new_tree.root };
    while (new_tree.nodes[curr_node_idx].children.size() > 0)
    {
      const prx_models::tree_msg_wrapper_t::NodeIdx child_idx{ new_tree.nodes[curr_node_idx].children[0] };
      const prx_models::Node& node{ new_tree.nodes[child_idx] };
      const prx_models::Edge& edge{ new_tree.edges[node.parent_edge] };

      ml4kp_bridge::copy(_plan, edge.plan);
      curr_node_idx = child_idx;
    }

    _x0->at(0) = std::get<0>(estimates)[0];
    _x0->at(1) = std::get<0>(estimates)[1];
    _x0->at(2) = std::get<0>(estimates)[2];
    _x0->at(3) = std::get<1>(estimates)[0];
    _x0->at(4) = std::get<1>(estimates)[1];
    _x0->at(5) = std::get<1>(estimates)[2];

    _sg->propagate(_x0, *_plan, *_traj);

    // TODO: collision check could be done via SDF
    const bool collision_free_traj{ default_valid_trajectory(*_traj, _valid_state) };

    prx_models::Tree sln_tree;
    const double max_edge_duration{ 0.1 };
    prx_models::tree_from_plan_traj(sln_tree, *_plan, *_traj, max_edge_duration);
    new_tree = prx_models::tree_msg_wrapper_t(sln_tree);
    return collision_free_traj;
  }

  // template <typename Params>
  // void set_params(const Params& params)
  void init(const prx::param_loader& params)
  {
    const std::string plant_name{ params["/name"].as<std::string>() };
    const std::string plant_path{ params["/path"].as<std::string>() };
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Failed to create plant");
    plant->init(params);

    prx::world_model_t world_model({ plant }, {});
    world_model.create_context("context", { plant_name }, {});
    auto context = world_model.get_context("context");
    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();
    // auto ps = context.first->get_parameter_space();
    _sg = prx::system_group(context);
    _cg = prx::collision_group(context);
    _plan = std::make_shared<prx::plan_t>(cs);
    _traj = std::make_shared<prx::trajectory_t>(ss);
    _x0 = ss->make_point();
    const std::vector<double> params_vec{ params["/parameter_space/values"].as<std::vector<double>>() };

    _valid_state = [&](prx::space_point_t& s) { return default_valid_state(s, _sg->get_state_space(), _cg); };

    _params[mushr_types::Control::vel_desired] = params_vec[mushr_types::Control::vel_desired];
    _params[mushr_types::Control::steering] = params_vec[mushr_types::Control::steering];
    _params[mushr_types::Control::friction] = params_vec[mushr_types::Control::friction];
    _params[mushr_types::Control::delta_offset] = params_vec[mushr_types::Control::delta_offset];
    _params[mushr_types::Control::delta_gain] = params_vec[mushr_types::Control::delta_gain];
    for (int i = 0; i < mushr_types::Control::PolyDeg; ++i)
    {
      _poly[i] = params_vec[i + 5];
    }
  }

  void log_params()
  {
    const Eigen::RowVectorXd mushr_parameters{ _params.transpose() };
    const Eigen::RowVectorXd mushr_polynomial{ _poly.transpose() };
    LOG_VARS(mushr_parameters);
    LOG_VARS(mushr_polynomial);
  }
  void print_params()
  {
    const Eigen::RowVectorXd mushr_parameters{ _params.transpose() };
    DEBUG_VARS(mushr_parameters);
  }

  // static inline Parameters default_params{ 1.50000, 0.20000, 0.90000, 0.90000, 1.05000 };
  static inline Parameters default_params{ .75, 0.2, 0.99, 0.90, 1.05 };
  static inline Poly default_poly{ -0.4397, 3.773e-5, 0.8677, 5.8e-6 };

  //   Poly default_poly{ 0.1045, 0.0212, 0.2357, 0.0486 };

protected:
  prx::valid_state_t _valid_state;
  prx::space_point_t _x0;

  std::shared_ptr<prx::collision_group_t> _cg;
  std::shared_ptr<prx::system_group_t> _sg;
  std::shared_ptr<prx::plan_t> _plan;
  std::shared_ptr<prx::trajectory_t> _traj;

  Parameters _params;
  Poly _poly;
  GraphValues aux_graph;
  // static inline std::size_t first{ std::numeric_limits<std::size_t>::max() };

  ros::Subscriber _sensor_subscriber;
  ros::Publisher _current_state_publisher;
  // std::pair<Observation, ros::Time> _last_observation;
  // const State _idle_state;
  // const StateDot _idle_state_dot;
  // const Control _idle_control;
  // const double _idle_dt;
  // bool _new_observation;

  // Control _ctrl_lower_bound;
  // Control _ctrl_upper_bound;
};

class mushrFG_t : public prx::plant_t
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;
  using MushrMjFactor = mushr_mj_factor_t<>;

public:
  mushrFG_t(const std::string& path)
    : plant_t(path)
    , _params_u(mushr_stela_t::default_params)
    , _delta_poly(mushr_stela_t::default_poly)
    , _state(0., 0., 0.)
    , _state_dot(StateDot::Zero())
    , _ctrl(mushr_types::Control::type::Zero())
    , _sensor_position(Eigen::Vector3d::Zero())
    , _sensor_quaternion(1., 0., 0., 0.)
  {
    state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
                     &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    state_space = new prx::space_t("EEREEE", state_memory, "mushr_state");
    state_space->set_bounds({ -100, -100, -prx::constants::pi, -10, -10, -10 },
                            { 100, 100, prx::constants::pi, 10, 10, 10 });

    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -1., -1. }, { 1., 1. });

    // derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    // derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");

    parameter_memory = { &_params_u[mushr_types::Control::vel_desired],
                         &_params_u[mushr_types::Control::steering],      // no-lint
                         &_params_u[mushr_types::Control::friction],      // no-lint
                         &_params_u[mushr_types::Control::delta_offset],  // no-lint
                         &_params_u[mushr_types::Control::delta_gain],    // no-lint
                         &_delta_poly[0],
                         &_delta_poly[1],
                         &_delta_poly[2],
                         &_delta_poly[3] };
    const std::string param_topology{ std::string(parameter_memory.size(), 'E') };
    parameter_space = new prx::space_t(param_topology, parameter_memory, "mushr_params");

    _sensor_memory = { &_sensor_position[0],    &_sensor_position[1],    &_sensor_position[2],  // no-lint
                       &_sensor_quaternion.w(), &_sensor_quaternion.x(), &_sensor_quaternion.y(),
                       &_sensor_quaternion.z() };
    _sensor_space = new prx::space_t("EEEEEEE", _sensor_memory, "mushr_sensors");
    _sensor_space->set_bounds({ -100, -100, -100, -100, -100, -100, -100 },
                              { +100, +100, +100, +100, +100, +100, +100 });

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();

    // DEBUG_PRINT
    // DEBUG_VARS(_propagation_factor)
  }
  ~mushrFG_t() {};

  virtual prx::param_loader initialization_parameters() override
  {
    prx::param_loader params{ prx::plant_t::initialization_parameters() };

    return params;
  }

  // static prx::param_loader init()
  // {
  //   prx::param_loader params{ prx::plant_t::init() };
  //   params["state_space"] = space_t::init();
  //   params["control_space"] = space_t::init();
  //   params["parameter_space"] = space_t::init();
  //   params["sensor_space"] = space_t::init();
  // }

  virtual void propagate(const double simulation_step) override final
  {
    // if (_propagation_factor == 0)
    // {
    // }

    _state = mushr_x_xdot_t::predict(_state, _state_dot, prx::simulation_step);
    _state_dot = mushr_CtrlAccel_t<>::predict(_state_dot, _ctrl, prx::simulation_step, _params_u, _delta_poly);
    // DEBUG_VARS(_state, _state_dot.transpose());
    // DEBUG_VARS(_state.matrix())
    // DEBUG_VARS(_state, _state_dot.transpose(), _ubar.transpose(), _ctrl.transpose(), simulation_step);
    // state_space->enforce_bounds();
  }

  virtual void sense() override
  {
    _sensor_position << _state[0], _state[1], 0.125;
    _sensor_quaternion = Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ());
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
    body->translation()[0] = _state[0];
    body->translation()[1] = _state[1];
    body->translation()[2] = 0.125;
  }
  virtual void compute_derivative() override final
  {
  }

protected:
  // State space
  mushr_types::State::type _state;
  mushr_types::StateDot::type _state_dot;

  // Control space
  mushr_types::Control::type _ctrl;
  // mushr_types::Ubar::type _ubar;

  // Parameter space
  mushr_types::Control::params _params_u;
  mushr_types::Control::Poly _delta_poly;

  // Sensor space
  Eigen::Vector3d _sensor_position;
  Eigen::Quaterniond _sensor_quaternion;

  // double _propagate_id;       // If mj prop using, it needs to reset if curr_propid != _propagate_id
  // double _curr_propagate_id;  // If mj prop using, it needs to reset if curr_propid != _propagate_id

  // std::shared_ptr<MushrMjFactor> _mj_factor;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushrFG_t, mushrFG)