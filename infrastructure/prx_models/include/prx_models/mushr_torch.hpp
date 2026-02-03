#pragma once

#include <limits>
#include <memory>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <string>

// Ros
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/mj_init_utils.hpp>
#include <prx_models/stela_robot_interface.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <interface/SensorDataStamped.h>
#include <torch_bridge/sysid_runtime.hpp>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include <vector>
#include "mujoco/mjmodel.h"

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>

namespace prx_models
{

struct mushr_torch_types_t
{
  static constexpr Eigen::Index DimX{ 3 };
  static constexpr Eigen::Index DimXdot{ 3 };
  // static constexpr Eigen::Index DimXdot{ 17 };
  static constexpr Eigen::Index DimU{ 2 };
  static constexpr Eigen::Index DimDt{ 1 };
  static constexpr Eigen::Index DimError{ 3 };
  static constexpr std::string_view plant_name = "mushrTorch";

  // using State = Eigen::Vector<double, DimX>;
  using State = prx::fg::SE2_t;
  using StateDot = Eigen::Vector<double, DimXdot>;
  using Control = Eigen::Vector<double, DimU>;
  using Error = Eigen::Vector<double, DimError>;
  using Observation = prx::fg::SE2_t;

  using StateKeys = std::array<gtsam::Key, 2>;
  using ControlKeys = std::array<gtsam::Key, 1>;
  using TimeKeys = std::array<gtsam::Key, 1>;

  using StateEstimates = std::tuple<State, StateDot>;
  using ControlEstimates = std::tuple<Control>;
};

template <typename... Types>
class mushr_torch_factor_t
  : public gtsam::NoiseModelFactorN<mushr_torch_types_t::StateDot, mushr_torch_types_t::StateDot,
                                    mushr_torch_types_t::Control, Types...>
{
  using StateDot = typename mushr_torch_types_t::StateDot;
  using Control = typename mushr_torch_types_t::Control;
  using Error = typename mushr_torch_types_t::Error;
  // using StateStateDot = std::pair<State, StateDot>;

  static constexpr Eigen::Index DimX{ mushr_torch_types_t::DimX };
  static constexpr Eigen::Index DimXdot{ mushr_torch_types_t::DimXdot };
  static constexpr Eigen::Index DimU{ mushr_torch_types_t::DimU };
  static constexpr Eigen::Index DimDt{ mushr_torch_types_t::DimDt };
  static constexpr Eigen::Index DimError{ mushr_torch_types_t::DimError };

  using Base = gtsam::NoiseModelFactorN<StateDot, StateDot, Control, Types...>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  static constexpr std::size_t NumTypes{ sizeof...(Types) };

  struct StructuredParams
  {
    static constexpr std::size_t friction{ prx_models::mushr_types::Control::friction };
    static constexpr std::size_t vel_desired{ prx_models::mushr_types::Control::vel_desired };
    static constexpr double L{ prx_models::mushr_types::Parameters::L };
  };

  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;
  using StructuredSysidRuntime = torch_bridge::StructuredSysidRuntime<MushrPlant, Params, Poly, StructuredParams>;

  mushr_torch_factor_t() = delete;
  mushr_torch_factor_t(const mushr_torch_factor_t& other) = delete;

public:
  template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>
  mushr_torch_factor_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const gtsam::Key dt,
                       const NoiseModel& cost_model, const std::string torch_model_path, const double nn_dt = 0.1)
    : Base(cost_model, xd1, xd0, u, dt), _dt(-1), _NN_DT(nn_dt), _NN_2(nn_dt * nn_dt)
  {
    const Params params{ Params(1.0, 1.0, 1.0, 0.0, 1.0) };
    const Poly poly{ Poly(0.0, 0.0, 1.0, 0.0) };
    _nn_interface = std::make_unique<StructuredSysidRuntime>(torch_model_path, params, poly, false, "float32");
    _nn_interface->set_dt(dt);
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(0 == Num), bool> = true>
  mushr_torch_factor_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const double& dt,
                       const NoiseModel& cost_model, const std::string torch_model_path, const double nn_dt = 0.1)
    : Base(cost_model, xd1, xd0, u), _dt(dt), _NN_DT(nn_dt), _NN_2(nn_dt * nn_dt)
  {
    const Params params{ Params(1.0, 1.0, 1.0, 0.0, 1.0) };
    const Poly poly{ Poly(0.0, 0.0, 1.0, 0.0) };
    _nn_interface = std::make_unique<StructuredSysidRuntime>(torch_model_path, params, poly, false, "float32");
    _nn_interface->set_dt(dt);
  }

  ~mushr_torch_factor_t() override
  {
  }

  StateDot predict(const StateDot& xd, const Control& u, const double& dt,  // no-lint
                   OptDeriv Hxd = boost::none, OptDeriv Hu = boost::none, OptDeriv Hdt = boost::none) const
  {
    Eigen::Matrix<double, 3, 3> xnn_H_xd{ Eigen::Matrix<double, 3, 3>::Identity() };
    Eigen::Matrix<double, 3, 2> xnn_H_u{ Eigen::Matrix<double, 3, 2>::Identity() };
    // auto xnn_H_xd = boost::make_optional(static_cast<bool>(Hxd), Eigen::Matrix<double, 3, 3>::Identity());
    const bool compute_derivative{ Hxd or Hu };
    const double epsilon{ dt - _NN_DT };
    const double eps_rate{ epsilon / _NN_2 };
    const double one_p_eps{ 1.0 + eps_rate };
    const StateDot xd_nn{ compute_derivative ? _nn_interface->call(xd, u) :  // no-lint
                                               _nn_interface->call(xd, u, xnn_H_xd, xnn_H_u) };
    // nullptr) };  // no-lint
    // no-lint Hu ? &xnn_H_u :
    // nullptr) };

    const StateDot xd_pred{ xd_nn * one_p_eps - xd * eps_rate };
    // DEBUG_VARS(xd_nn.transpose(), xd_pred.transpose())
    auto xin = xd.transpose();
    auto xNN = xd_nn.transpose();
    auto uin = u.transpose();
    DEBUG_VARS(dt, epsilon, _NN_DT, _NN_2)
    DEBUG_VARS(xin, xNN, uin)

    const Eigen::Matrix3d xpred_H_xnn{ one_p_eps * Eigen::Matrix3d::Identity() };
    if (Hxd)
    {
      const Eigen::Matrix3d xpred_H_xd{ -eps_rate * Eigen::Matrix3d::Identity() };
      *Hxd = xpred_H_xnn * xnn_H_xd + xpred_H_xd;
    }
    if (Hu)
    {
      *Hu = xpred_H_xnn * xnn_H_u;
    }
    if (Hdt)
    {
      const double dt_rate{ 1.0 / _NN_2 };
      const Eigen::Matrix<double, 3, 1> xpred_H_dt{ dt_rate * xd_nn - dt_rate * xd };
      *Hdt = xpred_H_dt;
    }

    return xd_pred;
  }

  // (x1,xd1) <- f( (x0,xd0), u, dt )
  virtual Eigen::VectorXd evaluateError(const StateDot& xd1, const StateDot& xd0, const Control& u,
                                        const Types&... dt01,  // no-lint
                                        OptDeriv Hxd1 = boost::none, OptDeriv Hxd0 = boost::none,
                                        OptDeriv Hu = boost::none, OptionalMatrix<Types>... H) const override
  {
    StateDot x1_pred;
    if constexpr (0 == NumTypes)
    {
      x1_pred = predict(xd0, u, _dt, Hxd0, Hu);
      // return error(x1, x0, xdot, _h, H1, H0, Hdot);
    }
    else
    {
      x1_pred = predict(xd0, u, dt01..., Hxd0, Hu, H...);
    }

    const Error error{ x1_pred - xd1 };
    if (Hxd1)
    {
      *Hxd1 = -Eigen::Matrix3d::Identity();
    }
    return error;
  }

private:
  const double _NN_DT;
  const double _NN_2;
  const double _dt;

  mutable std::unique_ptr<StructuredSysidRuntime> _nn_interface;
};

class mushr_torch_stela_t : public stela_robot_interface_t<mushr_torch_stela_t, mushr_torch_types_t>
{
  using This = mushr_torch_stela_t;
  using Base = stela_robot_interface_t<mushr_torch_stela_t, mushr_torch_types_t>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

  using XVelFactor = prx_models::mushr_x_xdot_t;
  using VelUbarFactor = prx_models::mushr_xdot_ub_t;
  using CtrlUbarFactor = prx_models::mushr_ub_u_xdot_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = typename mushr_torch_types_t::State;
  using StateDot = typename mushr_torch_types_t::StateDot;
  using Control = typename mushr_torch_types_t::Control;
  using Observation = typename mushr_torch_types_t::Observation;

  using StateKeys = typename mushr_torch_types_t::StateKeys;
  using ControlKeys = typename mushr_torch_types_t::ControlKeys;
  using TimeKeys = typename mushr_torch_types_t::TimeKeys;

  using StateEstimates = typename mushr_torch_types_t::StateEstimates;
  using ControlEstimates = typename mushr_torch_types_t::ControlEstimates;

  using Poly = mushr_types::Control::Poly;
  using Parameters = mushr_types::Control::params;
  using PrxPlant = mushrFG_t;

  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };

  mushr_torch_stela_t() : Base() {};

  mushr_torch_stela_t(ros::NodeHandle& nh) : Base(nh)
  {
    std::string sensor_topic_name;

    PARAM_SETUP(nh, sensor_topic_name)

    ros::NodeHandle nh_ctrl(nh, "control_space");

    std::vector<double> lower_bound;
    std::vector<double> upper_bound;

    PARAM_SETUP(nh_ctrl, lower_bound)
    PARAM_SETUP(nh_ctrl, upper_bound)

    _ctrl_lower_bound = Control(lower_bound.data());
    _ctrl_upper_bound = Control(upper_bound.data());

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<std_msgs::Float64>& zi{ msg->raw_sensor_data };
    _last_observation.first[0] = zi[0].data;
    _last_observation.first[1] = zi[1].data;
    // _last_observation.first[2] = zi[2].data;
    // const Eigen::Vector3d position{ zi[0].data, zi[1].data, zi[2].data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3].data, zi[4].data, zi[5].data, zi[6].data) };
    _last_observation.first[2] = prx::quaternion_to_euler(q)[2];
    // _last_observation.first = gtsam::Pose3(gtsam::Rot3(q), position);
    _last_observation.second = msg->header.stamp;
    _new_observation = true;
    // DEBUG_VARS(_new_observation, _last_observation.first[0], _last_observation.first[1], _last_observation.first[2])
  }

  // Factor graph for "Idle" state (i.e. before starting execution or after reaching the goal)
  virtual GraphValues idle_state_to_fg(const std::size_t parent, const std::size_t child,
                                       const bool time_as_variable = true) override
  {
    using IntegrationFactor = mushr_torch_factor_t<double>;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
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

    // const mjModel* mj_model;
    // mjData* mj_data;
    // const mushr_mujoco_types_t::StateHidden state_in;
    // const mushr_mujoco_types_t::StateDotHidden stateDot_in;

    graph_values.first.emplace_shared<IntegrationFactor>(k_xdot1, k_xdot0, k_u01, k_t01, integration_noise,
                                                         _torch_model_path, _nn_dt);
    graph_values.first.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_limit_noise);
    graph_values.first.addPrior(k_t01, _idle_dt, dt_noise);

    graph_values.first.addPrior(k_u01, _idle_control, u_prior_noise);
    graph_values.first.addPrior(k_xdot1, _idle_state_dot, xdot_prior_noise);

    graph_values.second.insert(k_t01, _idle_dt);
    graph_values.second.insert(k_x1, _idle_state);

    graph_values.second.insert(k_xdot1, _idle_state_dot);
    graph_values.second.insert(k_u01, _idle_control);

    return graph_values;
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
    for (int i = 0; i < 6; ++i)
    {
      xd[i] = pt.point[i];
    }
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
      // rotation = state.rotation().matrix();
      rotation = prx::euler_to_rotation<Eigen::Matrix3d>(vec, "Z");
      translation[0] = state.translation()[0];
      translation[1] = state.translation()[1];
      translation[2] = 0.0;
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
      // const Eigen::Matrix2d R{ state.rotation().matrix().template block<2, 2>(0, 0) };
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
      pt[0] = x.translation()[0];
      pt[1] = x.translation()[1];
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
      // const Eigen::Matrix2d R{ x0.rotation().matrix().template block<2, 2>(0, 0) };
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

  // virtual GraphValues node_edge_to_fg(const std::size_t parent, const std::size_t child,
  //                                     const ml4kp_bridge::SpacePoint& node_state, const ml4kp_bridge::Plan&
  //                                     edge_plan, const bool time_as_variable = true) override
  virtual GraphValues node_edge_to_fg(const prx_models::Node& node, const prx_models::Edge& edge) override
  {
    using IntegrationFactor = mushr_torch_factor_t<double>;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
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

    // const gtsam::Key k_xdotdot0{ keyXdotdot(1, parent) };

    const gtsam::Key k_u01{ keyU(parent, child) };
    const gtsam::Key k_t01{ keyT(parent, child) };

    NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 5e0) };
    NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e0) };
    NoiseModel dt_limit_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-1) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };
    NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-3) };

    const mjModel* mj_model;
    mjData* mj_data;
    const Eigen::VectorXd state_in;
    const Eigen::VectorXd stateDot_in;

    // mushr_torch_factor_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const gtsam::Key dt,
    // const NoiseModel& cost_model, const std::string torch_model_path, const double nn_dt = 0.1)

    graph_values.first.emplace_shared<IntegrationFactor>(k_xdot1, k_xdot0, k_u01, k_t01, integration_noise,
                                                         _torch_model_path, _nn_dt);
    graph_values.first.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_limit_noise);
    graph_values.first.addPrior(k_t01, dt, dt_noise);

    graph_values.second.insert(k_t01, dt);

    graph_values.first.addPrior(k_x1, x1);
    graph_values.first.addPrior(k_xdot1, xdot1);
    // graph_values.first.addPrior(k_u01, u01);
    // graph_values.first.addPrior(k_xdot1, xdot1, prior_noise);
    graph_values.second.insert(k_x1, x1);

    // aux_graph.first.addPrior(k_u01, u01, u_prior_noise);
    graph_values.second.insert(k_xdot1, xdot1);
    graph_values.second.insert(k_u01, u01);

    return graph_values;
  };

  // template <typename Params>
  void init(const prx::param_loader& params)
  {
    _torch_model_path = params["torch_model"].as<>();
    _nn_dt = params["nn_dt"].as<double>();
  }

  void log_params()
  {
  }
  void print_params()
  {
  }

protected:
  GraphValues aux_graph;

  std::string _torch_model_path;
  double _nn_dt;

  ros::Subscriber _sensor_subscriber;
};

class mushr_torch_t : public prx::plant_t
{
  // using State = mushr_types::State::type;
  // using StateDot = mushr_types::StateDot::type;
  // using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;
  // using MushrMjFactor = mushr_mujoco_factor_t<>;

public:
  mushr_torch_t(const std::string& path) : plant_t(path)
  {
    // state_memory = { &_state[0], &_state[1], &_state[2], &_ubar[0], &_ubar[1] };
    // state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
    // &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
                     &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    state_space = new prx::space_t("EEREEE", state_memory, "mushr_state");
    std::vector<double> min_bounds(6, -std::numeric_limits<double>::infinity());
    std::vector<double> max_bounds(6, std::numeric_limits<double>::infinity());
    state_space->set_bounds(min_bounds, max_bounds);

    // min_bounds.clear();
    // max_bounds.clear();
    // control_memory = { &_ctrl[0], &_ctrl[1] };
    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    // min_bounds = std::vector<double>(2, -std::numeric_limits<double>::infinity());
    // max_bounds = std::vector<double>(2, std::numeric_limits<double>::infinity());
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -1.0, -1.0 }, { 1.0, 1.0 });

    derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");

    parameter_memory = {};
    parameter_space = new prx::space_t("", parameter_memory, "mushr_params");
    // derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    // derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");
    // const std::string param_topology{ std::string(parameter_memory.size(), 'E') };

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }

  ~mushr_torch_t() {};

  virtual void init(const prx::param_loader& params) override
  {
    prx::plant_t::init(params);
    if (params.exists("torch_model") and params.exists("nn_dt"))
    {
      const std::string torch_model_path{ params["torch_model"].as<>() };
      const double nn_dt{ params["nn_dt"].as<double>() };

      DEBUG_VARS(torch_model_path)
      DEBUG_VARS(nn_dt)
      // t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const gtsam::Key dt,
      // const NoiseModel& cost_model, const std::string torch_model_path, const double nn_dt = 0.1)
      _mushr_factor =
          std::make_shared<mushr_torch_factor_t<>>(0, 1, 2, prx::simulation_step, nullptr, torch_model_path, nn_dt);
    }
    else
    {
      prx_throw("[mushr_torch_t::init] Parameters torch_model or nn_dt not found!")
    }
  }

  virtual void propagate(const double simulation_step) override final
  {
    _state = mushr_x_xdot_t::predict(_state, _state_dot, prx::simulation_step);
    _state_dot = _mushr_factor->predict(_state_dot, _ctrl, prx::simulation_step);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    // *body = _state.matrix();
    body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
    body->translation()[0] = _state[0];
    body->translation()[1] = _state[1];
    body->translation()[2] = 0.0;
  }
  virtual void compute_derivative() override final
  {
  }

protected:
  std::shared_ptr<mushr_torch_factor_t<>> _mushr_factor;

  mushr_torch_types_t::State _state;
  mushr_torch_types_t::StateDot _state_dot;
  mushr_torch_types_t::Control _ctrl;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushr_torch_t, mushrTorch)
