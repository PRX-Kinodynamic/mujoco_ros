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

struct mushr_mujoco_types_t
{
  static constexpr Eigen::Index DimX{ 6 };
  static constexpr Eigen::Index DimXdot{ 6 };
  // static constexpr Eigen::Index DimXdot{ 17 };
  static constexpr Eigen::Index DimU{ 2 };
  static constexpr Eigen::Index DimDt{ 1 };
  static constexpr Eigen::Index DimError{ 12 };
  static constexpr Eigen::Index DimXHidden{ 18 - DimX };
  static constexpr Eigen::Index DimXDotHidden{ 17 - DimXdot };
  static constexpr std::string_view plant_name = "mushrMujoco";

  // using State = Eigen::Vector<double, DimX>;
  using State = gtsam::Pose3;
  using StateDot = Eigen::Vector<double, DimXdot>;
  // using StateStateDot = Eigen::Vector<double, DimX + DimXdot>;
  using Control = Eigen::Vector<double, DimU>;
  using Error = Eigen::Vector<double, DimError>;
  using StateHidden = Eigen::Vector<double, DimXHidden>;
  using StateDotHidden = Eigen::Vector<double, DimXDotHidden>;
  using Observation = gtsam::Pose3;

  using StateKeys = std::array<gtsam::Key, 2>;
  using ControlKeys = std::array<gtsam::Key, 1>;
  using TimeKeys = std::array<gtsam::Key, 1>;

  using StateEstimates = std::tuple<State, StateDot>;
  using ControlEstimates = std::tuple<Control>;
};

template <typename... Types>
class mushr_mujoco_factor_t
  : public gtsam::NoiseModelFactorN<mushr_mujoco_types_t::State, mushr_mujoco_types_t::StateDot,  // no-lint
                                    mushr_mujoco_types_t::State, mushr_mujoco_types_t::StateDot,  // no-lint
                                    mushr_mujoco_types_t::Control,                                // no-lint
                                    Types...>
{
  using State = typename mushr_mujoco_types_t::State;
  using StateDot = typename mushr_mujoco_types_t::StateDot;
  // using StateStateDot = typename mushr_mujoco_types_t::StateStateDot;
  using Control = typename mushr_mujoco_types_t::Control;
  using Error = typename mushr_mujoco_types_t::Error;
  using StateHidden = typename mushr_mujoco_types_t::StateHidden;
  using StateDotHidden = typename mushr_mujoco_types_t::StateDotHidden;
  using StateStateDot = std::pair<State, StateDot>;

  static constexpr Eigen::Index DimX{ mushr_mujoco_types_t::DimX };
  static constexpr Eigen::Index DimXdot{ mushr_mujoco_types_t::DimXdot };
  static constexpr Eigen::Index DimU{ mushr_mujoco_types_t::DimU };
  static constexpr Eigen::Index DimDt{ mushr_mujoco_types_t::DimDt };
  static constexpr Eigen::Index DimError{ mushr_mujoco_types_t::DimError };
  static constexpr Eigen::Index DimXHidden{ mushr_mujoco_types_t::DimXHidden };
  static constexpr Eigen::Index DimXDotHidden{ mushr_mujoco_types_t::DimXDotHidden };
  // static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  // static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };
  // static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };

  using Base = gtsam::NoiseModelFactorN<State, StateDot, State, StateDot, Control, Types...>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  static constexpr std::size_t NumTypes{ sizeof...(Types) };

  using DerivWrapper = std::function<gtsam::Vector(const State&, const StateDot&, const State&, const StateDot&,
                                                   const Control&, const double&)>;
  // [](const StateDot& xd0, const double& dt, const Control& u, const Params& params) {
  // return MushrCtrl::predict(xd0, dt, u, params);
  // };
  // using WrapperState = std::function<Error(const State&, const Control&, const double&)>;
  // using WrapperStateDot = std::function<Error(const StateDot&, const Control&, const double&)>;
  // using WrapperControl = std::function<Error(const Control&, const double&, const StateDot&)>;
  // using WrapperDt = std::function<Error(const double&, const StateStateDot&, const Control&)>;

  // using PartialXdot0 =
  //     prx::math::first_order_derivative_t<WrapperXdot0, StateStateDot, 3, -1, const Control&, const double&>;
  // using PartialControl =
  //     prx::math::first_order_derivative_t<WrapperControl, Control, 3, -1, const double&, const StateStateDot&>;
  // using PartialDt = prx::math::first_order_derivative_t<WrapperDt, double, 3, -1, const StateStateDot&, const
  // Control&>;

  mushr_mujoco_factor_t() = delete;
  mushr_mujoco_factor_t(const mushr_mujoco_factor_t& other) = delete;

public:
  template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>
  mushr_mujoco_factor_t(const gtsam::Key x1, const gtsam::Key xd1, const gtsam::Key x0, const gtsam::Key xd0,
                        const gtsam::Key u, const gtsam::Key dt, const NoiseModel& cost_model, const mjModel* mj_model,
                        mjData* mj_data, const StateHidden state_in, const StateDotHidden stateDot_in)
    : Base(cost_model, x1, xd1, x0, xd0, u, dt)
    , _dt(-1)
    , _mj_model(mj_model)
    , _mj_data(mj_data)
    , _ctrl_vector(mj_data->ctrl, DimU, 1)
    , _qpos_vector(mj_data->qpos, _mj_model->nq, 1)
    , _qvel_vector(mj_data->qvel, _mj_model->nv, 1)
    , _qacc_vector(mj_data->qacc_warmstart, _mj_model->nv, 1)
    , _qpos_init(_qpos_vector)
    , _qvel_init(_qvel_vector)
    , _qacc_warmstart_init(Eigen::VectorXd::Zero(_mj_model->nv))
    , _deriv_wrapper([&](const State& x1, const StateDot& xd1, const State& x0, const StateDot& xd0, const Control& u,
                         const double& dt) { return compute_error(x1, xd1, x0, xd0, u, dt); })
    // , _wrapper_state([&](const StateDot& xd0, const Control& u, const double& dt) { return mj_predict(xd0, u, dt); })
    // , _wrapper_stateDot([&](const StateDot& xd0, const Control& u, const double& dt) { return mj_predict(xd0, u, dt);
    // }) , _wrapper_control([&](const Control& u, const double& dt, const StateDot& xd0) { return mj_predict(xd0, u,
    // dt); }) , _wrapper_dt([&](const double& dt, const StateDot& xd0, const Control& u) { return mj_predict(xd0, u,
    // dt); }) , _partial_state(_wrapper_state, h) , _partial_control(_wrapper_control, h) , _partial_dt(_wrapper_dt, h)
    , _mj_state_init(false)
    , _state_hidden(state_in.tail(DimXHidden))
    , _stateDot_hidden(stateDot_in.tail(DimXDotHidden))
  {
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(0 == Num), bool> = true>
  mushr_mujoco_factor_t(const gtsam::Key x1, const gtsam::Key xd1, const gtsam::Key x0, const gtsam::Key xd0,
                        const gtsam::Key u, const double& dt, const NoiseModel& cost_model, const mjModel* mj_model,
                        mjData* mj_data, const Eigen::VectorXd state_in, const Eigen::VectorXd stateDot_in)
    : Base(cost_model, x1, xd1, x0, xd0, u)
    , _dt(dt)
    , _mj_model(mj_model)
    , _mj_data(mj_data)
    , _ctrl_vector(_mj_data->ctrl, DimU, 1)
    , _qpos_vector(_mj_data->qpos, _mj_model->nq, 1)
    , _qvel_vector(_mj_data->qvel, _mj_model->nv, 1)
    , _qacc_vector(_mj_data->qacc_warmstart, _mj_model->nv, 1)
    , _qpos_init(_qpos_vector)
    , _qvel_init(_qvel_vector)
    , _qacc_warmstart_init(Eigen::VectorXd::Zero(_mj_model->nv))
    , _deriv_wrapper([&](const State& x1, const StateDot& xd1, const State& x0, const StateDot& xd0, const Control& u,
                         const double& dt) { return compute_error(x1, xd1, x0, xd0, u, dt); })
    , _mj_state_init(false)
    , _state_hidden(state_in.tail(DimXHidden))
    , _stateDot_hidden(stateDot_in.tail(DimXDotHidden))
  {
  }

  ~mushr_mujoco_factor_t() override
  {
  }

  template <typename Matrix>
  static boost::optional<Eigen::MatrixXd&> check_opt_H(const bool check, Matrix& matrix)
  {
    if (check)
      return matrix;
    return boost::none;
  }

  void reset_mj_state()
  {
    _mj_state_init = false;
  }

  Error compute_error(const State& x1, const StateDot& xd1, const State& x0, const StateDot& xd0, const Control& u,
                      const double& dt) const
  {
    // auto [x1_pred, x1dot_pred] = mj_fwd_prop(x0, xd0, u, dt, _state_hidden, _stateDot_hidden);
    mj_fwd_prop(x0, xd0, u, dt, _state_hidden, _stateDot_hidden);
    const State x1_pred{ mj_copy(_qpos_vector.head(7)) };
    const StateDot x1dot_pred{ _qvel_vector.head(6) };
    // return { _qpos_vector.head(7), _qvel_vector.head(6) };
    // const gtsam::Rot3 r1(gtsam::Quaternion(x1.template segment<4>(3)));
    // const gtsam::Rot3 r1_pred(gtsam::Quaternion(x1_pred.template segment<4>(3)));
    // const gtsam::Pose3 pose1(r1, x1.template segment<3>(0));
    // const gtsam::Pose3 pose1_pred(r1_pred, x1_pred.template segment<3>(0));

    const Eigen::Vector<double, 6> err_pose{ gtsam::traits<gtsam::Pose3>::Local(x1_pred, x1) };
    const Eigen::Vector<double, 6> err_vel{ x1dot_pred - xd1 };

    // LOG_VARS(_qvel_vector)
    // LOG_VARS(x0)
    return (Error() << err_pose, err_vel).finished();
  }

  virtual Eigen::VectorXd eval_error(const State& x1, const StateDot& xd1,                     // no-lint
                                     const State& x0, const StateDot& xd0,                     // no-lint
                                     const Control& u, const double& dt01,                     // no-lint
                                     OptDeriv Hx1 = boost::none, OptDeriv Hxd1 = boost::none,  // no-lint
                                     OptDeriv Hx0 = boost::none, OptDeriv Hxd0 = boost::none,  // no-lint
                                     OptDeriv Hu = boost::none, OptDeriv Hdt = boost::none) const
  {
    if (Hx1)
    {
      *Hx1 = gtsam::numericalDerivative61(_deriv_wrapper, x1, xd1, x0, xd0, u, dt01);
      // DEBUG_VARS(*Hx1);
    }
    if (Hxd1)
    {
      *Hxd1 = gtsam::numericalDerivative62(_deriv_wrapper, x1, xd1, x0, xd0, u, dt01);
      // DEBUG_VARS(*Hxd1);
    }
    if (Hx0)
    {
      *Hx0 = gtsam::numericalDerivative63(_deriv_wrapper, x1, xd1, x0, xd0, u, dt01);
      // DEBUG_VARS(*Hx0);
    }
    if (Hxd0)
    {
      *Hxd0 = gtsam::numericalDerivative64(_deriv_wrapper, x1, xd1, x0, xd0, u, dt01);
      // DEBUG_VARS(*Hxd0);
    }
    if (Hu)
    {
      *Hu = gtsam::numericalDerivative65(_deriv_wrapper, x1, xd1, x0, xd0, u, dt01);
      // DEBUG_VARS(*Hu);
    }
    if (Hdt)
    {
      *Hdt = gtsam::numericalDerivative66(_deriv_wrapper, x1, xd1, x0, xd0, u, dt01);
      // DEBUG_VARS(*Hdt);
    }

    return compute_error(x1, xd1, x0, xd0, u, dt01);
  }

  // (x1,xd1) <- f( (x0,xd0), u, dt )
  virtual Eigen::VectorXd evaluateError(const State& x1, const StateDot& xd1,                     // no-lint
                                        const State& x0, const StateDot& xd0,                     // no-lint
                                        const Control& u,                                         // no-lint
                                        const Types&... dt01,                                     // no-lint
                                        OptDeriv Hx1 = boost::none, OptDeriv Hxd1 = boost::none,  // no-lint
                                        OptDeriv Hx0 = boost::none, OptDeriv Hxd0 = boost::none,  // no-lint
                                        OptDeriv Hu = boost::none,                                // no-lint
                                        OptionalMatrix<Types>... H) const override
  {
    Error error;
    if constexpr (0 == NumTypes)
    {
      error = eval_error(x1, xd1, x0, xd0, u, _dt, Hx1, Hxd1, Hx0, Hxd0, Hu);
      // return error(x1, x0, xdot, _h, H1, H0, Hdot);
    }
    else
    {
      error = eval_error(x1, xd1, x0, xd0, u, dt01..., Hx1, Hxd1, Hx0, Hxd0, Hu, H...);
      // return error(x1, x0, xdot, xd..., H1, H0, Hdot, H...);
    }
    // if (Hx1)
    //   DEBUG_VARS(*Hx1);
    // if (Hxd1)
    //   DEBUG_VARS(*Hxd1);
    // if (Hx0)
    //   DEBUG_VARS(*Hx0);
    // if (Hxd0)
    //   DEBUG_VARS(*Hxd0);
    // if (Hu)
    //   DEBUG_VARS(*Hu);

    // if (Hxd1)
    // {
    //   *Hxd1 = -Eigen::Matrix<double, 3, 3>::Identity();
    // }
    // PRX_DBG_VARS(xdp1.transpose())
    // PRX_DBG_VARS(xd1.transpose())
    // PRX_DBG_VARS((xdp1 - xd1).transpose())

    return error;
  }

  Eigen::Vector<double, DimX + 1> state() const
  {
    return _qpos_vector.head(DimX + 1);
  }
  Eigen::Vector<double, DimXdot> stateDot() const
  {
    return _qvel_vector.head(DimXdot);
  }

  Eigen::Vector<double, DimXHidden> hidden_state() const
  {
    return _qpos_vector.tail(DimXHidden);
  }

  Eigen::Vector<double, DimXDotHidden> hidden_stateDot() const
  {
    return _qvel_vector.tail(DimXDotHidden);
  }

  // void init_mj_state(const StateDot& xd0, const Control& u, const double& dt) const
  // {
  //   if (not _mj_state_init)
  //   {
  //     PRINT_MSG("Resetting MJ Mushr");
  //     const StateDot xd1{ mj_predict(xd0, u, dt) };
  //     _qvel_init = _qvel_vector;
  //     _mj_state_init = true;
  //   }
  // }

  // StateStateDot mj_fwd_prop(const State& x0, const StateDot& xd0, const Control& u, const double& dt,
  //                           const StateHidden& state_hidden, const StateDotHidden& stateDot_hidden) const
  // {
  //   auto [x, xdot] = mj_fwd_prop_(x0, xd0, u, dt, state_hidden, stateDot_hidden);
  //   return { mj_copy(x), xdot };
  // }

  // std::pair<Eigen::Vector<double, DimX + 1>, Eigen::Vector<double, DimXdot>>
  void mj_fwd_prop(const State& x0, const StateDot& xd0, const Control& u, const double& dt,
                   const StateHidden& state_hidden, const StateDotHidden& stateDot_hidden) const
  {
    // d->qacc_warmstart[i] = 0;
    // _qacc_vector = _qacc_warmstart_init;
    // _qpos_vector = _qpos_init;
    // _qvel_vector = _qvel_init;
    const gtsam::Quaternion q0{ x0.rotation().toQuaternion() };

    _ctrl_vector = u;
    _qpos_vector.head(7) = mj_copy(x0);
    _qpos_vector.tail(DimXHidden) = state_hidden;
    // LOG_VARS(x0)
    // LOG_VARS(_qpos_vector.transpose());
    // _qpos_vector = State::Zero();
    // _qpos_vector.template segment<17>(0) = x0.template segment<17>(0);
    // _qpos_vector.template segment<11>(8) = Eigen::VectorXd::Zero(11);
    _qvel_vector.head(6) = xd0;
    _qvel_vector.tail(DimXDotHidden) = stateDot_hidden;
    // _qvel_vector = StateDot::Zero();
    // _qvel_vector.template segment<6>(0) = xd0.tmplate segment<6>(0);
    // _qvel_vector[1] = xd0[1];
    // _qvel_vector[5] = xd0[2];
    // mjtNum rot_mat[9], quat[4];
    // mju_quat2Mat(rot_mat, _qpos_vector.segment<4>(3).data());
    // mju_mat2Quat(quat, rot_mat);

    // auto xMJ = _qpos_vector.transpose();
    // auto xVelMJ = _qvel_vector.transpose();
    // auto uin = u.transpose();
    // auto mjAcc = _qacc_vector.transpose();
    // LOG_VARS(uin);
    // LOG_VARS(xMJ);
    // LOG_VARS(xVelMJ);
    // LOG_VARS(mjAcc);
    // dbg::variables::ofs_log << "Rot Matrix:\n";
    // dbg::variables::ofs_log << rot_mat[0] << " " << rot_mat[1] << " " << rot_mat[2] << "\n";
    // dbg::variables::ofs_log << rot_mat[3] << " " << rot_mat[4] << " " << rot_mat[5] << "\n";
    // dbg::variables::ofs_log << rot_mat[6] << " " << rot_mat[7] << " " << rot_mat[8] << "\n";
    // dbg::variables::ofs_log << "QuatMJ: " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << "\n";
    for (double ti = 0.0; ti < dt; ti += _mj_model->opt.timestep)
    {
      mj_step(_mj_model, _mj_data);
    }

    // const StateDot xd1{ _qvel_vector[0], _qvel_vector[1], _qvel_vector[5] };
    // const StateDot xd1{ _qvel_vector[0], _qvel_vector[1], _qvel_vector[5] };
    // return (StateStateDot() << _qpos_vector, _qvel_vector).finished();
    // LOG_VARS(_qpos_vector.head(7))
    // return { _qpos_vector.head(7), _qvel_vector.head(6) };
  }

private:
  const double _dt;

  const mjModel* _mj_model;
  mutable mjData* _mj_data;

  mutable Eigen::Map<Eigen::VectorXd> _ctrl_vector;
  mutable Eigen::Map<Eigen::VectorXd> _qpos_vector;
  mutable Eigen::Map<Eigen::VectorXd> _qvel_vector;
  mutable Eigen::Map<Eigen::VectorXd> _qacc_vector;

  const Eigen::VectorXd _qpos_init;
  const Eigen::VectorXd _qacc_warmstart_init;

  mutable Eigen::VectorXd _qvel_init;

  const Eigen::VectorXd _state_hidden, _stateDot_hidden;
  // const WrapperXdot0 _wrapper_state;
  // const WrapperControl _wrapper_control;
  // const WrapperDt _wrapper_dt;

  // const PartialXdot0 _partial_state;
  // const PartialControl _partial_control;
  // const PartialDt _partial_dt;
  mutable DerivWrapper _deriv_wrapper;
  mutable bool _mj_state_init;
};

class mushr_mujoco_stela_t : public stela_robot_interface_t<mushr_mujoco_types_t, mushr_mujoco_types_t>
{
  using This = mushr_mujoco_stela_t;
  using Base = stela_robot_interface_t<mushr_mujoco_types_t, mushr_mujoco_types_t>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

  using XVelFactor = prx_models::mushr_x_xdot_t;
  using VelUbarFactor = prx_models::mushr_xdot_ub_t;
  using CtrlUbarFactor = prx_models::mushr_ub_u_xdot_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = typename mushr_mujoco_types_t::State;
  using StateDot = typename mushr_mujoco_types_t::StateDot;
  using Control = typename mushr_mujoco_types_t::Control;
  using Observation = typename mushr_mujoco_types_t::Observation;

  using StateKeys = typename mushr_mujoco_types_t::StateKeys;
  using ControlKeys = typename mushr_mujoco_types_t::ControlKeys;
  using TimeKeys = typename mushr_mujoco_types_t::TimeKeys;

  using StateEstimates = typename mushr_mujoco_types_t::StateEstimates;
  using ControlEstimates = typename mushr_mujoco_types_t::ControlEstimates;

  using Poly = mushr_types::Control::Poly;
  using Parameters = mushr_types::Control::params;
  using PrxPlant = mushrFG_t;

  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };

  mushr_mujoco_stela_t() : Base() {};

  mushr_mujoco_stela_t(ros::NodeHandle& nh) : Base(nh)
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

  virtual void publish_current_state(const StateEstimates& estimates) override
  {
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<std_msgs::Float64>& zi{ msg->raw_sensor_data };
    // _last_observation.first[0] = zi[0].data;
    // _last_observation.first[1] = zi[1].data;
    // _last_observation.first[2] = zi[2].data;
    const Eigen::Vector3d position{ zi[0].data, zi[1].data, zi[2].data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3].data, zi[4].data, zi[5].data, zi[6].data) };
    // _last_observation.first[2] = prx::quaternion_to_euler(q)[2];
    _last_observation.first = gtsam::Pose3(gtsam::Rot3(q), position);
    _last_observation.second = msg->header.stamp;
    _new_observation = true;
    // DEBUG_VARS(_new_observation, _last_observation.first[0], _last_observation.first[1], _last_observation.first[2])
  }

  // Factor graph for "Idle" state (i.e. before starting execution or after reaching the goal)
  virtual GraphValues idle_state_to_fg(const std::size_t parent, const std::size_t child,
                                       const bool time_as_variable = true) override
  {
    using IntegrationFactor = mushr_mujoco_factor_t<double>;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
    GraphValues graph_values;

    const gtsam::Key k_x0{ keyX(1, parent) };
    const gtsam::Key k_x1{ keyX(1, child) };

    const gtsam::Key k_xdot0{ keyXdot(1, parent) };
    const gtsam::Key k_xdot1{ keyXdot(1, child) };

    const gtsam::Key k_u01{ keyU(parent, child) };
    const gtsam::Key k_t01{ keyT(parent, child) };

    NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e0) };
    NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e0) };
    NoiseModel dt_limit_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-1) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(12, 1e-1) };
    NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-1) };

    alloc();
    // mjModel* mj_model{ _mj_model_q.back() };
    mjData* mj_data{ _mj_data_q.back() };

    Eigen::Map<Eigen::VectorXd> qpos(mj_data->qpos, _mj_model->nq, 1);
    auto qposMJ = qpos.transpose();
    LOG_VARS(qposMJ)
    // _mj_model_q.pop_back();
    _mj_data_q.pop_back();

    mushr_mujoco_types_t::StateHidden state_hidden;
    mushr_mujoco_types_t::StateDotHidden stateDot_hidden;

    for (int i = 0; i < state_hidden.size(); ++i)
    {
      state_hidden[i] = mj_data->qpos[7 + i];
    }
    for (int i = 0; i < stateDot_hidden.size(); ++i)
    {
      stateDot_hidden[i] = mj_data->qvel[7 + i];
    }

    auto xHidden = state_hidden.transpose();
    auto xDotHidden = stateDot_hidden.transpose();
    // LOG_VARS(xHidden)
    // LOG_VARS(xDotHidden)

    graph_values.first.emplace_shared<IntegrationFactor>(k_x1, k_xdot1, k_x0, k_xdot0, k_u01, k_t01, integration_noise,
                                                         _mj_model, mj_data, state_hidden, stateDot_hidden);
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

    const gtsam::Quaternion q{ x.rotation().toQuaternion() };
    pt.point.resize(13);
    pt.point[0] = x.translation()[0];
    pt.point[1] = x.translation()[1];
    pt.point[2] = x.translation()[2];
    pt.point[3] = q.w();
    pt.point[4] = q.x();
    pt.point[5] = q.y();
    pt.point[6] = q.z();

    for (int i = 0; i < 6; ++i)
    {
      pt.point[7 + i] = xdot[i];
    }
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
    const Eigen::Vector3d position{ pt.point[0], pt.point[1], pt.point[2] };
    const gtsam::Quaternion q{ pt.point[3], pt.point[4], pt.point[5], pt.point[6] };
    x = gtsam::Pose3(gtsam::Rot3(q), position);
  }

  virtual void copy_stateDot(StateDot& xd, const ml4kp_bridge::SpacePoint& pt) override
  {
    for (int i = 0; i < 6; ++i)
    {
      xd[i] = pt.point[i];
    }
  }

  virtual void copy_hidden_state(mushr_mujoco_types_t::StateHidden& x, const ml4kp_bridge::SpacePoint& pt)
  {
    for (int i = 0; i < mushr_mujoco_types_t::DimXHidden; ++i)
    {
      x[i] = pt.point[mushr_mujoco_types_t::DimXHidden + i];
    }
  }

  virtual void copy_hidden_stateDot(mushr_mujoco_types_t::StateDotHidden& x, const ml4kp_bridge::SpacePoint& pt)
  {
    for (int i = 0; i < mushr_mujoco_types_t::DimXHidden; ++i)
    {
      x[i] = pt.point[mushr_mujoco_types_t::DimXHidden + i];
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
      // const Eigen::Vector<double, 1> vec{ state.angle() };
      rotation = state.rotation().matrix();
      // rotation = prx::euler_to_rotation<Eigen::Matrix3d>(vec, "Z");
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
      const Eigen::Matrix2d R{ state.rotation().matrix().template block<2, 2>(0, 0) };
      vec = -R.transpose() * vec;

      // const double Sth{ std::sin(state[2]) };
      // const double Cth{ std::cos(state[2]) };
      const double Sth{ R(1, 0) };
      const double Cth{ R(0, 0) };
      const double ax{ collision ? p1[0] : (p1[0] - state.x()) };
      const double ay{ collision ? p1[1] : (p1[1] - state.y()) };
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
      const Eigen::Matrix2d R{ x0.rotation().matrix().template block<2, 2>(0, 0) };
      // const Eigen::Matrix2d R{ x0.rotation<Eigen::Matrix2d>() };
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

  virtual GraphValues add_observation_factor(const std::size_t prev_id, const std::size_t curr_id,
                                             const ros::Time& ti) override
  {
    using ObservationFactor = prx::fg::lie_ode_observation_factor_t<State, StateDot>;

    GraphValues graph_values;
    if (not _new_observation)
    {
      return graph_values;
    }

    const gtsam::Key x0{ keyX(1, prev_id) };
    const gtsam::Key x1{ keyX(1, curr_id) };
    const gtsam::Key xdot0{ keyXdot(1, prev_id) };
    const gtsam::Key xdot1{ keyXdot(1, curr_id) };
    const gtsam::Key u01{ keyU(prev_id, curr_id) };

    NoiseModel z_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1.0e-0) };

    const Observation& zi{ _last_observation.first };
    const double dt{ (_last_observation.second - ti).toSec() };

    // LOG_MSG("Adding Observation Factor");
    // LOG_VARS(prev_id, curr_id, dt, zi);
    graph_values.first.emplace_shared<ObservationFactor>(x0, xdot0, z_noise, zi, dt, "Observation");

    _new_observation = false;
    return graph_values;
  }
  virtual GraphValues idle_root(const std::size_t root) override
  {
    GraphValues graph_values;

    const gtsam::Key k_x{ keyX(1, root) };
    const gtsam::Key k_xdot{ keyXdot(1, root) };

    NoiseModel x_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e0) };

    graph_values.first.addPrior(k_x, _idle_state, x_prior_noise);
    graph_values.first.addPrior(k_xdot, _idle_state_dot, xdot_prior_noise);

    graph_values.second.insert(k_x, _idle_state);
    graph_values.second.insert(k_xdot, _idle_state_dot);

    return graph_values;
  }

  // virtual GraphValues node_edge_to_fg(const std::size_t parent, const std::size_t child,
  //                                     const ml4kp_bridge::SpacePoint& node_state, const ml4kp_bridge::Plan&
  //                                     edge_plan, const bool time_as_variable = true) override
  virtual GraphValues node_edge_to_fg(const prx_models::Node& node, const prx_models::Edge& edge) override
  {
    using IntegrationFactor = mushr_mujoco_factor_t<double>;
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

    NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 5e0) };
    NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e0) };
    NoiseModel dt_limit_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-1) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(12, 1e-1) };
    NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-3) };

    alloc();
    // mjModel* mj_model{ _mj_model_q.back() };
    mjData* mj_data{ _mj_data_q.back() };

    // _mj_model_q.pop_back();
    _mj_data_q.pop_back();

    mushr_mujoco_types_t::StateHidden state_hidden;
    mushr_mujoco_types_t::StateDotHidden stateDot_hidden;

    int i = 0;
    for (; i < state_hidden.size(); ++i)
    {
      state_hidden[i] = node.parameters.point[i];
    }
    int j = 0;
    for (; i < stateDot_hidden.size(); ++i, ++j)
    {
      stateDot_hidden[j] = node.parameters.point[i];
    }

    graph_values.first.emplace_shared<IntegrationFactor>(k_x1, k_xdot1, k_x0, k_xdot0, k_u01, k_t01, integration_noise,
                                                         _mj_model, mj_data, state_hidden, stateDot_hidden);
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

  void init(const prx::param_loader& params)
  {
    _model_path = params["mj_model"].as<>();

    DEBUG_PRINT
    alloc();
    DEBUG_PRINT
  }
  void log_params()
  {
  }
  void print_params()
  {
  }

protected:
  void alloc()
  {
    // if (_mj_vfs == nullptr)
    // {
    //   mj_defaultVFS(_mj_vfs);
    //   mj_addFileVFS(_mj_vfs, nullptr, _model_path);
    // }
    if (_mj_model == NULL)
    {
      _mj_model = init_mj_model(_model_path);
    }
    if (_mj_data_q.size() == 0)
    {
      for (int i = 0; i < 100; ++i)
      {
        // _mj_model_q.push_back(init_mj_model(_model_path, _mj_vfs));
        _mj_data_q.push_back(init_mj_data(_mj_model));
      }
    }
  }

  mjVFS _mj_vfs;

  std::string _model_path;

  mjModel* _mj_model;  // queues of pre-alloc data
  // std::vector<mjModel*> _mj_model_q;  // queues of pre-alloc data
  std::vector<mjData*> _mj_data_q;  // queues of pre-alloc data

  GraphValues aux_graph;

  ros::Subscriber _sensor_subscriber;
};

class mushr_mujoco_t : public prx::plant_t
{
  // using State = mushr_types::State::type;
  // using StateDot = mushr_types::StateDot::type;
  // using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;
  // using MushrMjFactor = mushr_mujoco_factor_t<>;

public:
  mushr_mujoco_t(const std::string& path) : plant_t(path)
  {
    // state_memory = { &_state[0], &_state[1], &_state[2], &_ubar[0], &_ubar[1] };
    // state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
    // &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    std::string topology;
    std::vector<double> min_bounds, max_bounds;
    for (int i = 0; i < _state_proxy.size(); ++i)
    {
      state_memory.push_back(&_state_proxy[i]);
      topology += "E";
      min_bounds.push_back(-std::numeric_limits<double>::infinity());
      max_bounds.push_back(std::numeric_limits<double>::infinity());
    }
    for (int i = 0; i < _state_dot.size(); ++i)
    {
      state_memory.push_back(&_state_dot[i]);
      topology += "E";
      min_bounds.push_back(-std::numeric_limits<double>::infinity());
      max_bounds.push_back(std::numeric_limits<double>::infinity());
    }
    state_space = new prx::space_t(topology, state_memory, "mushr_state");
    state_space->set_bounds(min_bounds, max_bounds);

    topology = "";
    min_bounds.clear();
    max_bounds.clear();
    control_memory = { &_ctrl[0], &_ctrl[1] };
    for (int i = 0; i < _ctrl.size(); ++i)
    {
      topology += "E";
      min_bounds.push_back(-std::numeric_limits<double>::infinity());
      max_bounds.push_back(std::numeric_limits<double>::infinity());
    }
    input_control_space = new prx::space_t(topology, control_memory, "mushr_ctrl");
    input_control_space->set_bounds(min_bounds, max_bounds);

    // derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    // derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");

    parameter_memory = {};
    topology = "";
    for (int i = 0; i < _state_hidden.size(); ++i)
    {
      parameter_memory.push_back(&_state_hidden[i]);
      topology += "E";
    }
    for (int i = 0; i < _stateDot_hidden.size(); ++i)
    {
      parameter_memory.push_back(&_stateDot_hidden[i]);
      topology += "E";
    }
    parameter_space = new prx::space_t(topology, parameter_memory, "mushr_params");
    // const std::string param_topology{ std::string(parameter_memory.size(), 'E') };

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }

  ~mushr_mujoco_t() {};

  virtual void init(const prx::param_loader& params) override
  {
    prx::plant_t::init(params);
    if (params.exists("mj_model"))
    {
      const std::string model_path{ params["mj_model"].as<std::string>() };
      _mj_model = init_mj_model(model_path);
      _mj_data = init_mj_data(_mj_model);

      _mj_mushr_factor = std::make_shared<mushr_mujoco_factor_t<>>(
          0, 1, 2, 3, 4, prx::simulation_step, nullptr, _mj_model, _mj_data, mushr_mujoco_types_t::StateHidden::Zero(),
          mushr_mujoco_types_t::StateDotHidden::Zero());
      Eigen::Map<Eigen::VectorXd> qpos(_mj_data->qpos, _mj_model->nq, 1);
      Eigen::Map<Eigen::VectorXd> qvel(_mj_data->qvel, _mj_model->nv, 1);

      for (int i = 0; i < _mj_model->njnt; i++)
      {
        const std::string joint_name(_mj_model->names + _mj_model->name_jntadr[i]);
        DEBUG_VARS(joint_name)
      }

      // DEBUG_VARS(_mj_model->nu, _mj_model->nq, _mj_model->nv)
      // DEBUG_VARS(qpos.transpose())
      _state_proxy = qpos.head(7);
      _state_dot = qvel.head(6);
      _state_hidden = _mj_mushr_factor->hidden_state();
      _stateDot_hidden = _mj_mushr_factor->hidden_stateDot();
      // , _qacc_vector(_mj_data->qacc_warmstart, _mj_model->nv, 1)
    }
    else
    {
      prx_throw("[mushr_mujoco_t::init] No mj_model found!")
    }
  }

  virtual void propagate(const double simulation_step) override final
  {
    LOG_MSG("***** ***** ***** ***** ***** *****")
    // mushr_mujoco_factor_t<>::hidden_state(_mj_model, _mj_data);
    // mushr_mujoco_factor_t<>::hidden_stateDot(_mj_model, _mj_data);
    _state = mj_copy(_state_proxy);
    // std::tie(_state, _state_dot) =
    _mj_mushr_factor->mj_fwd_prop(_state, _state_dot, _ctrl, prx::simulation_step, _state_hidden, _stateDot_hidden);
    _state_proxy = _mj_mushr_factor->state();
    _state_dot = _mj_mushr_factor->stateDot();

    auto x_next = mj_copy(_state_proxy);
    auto q0 = _state.rotation().toQuaternion();
    auto q1 = x_next.rotation().toQuaternion();
    const double q0Dq1{ q0.dot(q1) };
    // LOG_VARS(_state)
    // LOG_VARS(_state_proxy.transpose())
    // LOG_VARS(q0)
    // LOG_VARS(q1)
    // LOG_VARS(q0Dq1)

    _state_hidden = _mj_mushr_factor->hidden_state();
    _stateDot_hidden = _mj_mushr_factor->hidden_stateDot();
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    *body = _state.matrix();
    // body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
    // body->translation()[0] = _state[0];
    // body->translation()[1] = _state[1];
    // body->translation()[2] = 0.0;
  }
  virtual void compute_derivative() override final
  {
  }

protected:
  mjData* _mj_data;
  mjModel* _mj_model;

  std::shared_ptr<mushr_mujoco_factor_t<>> _mj_mushr_factor;

  Eigen::Vector<double, 7> _state_proxy;
  mushr_mujoco_types_t::State _state;
  mushr_mujoco_types_t::StateDot _state_dot;
  mushr_mujoco_types_t::Control _ctrl;
  mushr_mujoco_types_t::StateHidden _state_hidden;
  mushr_mujoco_types_t::StateDotHidden _stateDot_hidden;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushr_mujoco_t, mushrMujoco)
