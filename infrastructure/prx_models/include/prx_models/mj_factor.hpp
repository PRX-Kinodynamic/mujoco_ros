#pragma once

#include <string>

// Ros
#include <Eigen/src/Core/Matrix.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <interface/SensorDataStamped.h>
#include <torch_bridge/query_utils.hpp>
// torch_bridge/query_utils.hpp
// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include "mujoco/mjmodel.h"
#include "torch_bridge/TorchQuery.h"

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>

namespace prx_models
{

template <typename... Types>
class mushr_mj_factor_t
  : public gtsam::NoiseModelFactorN<prx_models::mushr_types::StateDot::type, prx_models::mushr_types::StateDot::type,
                                    prx_models::mushr_types::Control::type, Types...>
{
  using State = prx_models::mushr_types::State::type;
  using StateDot = prx_models::mushr_types::StateDot::type;
  using StateDotDot = prx_models::mushr_types::StateDot::type;

  using Control = prx_models::mushr_types::Control::type;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };
  static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };

  using Base = gtsam::NoiseModelFactorN<StateDot, StateDot, Control, Types...>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  static constexpr std::size_t NumTypes{ sizeof...(Types) };

  using WrapperXdot0 = std::function<StateDot(const StateDot&, const Control&, const double&)>;
  using WrapperControl = std::function<StateDot(const Control&, const double&, const StateDot&)>;
  using WrapperDt = std::function<StateDot(const double&, const StateDot&, const Control&)>;

  using PartialXdot0 =
      prx::math::first_order_derivative_t<WrapperXdot0, StateDot, 3, -1, const Control&, const double&>;
  using PartialControl =
      prx::math::first_order_derivative_t<WrapperControl, Control, 3, -1, const double&, const StateDot&>;
  using PartialDt = prx::math::first_order_derivative_t<WrapperDt, double, 3, -1, const StateDot&, const Control&>;

  mushr_mj_factor_t() = delete;
  mushr_mj_factor_t(const mushr_mj_factor_t& other) = delete;

public:
  static mjModel* init_mj_model(const std::string model_path)
  {
    std::string error;
    error.reserve(1000);
    mjModel* mj_model{ mj_loadXML(model_path.c_str(), NULL, error.data(), error.capacity()) };
    if (!mj_model or error.size() != 0)
    {
      std::cerr << "Error in loading model." << std::endl;
      std::cout << error << std::endl;
    }
    return mj_model;
  }

  static mjData* init_mj_data(const mjModel* mj_model)
  {
    mjData* mj_data{ mj_makeData(mj_model) };
    for (int i = 0; i < 1.0 / mj_model->opt.timestep; i++)
    {
      mj_step(mj_model, mj_data);
    }
    return mj_data;
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>
  mushr_mj_factor_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const gtsam::Key dt,
                    const NoiseModel& cost_model, const mjModel* mj_model, mjData* mj_data, const double h = 0.01)
    : Base(cost_model, xd1, xd0, u, dt)
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
    , _wrapper_xdot0([&](const StateDot& xd0, const Control& u, const double& dt) { return mj_predict(xd0, u, dt); })
    , _wrapper_control([&](const Control& u, const double& dt, const StateDot& xd0) { return mj_predict(xd0, u, dt); })
    , _wrapper_dt([&](const double& dt, const StateDot& xd0, const Control& u) { return mj_predict(xd0, u, dt); })
    , _partial_xdot0(_wrapper_xdot0, h)
    , _partial_control(_wrapper_control, h)
    , _partial_dt(_wrapper_dt, h)
    , _mj_state_init(false)
  {
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(0 == Num), bool> = true>
  mushr_mj_factor_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const double& dt,
                    const NoiseModel& cost_model, const mjModel* mj_model, mjData* mj_data, const double h = 0.01)
    : Base(cost_model, xd1, xd0, u)
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
    , _wrapper_xdot0([&](const StateDot& xd0, const Control& u, const double& dt) { return mj_predict(xd0, u, dt); })
    , _wrapper_control([&](const Control& u, const double& dt, const StateDot& xd0) { return mj_predict(xd0, u, dt); })
    , _wrapper_dt([&](const double& dt, const StateDot& xd0, const Control& u) { return mj_predict(xd0, u, dt); })
    , _partial_xdot0(_wrapper_xdot0, h)
    , _partial_control(_wrapper_control, h)
    , _partial_dt(_wrapper_dt, h)
    , _mj_state_init(false)
  {
  }

  ~mushr_mj_factor_t() override
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

  Eigen::VectorXd predict(const StateDot& xd0, const Control& u, const double& dt,  // no-lint
                          gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,         // no-lint
                          gtsam::OptionalJacobian<3, 2> Hu = boost::none,           // no-lint
                          gtsam::OptionalJacobian<3, 1> Hdt = boost::none) const
  {
    if (Hxd0)
    {
      *Hxd0 = _partial_xdot0(xd0, u, dt);
    }
    if (Hu)
    {
      *Hu = _partial_control(u, dt, xd0);
    }
    if (Hdt)
    {
      *Hdt = _partial_dt(dt, xd0, u);
    }

    const Eigen::VectorXd xd1{ mj_predict(xd0, u, dt) };

    return xd1;
  }

  Eigen::VectorXd mj_predict(const StateDot& x0, const StateDot& xd0, const Control& u, const double& dt) const
  {
    _qpos_vector = x0;
    _qvel_vector = xd0;
    _ctrl_vector = u;

    for (double ti = 0.0; ti < dt; ti += _mj_model->opt.timestep)
    {
      mj_step(_mj_model, _mj_data);
    }

    // const StateDot xd1{ _qvel_vector[0], _qvel_vector[1], _qvel_vector[5] };
    // const StateDot xd1{ _qvel_vector[0], _qvel_vector[1], _qvel_vector[5] };

    return (Eigen::VectorXd() << _qpos_vector, _qvel_vector).finished();
  }

  virtual Eigen::VectorXd evaluateError(const StateDot& xd1, const StateDot& xd0, const Control& u,
                                        const Types&... dt01,  // no-lint
                                        OptDeriv Hxd1 = boost::none, OptDeriv Hxd0 = boost::none,
                                        OptDeriv Hu = boost::none, OptionalMatrix<Types>... H) const override
  {
    StateDot xdp1{};
    if constexpr (0 == NumTypes)
    {
      xdp1 = predict(xd0, u, _dt, Hxd0, Hu);
      // return error(x1, x0, xdot, _h, H1, H0, Hdot);
    }
    else
    {
      xdp1 = predict(xd0, u, dt01..., Hxd0, Hu, H...);
      // return error(x1, x0, xdot, xd..., H1, H0, Hdot, H...);
    }

    if (Hxd1)
    {
      *Hxd1 = -Eigen::Matrix<double, 3, 3>::Identity();
    }
    // PRX_DBG_VARS(xdp1.transpose())
    // PRX_DBG_VARS(xd1.transpose())
    // PRX_DBG_VARS((xdp1 - xd1).transpose())

    return xdp1 - xd1;
  }

private:
  void init_mj_state(const StateDot& xd0, const Control& u, const double& dt) const
  {
    if (not _mj_state_init)
    {
      PRINT_MSG("Resetting MJ Mushr");
      const StateDot xd1{ mj_predict(xd0, u, dt) };
      _qvel_init = _qvel_vector;
      _mj_state_init = true;
    }
  }

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

  const WrapperXdot0 _wrapper_xdot0;
  const WrapperControl _wrapper_control;
  const WrapperDt _wrapper_dt;

  const PartialXdot0 _partial_xdot0;
  const PartialControl _partial_control;
  const PartialDt _partial_dt;

  mutable bool _mj_state_init;
};
