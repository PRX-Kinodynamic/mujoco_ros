#include <thread>
#include "mujoco/mujoco.h"

#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include <utils/std_utils.cpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

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
    , _ctrl_vector(_mj_data->ctrl, DimU, 1)
    , _qpos_vector(mj_data->qpos, _mj_model->nq, 1)
    , _qvel_vector(mj_data->qvel, _mj_model->nv, 1)
    , _qpos_init(_qpos_vector)
    , _qvel_init(_qvel_vector)
    , _wrapper_xdot0([&](const StateDot& xd0, const Control& u, const double& dt) { return mj_predict(xd0, u, dt); })
    , _wrapper_control([&](const Control& u, const double& dt, const StateDot& xd0) { return mj_predict(xd0, u, dt); })
    , _wrapper_dt([&](const double& dt, const StateDot& xd0, const Control& u) { return mj_predict(xd0, u, dt); })
    , _partial_xdot0(_wrapper_xdot0, h)
    , _partial_control(_wrapper_control, h)
    , _partial_dt(_wrapper_dt, h)
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

  StateDot mj_predict(const StateDot& xd0, const Control& u, const double& dt) const
  {
    _ctrl_vector = u;
    _qvel_vector[0] = xd0[0];
    _qvel_vector[1] = xd0[1];
    _qvel_vector[5] = xd0[2];
    for (double ti = 0.0; ti < dt; ti += _mj_model->opt.timestep)
    {
      mj_step(_mj_model, _mj_data);
    }
    const StateDot xd1{ _qvel_vector[0], _qvel_vector[1], _qvel_vector[5] };

    return xd1;
  }

  StateDot predict(const StateDot& xd0, const Control& u, const double& dt,  // no-lint
                   gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,         // no-lint
                   gtsam::OptionalJacobian<3, 2> Hu = boost::none,           // no-lint
                   gtsam::OptionalJacobian<3, 1> Hdt = boost::none) const
  {
    if (Hxd0)
    {
      _qacc_vector = _qacc_warmstart_init;
      _qpos_vector = _qpos_init;
      _qvel_vector = _qvel_init;
      *Hxd0 = _partial_xdot0(xd0, u, dt);
    }
    if (Hu)
    {
      _qacc_vector = _qacc_warmstart_init;
      _qpos_vector = _qpos_init;
      _qvel_vector = _qvel_init;
      *Hu = _partial_control(u, dt, xd0);
    }
    if (Hdt)
    {
      _qacc_vector = _qacc_warmstart_init;
      _qpos_vector = _qpos_init;
      _qvel_vector = _qvel_init;
      *Hdt = _partial_dt(dt, xd0, u);
    }

    _qacc_vector = _qacc_warmstart_init;
    _qpos_vector = _qpos_init;
    _qvel_vector = _qvel_init;

    const StateDot xd1{ mj_predict(xd0, u, dt) };
    _qvel_init = _qvel_vector;

    return xd1;
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
    return xdp1 - xd1;
  }

private:
  const double _dt;

  const mjModel* _mj_model;
  mutable mjData* _mj_data;

  mutable Eigen::Map<Control> _ctrl_vector;
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
};

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MjFactorPlayground" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string model_path;
  // std::string file_out;
  // std::string plan_file;

  PARAM_SETUP(nh, model_path);
  // PARAM_SETUP(nh, file_out);
  // PARAM_SETUP(nh, plan_file);

  using MjFactor = mushr_mj_factor_t<>;
  // template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>

  mjModel* mj_model{ MjFactor::init_mj_model(model_path) };
  mjData* mj_data{ MjFactor::init_mj_data(mj_model) };

  // for (int i = 0; i < mj_model->nbody; ++i)
  // {
  //   const std::string body_name{ std::string(mj_model->names + mj_model->name_bodyadr[i]) };
  //   DEBUG_VARS(i, body_name);
  // }

  const gtsam::Key xd1{ gtsam::Symbol('V', 1) };
  const gtsam::Key xd0{ gtsam::Symbol('V', 0) };
  const gtsam::Key u{ gtsam::Symbol('U', 0) };
  // const double dt{ 0.01 };
  const double dt{ 0.01 };

  DEBUG_VARS(mj_model->opt.timestep);
  MjFactor mj_factor(xd1, xd0, u, dt, nullptr, mj_model, mj_data);

  prx_models::mushr_types::State::type x(1.0, 0.0, 0.0);
  prx_models::mushr_types::StateDot::type xdot;
  prx_models::mushr_types::Control::type ctrl;
  ctrl[0] = 0.30938896;   // 0.91120905;
  ctrl[1] = -0.31807874;  // -0.73472644;

  // DEBUG_VARS(x, xdot.transpose())
  for (double ti = 0; ti < 10.0; ti += dt)
  {
    xdot = mj_factor.predict(xdot, ctrl, dt);
    x = prx_models::mushr_x_xdot_t::predict(x, xdot, dt);

    LOG_VARS(ti, x, xdot.transpose());
    // DEBUG_VARS(ti, x, xdot.transpose())
  }

  // mj_factor.predict(xdot, ctrl, dt);
  return 0;
}