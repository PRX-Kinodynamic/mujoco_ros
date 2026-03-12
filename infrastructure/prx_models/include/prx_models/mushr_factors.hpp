#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <utils/dbg_utils.hpp>
// #include <utils/dbg_utils.hpp>

#include <prx/simulation/plant.hpp>
#include <prx/utilities/math/first_order_derivative.hpp>
#include <prx/factor_graphs/factors/noise_model_factors.hpp>
#include <prx/factor_graphs/lie_groups/se2.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>

#include <prx_models/mj_mushr.hpp>

#include "mujoco/mujoco.h"
// #include <prx/factor_graphs/factors/mushr_types.hpp>
// #include "prx/factor_graphs/factors/noise_model_factor.hpp"
// #include "prx/factor_graphs/utilities/perception/camera.hpp"
// #include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx_models
{
namespace mushr_types
{

namespace Parameters
{
constexpr double L{ 0.31 };
constexpr double mass{ 3.5 };
}  // namespace Parameters
namespace State
{
using type = prx::fg::SE2_t;
// using type = Eigen::Vector<double, 3>;
constexpr std::size_t x{ 0 };
constexpr std::size_t y{ 1 };
constexpr std::size_t theta{ 2 };
}  // namespace State

namespace StateDot
{
using type = Eigen::Vector<double, 3>;
constexpr std::size_t xdot{ 0 };
constexpr std::size_t ydot{ 1 };
constexpr std::size_t thetadot{ 2 };
}  // namespace StateDot

namespace Ubar
{
constexpr std::size_t Dim{ 2 };
constexpr std::size_t ParamsDim{ 3 };
using type = Eigen::Vector<double, Dim>;
using params = Eigen::Vector<double, ParamsDim>;

constexpr std::size_t velocity{ 0 };
constexpr std::size_t beta{ 1 };

constexpr std::size_t accel_slope{ 0 };
constexpr std::size_t steering_param{ 1 };
constexpr std::size_t max_vel_param{ 2 };

}  // namespace Ubar

namespace Control
{

constexpr std::size_t ParamsDim{ 5 };
constexpr std::size_t PolyDeg{ 3 };

using type = Eigen::Vector<double, 2>;
using params = Eigen::Vector<double, ParamsDim>;
using Poly = Eigen::Vector<double, PolyDeg + 1>;

constexpr std::size_t accel{ prx_models::mushr_t::control::velocity_idx };
constexpr std::size_t vel_desired{ prx_models::mushr_t::control::velocity_idx };
constexpr std::size_t steering{ prx_models::mushr_t::control::steering_idx };
constexpr std::size_t friction{ 2 };
constexpr std::size_t delta_offset{ 3 };
constexpr std::size_t delta_gain{ 4 };

double beta(const double& delta, gtsam::OptionalJacobian<1, 1> Hd = boost::none)
{
  if (Hd)
  {
    const double tan_d_2{ std::pow(std::tan(delta), 2) };
    (*Hd)(0, 0) = (tan_d_2 / 2.0 + 0.5) / (tan_d_2 / 4.0 + 1.0);
  }
  return std::atan(0.5 * std::tan(delta));
}

// f(x) = c0 x^n + c1 x^{n-1} + ... + c_{n-1} x^{n-n+1} + c_{n}
template <Eigen::Index I, std::enable_if_t<(I == 0), bool> = true>
inline double poly_eval(const Poly& poly, const double& x, gtsam::OptionalJacobian<1, 1> Hx = boost::none)
{
  // Given ax^n, this does ax^{n-1} ( avoid recomputing this for H)
  const double xaux{ poly[I] * std::pow(x, PolyDeg - 1) };

  if (Hx)
  {
    (*Hx)(0, 0) = (*Hx)(0, 0) + xaux * PolyDeg;
  }
  return xaux * x;
}

template <Eigen::Index I, std::enable_if_t<(I > 0), bool> = true>
inline double poly_eval(const Poly& poly, const double& x, gtsam::OptionalJacobian<1, 1> Hx = boost::none)
{
  constexpr int deg{ PolyDeg - I };
  const double xaux{ poly[I] * std::pow(x, deg - 1) };

  if (Hx)
  {
    (*Hx)(0, 0) = (*Hx)(0, 0) + xaux * deg;
  }

  return xaux * x + poly_eval<I - 1>(poly, x, Hx);
}

inline double evaluate_polynomial(const Poly& poly, const double& x, gtsam::OptionalJacobian<1, 1> Hx = boost::none)
{
  if (Hx)
  {
    *Hx = Eigen::Matrix<double, 1, 1>::Zero();
  }
  return poly[PolyDeg] + poly_eval<PolyDeg - 1>(poly, x, Hx);
}
}  // namespace Control

}  // namespace mushr_types

using mushr_x_xdot_nodT_t = prx::fg::lie_integration_factor_t<mushr_types::State::type, mushr_types::StateDot::type>;
using mushr_x_xdot_t = prx::fg::lie_integration_factor_t<mushr_types::State::type, mushr_types::StateDot::type, double>;

// CtrlUbarFactor
class mushr_ub_u_xdot_param_t
  : public prx::fg::noise_model_4factor_t<mushr_types::Ubar::type, mushr_types::Control::type, mushr_types::Ubar::type,
                                          mushr_types::Ubar::params>
{
  using Base = prx::fg::noise_model_4factor_t<mushr_types::Ubar::type, mushr_types::Control::type,
                                              mushr_types::Ubar::type, mushr_types::Ubar::params>;

public:
  using Xdot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;
  using Params = mushr_types::Ubar::params;
  using U = mushr_types::Control::type;

  mushr_ub_u_xdot_param_t(gtsam::Key ubar1, gtsam::Key u, gtsam::Key ubar0, gtsam::Key param,
                          const gtsam::noiseModel::Base::shared_ptr& cost_model, const double dt)
    : Base(ubar1, u, ubar0, param, cost_model, 0.01), _dt(dt)
  {
  }

  static Ubar dynamics(const U& u, const Ubar& ubar, const Params& params, const double& dt)
  {
    const double& v_current{ ubar[mushr_types::Ubar::velocity] };
    const double& steering{ u[mushr_types::Control::steering] };
    const double& v_desired{ u[mushr_types::Control::vel_desired] };

    const double& accel_slope{ params[mushr_types::Ubar::accel_slope] };
    const double& steering_param{ params[mushr_types::Ubar::steering_param] };
    const double& max_vel_param{ params[mushr_types::Ubar::max_vel_param] };

    const double dv{ max_vel_param * v_desired - v_current };
    const double v_next{ v_current + dt * dv * accel_slope };
    const double beta{ std::atan(0.5 * std::tan(steering * steering_param)) };

    Ubar ubar_next{};
    ubar_next[mushr_types::Ubar::beta] = beta;
    ubar_next[mushr_types::Ubar::velocity] = v_next;

    return ubar_next;
  }

  virtual Ubar predict(const U& u, const Ubar& ubar, const Params& params) const override
  {
    return dynamics(u, ubar, params, _dt);
  }

  virtual Ubar compute_error(const Ubar& ubar1, const U& u, const Ubar& ubar0, const Params& params) const override
  {
    return predict(u, ubar0, params) - ubar1;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Ubar ubar1{ values.at<Ubar>(key<1>()) };
    const U u{ values.at<U>(key<2>()) };
    const Ubar Ubar0{ values.at<Ubar>(key<3>()) };
    const Params params{ values.at<Params>(key<4>()) };

    os << ubar1.transpose() << " ";                                   // 1, 2, 3
    os << u.transpose() << " ";                                       // 4, 5, 6
    os << Ubar0.transpose() << " ";                                   // 7, 8
    os << params.transpose() << " ";                                  // 9
    os << compute_error(ubar1, u, Ubar0, params).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

private:
  const double _dt;
};

class mushr_xdot_ub_t : public gtsam::NoiseModelFactorN<mushr_types::StateDot::type, mushr_types::Ubar::type>
// public prx::fg::noise_model_2factor_t<mushr_types::StateDot::type, mushr_types::Ubar::type>
{
  // using Base = noise_model_2factor_t<mushr_types::StateDot::type, mushr_types::Ubar::type>;
  using Base = gtsam::NoiseModelFactorN<mushr_types::StateDot::type, mushr_types::Ubar::type>;
  using Error = Eigen::VectorXd;

public:
  using X = mushr_types::State::type;
  using Xdot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };
  static constexpr Eigen::Index DimUbar{ gtsam::traits<Ubar>::dimension };
  using XdotColumn = Eigen::Vector<double, DimXdot>;
  mushr_xdot_ub_t(gtsam::Key xdot, gtsam::Key ubar, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, xdot, ubar)
  // : Base(xdot, ubar, cost_model, 0.01)
  {
  }

  static Xdot dynamics(const Ubar& ubar, gtsam::OptionalJacobian<DimXdot, DimUbar> Hubar = boost::none)
  {
    const double& vt{ ubar[mushr_types::Ubar::velocity] };
    const double& beta{ ubar[mushr_types::Ubar::beta] };

    const double cBeta{ std::cos(beta) };
    const double sBeta{ std::sin(beta) };
    const double lr{ mushr_types::Parameters::L / 2.0 };  // L = lr + lf (rear|front)
    const double wt{ vt * sBeta / lr };

    // const Xdot t_x{ vt * cTh, vt * sTh, wt };
    // const Xdot t_x{ vt, 0, beta };
    // const Xdot t_x{ vt, 0, wt };
    const Xdot t_x{ vt * cBeta, vt * sBeta, wt };

    if (Hubar)
    {
      // (*Hubar) = Eigen::Matrix<double, DimXdot, DimUbar>::Zero();
      // DEBUG_VARS(*Hubar);
      (*Hubar).col(mushr_types::Ubar::velocity) = XdotColumn(cBeta, sBeta, sBeta / lr);
      (*Hubar).col(mushr_types::Ubar::beta) = XdotColumn(-vt * sBeta, vt * cBeta, (vt * cBeta) / lr);
      // [   cos(beta),     -v*sin(beta)]
      // [   sin(beta),      v*cos(beta)]
      // [sin(beta)/lr, (v*cos(beta))/lr]
    }

    return t_x;
  }

  virtual Xdot predict(const Ubar& ubar, gtsam::OptionalJacobian<DimXdot, DimUbar> Hubar = boost::none) const
  {
    return dynamics(ubar, Hubar);
  }

  // virtual Error compute_error(const X0& x0, const X1& x1, const X2& x2) const
  virtual Error evaluateError(const Xdot& xdot, const Ubar& ub,  // no-lint
                              boost::optional<Eigen::MatrixXd&> Hxdot = boost::none,
                              boost::optional<Eigen::MatrixXd&> Hubar = boost::none) const override
  {
    const Error error{ predict(ub, Hubar) - xdot };
    if (Hxdot)
    {
      *Hxdot = -Eigen::Matrix<double, DimXdot, DimXdot>::Identity();
    }

    return error;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Xdot xdot{ values.at<Xdot>(key<1>()) };
    const Ubar ubar{ values.at<Ubar>(key<2>()) };

    os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xdot.transpose() << " ";  // 4, 5, 6
    os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << ubar.transpose() << " ";  // 7, 8
    os << "Error: " << evaluateError(xdot, ubar).transpose() << " ";                         //, 5, 6
    os << "\n";
  }

private:
};

class mushr_ub_u_xdot_t : public prx::fg::noise_model_4factor_t<mushr_types::Ubar::type, mushr_types::Control::type,
                                                                mushr_types::Ubar::type, double>
{
  using Base = prx::fg::noise_model_4factor_t<mushr_types::Ubar::type, mushr_types::Control::type,
                                              mushr_types::Ubar::type, double>;

public:
  using Xdot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;
  using Params = mushr_types::Ubar::params;
  using U = mushr_types::Control::type;

  mushr_ub_u_xdot_t(gtsam::Key ubar1, gtsam::Key u, gtsam::Key ubar0, gtsam::Key t01, const Params params,
                    const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(ubar1, u, ubar0, t01, cost_model, 0.01), _params{ params }
  {
  }

  inline static Ubar dynamics(const U& u, const Ubar& ubar, const Params& params, const double& dt)
  {
    return mushr_ub_u_xdot_param_t::dynamics(u, ubar, params, dt);
  }

  virtual Ubar predict(const U& u, const Ubar& ubar, const double& dt) const override
  {
    return dynamics(u, ubar, _params, dt);
  }

  virtual Ubar compute_error(const Ubar& ubar1, const U& u, const Ubar& ubar0, const double& dt) const override
  {
    return predict(u, ubar0, dt) - ubar1;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Ubar ubar1{ values.at<Ubar>(key<1>()) };
    const U u{ values.at<U>(key<2>()) };
    const Ubar Ubar0{ values.at<Ubar>(key<3>()) };
    const double dt{ values.at<double>(key<4>()) };

    os << ubar1.transpose() << " ";
    os << u.transpose() << " ";
    os << Ubar0.transpose() << " ";
    os << _params.transpose() << " ";
    os << dt << " ";
    os << compute_error(ubar1, u, Ubar0, dt).transpose() << " ";
    os << "\n";
  }

private:
  const Params _params;
};

class mushr_observation_factor_t : public gtsam::NoiseModelFactorN<mushr_types::State::type, mushr_types::Ubar::type>
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using Control = mushr_types::Control::type;
  using Ubar = mushr_types::Ubar::type;
  using Params = mushr_types::Ubar::params;
  using Observation = mushr_types::State::type;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };
  static constexpr Eigen::Index DimUbar{ gtsam::traits<Ubar>::dimension };

  using Base = gtsam::NoiseModelFactorN<State, Ubar>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using PartialUbar = std::function<Ubar(const Ubar&)>;
  using PartialXdot = std::function<StateDot(const Ubar&)>;
  using FirstOrderDerivativeUbar = prx::math::first_order_derivative_t<PartialUbar, Ubar, 4>;
  using FirstOrderDerivativeXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 4>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  mushr_observation_factor_t() = delete;
  mushr_observation_factor_t(const mushr_observation_factor_t& other) = delete;

public:
  mushr_observation_factor_t(const gtsam::Key key_x, const gtsam::Key key_ubar, const Observation zx, const Control zu,
                             const double dt, const Params& params_ubar, const NoiseModel& cost_model,  // no-lint
                             const std::string label = "LieOdeIntegration", const double h = 0.01)
    : Base(cost_model, key_x, key_ubar)
    , _dt(dt)
    , _zx(zx)
    , _zu(zu)
    , _params_ubar(params_ubar)
    , _label(label)
    , _partial_ubar([&](const Ubar& ubar) { return mushr_ub_u_xdot_param_t::dynamics(_zu, ubar, _params_ubar, _dt); })
    , _partial_xdot_ubar([](const Ubar& ubar) { return mushr_xdot_ub_t::dynamics(ubar); })
    , _derivative_ubar(_partial_ubar, h)
    , _derivative_xdot_ubar(_partial_xdot_ubar, h)
  {
  }

  ~mushr_observation_factor_t() override
  {
  }

  virtual Eigen::VectorXd evaluateError(const State& x0, const Ubar& ubar0,  // no-lint
                                        OptDeriv Hx = boost::none, OptDeriv Hubar = boost::none) const override
  {
    // DEBUG_VARS(x0);
    err_H_xde = Hubar;
    // xde_H_ube = Hubar ? ;
    const Ubar ubar_eps{ _partial_ubar(ubar0) };
    // const StateDot xdot_eps{ _partial_xdot_ubar(ubar_eps) };
    const StateDot xdot_eps{ mushr_xdot_ub_t::dynamics(ubar_eps, Hubar ? &xde_H_ube : nullptr) };
    // DEBUG_VARS(xde_H_ube);
    // const State x_eps{ mushr_x_xdot_t::predict(x0, xdot_eps, dt) };

    // Eigen::Matrix<double, DimX, DimXdot> err_H_xde;  // Deriv error wrt between
    const Eigen::VectorXd error{ mushr_x_xdot_t::error(_zx, x0, xdot_eps, _dt,  // no-lint
                                                       boost::none,             // no-lint
                                                       Hx,                      // no-lint
                                                       err_H_xde) };

    if (Hubar)
    {
      // DEBUG_VARS(*err_H_xde);
      ube_H_ub0 = _derivative_ubar(ubar0);  // Deriv ubar_eps wrt ubar0
      // auto deriv = _derivative_xdot_ubar(ubar_eps);  // Deriv xdot_eps wrt
      // xde_H_ube = _derivative_xdot_ubar(ubar_eps);  // Deriv xdot_eps wrt
      // DEBUG_VARS(xde_H_ube);
      // DEBUG_VARS(ube_H_ub0);
      *Hubar = (*err_H_xde) * xde_H_ube * ube_H_ub0;
    }
    return error;
  }

private:
  const double _dt;
  const Observation _zx;
  const Control _zu;
  const Params _params_ubar;

  const PartialUbar _partial_ubar;
  const PartialXdot _partial_xdot_ubar;

  const FirstOrderDerivativeUbar _derivative_ubar;
  const FirstOrderDerivativeXdot _derivative_xdot_ubar;

  const std::string _label;

  mutable OptDeriv err_H_xde;  // Deriv error wrt between
  mutable Eigen::Matrix<double, DimUbar, DimUbar> ube_H_ub0;
  mutable Eigen::Matrix<double, DimXdot, DimUbar> xde_H_ube;
  // const DerivativeX _negative_identity;
};

class mushr_u_xddot01_t : public gtsam::NoiseModelFactorN<mushr_types::StateDot::type, mushr_types::StateDot::type,
                                                          double, mushr_types::Control::type>
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using Control = mushr_types::Control::type;
  using Ubar = mushr_types::Ubar::type;
  using Params = mushr_types::Ubar::params;
  using Observation = mushr_types::State::type;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };
  static constexpr Eigen::Index DimUbar{ gtsam::traits<Ubar>::dimension };

  using Base = gtsam::NoiseModelFactorN<StateDot, StateDot, double, Control>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Error = Eigen::VectorXd;

  using PartialUbar = std::function<Ubar(const Ubar&)>;
  using PartialXdot = std::function<StateDot(const Ubar&)>;
  using FirstOrderDerivativeUbar = prx::math::first_order_derivative_t<PartialUbar, Ubar, 4>;
  using FirstOrderDerivativeXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 4>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  mushr_u_xddot01_t() = delete;
  mushr_u_xddot01_t(const mushr_u_xddot01_t& other) = delete;

public:
  mushr_u_xddot01_t(const gtsam::Key key_xd1, const gtsam::Key key_xd0, const gtsam::Key key_dt, const gtsam::Key key_u,
                    const Params& params, const NoiseModel& cost_model)
    : Base(cost_model, key_xd1, key_xd0, key_dt, key_u), _params(params)
  {
  }

  ~mushr_u_xddot01_t() override
  {
  }

  static StateDot desired_xdot(const Control& u, gtsam::OptionalJacobian<3, 2> Hu = boost::none)
  {
    const double& Vin{ u[mushr_types::Control::vel_desired] };
    const double& delta{ u[mushr_types::Control::steering] };
    const double& L{ mushr_types::Parameters::L };

    Eigen::Matrix<double, 1, 1> b_H_delta;
    Eigen::Matrix3d vd_H_u, xdotd_H_tbeta, xdotd_H_vd;
    // Eigen::Matrix3d xdotd_H_tbeta, xdotD_H_vd;

    const double beta{ mushr_types::Control::beta(delta, Hu ? &b_H_delta : nullptr) };

    const double omega{ 2.0 * std::sin(beta) / L };
    const StateDot Vd{ Vin * StateDot(1.0, 0.0, omega) };

    const State T_beta{ 0.0, 0.0, beta };
    const StateDot xdot_d{ T_beta.adjoint(Vd, Hu ? &xdotd_H_tbeta : nullptr, Hu ? &xdotd_H_vd : nullptr) };

    if (Hu)
    {
      // PRX_DBG_VARS(b_H_delta);
      // PRX_DBG_VARS(xdotd_H_tbeta);
      // PRX_DBG_VARS(xdotd_H_vd);
      Eigen::Vector3d TBeta_H_beta{ 0, 0, 1 };
      Eigen::Vector3d vd_H_b, vd_H_vin;
      vd_H_b << 0, 0, (2.0 * Vin * std::cos(beta)) / L;
      vd_H_vin << 1, 0, (2.0 * std::sin(beta)) / L;
      // PRX_DBG_VARS(vd_H_b);
      // PRX_DBG_VARS(vd_H_vin);

      (*Hu).col(mushr_types::Control::vel_desired) = xdotd_H_vd * vd_H_vin;
      (*Hu).col(mushr_types::Control::steering) =
          xdotd_H_tbeta * TBeta_H_beta * b_H_delta + xdotd_H_vd * vd_H_b * b_H_delta;
    }

    return xdot_d;
  }

  static StateDot velocity_delta(const StateDot& xd0, const double& dt, const StateDot& xdot_d,
                                 const double& K,  // no-lint
                                 gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,
                                 gtsam::OptionalJacobian<3, 1> Hdt = boost::none,
                                 gtsam::OptionalJacobian<3, 3> Hxdotd = boost::none,
                                 gtsam::OptionalJacobian<3, 1> HK = boost::none)
  {
    const StateDot xddot{ K * (xdot_d - xd0) };
    // const StateDot xddot{ K * (xdot_d - xd0) / dt};

    if (Hxd0)
    {
      *Hxd0 = -(K)*Eigen::Matrix3d::Identity();
    }
    if (Hdt)
    {
      *Hdt = -(0.0) * (xdot_d - xd0);
      // *Hdt = -(K / std::pow(dt, 2)) * (xdot_d - xd0);
    }
    if (Hxdotd)
    {
      *Hxdotd = (K)*Eigen::Matrix3d::Identity();
    }
    if (HK)
    {
      *HK = (1.0) * (xdot_d - xd0);
    }
    return xddot;
  }

  template <typename Matrix>
  static boost::optional<Eigen::MatrixXd&> check_opt_H(const bool check, Matrix& matrix)
  {
    if (check)
      return matrix;
    return boost::none;
  }

  // Vb= Ad(0,0,beta)*[xr/dt;0;th1/dt]*dt;
  // T(x,y,th)*Exp(Vb(1),Vb(2),Vb(3))
  static StateDot predict(const StateDot& xd0, const double& dt, const Control& u, const Params& params,
                          gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,
                          gtsam::OptionalJacobian<3, 1> Hdt = boost::none,
                          gtsam::OptionalJacobian<3, 2> Hu = boost::none,
                          gtsam::OptionalJacobian<3, 3> HParams = boost::none)
  {
    using StateDDot = Eigen::Vector3d;
    using Integration = prx::fg::euler_integration_factor_t<StateDot, StateDDot, double>;

    Eigen::Matrix3d xdotd_H_tbeta, xdotd_H_vdesired;
    Eigen::MatrixXd xdot1_H_xddot, xdot1_H_dt, xdot1_H_xd0;
    Eigen::Matrix<double, 3, 2> xdotD_H_u;
    Eigen::Matrix<double, 3, 3> xddot_H_xd0, xddot_H_xdotD;
    Eigen::Matrix<double, 3, 1> xddot_H_dt, xddot_H_K;
    Eigen::Matrix<double, 3, 3> xddot_H_xdotd;
    // boost::optional<Eigen::MatrixXd&> xdot1_H_xddot_opt{ (Hxd0 or Hdt or Hu) ? xdot1_H_xddot : boost::none };

    const double& K{ params[mushr_types::Ubar::accel_slope] };
    const double& steering_param{ params[mushr_types::Ubar::steering_param] };

    // const double& v_in{ u[mushr_types::Control::vel_desired] };
    // const double& delta{ u[mushr_types::Control::steering] };
    // const double beta{ mushr_types::Control::beta(delta * steering_param) };
    // const double omega{ 2.0 * v_in * std::sin(beta) / mushr_types::Parameters::L };
    // const State T_beta{ 0.0, 0.0, beta };
    // const Eigen::Vector3d v_desired{ v_in, 0.0, omega };
    const StateDot xdot_d{ desired_xdot(u, Hu ? &xdotD_H_u : nullptr) };

    const StateDDot xddot{ velocity_delta(xd0, dt, xdot_d, K,             // no-lint
                                          Hxd0 ? &xddot_H_xd0 : nullptr,  // no-lint
                                          Hdt ? &xddot_H_dt : nullptr,    // no-lint
                                          Hu ? &xddot_H_xdotD : nullptr,  // no-lint
                                          HParams ? &xddot_H_K : nullptr) };
    const StateDot xdot_t1{ Integration::predict(xd0, xddot, dt,                                   // no-lint
                                                 check_opt_H((Hxd0 or Hdt or Hu), xdot1_H_xd0),    // no-lint
                                                 check_opt_H((Hxd0 or Hdt or Hu), xdot1_H_xddot),  // no-lint
                                                 check_opt_H((Hxd0 or Hdt or Hu), xdot1_H_dt)) };

    if (Hxd0)
    {
      // PRX_DBG_VARS(xddot_H_xd0);
      // PRX_DBG_VARS(xdot1_H_xd0);
      *Hxd0 = xdot1_H_xddot * xddot_H_xd0 + xdot1_H_xd0;
    }
    if (Hdt)
    {
      *Hdt = xdot1_H_xddot * xddot_H_dt + xdot1_H_dt;
    }
    if (Hu)
    {
      *Hu = xdot1_H_xddot * xddot_H_xdotD * xdotD_H_u;
    }
    if (HParams)
    {
      (*HParams) = Eigen::Matrix3d::Zero();  // To be fixed
      (*HParams).col(mushr_types::Ubar::accel_slope) = xdot1_H_xddot * xddot_H_K;
      // *HParams.col(mushr_types::Ubar::steering) *= delta;
    }
    return xdot_t1;
  }

  virtual Error evaluateError(const StateDot& xd1, const StateDot& xd0, const double& dt,
                              const Control& u,  // no-lint
                              OptDeriv Hxd1 = boost::none, OptDeriv Hxd0 = boost::none, OptDeriv Hdt = boost::none,
                              OptDeriv Hu = boost::none) const override
  {
    const StateDot xdp1{ predict(xd0, dt, u, _params, Hxd0, Hdt, Hu) };
    if (Hxd1)
    {
      *Hxd1 = -Eigen::Matrix<double, 3, 3>::Identity();
    }
    return xdp1 - xd1;
  }

private:
  const Params _params;
};

template <typename... Types>
class mushr_CtrlAccel_t : public gtsam::NoiseModelFactorN<mushr_types::StateDot::type, mushr_types::StateDot::type,
                                                          mushr_types::Control::type, Types...>
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using StateDotDot = mushr_types::StateDot::type;

  using Params = mushr_types::Control::params;
  using Control = mushr_types::Control::type;
  using Polynomial = mushr_types::Control::Poly;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };

  static constexpr Eigen::Index DimParams{ mushr_types::Control::ParamsDim };

  using Base = gtsam::NoiseModelFactorN<StateDot, StateDot, Control, Types...>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;
  static constexpr std::size_t NumTypes{ sizeof...(Types) };

  mushr_CtrlAccel_t() = delete;
  mushr_CtrlAccel_t(const mushr_CtrlAccel_t& other) = delete;

public:
  template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>
  mushr_CtrlAccel_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const gtsam::Key dt,
                    const NoiseModel& cost_model, const Params params, const Polynomial& steering_poly)
    : Base(cost_model, xd1, xd0, u, dt), _params(params), _steering_poly(steering_poly), _dt(-1)
  {
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(0 == Num), bool> = true>
  mushr_CtrlAccel_t(const gtsam::Key xd1, const gtsam::Key xd0, const gtsam::Key u, const double& dt,
                    const NoiseModel& cost_model, const Params params, const Polynomial& steering_poly)
    : Base(cost_model, xd1, xd0, u), _params(params), _steering_poly(steering_poly), _dt(dt)
  {
  }

  ~mushr_CtrlAccel_t() override
  {
  }

  template <typename Matrix>
  static boost::optional<Eigen::MatrixXd&> check_opt_H(const bool check, Matrix& matrix)
  {
    if (check)
      return matrix;
    return boost::none;
  }

  // Vb= Ad(0,0,beta)*[xr/dt;0;th1/dt]*dt;
  // T(x,y,th)*Exp(Vb(1),Vb(2),Vb(3))
  static StateDot predict(const StateDot xd0, const Control u, const double dt,  // no-lint
                          const Params& params, const Polynomial steering_poly,  // no-lint
                          gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,
                          gtsam::OptionalJacobian<3, 2> Hu = boost::none,
                          gtsam::OptionalJacobian<3, 1> Hdt = boost::none,
                          gtsam::OptionalJacobian<3, DimParams> Hparams = boost::none)
  {
    // PRINT_MSG("---------------------");
    using StateDDot = Eigen::Vector3d;
    using Integration = prx::fg::euler_integration_factor_t<StateDot, StateDDot, double>;

    Eigen::MatrixXd xd1Z_H_qd0, xd1Zero_H_qdd, xd1Zero_H_dt;
    Eigen::Matrix<double, 3, 3> xdd_H_Tb, xdd_H_stateDD;
    Eigen::Matrix<double, 3, 3> qd0_H_Tbpinv, qd0_H_xd0;
    Eigen::Matrix<double, 3, 3> Tpbinv_H_Tbprev;
    Eigen::Matrix<double, 1, 1> beta_H_delta;
    Eigen::Matrix<double, 3, 3> xd1Adj_H_xd1Z, xd1Adj_H_Tbeta;
    Eigen::Matrix<double, 1, 1> delta_H_deltaIn;

    const double& L{ mushr_types::Parameters::L };
    const double& mass{ mushr_types::Parameters::mass };
    const double& param_AccIn{ params[mushr_types::Control::vel_desired] };
    const double& friction{ params[mushr_types::Control::friction] };

    const double& deltaIn{ u[mushr_types::Control::steering] };
    const double delta{ mushr_types::Control::evaluate_polynomial(steering_poly, deltaIn, delta_H_deltaIn) };

    const double Uaccel{ u[mushr_types::Control::vel_desired] };
    const double AccIn{ Uaccel * param_AccIn };

    // DEBUG_VARS(deltaIn, delta);
    // DEBUG_VARS(Uaccel, AccIn);
    const double Vprev_pos{ xd0.head(2).norm() };
    const double Vprev{ std::copysign(Vprev_pos, Uaccel) };
    const Eigen::RowVector3d Vprev_H_xd0{ Vprev_pos < 1e-8 ? Eigen::RowVector3d::Zero() :
                                                             Eigen::RowVector3d(xd0[0] / Vprev, xd0[1] / Vprev, 0.0) };

    Eigen::Matrix<double, 1, 1> accIn_H_paramAccIn{ Uaccel };
    Eigen::Matrix<double, 1, 1> delta_H_paramDelta{ u[mushr_types::Control::steering] };
    Eigen::Matrix<double, 1, 1> accIn_H_UaccIn{ param_AccIn };
    // Eigen::Matrix<double, 1, 1> delta_H_Udelta{ param_delta };

    const double beta{ mushr_types::Control::beta(delta, Hu ? &beta_H_delta : nullptr) };
    const double beta_prev{ std::atan2(xd0[1], xd0[0]) };
    const double norm2{ xd0.head(2).squaredNorm() };
    const Eigen::RowVector3d bprev_H_xd0{ norm2 < 1e-6 ? Eigen::RowVector3d::Zero() :
                                                         Eigen::RowVector3d(-xd0[1] / norm2, xd0[0] / norm2, 0.0) };
    // [-y/(x^2 + y^2), x/(x^2 + y^2)]

    const double omega{ 2.0 * std::sin(beta) / L };
    const double omega_prev{ 2.0 * std::sin(beta_prev) / L };
    const Eigen::Matrix<double, 1, 1> omega_H_beta{ 2.0 * std::cos(beta) / L };
    const Eigen::Matrix<double, 1, 1> omegaPrev_H_bPrev{ 2.0 * std::cos(beta_prev) / L };

    // DEBUG_VARS(omega, omega_prev);
    // DEBUG_VARS(beta, beta_prev);

    const State T_beta{ 0.0, 0.0, beta };
    const State T_beta_prev{ 0.0, 0.0, beta_prev };
    const Eigen::Vector3d Tb_H_beta{ 0, 0, 1 };
    const Eigen::Vector3d Tbprev_H_bprev{ 0, 0, 1 };

    const State Tbpinv{ T_beta_prev.inverse(Tpbinv_H_Tbprev) };
    const double qd0_sign{ std::copysign(1.0, Uaccel) };
    const StateDot qd0_adj{ Tbpinv.adjoint(xd0, Hu ? &qd0_H_Tbpinv : nullptr, Hxd0 ? &qd0_H_xd0 : nullptr) };
    const StateDot qd0{ qd0_sign * qd0_adj };
    const StateDotDot qdd{ AccIn, 0.0, 0.0 };
    const StateDot xd1_zero{ Integration::integrate(qd0, qdd, dt, xd1Z_H_qd0, xd1Zero_H_qdd, xd1Zero_H_dt) };
    const Eigen::Matrix<double, 3, 1> qdd_H_AccIn{ 1.0, 0.0, 0.0 };
    // DEBUG_VARS(xd0.transpose())
    // DEBUG_VARS(Tbpinv.matrix())
    // DEBUG_VARS(qd0_adj.transpose())
    // PRX_DBG_VARS(qd0.transpose(), xd1_zero.transpose())

    const double Vcurr_pos{ xd1_zero.head(2).norm() };
    const double Vcurr{ std::copysign(Vcurr_pos, Uaccel) };
    // DEBUG_VARS(T_beta_prev);
    // DEBUG_VARS(Tbpinv);
    // DEBUG_VARS(xd0.transpose());
    // DEBUG_VARS(qd0.transpose());
    // DEBUG_VARS(qdd.transpose());
    // DEBUG_VARS(xd1_zero.transpose());
    // DEBUG_VARS(Vprev, Uaccel, AccIn, Vcurr);
    const Eigen::RowVector3d VCurr_H_xd1Zero{ std::fabs(Vcurr) < 1e-8 ?
                                                  Eigen::RowVector3d::Zero() :
                                                  Eigen::RowVector3d(xd1_zero[0] / Vcurr, xd1_zero[1] / Vcurr, 0.0) };

    const double thd_prev{ omega_prev * Vprev };
    const double thd_curr{ omega * Vcurr };
    const StateDot w_new{ 0, 0, (thd_curr - thd_prev) * friction };
    const StateDot xd1Adj{ T_beta.adjoint(xd1_zero, xd1Adj_H_Tbeta, xd1Adj_H_xd1Z) };
    const StateDot xd1{ xd1Adj + w_new };
    // DEBUG_VARS(T_beta.matrix());
    // DEBUG_VARS(xd1_zero.transpose());
    // DEBUG_VARS(xd1Adj.transpose());
    // DEBUG_VARS(w_new.transpose());

    const double thdPrev_H_omegaPrev{ Vprev };
    const double thdPrev_H_Vprev{ omega_prev };

    const double thdCurr_H_omega{ Vcurr };
    const double thdCurr_H_Vcurr{ omega };

    const Eigen::Matrix<double, 3, 1> wNew_H_friction{ 0.0, 0.0, (thd_curr - thd_prev) };
    const Eigen::Matrix<double, 3, 1> wNew_H_thdCurr{ 0.0, 0.0, friction };
    const Eigen::Matrix<double, 3, 1> wNew_H_thdPrev{ 0.0, 0.0, -friction };
    const Eigen::Matrix3d xd1_H_xd1Adj{ Eigen::Matrix3d::Identity() };
    const Eigen::Matrix3d xd1_H_wNew{ Eigen::Matrix3d::Identity() };
    if (Hxd0)
    {
      const Eigen::Matrix3d xd1Zero_H_xd0{
        qd0_sign * xd1Z_H_qd0 * (qd0_H_xd0 + qd0_H_Tbpinv * Tpbinv_H_Tbprev * Tbprev_H_bprev * bprev_H_xd0)
      };
      *Hxd0 =                            // no-lint
          (xd1_H_xd1Adj * xd1Adj_H_xd1Z  // no-lint
           + xd1_H_wNew * wNew_H_thdCurr * thdCurr_H_Vcurr * VCurr_H_xd1Zero) *
              xd1Zero_H_xd0                // no-lint
          + xd1_H_wNew * wNew_H_thdPrev *  // no-lint
                (thdPrev_H_Vprev * Vprev_H_xd0 + thdPrev_H_omegaPrev * omegaPrev_H_bPrev * bprev_H_xd0);
      // PRX_DBG_VARS(*Hxd0);
    }
    if (Hdt)
    {
      *Hdt = xd1_H_xd1Adj * xd1Adj_H_xd1Z * xd1Zero_H_dt  // no-lint
             + xd1_H_wNew * wNew_H_thdCurr * thdCurr_H_Vcurr * VCurr_H_xd1Zero * xd1Zero_H_dt;
      // *Hdt += Eigen::Matrix<double, 3, 1>(0.01, 0.01, 0.01);
    }
    if (Hu)
    {
      const Eigen::Matrix<double, 3, 1> xd1Z_H_acc{ xd1Zero_H_qdd * qdd_H_AccIn * accIn_H_UaccIn };

      (*Hu).col(mushr_types::Control::vel_desired) =  // no-lint
          (xd1_H_xd1Adj * xd1Adj_H_xd1Z               // no-lint
           + xd1_H_wNew * wNew_H_thdCurr * thdCurr_H_Vcurr * VCurr_H_xd1Zero) *
          xd1Z_H_acc;
      (*Hu).col(mushr_types::Control::steering) =     // no-lint
          (xd1_H_xd1Adj * xd1Adj_H_Tbeta * Tb_H_beta  // no-lint
           + xd1_H_wNew * wNew_H_thdCurr * thdCurr_H_omega * omega_H_beta) *
          beta_H_delta * delta_H_deltaIn;  // no-lint
                                           // *Hu += Eigen::Matrix<double, 3, 2>::Identity() * 0.01;
      // DEBUG_VARS(xd1_H_xd1Adj)
      // DEBUG_VARS(xd1Adj_H_Tbeta)
      // DEBUG_VARS(Tb_H_beta)
      // DEBUG_VARS(xd1_H_wNew)
      // DEBUG_VARS(wNew_H_thdCurr)
      // DEBUG_VARS(thdCurr_H_omega)
      // DEBUG_VARS(omega_H_beta)
      // DEBUG_VARS(beta_H_delta)
      // DEBUG_VARS(delta_H_deltaIn)
    }
    if (Hparams)
    {
      (*Hparams).col(mushr_types::Control::vel_desired) =
          xd1_H_xd1Adj * xd1Adj_H_xd1Z * xd1Zero_H_qdd * qdd_H_AccIn * accIn_H_paramAccIn  // no-lint
          + xd1_H_wNew * wNew_H_thdCurr * thdCurr_H_Vcurr * VCurr_H_xd1Zero * xd1Zero_H_qdd * qdd_H_AccIn *
                accIn_H_paramAccIn;
      (*Hparams).col(mushr_types::Control::steering) = Eigen::Vector3d::Zero();
      (*Hparams).col(mushr_types::Control::friction) = wNew_H_friction;  // xd1_H_xddF * xddF_H_F;
    }
    // DEBUG_VARS(xd1.transpose());

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
      xdp1 = predict(xd0, u, _dt, _params, _steering_poly, Hxd0, Hu);
      // return error(x1, x0, xdot, _h, H1, H0, Hdot);
    }
    else
    {
      xdp1 = predict(xd0, u, dt01..., _params, _steering_poly, Hxd0, Hu, H...);
      // return error(x1, x0, xdot, xd..., H1, H0, Hdot, H...);
    }

    if (Hxd1)
    {
      *Hxd1 = -Eigen::Matrix<double, 3, 3>::Identity();
    }
    return xdp1 - xd1;
  }

private:
  const Polynomial _steering_poly;
  const Params _params;

  const double _dt;
};

// Non-holonomic constraints
class mushr_NHC_t : public gtsam::NoiseModelFactorN<mushr_types::StateDot::type, mushr_types::Control::type>
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using StateDotDot = mushr_types::StateDot::type;

  using Params = mushr_types::Control::params;
  using Control = mushr_types::Control::type;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };

  using Base = gtsam::NoiseModelFactorN<StateDot, Control>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  mushr_NHC_t() = delete;
  mushr_NHC_t(const mushr_NHC_t& other) = delete;

public:
  mushr_NHC_t(const gtsam::Key xd, const gtsam::Key u, const NoiseModel& cost_model, const Params params)
    : Base(cost_model, xd, u), _params(params)
  {
  }

  ~mushr_NHC_t() override
  {
  }

  template <typename Matrix>
  static boost::optional<Eigen::MatrixXd&> check_opt_H(const bool check, Matrix& matrix)
  {
    if (check)
      return matrix;
    return boost::none;
  }

  static double velocity_constraint(const StateDot xd, const double& beta,  // no-lint
                                    gtsam::OptionalJacobian<1, 3> Hxd = boost::none,
                                    gtsam::OptionalJacobian<1, 1> Hbeta = boost::none)
  {
    const double& xdot{ xd[0] };
    const double& ydot{ xd[1] };
    const double tan_beta{ std::tan(beta) };
    const double error{ ydot / xdot - tan_beta };
    if (Hxd)
    {
      //[-ydot/xdot^2, 1/xdot, 0];
      (*Hxd)(0, 0) = -ydot / (std::pow(xdot, 2));
      (*Hxd)(0, 1) = 1.0 / xdot;
      (*Hxd)(0, 2) = 0.0;
    }
    if (Hbeta)
    {
      (*Hbeta)(0, 0) = -std::pow(tan_beta, 2) - 1;
    }

    return error;
  }

  static double omega_constraint(const StateDot xd, const double& beta,  // no-lint
                                 gtsam::OptionalJacobian<1, 3> Hxd = boost::none,
                                 gtsam::OptionalJacobian<1, 1> Hbeta = boost::none)
  {
    const double& L{ mushr_types::Parameters::L };

    const double& xdot{ xd[0] };
    const double& ydot{ xd[1] };
    const double& thetadot{ xd[2] };
    const double sBeta{ std::sin(beta) };
    const double cBeta{ std::cos(beta) };
    const double error{ 2.0 * (xdot * cBeta + ydot * sBeta) * sBeta / L - thetadot };

    if (Hxd)  // [(2*cos(beta)*sin(beta))/L, (2*sin(beta)^2)/L, -1]
    {
      (*Hxd)(0, 0) = (2.0 * cBeta * sBeta) / L;
      (*Hxd)(0, 1) = (2.0 * std::pow(sBeta, 2)) / L;
      (*Hxd)(0, 2) = -1.0;
    }
    if (Hbeta)
    {
      (*Hbeta)(0, 0) = (cBeta * (2.0 * xdot * cBeta + 2.0 * ydot * sBeta)) / L +
                       (sBeta * (2.0 * ydot * cBeta - 2.0 * xdot * sBeta)) / L;
    }

    return error;
  }

  virtual Error evaluateError(const StateDot& xd, const Control& u,  // no-lint
                              OptDeriv Hxd = boost::none, OptDeriv Hu = boost::none) const override
  {
    const double& param_delta{ _params[mushr_types::Control::steering] };
    const double delta{ u[mushr_types::Control::steering] * param_delta };
    const Eigen::Matrix<double, 1, 1> delta_H_Udelta{ param_delta };
    Eigen::Matrix<double, 1, 1> beta_H_delta, Vc_H_beta, Wc_H_beta;
    Eigen::Matrix<double, 1, 3> Vc_H_xd, Wc_H_xd;

    const double beta{ mushr_types::Control::beta(delta, Hu ? &beta_H_delta : nullptr) };

    const double Vc{ velocity_constraint(xd, beta, Vc_H_xd, Vc_H_beta) };
    const double Wc{ omega_constraint(xd, beta, Wc_H_xd, Wc_H_beta) };

    const Error error{ Eigen::Vector2d(Vc, Wc) };

    if (Hxd)  // (2x3)
    {
      *Hxd = Eigen::Matrix<double, 2, 3>::Zero();
      *Hxd << Vc_H_xd, Wc_H_xd;
    }
    if (Hu)  // (2x2)
    {
      *Hu = Eigen::Matrix<double, 2, 2>::Zero();
      (*Hu).col(mushr_types::Control::steering) << Vc_H_beta * beta_H_delta * delta_H_Udelta,
          Wc_H_beta * beta_H_delta * delta_H_Udelta;
    }

    return error;
  }

private:
  const Params _params;
};

class mushr_params_sysid_t : public gtsam::NoiseModelFactorN<mushr_types::Control::params>
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using StateDotDot = mushr_types::StateDot::type;

  using Params = mushr_types::Control::params;
  using Control = mushr_types::Control::type;
  using Polynomial = mushr_types::Control::Poly;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };

  static constexpr Eigen::Index DimParams{ mushr_types::Control::ParamsDim };

  using Base = gtsam::NoiseModelFactorN<Params>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  using MushrCtrlAccel = mushr_CtrlAccel_t<>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  mushr_params_sysid_t() = delete;
  mushr_params_sysid_t(const mushr_params_sysid_t& other) = delete;

public:
  mushr_params_sysid_t(const gtsam::Key key_param,                                                // no-lint
                       const StateDot xd1, const StateDot xd0, const Control u, const double dt,  // no-lint
                       const Polynomial& steering_poly, const NoiseModel& cost_model)
    : Base(cost_model, key_param), _xd1(xd1), _xd0(xd0), _u(u), _dt(dt), _steering_poly(steering_poly)
  {
  }

  ~mushr_params_sysid_t() override
  {
  }

  // Vb= Ad(0,0,beta)*[xr/dt;0;th1/dt]*dt;
  // T(x,y,th)*Exp(Vb(1),Vb(2),Vb(3))

  virtual Eigen::VectorXd evaluateError(const Params& params, OptDeriv Hparams = boost::none) const override
  {
    const StateDot xdp1{ MushrCtrlAccel::predict(_xd0, _u, _dt, params, _steering_poly, boost::none, boost::none,
                                                 boost::none, Hparams) };

    return xdp1 - _xd1;
  }

private:
  const StateDot _xd0, _xd1;
  const Control _u;
  const Polynomial _steering_poly;
  const Params _params;

  const double _dt;
};

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

  StateDot predict(const StateDot& xd0, const Control& u, const double& dt,  // no-lint
                   gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,         // no-lint
                   gtsam::OptionalJacobian<3, 2> Hu = boost::none,           // no-lint
                   gtsam::OptionalJacobian<3, 1> Hdt = boost::none) const
  {
    init_mj_state(xd0, u, dt);

    if (Hxd0)
    {
      // _qacc_vector = _qacc_warmstart_init;
      // _qpos_vector = _qpos_init;
      // _qvel_vector = _qvel_init;
      *Hxd0 = _partial_xdot0(xd0, u, dt);
    }
    if (Hu)
    {
      // _qacc_vector = _qacc_warmstart_init;
      // _qpos_vector = _qpos_init;
      // _qvel_vector = _qvel_init;
      *Hu = _partial_control(u, dt, xd0);
    }
    if (Hdt)
    {
      // _qacc_vector = _qacc_warmstart_init;
      // _qpos_vector = _qpos_init;
      // _qvel_vector = _qvel_init;
      *Hdt = _partial_dt(dt, xd0, u);
    }

    const StateDot xd1{ mj_predict(xd0, u, dt) };
    // _qvel_init = _qvel_vector;

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

  StateDot mj_predict(const StateDot& xd0, const Control& u, const double& dt) const
  {
    _qacc_vector = _qacc_warmstart_init;
    _qpos_vector = _qpos_init;
    _qvel_vector = _qvel_init;

    _ctrl_vector = u;
    _qvel_vector[0] = xd0[0];
    _qvel_vector[1] = xd0[1];
    _qvel_vector[5] = xd0[2];

    for (double ti = 0.0; ti < dt; ti += _mj_model->opt.timestep)
    {
      mj_step(_mj_model, _mj_data);
    }

    const StateDot xd1{ _qvel_vector[0], _qvel_vector[1], _qvel_vector[5] };

    // PRX_DBG_VARS(_qpos_vector.transpose())
    // PRX_DBG_VARS(_qvel_vector.transpose())
    // PRX_DBG_VARS(xd0.transpose())
    // PRX_DBG_VARS(u.transpose(), dt)
    // PRX_DBG_VARS(xd1.transpose())
    return xd1;
  }

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

  mutable bool _mj_state_init;
};
}  // namespace prx_models
