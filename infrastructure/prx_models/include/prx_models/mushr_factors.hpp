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

#include <prx/simulation/plant.hpp>
#include <prx/utilities/math/first_order_derivative.hpp>
#include <prx/factor_graphs/factors/noise_model_factors.hpp>
#include <prx/factor_graphs/lie_groups/se2.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>

#include <prx_models/mj_mushr.hpp>
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

}
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
using type = Eigen::Vector<double, 2>;
constexpr std::size_t vel_desired{ prx_models::mushr_t::control::velocity_idx };
constexpr std::size_t steering{ prx_models::mushr_t::control::steering_idx };

// constexpr std::size_t vel_desired{ prx_models::mushr_t::control::steering_idx };
// constexpr std::size_t steering{ prx_models::mushr_t::control::velocity_idx };

}  // namespace Control

}  // namespace mushr_types

using mushr_x_xdot_t = prx::fg::lie_integration_factor_t<mushr_types::State::type, mushr_types::StateDot::type, double>;
// template <typename X, typename Xdot, typename... Types>
// class mushr_x_xdot_t : public gtsam::NoiseModelFactorN<mushr_types::State::type, mushr_types::State::type,
//                                                        mushr_types::StateDot::type, mushr_types::Ubar::type, double>
// {
//   using X = mushr_types::State::type;
//   using Xdot = mushr_types::StateDot::type;
//   using Ubar = mushr_types::Ubar::type;

//   using Base = gtsam::NoiseModelFactorN<X, X, Xdot, Ubar, double>;
//   using LieIntegrator = prx::fg::lie_integrator_t<X, Xdot, double>;
//   using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
//   static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
//   static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };
//   static constexpr Eigen::Index DimUbar{ gtsam::traits<Ubar>::dimension };

//   // static constexpr std::size_t NumTypes{ sizeof...(Types) };

//   using DerivativeX = Eigen::Matrix<double, DimX, DimX>;
//   using OptDeriv = boost::optional<Eigen::MatrixXd&>;
//   template <typename T>
//   using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

//   using MatXX = Eigen::Matrix<double, DimX, DimX>;
//   using MatXXdot = Eigen::Matrix<double, DimX, DimXdot>;
//   using MatXdotXdot = Eigen::Matrix<double, DimXdot, DimXdot>;
//   using MatXdotDt = Eigen::Matrix<double, DimXdot, 1>;

//   mushr_x_xdot_t() = delete;
//   mushr_x_xdot_t(const mushr_x_xdot_t& other) = delete;

// public:
//   mushr_x_xdot_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
//                  const gtsam::Key key_ubar, const gtsam::Key key_dt, const NoiseModel& cost_model,
//                  const std::string label = "MushrLieOdeIntegration")
//     : Base(cost_model, key_xt1, key_xt0, key_xdot, key_ubar, key_dt), _h(0.0), _label(label)
//   {
//   }

//   ~mushr_x_xdot_t() override
//   {
//   }

//   // virtual X0 predict(const X1& x1, const X2& x2) const = 0;
//   static X predict(const X& x, const Xdot& xdot, const Ubar& ubar, const double dt,  // no-lint
//                    gtsam::OptionalJacobian<DimX, DimX> Hx = boost::none,             // no-lint
//                    gtsam::OptionalJacobian<DimX, DimXdot> Hxdot = boost::none,       // no-lint
//                    gtsam::OptionalJacobian<DimX, DimUbar> Hubar = boost::none,       // no-lint
//                    gtsam::OptionalJacobian<DimX, 1> Hdt = boost::none)
//   {
//     const double& beta{ ubar[mushr_types::Ubar::beta] };
//     const X x_beta(Eigen::Vector2d::Zero(), beta);
//     // const X xp{ x * x_beta };  // x' <= beta + theta
//     const X xp{ gtsam::traits<X>::Compose(x, x_beta) };
//     // DEBUG_VARS(x);
//     // DEBUG_VARS(beta, x_beta);
//     // DEBUG_VARS(xp);
//     // const X xp{ x };
//     // return gtsam::traits<prx::fg::SE2_t>::Compose(x, exmap);

//     const X result{ LieIntegrator::integrate(xp, xdot, dt, Hx, Hxdot, Hdt) };

//     return result;
//   }

//   Eigen::VectorXd error(const X& x1, const X& x0, const Xdot& xdot, const Ubar& ubar, const double& dt,
//                         boost::optional<Eigen::MatrixXd&> Hx1 = boost::none,
//                         boost::optional<Eigen::MatrixXd&> Hx0 = boost::none,
//                         boost::optional<Eigen::MatrixXd&> Hxdot = boost::none,
//                         boost::optional<Eigen::MatrixXd&> Hubar = boost::none,
//                         boost::optional<Eigen::MatrixXd&> Hdt = boost::none) const
//   {
//     Eigen::Matrix<double, DimX, DimX> err_H_b;       // Deriv error wrt between
//     Eigen::Matrix<double, DimX, DimX> b_H_q1;        // Deriv between wrt x1
//     Eigen::Matrix<double, DimX, DimX> b_H_qp;        // Deriv between wrt predicted
//     Eigen::Matrix<double, DimX, DimX> qp_H_q0;       // Deriv predicted wrt x0
//     Eigen::Matrix<double, DimX, DimXdot> qp_H_qdot;  // Deriv predicted wrt xdot
//     Eigen::Matrix<double, DimX, DimUbar> qp_H_ubar;  // Deriv predicted wrt dt
//     Eigen::Matrix<double, DimX, 1> qp_H_qdt;         // Deriv predicted wrt dt

//     const X prediction{ predict(x0, xdot, ubar, dt,            // no-lint
//                                 Hx0 ? &qp_H_q0 : nullptr,      // no-lint
//                                 Hxdot ? &qp_H_qdot : nullptr,  // no-lint
//                                 Hubar ? &qp_H_ubar : nullptr,  // no-lint
//                                 Hdt ? &qp_H_qdt : nullptr) };
//     // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
//     const X between{ x1.between(prediction,                                 // no-lint
//                                 (Hx0 or Hxdot or Hdt) ? &b_H_q1 : nullptr,  // no-lint
//                                 (Hx0 or Hxdot or Hdt) ? &b_H_qp : nullptr) };
//     const Eigen::VectorXd error{ X::Logmap(between, (Hx0 or Hxdot or Hdt) ? &err_H_b : nullptr) };

//     if (Hx1)
//     {
//       *Hx1 = err_H_b * b_H_q1;
//     }
//     if (Hx0)
//     {
//       *Hx0 = err_H_b * b_H_qp * qp_H_q0;
//     }
//     if (Hxdot)
//     {
//       *Hxdot = err_H_b * b_H_qp * qp_H_qdot;
//     }
//     if (Hdt)
//     {
//       *Hdt = err_H_b * b_H_qp * qp_H_qdt;
//     }

//     return error;
//   }

//   virtual Eigen::VectorXd evaluateError(const X& x1, const X& x0, const Xdot& xdot, const Ubar& ubar,
//                                         const double& dt,  // no-lint
//                                         OptDeriv H1 = boost::none, OptDeriv H0 = boost::none,
//                                         OptDeriv Hdot = boost::none, OptDeriv Hubar = boost::none,
//                                         OptDeriv Hdt = boost::none) const override
//   {
//     return error(x1, x0, xdot, ubar, dt, H1, H0, Hdot, Hubar, Hdt);
//   }

// private:
//   const double _h;
//   const std::string _label;
//   // const DerivativeX _negative_identity;
// };

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
      // (*Hubar) << -vt * sBeta, 0, cBeta,  // no-lint
      //     vt * cBeta, 0, sBeta,           // no-lint
      //     (vt * cBeta) / lr, -(vt * sBeta) / (lr * lr), sBeta / lr;
      // DEBUG_VARS(*Hubar);
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

// class mushr_ubar_u_observation_t
//   : public prx::fg::noise_model_2factor_t<mushr_types::Ubar::type, mushr_types::Control::type>
// {
//   using Base = prx::fg::noise_model_2factor_t<mushr_types::Ubar::type, mushr_types::Control::type>;

// public:
//   using Xdot = mushr_types::StateDot::type;
//   using Ubar = mushr_types::Ubar::type;
//   using Params = mushr_types::Ubar::params;
//   using U = mushr_types::Control::type;

//   mushr_ubar_u_observation_t(gtsam::Key ubar, gtsam::Key u, const Control zu, const double dt0, const double dt1,
//                              const Params params, const gtsam::noiseModel::Base::shared_ptr& cost_model)
//     : Base(ubar, u, cost_model, 0.01), _zu(zu), _params{ params }
//   {
//   }

//   virtual Ubar compute_error(const U& u, const Ubar& ubar) const override
//   {
//     const Ubar ubar_eps{ mushr_ub_u_xdot_param_t::dynamics(_zu, ubar, _params, dt) };
//     const Ubar ubar_eps{ mushr_ub_u_xdot_param_t::dynamics(_zu, ubar, _params, dt) };
//     return predict(u, ubar0, dt) - ubar1;
//   }

//   void eval_to_stream(gtsam::Values& values, std::ostream& os)
//   {
//     const Ubar ubar1{ values.at<Ubar>(key<1>()) };
//     const U u{ values.at<U>(key<2>()) };
//     const Ubar Ubar0{ values.at<Ubar>(key<3>()) };
//     const double dt{ values.at<double>(key<4>()) };

//     os << ubar1.transpose() << " ";
//     os << u.transpose() << " ";
//     os << Ubar0.transpose() << " ";
//     os << _params.transpose() << " ";
//     os << dt << " ";
//     os << compute_error(ubar1, u, Ubar0, dt).transpose() << " ";
//     os << "\n";
//   }

// private:
//   const Params _params;
// };
}  // namespace prx_models
