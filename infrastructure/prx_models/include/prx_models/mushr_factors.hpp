#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

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
constexpr double L{ 0.2965 };

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

}  // namespace Control

}  // namespace mushr_types

using mushr_x_xdot_t = prx::fg::lie_integration_factor_t<mushr_types::State::type, mushr_types::StateDot::type>;

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

    // const double dv{ v_desired - v_current };
    const double v_next{ v_current + dt * accel_slope };
    const double beta{ std::atan(0.5 * std::tan(steering * steering_param)) };

    Ubar ubar_next{};
    ubar_next[mushr_types::Ubar::beta] = beta;
    ubar_next[mushr_types::Ubar::velocity] = max_vel_param * v_next;

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

class mushr_xdot_ub_t : public prx::fg::noise_model_2factor_t<mushr_types::StateDot::type, mushr_types::Ubar::type>
{
  using Base = noise_model_2factor_t<mushr_types::StateDot::type, mushr_types::Ubar::type>;
  using Error = Base::Error;

public:
  using X = mushr_types::State::type;
  using Xdot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;

  mushr_xdot_ub_t(gtsam::Key xdot, gtsam::Key ubar, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(xdot, ubar, cost_model, 0.01)
  {
  }

  static Xdot dynamics(const Ubar& ubar)
  {
    const double& vt{ ubar[mushr_types::Ubar::velocity] };
    const double& beta{ ubar[mushr_types::Ubar::beta] };

    const double wt{ 2.0 * vt * std::sin(beta) / mushr_types::Parameters::L };

    // const Xdot t_x{ vt * cTh, vt * sTh, wt };
    // const Xdot t_x{ vt, 0, beta };
    const Xdot t_x{ vt, 0, wt };
    // return x.AdjointMap().inverse() * t_x;
    return t_x;
  }

  virtual Xdot predict(const Ubar& ubar) const override
  {
    return dynamics(ubar);
  }

  // virtual Error compute_error(const X0& x0, const X1& x1, const X2& x2) const
  virtual Error compute_error(const Xdot& xdot, const Ubar& ub) const override
  {
    return predict(ub) - xdot;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Xdot xdot{ values.at<Xdot>(key<1>()) };
    const Ubar ubar{ values.at<Ubar>(key<2>()) };

    os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xdot.transpose() << " ";  // 4, 5, 6
    os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << ubar.transpose() << " ";  // 7, 8
    os << "Error: " << compute_error(xdot, ubar).transpose() << " ";                         //, 5, 6
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

// class mushr_ub_ufix_xdot_param_t
//   : public prx::fg::noise_model_3factor_t<mushr_types::Ubar::type, mushr_types::Ubar::type,
//   mushr_types::Ubar::params>
// {
//   using Base =
//       prx::fg::noise_model_3factor_t<mushr_types::Ubar::type, mushr_types::Ubar::type, mushr_types::Ubar::params>;

// public:
//   using Xdot = mushr_types::StateDot::type;
//   using Ubar = mushr_types::Ubar::type;
//   using Params = mushr_types::Ubar::params;
//   using U = mushr_types::Control::type;

//   mushr_ub_ufix_xdot_param_t(gtsam::Key ubar, gtsam::Key xdot, gtsam::Key param, const U u,
//                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
//     : Base(ubar, xdot, param, cost_model, 0.01), _u(u)
//   {
//   }

//   virtual Ubar predict(const Ubar& ubar, const Params& params) const override
//   {
//     return mushr_ub_u_xdot_param_t::dynamics(_u, ubar, params);
//   }

//   virtual Ubar compute_error(const Ubar& ubar1, const Ubar& ubar0, const Params& params) const override
//   {
//     return predict(ubar0, params) - ubar1;
//   }

//   void eval_to_stream(gtsam::Values& values, std::ostream& os)
//   {
//     const Ubar ubar1{ values.at<Ubar>(key<1>()) };
//     const Ubar Ubar0{ values.at<Ubar>(key<2>()) };
//     const Params params{ values.at<Params>(key<3>()) };

//     os << ubar1.transpose() << " ";                                // 1, 2, 3
//     os << _u.transpose() << " ";                                   // 4, 5, 6
//     os << Ubar0.transpose() << " ";                                // 7, 8
//     os << params.transpose() << " ";                               // 9
//     os << compute_error(ubar1, Ubar0, params).transpose() << " ";  // 4, 5, 6
//     os << "\n";
//   }

// private:
//   const U _u;
// };

}  // namespace prx_models
