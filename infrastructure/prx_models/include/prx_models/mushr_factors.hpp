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
constexpr std::size_t vel_desired{ 0 };
constexpr std::size_t steering{ 1 };
}  // namespace Control

}  // namespace mushr_types

using mushr_x_xdot_t = prx::fg::lie_integration_factor_t<mushr_types::State::type, mushr_types::StateDot::type>;
// template <typename X, typename Xdot>
// class lie_integration_factor_t
// X_j = X_i + \dpt{x}_i * dt
// class mushr_x_xdot_t : public prx::fg::noise_model_3factor_t<mushr_types::State::type, mushr_types::StateDot::type,
//                                                              mushr_types::State::type>
// {
//   using Base = noise_model_3factor_t<mushr_types::State::type, mushr_types::StateDot::type,
//   mushr_types::State::type>;

// public:
//   using X = mushr_types::State::type;
//   using Xdot = mushr_types::StateDot::type;

//   mushr_x_xdot_t(gtsam::Key xi, gtsam::Key xdot, gtsam::Key xj, const double dt,
//                  const gtsam::noiseModel::Base::shared_ptr& cost_model)
//     : Base(xi, xdot, xj, cost_model, 0.01), _dt(dt)
//   {
//   }

//   static X dynamics(const X& x, const Xdot& xdot, const double dt)
//   {
//     // const gtsam::Pose2 res{ gtsam::Pose2(x[0], x[1], x[2]) * gtsam::Pose2::Expmap(xdot * dt) };
//     // return X{ res.x(), res.y(), res.theta() };
//     return x + xdot * dt;
//   }

//   virtual X predict(const X& x, const Xdot& xdot) const override
//   {
//     return dynamics(x, xdot, _dt);
//   }

//   virtual X compute_error(const X& xi, const Xdot& xdot, const X& xj) const override
//   {
//     const X prediction{ predict(xi, xdot) };
//     X error{ prediction - xj };
//     error[2] = prx::angle_diff(prediction[2], xj[2]);
//     return error;
//   }

//   void eval_to_stream(gtsam::Values& values, std::ostream& os)
//   {
//     const X xi{ values.at<X>(key<1>()) };
//     const Xdot xdot{ values.at<Xdot>(key<2>()) };
//     const X xj{ values.at<X>(key<3>()) };

//     os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xi.transpose() << " ";    // 1, 2, 3
//     os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << xdot.transpose() << " ";  // 4, 5, 6
//     os << prx::fg::symbol_factory_t::formatter(key<3>()) << " " << xj.transpose() << " ";    // 7, 8, 9
//     os << "dt " << _dt << " ";                                                               // 10
//     os << "Error " << compute_error(xi, xdot, xj).transpose() << " ";                        // 4, 5, 6
//     os << "\n";
//   }

// private:
//   const double _dt;
// };

class mushr_x_xdot_ub_t : public prx::fg::noise_model_3factor_t<mushr_types::StateDot::type, mushr_types::State::type,
                                                                mushr_types::Ubar::type>
{
  using Base = noise_model_3factor_t<mushr_types::StateDot::type, mushr_types::State::type, mushr_types::Ubar::type>;
  using Error = Base::Error;

public:
  using X = mushr_types::State::type;
  using Xdot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;

  mushr_x_xdot_ub_t(gtsam::Key xdot, gtsam::Key x, gtsam::Key ubar,
                    const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(xdot, x, ubar, cost_model, 0.01)
  {
  }

  static Xdot dynamics(const X& x, const Ubar& ubar)
  {
    const double& vt{ ubar[mushr_types::Ubar::velocity] };
    const double& beta{ ubar[mushr_types::Ubar::beta] };

    const double& theta{ x[mushr_types::State::theta] };

    const double cTh{ std::cos(theta + beta) };
    const double sTh{ std::sin(theta + beta) };
    const double wt{ 2.0 * vt * std::sin(beta) / mushr_types::Parameters::L };

    return Xdot{
      vt * cTh,  // no-indent
      vt * sTh,  // no-indent
      wt         // no-indent
    };
  }

  virtual Xdot predict(const X& x, const Ubar& ubar) const override
  {
    return dynamics(x, ubar);
  }

  // virtual Error compute_error(const X0& x0, const X1& x1, const X2& x2) const
  virtual Error compute_error(const Xdot& xdot, const X& xi, const Ubar& ub) const override
  {
    return predict(xi, ub) - xdot;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Xdot xdot{ values.at<Xdot>(key<1>()) };
    const X xi{ values.at<X>(key<2>()) };
    const Ubar ubar{ values.at<Ubar>(key<3>()) };

    os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xdot.transpose() << " ";  // 4, 5, 6
    os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << xi << " ";                // 1, 2, 3
    os << prx::fg::symbol_factory_t::formatter(key<3>()) << " " << ubar.transpose() << " ";  // 7, 8
    os << "Error: " << compute_error(xdot, xi, ubar).transpose() << " ";                     // 9, 10, 11
    os << "\n";
  }

private:
};

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

  mushr_ub_u_xdot_param_t(gtsam::Key ubar, gtsam::Key u, gtsam::Key xdot, gtsam::Key param,
                          const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(ubar, u, xdot, param, cost_model, 0.01)
  {
  }

  static Ubar dynamics(const U& u, const Ubar& ubar, const Params& params)
  {
    const double& v_current{ ubar[mushr_types::Ubar::velocity] };
    const double& steering{ u[mushr_types::Control::steering] };
    const double& v_desired{ u[mushr_types::Control::vel_desired] };

    const double& accel_slope{ params[mushr_types::Ubar::accel_slope] };
    const double& steering_param{ params[mushr_types::Ubar::steering_param] };
    const double& max_vel_param{ params[mushr_types::Ubar::max_vel_param] };

    const double dv{ v_desired - v_current };
    const double v_next{ v_current + dv * accel_slope };
    const double beta{ std::atan(0.5 * std::tan(steering * steering_param)) };

    Ubar ubar_next{};
    ubar_next[mushr_types::Ubar::beta] = beta;
    ubar_next[mushr_types::Ubar::velocity] = max_vel_param * v_next;

    return ubar_next;
  }

  virtual Ubar predict(const U& u, const Ubar& ubar, const Params& params) const override
  {
    return dynamics(u, ubar, params);
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
};

class mushr_ub_ufix_xdot_param_t
  : public prx::fg::noise_model_3factor_t<mushr_types::Ubar::type, mushr_types::Ubar::type, mushr_types::Ubar::params>
{
  using Base =
      prx::fg::noise_model_3factor_t<mushr_types::Ubar::type, mushr_types::Ubar::type, mushr_types::Ubar::params>;

public:
  using Xdot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;
  using Params = mushr_types::Ubar::params;
  using U = mushr_types::Control::type;

  mushr_ub_ufix_xdot_param_t(gtsam::Key ubar, gtsam::Key xdot, gtsam::Key param, const U u,
                             const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(ubar, xdot, param, cost_model, 0.01), _u(u)
  {
  }

  virtual Ubar predict(const Ubar& ubar, const Params& params) const override
  {
    return mushr_ub_u_xdot_param_t::dynamics(_u, ubar, params);
  }

  virtual Ubar compute_error(const Ubar& ubar1, const Ubar& ubar0, const Params& params) const override
  {
    return predict(ubar0, params) - ubar1;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Ubar ubar1{ values.at<Ubar>(key<1>()) };
    const Ubar Ubar0{ values.at<Ubar>(key<2>()) };
    const Params params{ values.at<Params>(key<3>()) };

    os << ubar1.transpose() << " ";                                // 1, 2, 3
    os << _u.transpose() << " ";                                   // 4, 5, 6
    os << Ubar0.transpose() << " ";                                // 7, 8
    os << params.transpose() << " ";                               // 9
    os << compute_error(ubar1, Ubar0, params).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

private:
  const U _u;
};

// class mushr_x_observation_t : public prx::fg::noise_model_1factor_t<mushr_types::State::type>
// {
//   using Base = noise_model_1factor_t<mushr_types::State::type>;

// public:
//   using X = mushr_types::State::type;

//   mushr_x_observation_t(const gtsam::Key x, const X z, const gtsam::noiseModel::Base::shared_ptr& cost_model)
//     : Base(x, cost_model, 0.01), _z(z)
//   {
//   }

//   virtual X compute_error(const X& x) const override
//   {
//     X error{ x - _z };
//     error[2] = prx::angle_diff(x[2], _z[2]);
//     return error;
//   }

//   void eval_to_stream(gtsam::Values& values, std::ostream& os)
//   {
//     const X x{ values.at<X>(key<1>()) };

//     os << x.transpose() << " ";                 // 1, 2, 3
//     os << _z.transpose() << " ";                // 4, 5, 6
//     os << compute_error(x).transpose() << " ";  // 4, 5, 6
//     os << "\n";
//   }

// protected:
//   const X _z;
// };

}  // namespace prx_models
