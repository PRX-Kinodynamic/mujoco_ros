#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace prx
{
namespace fg
{
template <typename X, typename Xdot>
class lie_ode_observation_factor_t : public gtsam::NoiseModelFactorN<X, Xdot>
{
  using Base = gtsam::NoiseModelFactorN<X, Xdot>;
  using Derived = lie_ode_observation_factor_t<X, Xdot>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using LieIntegrator = lie_integrator_t<X, Xdot>;

  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };

  lie_ode_observation_factor_t() = delete;

public:
  lie_ode_observation_factor_t(const lie_ode_observation_factor_t& other) = delete;

  lie_ode_observation_factor_t(const gtsam::Key key_x, const gtsam::Key key_xdot, const NoiseModel& cost_model,
                               const X x_tepsilon, const double dt, const std::string label = "LieOdeZ")
    : Base(cost_model, key_x, key_xdot), _dt(dt), _xte(x_tepsilon), _label(label)
  {
  }

  ~lie_ode_observation_factor_t() override
  {
  }

  static X predict(const X& x, const Xdot& xdot, const double& dt,  // no-lint
                   gtsam::OptionalJacobian<DimX, DimX> Hx = boost::none,
                   gtsam::OptionalJacobian<DimX, DimXdot> Hxdot = boost::none)
  {
    return LieIntegrator::integrate(x, xdot, dt, Hx, Hxdot);
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const bool activated{ _dt > 0 };
    // DEBUG_VARS(activated);
    return activated;
  }

  // Error is: z (-) q_^{predicted}_1; where q_^{predicted}_1 = q0 (+) qdot dt, for a fix (known) dt
  virtual Eigen::VectorXd evaluateError(const X& x, const Xdot& xdot,  // no-lint
                                        OptDeriv Hx = boost::none, OptDeriv Hxdot = boost::none) const override
  {
    // DEBUG_VARS(x);
    Eigen::Matrix<double, DimX, DimX> err_H_b;      // Deriv error wrt between
    Eigen::Matrix<double, DimX, DimX> b_H_z;        // Deriv between wrt z
    Eigen::Matrix<double, DimX, DimX> b_H_p;        // Deriv between wrt predicted
    Eigen::Matrix<double, DimX, DimX> p_H_x;        // Deriv predicted wrt x
    Eigen::Matrix<double, DimX, DimXdot> p_H_xdot;  // Deriv predicted wrt xdot

    const X prediction{ predict(x, xdot, _dt,           // no-lint
                                Hx ? &p_H_x : nullptr,  // no-lint
                                Hxdot ? &p_H_xdot : nullptr) };
    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const X between{ _xte.between(prediction,                        // no-lint
                                  (Hx or Hxdot) ? &b_H_z : nullptr,  // no-lint
                                  (Hx or Hxdot) ? &b_H_p : nullptr) };
    const Eigen::Vector<double, DimX> error{ X::Logmap(between, (Hx or Hxdot) ? &err_H_b : nullptr) };

    if (Hx)
    {
      *Hx = err_H_b * b_H_p * p_H_x;
    }
    if (Hxdot)
    {
      *Hxdot = err_H_b * b_H_p * p_H_xdot;
    }
    // DEBUG_VARS(error.transpose());

    return error;
  }

  void to_stream(std::ostream& os, const gtsam::Values& values) const
  {
    const char sp{ prx::constants::separating_value };

    const gtsam::Key kx{ this->template key<1>() };
    const gtsam::Key kxdot{ this->template key<2>() };

    const X x{ values.at<X>(kx) };
    const Xdot xdot{ values.at<Xdot>(kxdot) };

    os << _label << sp;
    os << symbol_factory_t::formatter(kx) << " " << x << sp;
    os << symbol_factory_t::formatter(kxdot) << " " << xdot << sp;
    os << "Z: " << _xte << " dt:" << _dt << sp;
    os << "\n";
  }

private:
  const double _dt;
  const X _xte;
  const std::string _label;
};
}  // namespace fg
}  // namespace prx