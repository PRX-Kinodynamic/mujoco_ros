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
    return activated;
  }

  // Error is: z (-) q_^{predicted}_1; where q_^{predicted}_1 = q0 (+) qdot dt, for a fix (known) dt
  virtual Eigen::VectorXd evaluateError(const X& x, const Xdot& xdot,  // no-lint
                                        OptDeriv Hx = boost::none, OptDeriv Hxdot = boost::none) const override
  {
    Eigen::Matrix<double, DimX, DimX> err_H_b;       // Deriv error wrt between
    Eigen::Matrix<double, DimX, DimX> b_H_q1;        // Deriv between wrt x1
    Eigen::Matrix<double, DimX, DimX> b_H_qp;        // Deriv between wrt predicted
    Eigen::Matrix<double, DimX, DimX> qp_H_q0;       // Deriv predicted wrt x0
    Eigen::Matrix<double, DimX, DimXdot> qp_H_qdot;  // Deriv predicted wrt xdot
    Eigen::Matrix<double, DimX, 1> qp_H_qdt;         // Deriv predicted wrt dt

    const X prediction{ predict(x, xdot, _dt,             // no-lint
                                Hx ? &qp_H_q0 : nullptr,  // no-lint
                                Hxdot ? &qp_H_qdot : nullptr) };
    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const X between{ _xte.between(prediction,                         // no-lint
                                  (Hx or Hxdot) ? &b_H_q1 : nullptr,  // no-lint
                                  (Hx or Hxdot) ? &b_H_qp : nullptr) };
    const Eigen::VectorXd error{ X::Logmap(between, (Hx or Hxdot) ? &err_H_b : nullptr) };

    if (Hx)
    {
      *Hx = err_H_b * b_H_q1;
    }
    if (Hxdot)
    {
      *Hxdot = err_H_b * b_H_qp * qp_H_qdot;
    }

    // // Derivatives of q1p with respect to q/qdot
    // Eigen::Matrix<double, DimX, DimX> q1p_H_q0;
    // Eigen::Matrix<double, DimX, DimXdot> q1p_H_qdot;
    // // Derivative of between with respect to q1p
    // Eigen::Matrix<double, DimX, DimXdot> c_H_q1p;
    // OptDeriv c_H_q1p{ (Hxt or Hxdot) ? Eigen::MatrixXd::Zero(DimX, DimX) : nullptr };
    // // Derivative of Err with respect to C
    // OptDeriv err_H_c{ (Hxt or Hxdot) ? Eigen::MatrixXd::Zero(DimX, DimX) : nullptr };

    // // Predict q1
    // const X q1p{ predict(xt, xdot, _dt, q1p_H_q0, q1p_H_qdotdt) };

    // const X between{ gtsam::traits<X>::Between(_xte, prediction.inverse(), nullptr, c_H_q1p) };
    // const X error{ X::Logmap(composition, err_H_c) };

    // if (Hxt)
    // {
    //   *Hxt = err_H_c * c_H_q1p * q1p_H_q0;
    // }
    // if (Hxdot)
    // {
    //   *Hxdot = err_H_c * c_H_q1p * q1p_H_qdotdt;
    // }
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