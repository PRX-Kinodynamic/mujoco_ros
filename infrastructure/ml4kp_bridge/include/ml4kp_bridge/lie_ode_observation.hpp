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
// The trivial observation (where the observation is the full state)
// In general, an observation is: Z = h(X) + Noise
// This trivial case considers: Z = X + Noise => h(X) = X (Jacobian is identity)
// This template implements the trivial h(.)
template <typename State, typename Observation>
struct trivial_observation_function_t
{
  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimZ{ gtsam::traits<Observation>::dimension };
  static Observation predict(const State& x, gtsam::OptionalJacobian<DimZ, DimX> H = boost::none)
  {
    if (H)
    {
      *H = Eigen::Matrix<double, DimZ, DimX>::Identity();
    }
    return x;
  }
};

template <typename X, typename Xdot, typename Z = X,
          typename ObservationFunction = trivial_observation_function_t<X, Z>>
class lie_ode_observation_factor_t : public gtsam::NoiseModelFactorN<X, Xdot>
{
  using Base = gtsam::NoiseModelFactorN<X, Xdot>;
  using Derived = lie_ode_observation_factor_t<X, Xdot>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using LieIntegrator = lie_integrator_t<X, Xdot>;
  using SF = prx::fg::symbol_factory_t;

  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };
  static constexpr Eigen::Index DimZ{ gtsam::traits<Z>::dimension };

  lie_ode_observation_factor_t() = delete;

public:
  lie_ode_observation_factor_t(const lie_ode_observation_factor_t& other) = delete;

  lie_ode_observation_factor_t(const gtsam::Key key_x, const gtsam::Key key_xdot, const NoiseModel& cost_model,
                               const Z z_tepsilon, const double dt, const std::string label = "LieOdeZ")
    : Base(cost_model, key_x, key_xdot), _dt(dt), _zte(z_tepsilon), _label(label)
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
    Eigen::Matrix<double, DimZ, DimZ> err_H_b;      // Deriv error wrt between
    Eigen::Matrix<double, DimZ, DimZ> b_H_z;        // Deriv between wrt z
    Eigen::Matrix<double, DimZ, DimZ> b_H_zp;       // Deriv between wrt predicted
    Eigen::Matrix<double, DimX, DimX> p_H_x;        // Deriv predicted wrt x
    Eigen::Matrix<double, DimX, DimXdot> p_H_xdot;  // Deriv predicted wrt xdot
    Eigen::Matrix<double, DimZ, DimX> zp_H_p;       // Deriv predicted wrt xdot

    const X prediction{ predict(x, xdot, _dt,           // no-lint
                                Hx ? &p_H_x : nullptr,  // no-lint
                                Hxdot ? &p_H_xdot : nullptr) };

    const Z z_prediction{ ObservationFunction::predict(prediction, Hx ? &zp_H_p : nullptr) };

    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const Z between{ gtsam::traits<Z>::Between(_zte, z_prediction,  // no-lint
                                               boost::none,         // no-lint
                                               (Hx or Hxdot) ? &b_H_zp : nullptr) };
    const Eigen::Vector<double, DimZ> error{ gtsam::traits<Z>::Logmap(between, (Hx or Hxdot) ? &err_H_b : nullptr) };

    if (Hx)
    {
      *Hx = err_H_b * b_H_zp * zp_H_p * p_H_x;
    }
    if (Hxdot)
    {
      *Hxdot = err_H_b * b_H_zp * zp_H_p * p_H_xdot;
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
    os << "Z: " << _zte << " dt:" << _dt << sp;
    os << "\n";
  }

  void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = SF::formatter) const override
  {
    const char sp{ prx::constants::separating_value };

    const gtsam::Key kx{ this->template key<1>() };
    const gtsam::Key kxdot{ this->template key<2>() };

    std::cout << _label << sp;
    std::cout << symbol_factory_t::formatter(kx) << sp;
    std::cout << symbol_factory_t::formatter(kxdot) << sp;
    std::cout << "Z: " << _zte << " dt:" << _dt << sp;
    std::cout << "\n";
  }

private:
  const double _dt;
  const Z _zte;
  const std::string _label;
};
}  // namespace fg
}  // namespace prx