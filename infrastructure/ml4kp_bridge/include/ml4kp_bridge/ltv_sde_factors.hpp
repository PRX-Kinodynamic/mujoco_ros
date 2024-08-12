#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
template <typename X, typename Xdot>
class euler_observation_factor_t : public gtsam::NoiseModelFactorN<X, Xdot>
{
  using Base = gtsam::NoiseModelFactorN<X, Xdot>;
  using Derived = euler_observation_factor_t<X, Xdot>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };

  euler_observation_factor_t() = delete;

public:
  euler_observation_factor_t(const euler_observation_factor_t& other) = delete;

  euler_observation_factor_t(const gtsam::Key key_x, const gtsam::Key key_xdot, const NoiseModel& cost_model,
                             const X x_tepsilon, const double dt, const std::string label = "EulerZ")
    : Base(cost_model, key_x, key_xdot), _dt(dt), _xte(x_tepsilon), _label(label)
  {
  }

  ~euler_observation_factor_t() override
  {
  }

  static X predict(const X& x, const Xdot& xdot, const double& dt,  // no-lint
                   OptDeriv Hx = boost::none, OptDeriv Hxdot = boost::none)
  {
    // clang-format off
    if (Hx){ *Hx = -Eigen::Matrix<double, DimX, DimX>::Identity(); }
    if (Hxdot){ *Hxdot = -dt * Eigen::Matrix<double, DimXdot, DimXdot>::Identity(); }
    // clang-format on
    return x + xdot * dt;
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const bool activated{ _dt > 0 };
    return activated;
  }

  // template <std::enable_if_t<not DtKey, bool> = true>
  virtual Eigen::VectorXd evaluateError(const X& xt, const Xdot& xdot,  // no-lint
                                        OptDeriv Hxt = boost::none, OptDeriv Hxdot = boost::none) const override
  {
    const X prediction{ predict(xt, xdot, _dt, Hxt, Hxdot) };
    return _xte - prediction;
  }

  void to_stream(std::ostream& os, const gtsam::Values& values) const
  {
    const char sp{ prx::constants::separating_value };

    const gtsam::Key kx{ this->template key<1>() };
    const gtsam::Key kxdot{ this->template key<2>() };

    const X x{ values.at<X>(kx) };
    const Xdot xdot{ values.at<Xdot>(kxdot) };

    os << _label << sp;
    os << symbol_factory_t::formatter(kx) << " " << x.transpose() << sp;
    os << symbol_factory_t::formatter(kxdot) << " " << xdot.transpose() << sp;
    os << "Z: " << _xte.transpose() << " dt:" << _dt << sp;
    os << "\n";
  }

private:
  const double _dt;
  const X _xte;
  const std::string _label;
};
}  // namespace fg
}  // namespace prx