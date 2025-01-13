#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "prx/external/PQP/PQP_Eigen.hpp"
#include "prx/utilities/geometry/geometry.hpp"
#include "prx/utilities/geometry/movable_object.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include <utils/signed_distance_field.hpp>

namespace motion_planning
{
// using StateToConfiguration = void (*)(const std::size_t, const ml4kp_bridge::SpacePoint&);
template <typename State, typename ConfigurationFromState>
class sdf_factor_t : public gtsam::NoiseModelFactor1<State>
{
  using Base = gtsam::NoiseModelFactor1<State>;
  using Derived = sdf_factor_t<State, ConfigurationFromState>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Sdf = utils::signed_distance_field_t;
  using SdfPtr = std::shared_ptr<Sdf>;

  using Key = gtsam::Key;
  using SF = prx::fg::symbol_factory_t;

public:
  sdf_factor_t(sdf_factor_t&& other)
    : Base(other), _eps_distance(other._eps_distance), _sdf(other.sdf), _config_from_state(other._config_from_state)
  {
  }

  sdf_factor_t(const gtsam::Key& pose_key, const double safety_distance, const SdfPtr sdf,
               const NoiseModel& cost_model = nullptr)
    : Base(cost_model, pose_key), _eps_distance(safety_distance), _sdf(sdf)
  {
  }

  inline double distance(const State& x, gtsam::OptionalJacobian<1, 2> Hx = boost::none) const
  {
    _config_from_state.configuration(_pt, x);
    const double dist{ _sdf->distance(_pt) };

    if (Hx)
    {
      *Hx = _sdf->jacobian(_pt);
    }
    return dist;
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const State state{ values.at<State>(this->template key<1>()) };
    const double dist{ distance(state) };
    const bool is_close{ dist < _eps_distance };

    return is_close;
  }

  virtual bool sendable() const override
  {
    return true;
  }

  virtual Eigen::VectorXd evaluateError(const State& x0,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const double dist{ distance(x0, H0 ? &_Hconfig : nullptr) };

    const double activated_dist{ -dist + _eps_distance };

    const Eigen::VectorXd error{ { activated_dist } };
    // const std::string key{ SF::formatter(this->template key<1>()) };
    if (H0)
    {
      _config_from_state.jacobian(x0, _Hconfig, *H0);
      *H0 = -1 * (*H0);
    }

    return error;
  }

  void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = SF::formatter) const override
  {
    std::cout << s << "SdfFactor: " << keyFormatter(this->template key<1>());
    std::cout << " epsilon_distance: " << _eps_distance;
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
    std::cout << "\n";
  }

protected:
  const SdfPtr _sdf;
  const double _eps_distance;
  mutable ConfigurationFromState _config_from_state;

  mutable Eigen::Vector2d _pt;
  mutable Eigen::Matrix<double, 1, 2> _Hconfig;
};
}  // namespace motion_planning