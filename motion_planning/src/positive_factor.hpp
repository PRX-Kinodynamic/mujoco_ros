#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace prx
{
namespace fg
{
// ComparisonFn: Returns true when the constrain is NOT satisfied.
//               Activate when: (value *FN* constraint) is true. eg: *value* < (constraint=0) (using std::less)
template <typename Type, typename ComparisonFn>
class constraint_factor_t : public gtsam::NoiseModelFactor1<double>
{
  using Type = double;
  using Base = gtsam::NoiseModelFactor1<double>;
  using Derived = constraint_factor_t;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  static constexpr Eigen::Index Dim{ gtsam::traits<Type>::dimension };

  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;
  using ErrorVector = Eigen::Vector<double, Dim>;
  using Jacobian = Eigen::Matrix<double, Dim, Dim>;

public:
  constraint_factor_t(const gtsam::Key& key, const Type& constrain, const NoiseModel& cost_model)
    : Base(cost_model, key), _constrain(constrain)
  {
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const Type val{ values.at<Type>(this->template key<1>()) };
    const bool activated{ ComparisonFn(val, _constrain) };
    return activated;
  }

  // Error and jacobian is in the same dim as Type. Could be parametrized somehow
  virtual Eigen::VectorXd evaluateError(const Type& x0, OptionalMatrix H0 = boost::none) const override
  {
    const ErrorVector error{ ErrorVector::Ones() };

    if (H0)
    {
      *H0 = Jacobian::Ones();
    }
    return error;
  }

private:
  Type _constrain;
  ComparisonFn _comparison;
};
}  // namespace fg
}  // namespace prx