#pragma once

namespace motion_planning
{
namespace metrics
{
// EigenVector as in Eigen::Vector<> (aka has norm() function)
template <typename EigenVector>
struct eucleadian
{
  double operator()(const EigenVector& a, const EigenVector& b) const
  {
    return (a - b).norm();
  }
};
}  // namespace metrics
}  // namespace motion_planning
