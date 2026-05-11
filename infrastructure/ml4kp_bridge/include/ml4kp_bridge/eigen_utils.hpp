#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include "prx/utilities/general/template_utils.hpp"

#define PRX_EIGEN_IS_VECTOR(Type, Dimension)                                                                           \
  template <>                                                                                                          \
  struct is_eigen_vector<Eigen::Vector<Type, Dimension>> : std::true_type                                              \
  {                                                                                                                    \
  };

namespace ml4kp_bridge
{

template <class T>
struct is_eigen_vector : std::false_type
{
};

PRX_EIGEN_IS_VECTOR(double, 1)
PRX_EIGEN_IS_VECTOR(double, 2)
PRX_EIGEN_IS_VECTOR(double, 3)
PRX_EIGEN_IS_VECTOR(double, 4)
PRX_EIGEN_IS_VECTOR(double, 5)
PRX_EIGEN_IS_VECTOR(double, 6)
PRX_EIGEN_IS_VECTOR(double, 7)
PRX_EIGEN_IS_VECTOR(double, 8)
PRX_EIGEN_IS_VECTOR(double, 9)
}  // namespace ml4kp_bridge