
#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <ml4kp_bridge/PlanStepStampedArray.h>
#include <ml4kp_bridge/SpacePointStampedArray.h>
#include <prx/simulation/system.hpp>
#include <prx/simulation/forward_propagation.hpp>
#include <prx/factor_graphs/factors/constraint_factor.hpp>
#include <prx/utilities/math/lie_utils.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

namespace prx
{
// template <int Row, int Col>  // primary template
// struct stream_specialization<Eigen::Matrix<double, Row, Col>> : std::true_type
// {
// };

// // template <int Dim>
// template <int Row, int Col>
// struct streamer_t<Eigen::Matrix<double, Row, Col>> : std::true_type
// {
// public:
//   // using Vector = Eigen::Vector<double, Row>;
//   // using Matrix = Eigen::Matrix<double, Row, Col>;

//   template <typename Vector, std::enable_if_t<Vector::ColsAtCompileTime == 1, bool> = true>
//   static void to_stream(std::ostream& os, const Vector& v)
//   {
//     os << v.transpose() << " ";
//   }

//   template <typename Matrix, std::enable_if_t<Matrix::ColsAtCompileTime != 1, bool> = true>
//   static void to_stream(std::ostream& os, const Matrix& m)
//   {
//     os << m << " ";
//   }
// };
}  // namespace prx