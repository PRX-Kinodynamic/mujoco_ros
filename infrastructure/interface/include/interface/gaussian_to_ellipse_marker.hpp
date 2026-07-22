#pragma once
#include <visualization_msgs/MarkerArray.h>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

namespace interface
{

struct gaussian_params_t
{
  int idx;
  std::string frame_id;
  std::string ns;
  Eigen::Vector<double, 3> position;
  Eigen::Quaterniond orientation;
  Eigen::Vector<double, 4> color;  // alpha first: ARGB
  double confidence;
  Eigen::Vector<double, 3> axis;

  gaussian_params_t()
    : idx(0)
    , frame_id("world")
    , color(1, 0, 0, 0)
    , confidence(7.815)
    , ns("confidence_ellipse")
    , axis(Eigen::Vector3d::Zero())
  {
  }

  // Given a covariance matrix (NxN), obtain the marginal (MxM), where M < N
  template <int MarginalDim, int CovarianceDim>
  static Eigen::Matrix<double, MarginalDim, MarginalDim>
  marginal(const Eigen::Matrix<double, CovarianceDim, CovarianceDim>& cov, const int marginal_dim_init)
  {
    using Amat = Eigen::Matrix<double, MarginalDim, MarginalDim>;
    using Bmat = Eigen::Matrix<double, MarginalDim, CovarianceDim - MarginalDim>;
    using Cmat = Eigen::Matrix<double, CovarianceDim - MarginalDim, CovarianceDim - MarginalDim>;

    const int rest_start{ marginal_dim_init + MarginalDim };
    // const int col_rest_start{ marginal_dim_init + MarginalDim };

    //  Covariance matrix is:
    // [ A  | B ]
    // [ B' | C ]
    const Amat A{ cov.template block<MarginalDim, MarginalDim>(marginal_dim_init, marginal_dim_init) };
    const Bmat B{ cov.template block<MarginalDim, CovarianceDim - MarginalDim>(marginal_dim_init, rest_start) };
    const Cmat C{ cov.template block<CovarianceDim - MarginalDim, CovarianceDim - MarginalDim>(rest_start,
                                                                                               rest_start) };

    // Marginal is: A - B * C^-1 * B'
    //            (MxM) - (Mx(N-M)) * ((N-M)x(N-M)) * ((N-M)xM)
    const Amat cov_marginal{ A - B * C.inverse() * B.transpose() };
    return cov_marginal;
  }

  template <int CovarianceDim>
  void cov_to_3Dellipse(const Eigen::Matrix<double, CovarianceDim, CovarianceDim>& cov, const bool verbose = false)
  {
    using Covariance = Eigen::Matrix<double, CovarianceDim, CovarianceDim>;
    using CovVector = Eigen::Vector<double, CovarianceDim>;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
    const Eigen::Vector<double, CovarianceDim> D_marginal{ es.eigenvalues() };
    double sign{ 1. };
    if (es.eigenvectors().determinant() < 0)
    {
      sign = -1.;
      if (verbose)
      {
        DEBUG_VARS(es.eigenvectors(), es.eigenvectors().determinant());
      }
    }
    const Covariance V_marginal{ sign * es.eigenvectors() };

    Eigen::Matrix3d rot{ Eigen::Matrix3d::Identity() };
    // CovarianceDim could be less than 3
    rot.block<CovarianceDim, CovarianceDim>(0, 0) = V_marginal;
    orientation = Eigen::Quaterniond(rot);

    if (verbose)
    {
      DEBUG_VARS(cov);
      DEBUG_VARS(D_marginal);
      DEBUG_VARS(V_marginal);
      DEBUG_VARS(rot);
      DEBUG_VARS(orientation);
    }

    axis = Eigen::Vector3d::Zero();
    axis.head<CovarianceDim>() = D_marginal;
  }
};

static void gaussian_to_ellipse_marker(visualization_msgs::Marker& marker, const gaussian_params_t& input)
{
  marker.header.frame_id = input.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = input.ns;
  marker.id = input.idx;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = input.position[0];
  marker.pose.position.y = input.position[1];
  marker.pose.position.z = input.position[2];
  marker.pose.orientation.x = input.orientation.x();
  marker.pose.orientation.y = input.orientation.y();
  marker.pose.orientation.z = input.orientation.z();
  marker.pose.orientation.w = input.orientation.w();
  marker.color.a = input.color[0];
  marker.color.r = input.color[1];
  marker.color.g = input.color[2];
  marker.color.b = input.color[3];
  marker.type = visualization_msgs::Marker::SPHERE;
  // input.confidence = sqrt(Xi^2_{N,alpha})
  // In other sources, scale.x = std::sqrt(confidence * input.axis[0])
  // (or even 2*sqrt(...), if the input expects the total length of the axis)
  marker.scale.x = 2 * std::sqrt(input.confidence * input.axis[0]);
  marker.scale.y = 2 * std::sqrt(input.confidence * input.axis[1]);
  marker.scale.z = 2 * std::sqrt(input.confidence * input.axis[2]);
}

static visualization_msgs::Marker gaussian_to_ellipse_marker(const gaussian_params_t& input)
{
  visualization_msgs::Marker marker;
  gaussian_to_ellipse_marker(marker, input);
  return marker;
}

}  // namespace interface
