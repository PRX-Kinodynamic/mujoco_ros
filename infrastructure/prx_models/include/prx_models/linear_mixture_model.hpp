#pragma once

#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <string>

// Ros
#include <Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/tree_msg_wrapper.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/stela_robot_interface.hpp>
#include <prx_models/Edge.h>
#include <prx_models/tree_utils.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <interface/SensorDataStamped.h>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>

namespace prx_models
{

template <int Xdim, int Udim>
class linear_gaussian_model_t
{
public:
  using Amatrix = Eigen::Matrix<double, Xdim, Xdim>;
  using Bmatrix = Eigen::Matrix<double, Xdim, Udim>;
  using Zmatrix = Eigen::Matrix<double, Xdim + Udim, Xdim + Udim>;
  using Covariance = Eigen::Matrix<double, Xdim + Udim, Xdim + Udim>;

  using X = Eigen::Vector<double, Xdim>;
  using U = Eigen::Vector<double, Udim>;
  using Value = Eigen::Vector<double, Xdim + Udim>;

  // (1./sqrt(det(cov)*((2*pi)^5)) ) * exp(-0.5 * eps'*inv(cov)*eps)
  linear_gaussian_model_t(Amatrix A, Bmatrix B, Covariance cov, Value mean)
    : _A(A), _B(B), _inv_covariance(cov.inverse()), _mean(mean), _cte_pdf(constant_pdf_part(cov))
  {
  }
  linear_gaussian_model_t(Zmatrix Z, Covariance cov, Value mean)
    : linear_gaussian_model_t(Z.block<Xdim, Xdim>(0, 0), Z.block<Xdim, Udim>(0, Xdim + 1), cov, mean)
  {
  }

  X evaluate(const X& x, const U& u,  // no-lint
             gtsam::OptionalJacobian<Xdim, Xdim> Hx = boost::none,
             gtsam::OptionalJacobian<Xdim, Udim> Hu = boost::none) const
  {
    if (Hx)
    {
      *Hx = _A;
    }
    if (Hu)
    {
      *Hu = _B;
    }
    return _A * x + _B * u;
  }

  // diff = x-d
  // d/dx exp(-0.5*(x-d)' *A*(x-d)) =
  // -(0.5*exp(-0.5*diff'*A*diff)*A*diff+0.5*exp(-0.5*diff'*A'*diff)*A'*diff)

  double pdf(const Value& value, gtsam::OptionalJacobian<1, Xdim + Udim> Hv = boost::none) const
  {
    const Value btw{ gtsam::traits<Value>::Between(value, _mean) };
    const Eigen::Vector<double, Xdim + Udim> diff{ gtsam::traits<Value>::Logmap(btw) };
    // const Value diff{ value - _mean };
    const auto sigma_diff = _inv_covariance * diff;
    const auto sigmaT_diff = _inv_covariance.transpose() * diff;
    const double val{ std::exp(-0.5 * diff.transpose() * sigma_diff) };

    if (Hv)
    {
      // If _inv_covariance == _inv_covariance^T
      // then:
      // *Hv = - val * sigma_diff; ????
      *Hv = -(0.5 * val * sigma_diff + 0.5 * val * sigmaT_diff);
      // *Hv = result * aux;
    }
    return _cte_pdf * val;
  }

  double pdf(const X& x, const U& u,                             // no-lint
             gtsam::OptionalJacobian<1, Xdim> Hx = boost::none,  // no-lint
             gtsam::OptionalJacobian<1, Udim> Hu = boost::none) const
  {
    const bool compute_jacs{ Hx or Hu };
    Eigen::Matrix<double, 1, Xdim + Udim> jacobian;
    const Value v{ (Value() << x, u).finished() };
    const double result{ pdf(v, compute_jacs ? &jacobian : nullptr) };
    if (Hx)
    {
      *Hx = jacobian.block<1, Xdim>(0, 0);
    }
    if (Hu)
    {
      *Hu = jacobian.block<1, Udim>(0, Xdim + 1);
    }

    return result;
  }

  static double constant_pdf_part(const Covariance& cov)
  {
    // (1./sqrt(det(cov)*((2*pi)^5)) ;
    const double determinant{ cov.determinant() };
    const double val{ std::sqrt(determinant * std::pow(2 * prx::constants::pi, Xdim + Udim)) };
    return 1. / val;
  }

private:
  const double _cte_pdf;
  const Amatrix _A;
  const Bmatrix _B;
  const Value _mean;
  const Covariance _inv_covariance;
};

template <int Xdim, int Udim>
class linear_mixture_model_t
{
  using Covariance = Eigen::Matrix<double, Xdim + Udim, Xdim + Udim>;
  using Mean = Eigen::Vector<double, Xdim + Udim>;
  using Zmatrix = Eigen::Matrix<double, Xdim, Xdim + Udim>;

public:
  using LinearGaussianModel = linear_gaussian_model_t<Xdim, Udim>;
  using LinearMixtureModelPtr = std::shared_ptr<linear_mixture_model_t>;

  static LinearMixtureModelPtr from_files(const std::string linear_systems_file, const std::string covariances_file,
                                          const std::string means_file)
  {
    using prx::utilities::convert_to;

    std::vector<int> ids;
    std::map<int, Zmatrix> linear_systems;
    std::map<int, Cov> covs;
    std::map<int, Mean> means;
    prx::utilities::csv_reader_t linear_systems_reader(linear_systems_file);
    prx::utilities::csv_reader_t covs_reader(covariances_file);
    prx::utilities::csv_reader_t means_reader(means_file);

    std::vector<std::string> line;
    const int mat_size{ Xdim * (Xdim + Udim) };
    while (linear_systems_reader.next_valid_line(line))
    {
      const int id{ convert_to<int>(line[0]) };
      ids.push_back(id);
      Eigen::MatrixXd m_in(mat_size, 1);
      for (int i = 0; i < mat_size; ++i)
      {
        m_in(i, 0) = convert_to<double>(line[i + 1]);
      }
      linear_systems[id] = m_in.reshaped(Xdim, Xdim + Udim);
    }

    const int cov_size{ (Xdim + Udim) * (Xdim + Udim) };
    while (covs_reader.next_valid_line(line))
    {
      const int id{ convert_to<int>(line[0]) };
      Eigen::MatrixXd m_in(cov_size, 1);
      for (int i = 0; i < cov_size; ++i)
      {
        m_in(i, 0) = convert_to<double>(line[i + 1]);
      }
      covs[id] = m_in.reshaped(Xdim + Udim, Xdim + Udim);
    }

    const int mean_size{ Xdim + Udim };
    while (means_reader.next_valid_line(line))
    {
      const int id{ convert_to<int>(line[0]) };
      Mean mu;
      for (int i = 0; i < mean_size; ++i)
      {
        mu[i] = convert_to<double>(line[i + 1]);
      }
      means[id] = mu;
    }

    LinearMixtureModelPtr model{ std::make_shared<linear_mixture_model_t>() };

    for (auto id : ids)
    {
      const Zmatrix z{ linear_systems[id] };
      const Covariance cov{ covs[id] };
      const Mean mean{ means[id] };
      model->_models.emplace_back(z, cov, mean);
    }
    return model;
  }

private:
  std::vector<LinearGaussianModel> _models;
};
}  // namespace prx_models