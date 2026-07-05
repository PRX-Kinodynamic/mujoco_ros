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
#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>
#include "prx/utilities/math/lie_utils.hpp"
#include <motion_planning/nonlinear_clustering.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Lie.h>

namespace prx_models
{

template <typename State, typename Control>
class linear_gaussian_model_t
{
public:
  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };
  static constexpr Eigen::Index DimZ{ DimX + DimU };

  // Z = X \times U
  using Z = gtsam::ProductLieGroupV43<State, Control>;

  using Amatrix = Eigen::Matrix<double, DimX, DimX>;
  using Bmatrix = Eigen::Matrix<double, DimX, DimU>;
  using Zmatrix = Eigen::Matrix<double, DimX, DimZ>;
  using Covariance = Eigen::Matrix<double, DimZ, DimZ>;

  using DeltaX = Eigen::Vector<double, DimX>;
  using DeltaU = Eigen::Vector<double, DimU>;
  using DeltaZ = Eigen::Vector<double, DimZ>;

  template <int Rows, int Cols>
  using Jacobian = Eigen::Matrix<double, Rows, Cols>;

  template <int Rows, int Cols>
  using OptionalJacobian = Eigen::Matrix<double, Rows, Cols>*;

  // (1./sqrt(det(cov)*((2*pi)^5)) ) * exp(-0.5 * eps'*inv(cov)*eps)
  linear_gaussian_model_t(Amatrix A, Bmatrix B, Covariance cov, const State x_mean, const Control u_mean,
                          const std::size_t idx = 0)
    : _A(A)
    , _B(B)
    , _inv_covariance(cov.inverse())
    , _mean(x_mean, u_mean)
    , _cte_pdf(constant_pdf_part(cov))
    , _id(idx)
    , _gaussian_sampler(cov, /** bounded **/ false)
  {
  }

  linear_gaussian_model_t(Zmatrix z, Covariance cov, const Z mean, const std::size_t idx = 0)
    : linear_gaussian_model_t(z.template block<DimX, DimX>(0, 0), z.template block<DimX, DimU>(0, DimX), cov,
                              mean.first, mean.second, idx)
  {
  }

  // template <typename DX, typename DXU, int DimX = gtsam::traits<DX>::dimension,
  // int DimXU = gtsam::traits<DXU>::dimension>
  // Eigen::Matrix<double, DimX, DimXU>
  // Least Squares Estimation of AB, where AB corresponds to the linear system x_{t+1} = A*x_t + B*u_t; z_t = [x_t;u_t]
  static Eigen::Matrix<double, DimX, DimZ> LSE_AB(const std::vector<DeltaZ> all_zts, const std::vector<DeltaX> all_x1s)
  {
    Eigen::Matrix<double, Eigen::Dynamic, DimZ> theta(all_zts.size(), DimZ);  // (Nx5)
    Eigen::MatrixXd xs1(all_x1s.size(), DimX);                                // (Nx3)

    for (int i = 0; i < all_zts.size(); ++i)
    {
      theta.row(i) = all_zts[i];
      xs1.row(i) = all_x1s[i];
    }

    //                             (ZxZ)  =     (ZxN)               (NxZ)              (ZxN)              (NxX)
    const Jacobian<DimZ, DimX> theta_estimate{ (theta.transpose() * theta).inverse() * theta.transpose() * xs1 };
    // const Eigen::Matrix<double, DimZ, DimX> theta_estimate{ th2_inv * theta.transpose() * xs1 };

    const Eigen::Matrix<double, DimX, DimZ> AB{ theta_estimate.transpose() };
    return AB;
  }

  // template <typename Data>
  static Eigen::Matrix<double, DimX, DimZ> LSE_AB(const std::vector<std::tuple<State, Control, State>>& data)
  {
    using LGM = prx_models::linear_gaussian_model_t<State, Control>;
    std::vector<DeltaZ> all_zts;
    std::vector<DeltaX> all_x1s;

    // const State& xmean{ element.first };
    for (auto [x0, u0, x1] : data)
    {
      const Z z0{ Z(x0, u0) };
      const DeltaZ tg0{ gtsam::traits<Z>::Logmap(z0) };
      const DeltaX tg1{ gtsam::traits<State>::Logmap(x1) };

      all_zts.push_back(tg0);
      all_x1s.push_back(tg1);
    }

    return LGM::LSE_AB(all_zts, all_x1s);
  }

  DeltaX predict(const DeltaX& dx, const DeltaU& du,         // no-lint
                 OptionalJacobian<DimX, DimX> Hx = nullptr,  // no-lint
                 OptionalJacobian<DimX, DimU> Hu = nullptr) const
  {
    if (Hx)
    {
      *Hx = _A;
    }
    if (Hu)
    {
      *Hu = _B;
    }
    return _A * dx + _B * du;
  }

  DeltaX delta(const Z& z, OptionalJacobian<DimX, DimZ> Hz = nullptr) const
  {
    // const DeltaZ tg{ prx::TangentBetween(_mean, z) };
    const DeltaZ tg{ gtsam::traits<Z>::Logmap(z) };
    const DeltaX& dx{ tg.template head<DimX>() };
    const DeltaU& du{ tg.template tail<DimU>() };
    return predict(dx, du);
  }

  DeltaX delta(const State& x, const Control& u,  // no-lint
               OptionalJacobian<DimX, DimX> Hx = nullptr, OptionalJacobian<DimX, DimU> Hu = nullptr) const
  {
    Eigen::Matrix<double, DimX, DimZ> Hz;

    const Z z{ Z(x, u) };
    const DeltaX dx{ delta(z, (Hx or Hu) ? &Hz : nullptr) };

    return dx;
  }

  State evaluate(const Z& z, OptionalJacobian<DimX, DimZ> Hz = nullptr) const
  {
    // const Z btw{ gtsam::traits<Z>::Between(z, _mean) };
    // const ZTangentElement tg{ gtsam::traits<Z>::Logmap(z) };

    // const State& x0{ _mean.first };

    Jacobian<DimX, DimX> xexp_H_dx;
    Jacobian<DimX, DimZ> dx_H_z;

    const DeltaX dz{ gtsam::traits<State>::Logmap(_mean.first) };
    const DeltaX dx{ delta(z, Hz ? &dx_H_z : nullptr) };
    const State xexp{ gtsam::traits<State>::Expmap(dx, Hz ? &xexp_H_dx : nullptr) };

    if (Hz)
    {
      *Hz = xexp_H_dx * dx_H_z;
    }
    return xexp;
    // return x1;
  }

  State evaluate(const State& x, const Control& u,           // no-lint
                 OptionalJacobian<DimX, DimX> Hx = nullptr,  // no-lint
                 OptionalJacobian<DimX, DimU> Hu = nullptr) const
  {
    Eigen::Matrix<double, DimX, DimZ> Hz;
    const Z z{ Z(x, u) };
    const State res{ evaluate(z, (Hx or Hu) ? &Hz : nullptr) };

    return res;
  }

  // diff = x-d
  // d/dx exp(-0.5*(x-d)' *A*(x-d)) =
  // -(0.5*exp(-0.5*diff'*A*diff)*A*diff+0.5*exp(-0.5*diff'*A'*diff)*A'*diff)
  double pdf(const Z& z, OptionalJacobian<1, DimZ> Hz = nullptr) const
  {
    const Z btw{ gtsam::traits<Z>::Between(z, _mean) };
    const Eigen::Vector<double, DimZ> diff{ gtsam::traits<Z>::Logmap(btw) };
    // const Value diff{ value - _mean };
    const auto sigma_diff = _inv_covariance * diff;
    const double val{ std::exp(-0.5 * diff.transpose() * sigma_diff) };

    if (Hz)
    {
      // If _inv_covariance == _inv_covariance^T
      // then:
      // *Hz = - val * sigma_diff; ????
      const auto sigmaT_diff = _inv_covariance.transpose() * diff;
      *Hz = -(0.5 * val * sigma_diff + 0.5 * val * sigmaT_diff);
      // *Hz = result * aux;
    }
    return _cte_pdf * val;
  }

  double pdf(const State& x, const Control& u,  // no-lint
             OptionalJacobian<1, DimX> Hx = nullptr, OptionalJacobian<1, DimU> Hu = nullptr) const
  {
    Eigen::Matrix<double, 1, DimZ> Hz;
    const Z z{ Z(x, u) };
    const double p{ pdf(z, (Hx and Hu) ? &Hz : nullptr) };
    if (Hx)
    {
      *Hx = Hz.template block<1, DimX>(0, 0);
    }
    if (Hu)
    {
      *Hu = Hz.template block<1, DimU>(1, DimX);
    }

    return p;
  }

  static double constant_pdf_part(const Covariance& cov)
  {
    // (1./sqrt(det(cov)*((2*pi)^5)) ;
    const double determinant{ cov.determinant() };
    const double val{ std::sqrt(determinant * std::pow(2 * prx::constants::pi, DimZ)) };
    return 1. / val;
  }

  std::size_t id() const
  {
    return _id;
  }

  void verbose(const bool verbose_)
  {
    _verbose = verbose_;
  }

  // Algorithm 2 of [1]
  std::pair<double, double> compute_error_bounds(const double confidence_delta, const int M, const double sigma_w)
  {
    using NormalW = prx::multivariate_gaussian_t<DimX>;
    // static Eigen::Matrix<double, DimX, DimZ> LSE_AB(const std::vector<DeltaZ> all_zts,
    //                                               const std::vector<DeltaX> all_xdots)

    // From Prop 1 of [1], note that N is number of "values", so each value X is DimX "values"
    // Theoretically, only O(DimX + DimU) are needed. This higher N would just add robustness (?)
    const double N_raw{ 8. * DimZ + 16. * std::log(4. / confidence_delta) };
    const std::size_t N{ static_cast<std::size_t>(N_raw / DimX) };
    const Eigen::Matrix<double, DimX, DimX> wI{ sigma_w * Eigen::Matrix<double, DimX, DimX>::Identity() };
    NormalW w_sampler{ NormalW(wI) };

    std::vector<double> error_bound_A, error_bound_B;
    for (int i = 0; i < M; ++i)
    {
      std::vector<DeltaZ> all_zts;
      std::vector<DeltaX> all_x1s;
      for (int i = 0; i < N; ++i)
      {
        const DeltaZ dz{ _gaussian_sampler() };

        // const Z z_local{ gtsam::traits<Z>::Expmap(dz) };
        // const Z z{ gtsam::traits<Z>::Compose(_mean, z_local) };
        const DeltaX dx{ dz.template head<DimX>() };
        const DeltaU du{ dz.template tail<DimU>() };
        const DeltaX w{ w_sampler() };

        const DeltaX dx1{ predict(dx, du) + w };  // dx1 = A * dx + B * du + w
        all_zts.push_back(dz);
        all_x1s.push_back(dx1);
      }
      const Eigen::Matrix<double, DimX, DimZ> AB_tilde{ LSE_AB(all_zts, all_x1s) };
      const Eigen::Matrix<double, DimX, DimX> A_tilde{ AB_tilde.template block<DimX, DimX>(0, 0) };
      const Eigen::Matrix<double, DimX, DimU> B_tilde{ AB_tilde.template block<DimX, DimU>(0, DimX) };

      const double errA{ (_A - A_tilde).norm() };
      const double errB{ (_B - B_tilde).norm() };
      error_bound_A.push_back(errA);
      error_bound_B.push_back(errB);
    }

    std::sort(error_bound_A.begin(), error_bound_A.end());
    std::sort(error_bound_B.begin(), error_bound_B.end());

    // const double delta_percent{ M * confidence_delta };
    // const std::size_t total_worst{ std::max(std::size_t(1), static_cast<std::size_t>(delta_percent)) };

    // error_bound_A.erase(error_bound_A.begin(), error_bound_A.end() - total_worst);
    // error_bound_B.erase(error_bound_B.begin(), error_bound_B.end() - total_worst);
    // DEBUG_VARS(error_bound_A, error_bound_B)
    return { error_bound_A.back(), error_bound_B.back() };
  }

private:
  bool _verbose;
  const std::size_t _id;
  const double _cte_pdf;
  const Amatrix _A;
  const Bmatrix _B;
  const Z _mean;
  const Covariance _inv_covariance;
  const prx::multivariate_gaussian_t<DimZ> _gaussian_sampler;
  // [1] Dean, Sarah, Horia Mania, Nikolai Matni, Benjamin Recht, and Stephen Tu. "On the sample
  //     complexity of the linear quadratic regulator." Foundations of Computational Mathematics
  //     20, no. 4 (2020): 633-679.
};

// template <int DimX, int DimU>
template <typename State, typename Control>
class linear_mixture_model_t
{
  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };
  using Covariance = Eigen::Matrix<double, DimX + DimU, DimX + DimU>;
  using Mean = Eigen::Vector<double, DimX + DimU>;
  using Zmatrix = Eigen::Matrix<double, DimX, DimX + DimU>;

public:
  using LinearGaussianModel = linear_gaussian_model_t<State, Control>;
  using LinearMixtureModelPtr = std::shared_ptr<linear_mixture_model_t>;

  linear_mixture_model_t() : _tolerance(0.001)
  {
  }

  static LinearMixtureModelPtr from_files(const std::string linear_systems_file, const std::string covariances_file,
                                          const std::string means_file)
  {
    using prx::utilities::convert_to;

    std::vector<int> ids;
    std::map<int, Zmatrix> linear_systems;
    std::map<int, Covariance> covs;
    std::map<int, Mean> means;
    prx::utilities::csv_reader_t linear_systems_reader(linear_systems_file);
    prx::utilities::csv_reader_t covs_reader(covariances_file);
    prx::utilities::csv_reader_t means_reader(means_file);

    std::vector<std::string> line;
    const int mat_size{ DimX * (DimX + DimU) };
    while (linear_systems_reader.next_valid_line(line))
    {
      const int id{ convert_to<int>(line[0]) };
      ids.push_back(id);
      Eigen::MatrixXd m_in(mat_size, 1);
      for (int i = 0; i < mat_size; ++i)
      {
        m_in(i, 0) = convert_to<double>(line[i + 1]);
      }
      linear_systems[id] = m_in.reshaped(DimX, DimX + DimU);
    }

    const int cov_size{ (DimX + DimU) * (DimX + DimU) };
    while (covs_reader.next_valid_line(line))
    {
      const int id{ convert_to<int>(line[0]) };
      Eigen::MatrixXd m_in(cov_size, 1);
      for (int i = 0; i < cov_size; ++i)
      {
        m_in(i, 0) = convert_to<double>(line[i + 1]);
      }
      covs[id] = m_in.reshaped(DimX + DimU, DimX + DimU);
    }

    const int mean_size{ DimX + DimU };
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

  template <typename... Args>
  void emplace(const Args... args)
  {
    _models.emplace_back(args...);
  }

  void verbose(const bool verbose_)
  {
    _verbose = verbose_;
  }

  State predict(const State x0, const Control u) const
  {
    using DeltaX = typename LinearGaussianModel::DeltaX;
    using DeltaXProb = std::pair<DeltaX, double>;
    double sum_probs{ 0. };
    std::vector<DeltaXProb> deltas;
    if (_verbose)
      LOG_VARS(x0, u)
    for (auto&& mi : _models)
    {
      const double pr_i{ mi.pdf(x0, u) };
      if (pr_i > _tolerance)
      {
        const DeltaX dx{ mi.delta(x0, u) };
        deltas.push_back({ dx, pr_i });
        sum_probs += pr_i;
        if (_verbose)
        {
          const std::size_t idx{ mi.id() };
          LOG_VARS(idx, pr_i, dx)
        }
      }
      else
      {
        const std::size_t rejected_idx{ mi.id() };
        if (_verbose)
          LOG_VARS(rejected_idx, pr_i)
      }
    }
    typename LinearGaussianModel::DeltaX w_delta{ LinearGaussianModel::DeltaX::Zero() };
    if (deltas.size() == 0)
    {
      prx_warn("No cluster found")
    }
    for (auto& [di, pr_i] : deltas)
    {
      const double p_normed{ pr_i / sum_probs };
      w_delta += p_normed * di;
      if (_verbose)
        LOG_VARS(pr_i, p_normed, di, w_delta)
    }
    const State xbar{ gtsam::traits<State>::Expmap(w_delta) };
    if (_verbose)
      LOG_VARS(w_delta, xbar)
    return xbar;
  }

  std::pair<bool, State> predict_safe(const State x0, const Control u) const
  {
    using DeltaX = typename LinearGaussianModel::DeltaX;
    using DeltaXProb = std::pair<DeltaX, double>;
    double sum_probs{ 0. };
    std::vector<DeltaXProb> deltas;
    if (_verbose)
      LOG_VARS(x0, u)
    for (auto&& mi : _models)
    {
      const double pr_i{ mi.pdf(x0, u) };
      if (pr_i > _tolerance)
      {
        const DeltaX dx{ mi.delta(x0, u) };
        deltas.push_back({ dx, pr_i });
        sum_probs += pr_i;
        if (_verbose)
        {
          const std::size_t idx{ mi.id() };
          LOG_VARS(idx, pr_i, dx)
        }
      }
      else
      {
        const std::size_t rejected_idx{ mi.id() };
        if (_verbose)
          LOG_VARS(rejected_idx, pr_i)
      }
    }
    if (deltas.size() == 0)
    {
      return { false, State() };
    }

    typename LinearGaussianModel::DeltaX w_delta{ LinearGaussianModel::DeltaX::Zero() };
    for (auto& [di, pr_i] : deltas)
    {
      const double p_normed{ pr_i / sum_probs };
      w_delta += p_normed * di;
      if (_verbose)
        LOG_VARS(pr_i, p_normed, di, w_delta)
    }
    const State xbar{ gtsam::traits<State>::Expmap(w_delta) };
    if (_verbose)
      LOG_VARS(w_delta, xbar)
    return { true, xbar };
  }

private:
  bool _verbose;
  double _tolerance;
  std::vector<LinearGaussianModel> _models;
};
}  // namespace prx_models