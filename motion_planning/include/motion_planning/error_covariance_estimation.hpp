#pragma once
#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>

#include <iterator>
#include <memory>
#include <prx/simulation/forward_propagation.hpp>
#include <string>
#include <std_msgs/Bool.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <interface/PlannerClock.h>
#include <visualization_msgs/MarkerArray.h>

#include <motion_planning/utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <prx/simulation/collision_checking/pqp_collision_checker.hpp>
#include <utils/rosparams_utils.hpp>
#include <prx/external/thread_pool/BS_thread_pool.hpp>
#include <prx/utilities/data_structures/implicit_grid.hpp>
#include <prx/utilities/general/transforms.hpp>

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>

namespace motion_planning
{

template <int Dim>
struct incremental_covariance_t
{
  using Element = Eigen::Vector<double, Dim>;
  using Covariance = Eigen::Matrix<double, Dim, Dim>;

  incremental_covariance_t() : _A(Covariance::Zero()), _b(Element::Zero()), _sample_mean(Element::Zero())
  {
  }

  void update_covariance_values(const Element new_element, const bool increment)
  {
    if (increment)
    {
      _A += new_element * new_element.transpose();
      _b += new_element;
      _sample_mean = (1. / (_total_samples + 1)) * _b;
      _total_samples++;
    }
    else
    {
      _A -= new_element * new_element.transpose();
      _b -= new_element;
      _sample_mean = (1. / _total_samples) * _b;
      _total_samples--;
    }
  }

  Covariance compute_covariance() const
  {
    const double rate{ 1. / _total_samples };

    return rate * (_A -                             // no-lint
                   _sample_mean * _b.transpose() -  // no-lint
                   _b * _sample_mean.transpose() +  // no-lint
                   _total_samples * _sample_mean * _sample_mean.transpose());
  }

  int _total_samples;

  Eigen::Matrix<double, Dim, Dim> _A;
  Eigen::Vector<double, Dim> _b;
  Eigen::Vector<double, Dim> _sample_mean;
};

template <typename DynamicalSystem>
class error_covariance_estimation_t
{
public:
  using TrajectoryMsg = std::vector<ml4kp_bridge::SpacePointStamped>;
  using PlanMsg = ml4kp_bridge::PlanStepStampedArray;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Trajectory = std::vector<State>;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };

  using Error = Eigen::Vector<double, DimX>;

  using Covariance = Eigen::Matrix<double, DimX, DimX>;

  error_covariance_estimation_t(ros::NodeHandle nh)
  {
    // PRX FILES
    std::string plant_parameters;

    using prx::simulation_step;

    int& max_samples{ _max_samples };
    PARAM_SETUP_WITH_DEFAULT(nh, max_samples, 100);

    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(simulation_step);

    _plant = std::make_shared<DynamicalSystem>(plant_parameters);
  }

  ~error_covariance_estimation_t()
  {
  }

  Error residual(const State x, const State xp)
  {
    const State xbtw{ gtsam::traits<State>::Between(x, xp) };
    const Error error{ gtsam::traits<State>::Logmap(xbtw) };
    return error;
  }

  State from_observation(const State x1, const State x0, const Control u0, const double dt, const double dtz)
  {
  }

  void add_estimate(const State x1hat, const State& x0, const Control u0, const double dt)
  {
    const State x1{ _plant->propagate(x0, u0, dt) };
    const Error wi{ residual(x1, x1hat) };

    _inc_covariance.update_covariance_values(wi, /** increment **/ true);
    _errors.push_back(wi);

    if (_errors.size() > _max_samples)
    {
      _inc_covariance.update_covariance_values(_errors.front(), /** increment **/ false);
      _errors.erase(_errors.begin());
    }
  }

  Eigen::Matrix<double, DimX, DimX> covariance() const
  {
    return _inc_covariance.compute_covariance();
  }

private:
  incremental_covariance_t<DimX> _inc_covariance;
  // void update_covariance_values(const Error w_update, const bool increment)
  // {
  //   if (increment)
  //   {
  //     _A += w_update * w_update.transpose();
  //     _b += w_update;
  //     _sample_mean = (1. / (_total_samples + 1)) * _b;
  //     _total_samples++;
  //   }
  //   else
  //   {
  //     _A -= w_update * w_update.transpose();
  //     _b -= w_update;
  //     _sample_mean = (1. / _total_samples) * _b;
  //     _total_samples--;
  //   }
  // }

  // Eigen::Matrix<double, DimX, DimX> compute_covariance() const
  // {
  //   const double rate{ 1. / _total_samples };

  //   return rate * (_A -                             // no-lint
  //                  _sample_mean * _b.transpose() -  // no-lint
  //                  _b * _sample_mean.transpose() +  // no-lint
  //                  _total_samples * _sample_mean * _sample_mean.transpose());
  // }

  int _max_samples;

  // Eigen::Matrix<double, DimX, DimX> _A;
  // Eigen::Vector<double, DimX> _b;
  // Eigen::Vector<double, DimX> _sample_mean;

  // int _total_samples;

  bool _visualize;

  std::shared_ptr<DynamicalSystem> _plant;

  std::vector<Error> _errors;
  // State _state;

  // std::vector<std::shared_ptr<prx::geometry_t>> _system_geoms;
  // Query:
  //   PQP_CollideResult collision_result;
  //   std::vector<std::shared_ptr<PQP_Model>> pqp_models;
  //   std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> plant_configurations;
  // std::vector<std::shared_ptr<CollisionQuery>> _queries;
  // std::vector<std::shared_ptr<prx::collision_checking::pqp::rigid_body_t>> _obstacles_bodies;

  // int _total_threads, _half_threads;
  // BS::thread_pool<BS::tp::pause | BS::tp::priority> _pool;

  // StateSampler _x0_sampler;
  // StateSampler _w_sampler;
};
}  // namespace motion_planning
