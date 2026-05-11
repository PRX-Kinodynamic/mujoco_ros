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
#include "prx/external/thread_pool/BS_thread_pool.hpp"

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>

namespace motion_planning
{

template <typename DynamicalSystem, typename Controller>
class randup_t
{
public:
  using TrajectoryMsg = std::vector<ml4kp_bridge::SpacePointStamped>;
  using PlanMsg = ml4kp_bridge::PlanStepStampedArray;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Trajectory = std::vector<State>;

  using FwdProp = prx::forward_propagation_t<DynamicalSystem, Trajectory, Controller>;

  using StateSampler = prx::lie_group_gaussian_noise_t<State>;
  using Covariance = typename StateSampler::Covariance;

  using CollisionQuery = prx::collision_checking::pqp::query_t;

  randup_t(ros::NodeHandle nh)
  {
    // PRX FILES
    std::string environment;
    std::string plant_parameters;

    // PRX PARAM LOADERS FOR PRX FILES
    prx::param_loader plant_params, env_params;

    using prx::simulation_step;

    int& total_threads{ _total_threads };
    bool& visualize{ _visualize };

    PARAM_SETUP_WITH_DEFAULT(nh, total_threads, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, visualize, true);

    GLOBAL_PARAM_BLOCKER(environment);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(simulation_step);

    _pool.reset(_total_threads);
    _half_threads = std::max(static_cast<int>(_total_threads / 2.0), 1);

    env_params.from_string(environment);
    plant_params.from_string(plant_parameters);

    // DEBUG_VARS(plant_params);
    _plant = std::make_shared<DynamicalSystem>(plant_params);

    _obstacles_bodies = prx::collision_checking::pqp::create_obstacles(env_params);

    _system_geoms = _plant->geometries();

    _markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/randup/trajectories/marker", 1);
    _collision_publisher = nh.advertise<std_msgs::Bool>("/randup/collision", 1);
  }

  ~randup_t()
  {
  }

  void collion_check()
  {
    if (_collision_found)  // short-circuit
      return;

    Trajectory traj;
    {
      std::scoped_lock lock(_trajectories_mutex);
      traj = _trajectories.back();
      _trajectories.pop_back();
    }

    std::shared_ptr<CollisionQuery> query;
    {
      std::scoped_lock lock(_queries_mutex);
      if (_queries.size() == 0)
      {
        query = std::make_shared<CollisionQuery>();
        query->pqp_models = prx::collision_checking::pqp::create_pqp_models(_system_geoms);
      }
      else
      {
        query = _queries.back();
        _queries.pop_back();
      }
    }

    bool collision{ false };
    for (auto&& state : traj)
    {
      query->plant_configurations = _plant->configuration(state);

      collision = prx::collision_checking::pqp::collision(*query, _obstacles_bodies);
      if (collision)
        break;
    }

    {
      std::scoped_lock lock(_queries_mutex);
      _queries.push_back(query);
    }

    _collision_found = _collision_found or collision;
    std::scoped_lock lock(_checked_trajectories_mutex);
    _checked_trajectories.push_back(traj);
  }

  void propagate()
  {
    if (_collision_found)  // short-circuit
      return;
    const State x0_noise{ _x0_sampler(_state) };
    Trajectory traj;
    FwdProp::propagate(traj, x0_noise, _controller, _plant, _w_sampler);

    {
      std::scoped_lock lock(_trajectories_mutex);
      _trajectories.push_back(traj);
      _unchecked_trajectories++;
    }

    _pool.detach_task([&] { this->collion_check(); }, BS::pr::highest);
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& x0,
               const Covariance& w, const std::chrono::time_point<std::chrono::steady_clock>& limit)
  {
    copy(_controller, plan_in);
    copy(_state, x_hat);

    _x0_sampler.set(x0);
    _w_sampler.set(w);

    _collision_found = false;

    const std::size_t total_threads{ _pool.get_thread_count() };

    while (std::chrono::steady_clock::now() < limit)
    {
      if (_collision_found)
      {
        break;
      }

      if (_pool.get_tasks_total() < total_threads)
      {
        _pool.detach_task([&] { this->propagate(); });
      }
    }

    _collision_msg.data = _collision_found;
    _collision_publisher.publish(_collision_msg);
    _pool.detach_task([&] { this->trajectories_to_marker(); });
    return _collision_found;
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
               const Covariance& cov_w, const int total_trajectories)
  {
    ml4kp_bridge::copy(_controller, plan_in);
    ml4kp_bridge::copy(_state, x_hat);

    _x0_sampler.set(cov_x0);
    _w_sampler.set(cov_w);

    _collision_found = false;
    for (int i = 0; i < total_trajectories; ++i)
    {
      _pool.detach_task([&] { this->propagate(); });
    }
    _pool.wait();

    _collision_msg.data = _collision_found;
    _collision_publisher.publish(_collision_msg);
    _pool.detach_task([&] { this->trajectories_to_marker(); });

    return _collision_found;
  }

  void trajectories_to_marker()
  {
    if (_visualize)
    {
      visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 }) };
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::DELETEALL;
      _trajectory_markers.markers.push_back(marker);
      _markers_publisher.publish(_trajectory_markers);

      _trajectory_markers.markers.clear();
      marker.action = visualization_msgs::Marker::ADD;
      std::scoped_lock lock(_checked_trajectories_mutex);
      while (_checked_trajectories.size() > 0)
      {
        ml4kp_bridge::update_marker(marker, _checked_trajectories.back(), 0, 1, 0.0);
        _checked_trajectories.pop_back();
        _trajectory_markers.markers.push_back(marker);
      }

      DEBUG_VARS(_trajectory_markers.markers.size())
      _markers_publisher.publish(_trajectory_markers);
      _trajectory_markers.markers.clear();
    }

    std::scoped_lock lock(_checked_trajectories_mutex);
    _checked_trajectories.clear();
  }

private:
  bool _visualize;
  std::atomic<int> _trajectories_to_viz;

  std::atomic<bool> _collision_found;
  std::atomic<int> _total_checked_trajectories;
  std::atomic<int> _unchecked_trajectories, _collisions_in_check;
  std::mutex _trajectories_mutex, _checked_trajectories_mutex, _queries_mutex;
  std::vector<Trajectory> _trajectories;
  std::vector<Trajectory> _checked_trajectories;

  Controller _controller;

  std_msgs::Bool _collision_msg;
  ros::Publisher _markers_publisher, _collision_publisher;

  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::world_model_t> _planning_model;
  std::shared_ptr<prx::collision_group_t> _collision_group;

  visualization_msgs::MarkerArray _trajectory_markers;
  visualization_msgs::MarkerArray _prev_trajectory_markers;
  //
  std::shared_ptr<DynamicalSystem> _plant;

  State _state;

  std::vector<std::shared_ptr<prx::geometry_t>> _system_geoms;
  // Query:
  //   PQP_CollideResult collision_result;
  //   std::vector<std::shared_ptr<PQP_Model>> pqp_models;
  //   std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> plant_configurations;
  std::vector<std::shared_ptr<CollisionQuery>> _queries;
  std::vector<std::shared_ptr<prx::collision_checking::pqp::rigid_body_t>> _obstacles_bodies;

  int _total_threads, _half_threads;
  BS::thread_pool<BS::tp::pause | BS::tp::priority> _pool;

  StateSampler _x0_sampler;
  StateSampler _w_sampler;
};
}  // namespace motion_planning
