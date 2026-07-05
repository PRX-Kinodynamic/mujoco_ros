#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>

#include <iterator>
#include <memory>
#include <prx/simulation/forward_propagation.hpp>
#include <prx/utilities/math/chi_squared.hpp>
#include <prx/utilities/math/lie_utils.hpp>
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
#include <CGAL/Polygon_mesh_processing/measure.h>

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_2.h>

namespace motion_planning
{

template <typename DynamicalSystem, typename Controller>
class gotube_t
{
public:
  using TrajectoryMsg = std::vector<ml4kp_bridge::SpacePointStamped>;
  using PlanMsg = ml4kp_bridge::PlanStepStampedArray;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Trajectory = std::vector<State>;

  static constexpr int DimX{ gtsam::traits<State>::dimension };
  using Tangent = Eigen::Vector<double, DimX>;

  using FwdProp = prx::forward_propagation_t<DynamicalSystem, Trajectory, Controller>;

  using StateSampler = prx::lie_group_gaussian_noise_t<State>;
  using StateDotSampler = prx::multivariate_gaussian_t<DimX>;
  using Covariance = typename StateSampler::Covariance;

  using CollisionQuery = prx::collision_checking::pqp::query_t;

  // using Polyhedron = CGAL::Polyhedron_3<cgal_bridge::CgalEpicKernel>;

  gotube_t(ros::NodeHandle nh)
  {
    // PRX FILES
    std::string environment;
    std::string plant_parameters;

    // PRX PARAM LOADERS FOR PRX FILES
    prx::param_loader env_params;

    using prx::simulation_step;

    int& total_threads{ _total_threads };
    bool& visualize{ _visualize };
    bool& short_circuit{ _short_circuit };
    int& ball_step{ _ball_step };
    // int& convex_hulls_step{ _convex_hulls_step };

    // double chi_tolerance{ 0.05 };  // 1 - 0.95
    // PARAM_SETUP(nh, convex_hulls_step);

    // PARAM_SETUP_WITH_DEFAULT(nh, chi_tolerance, chi_tolerance)
    PARAM_SETUP_WITH_DEFAULT(nh, ball_step, 10)
    PARAM_SETUP_WITH_DEFAULT(nh, total_threads, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, visualize, true);
    PARAM_SETUP_WITH_DEFAULT(nh, short_circuit, true);

    GLOBAL_PARAM_BLOCKER(environment);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(simulation_step);

    // _chi2 = std::make_shared<prx::chi_squared>(chi_tolerance);
    _pool.reset(_total_threads);
    _half_threads = std::max(static_cast<int>(_total_threads / 2.0), 1);

    DEBUG_VARS(_pool.get_thread_count())

    env_params.from_string(environment);
    // plant_params.from_string(plant_parameters);

    // _plant = std::make_shared<DynamicalSystem>(plant_parameters);
    _plant = DynamicalSystem::create(plant_parameters);

    _obstacles_bodies = prx::collision_checking::pqp::create_obstacles(env_params);

    _system_geoms = _plant->geometries();

    _center_traj_publisher = nh.advertise<visualization_msgs::Marker>("/gotube/trajectories/center/marker", 1);
    // _collision_publisher = nh.advertise<std_msgs::Bool>("/gotube/collision", 1);
    _balls_publisher = nh.advertise<visualization_msgs::MarkerArray>("/gotube/balls", 1);
    // _end_points_publisher = nh.advertise<visualization_msgs::MarkerArray>("/gotube/end_points", 1);
    // _collision_markers_publisher = nh.advertise<visualization_msgs::Marker>("/gotube/collisions/marker", 1);

    std::string output_directory, file_prefix;
    PARAM_SETUP_WITH_DEFAULT(nh, output_directory, "/tmp/");
    PARAM_SETUP_WITH_DEFAULT(nh, file_prefix, "gotube");

    const std::string timestamp{ utils::timestamp() };
    const std::string OUTPUT_FILE{ output_directory + "/" + file_prefix + "_volumes_" + timestamp + ".txt" };
    const std::string CONVEX_HULL_OUTPUT_FILE{ output_directory + "/" + file_prefix + "_convex_hulls_" + timestamp +
                                               ".txt" };

    DEBUG_VARS(OUTPUT_FILE)
    DEBUG_VARS(CONVEX_HULL_OUTPUT_FILE)
    _ofs.open(OUTPUT_FILE);
    _ofs_convex_hulls.open(CONVEX_HULL_OUTPUT_FILE);
  }

  ~gotube_t()
  {
  }

  // const double distribution_bound(const Covariance& cov)
  // {
  //   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
  //   const Tangent D_marginal{ es.eigenvalues().cwiseSqrt() * _Chi2_confidence };
  //   const double C_bound{ D_marginal.maxCoeff() };
  //   return std::pow(C_bound, 2);
  // }

  void collion_check()
  {
    if (_short_circuit and _collision_found)  // short-circuit
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
        const bool new_q{ 1 };
      }
      else
      {
        query = _queries.back();
        _queries.pop_back();
        const bool new_q{ 0 };
      }
    }

    bool collision{ false };
    // Check only the states inside the convex hull
    // The original gotube paper is unclear about this... But the RRT-gotube paper seems to only check the convex hull
    // A conservative approach is to check every state in the trajectory, but the point of the convex hull is to speed
    // this up However, computing convex hull and then checking collisions is problematic because the convex hull
    // computation is expensive and done at the end
    // So... The compromise here is to check only those states that will be used to construct the convex hull.
    // for (auto&& state : traj)
    for (int state_idx = 0; state_idx < traj.size(); ++state_idx)
    {
      const State& state{ traj[state_idx] };
      query->plant_configurations = _plant->configuration(state);

      collision = prx::collision_checking::pqp::collision(*query, _obstacles_bodies);
      if (collision)
      {
        _colliding_states.push_back(state);
        break;
      }
    }

    for (int bi = 0; bi < traj.size(); bi += _ball_step)
    {
      const State& ci{ _center_traj[bi] };
      const State& xi{ traj[bi] };
      const Tangent tgi{ prx::TangentBetween(ci, xi) };
      const double error{ tgi.norm() };

      std::scoped_lock lock(_rads_mutex[bi]);
      _max_rads[bi] = std::max(_max_rads[bi], error);
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
    if (_short_circuit and _collision_found)  // short-circuit
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

  void init_query(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
                  const Covariance& cov_w)
  {
    _convex_hull_volumes.clear();
    _colliding_states.clear();

    // copy(_controller, plan_in);
    // copy(_state, x_hat);
    ml4kp_bridge::copy(_controller, plan_in);
    ml4kp_bridge::copy(_state, x_hat);

    _x0_sampler.set(cov_x0);
    _w_sampler.set(cov_w);

    _collision_found = false;

    if (_visualize)
    {
      for (auto&& marker : _marker_balls.markers)
      {
        marker.action = visualization_msgs::Marker::DELETEALL;
      }
      _balls_publisher.publish(_marker_balls);
    }

    _marker_balls.markers.clear();
    _marker_pts.markers.clear();

    _center_traj.clear();
    _max_rads.clear();
    _rads_mutex.clear();
    FwdProp::propagate(_center_traj, _state, _controller, _plant);

    _rads_mutex = std::vector<std::mutex>(_center_traj.size());
    _max_rads.resize(_center_traj.size(), 0.);
    // for (int i = 0; i < _center_traj.size(); ++i)
    // {
    //   _rads_mutex.emplace_back();
    //   _max_rads.push_back(0.);
    // }
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
               const Covariance& cov_w, const std::chrono::time_point<std::chrono::steady_clock>& limit)
  {
    init_query(x_hat, plan_in, cov_x0, cov_w);
    const std::size_t total_threads{ _pool.get_thread_count() };

    while (std::chrono::steady_clock::now() < limit)
    {
      if (_short_circuit and _collision_found)  // short-circuit
      {
        break;
      }

      if (_pool.get_tasks_total() < total_threads)
      {
        _pool.detach_task([&] { this->propagate(); });
      }
    }
    // compute_convex_hulls();
    _collision_msg.data = _collision_found;
    // _collision_publisher.publish(_collision_msg);
    _pool.detach_task([&] { this->trajectories_to_marker(); });
    return _collision_found;
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
               const Covariance& cov_w, const int total_trajectories)
  {
    init_query(x_hat, plan_in, cov_x0, cov_w);

    ros::Time start{ ros::Time::now() };
    for (int i = 0; i < total_trajectories; ++i)
    {
      _pool.detach_task([&] { this->propagate(); });
    }
    _pool.wait();

    // compute_convex_hulls();

    const ros::Time end{ ros::Time::now() };
    const double gotube_time{ (end - start).toSec() };
    const bool collision{ _collision_found };

    // _convex_hull_volumes.back().push_back(convex_hull_volume);

    _ofs << total_trajectories << " ";
    _ofs << collision << " ";
    _ofs << gotube_time << " ";
    for (auto&& vol : _convex_hull_volumes)
    {
      _ofs << vol << " ";
    }
    _ofs << "\n";
    // VARS_TO_STREAM(_ofs, gotube_time, collision);

    _collision_msg.data = _collision_found;
    // _collision_publisher.publish(_collision_msg);
    _pool.detach_task([&] { this->trajectories_to_marker(); });

    return _collision_found;
  }

  void trajectories_to_marker()
  {
    if (_visualize)
    {
      visualization_msgs::Marker center_marker{ ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 }) };
      center_marker.type = visualization_msgs::Marker::LINE_LIST;
      center_marker.action = visualization_msgs::Marker::DELETEALL;
      // _trajectory_markers.markers.push_back(marker);
      _center_traj_publisher.publish(center_marker);

      // _trajectory_markers.markers.clear();
      center_marker.action = visualization_msgs::Marker::ADD;

      ml4kp_bridge::update_marker(center_marker, _center_traj, 0, 1, -0.1);
      _center_traj_publisher.publish(center_marker);

      // for (auto& r : _max_rads)
      // DEBUG_VARS(_max_rads)
      for (int i = 0; i < _max_rads.size(); ++i)
      {
        const double& r{ _max_rads[i] };
        visualization_msgs::Marker ball_marker{ ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 }) };
        ball_marker.type = visualization_msgs::Marker::SPHERE;
        ball_marker.id = i;
        ball_marker.scale.x = r;
        ball_marker.scale.y = r;
        ball_marker.scale.z = r;

        ball_marker.color.a = 0.5;
        ball_marker.color.r = 0.92;
        ball_marker.color.g = 0.91;
        ball_marker.color.b = 0.1;

        const State& xc{ _center_traj[i] };
        ball_marker.points.emplace_back();
        ml4kp_bridge::update_pose(ball_marker.pose, xc, 0, 1, 0.0);

        DEBUG_VARS(i, r, xc)

        _marker_balls.markers.push_back(ball_marker);
      }

      // for (auto&& state : _colliding_states)
      // {
      //   collision_marker.points.emplace_back();
      //   ml4kp_bridge::update_point(collision_marker.points.back(), state, 0, 1, 0.0);
      // }
      _balls_publisher.publish(_marker_balls);
      // _collision_markers_publisher.publish(_marker_balls);
      // _trajectory_markers.markers.clear();
    }

    // std::scoped_lock lock(_checked_trajectories_mutex);
    // _checked_trajectories.clear();
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

  visualization_msgs::MarkerArray _marker_balls, _marker_pts;

  Controller _controller;

  std_msgs::Bool _collision_msg;
  // ros::Publisher _center_traj_publisher, _collision_publisher, _collision_markers_publisher;
  // ros::Publisher _end_points_publisher, _balls_publisher;
  ros::Publisher _center_traj_publisher, _balls_publisher;

  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::world_model_t> _planning_model;
  std::shared_ptr<prx::collision_group_t> _collision_group;

  // visualization_msgs::MarkerArray _trajectory_markers;
  // visualization_msgs::MarkerArray _prev_trajectory_markers;
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
  StateDotSampler _w_sampler;

  bool _short_circuit;
  int _convex_hulls_step;

  std::ofstream _ofs, _ofs_convex_hulls;

  std::vector<State> _colliding_states;
  std::vector<double> _convex_hull_volumes;

  std::vector<std::mutex> _rads_mutex;
  std::vector<double> _max_rads;
  int _ball_step;
  Trajectory _center_traj;
  // double _Chi2_confidence;
  // std::shared_ptr<prx::chi_squared> _chi2;
};
}  // namespace motion_planning
