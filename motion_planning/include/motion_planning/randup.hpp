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
#include <CGAL/Polygon_mesh_processing/measure.h>

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_2.h>
namespace cgal_bridge
{
using CgalEpicKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
inline CgalEpicKernel::Point_3 cgal_create(const gtsam::ProductLieGroupV43<gtsam::Rot2, double>& state)
{
  // const Eigen::Vector2d tg{ gtsam::traits<gtsam::ProductLieGroupV43<gtsam::Rot2, double>>::Logmap(state) };
  const double x{ state.first.theta() };
  const double y{ state.second };

  // CgalEpicKernel::Point_3 pt(tg[0], tg[1], 0.);
  CgalEpicKernel::Point_3 pt(x, y, 0.);

  return pt;
}

inline CgalEpicKernel::Point_3 cgal_create(const gtsam::ProductLieGroupV43<gtsam::Pose2, Eigen::Vector3d>& state)
{
  const double x{ state.first.x() };
  const double y{ state.first.y() };
  const double z{ 0. };
  CgalEpicKernel::Point_3 pt(x, y, 0.);
  return pt;
}

}  // namespace cgal_bridge

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

  using Polyhedron = CGAL::Polyhedron_3<cgal_bridge::CgalEpicKernel>;

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
    bool& short_circuit{ _short_circuit };

    int& convex_hulls_step{ _convex_hulls_step };

    PARAM_SETUP(nh, convex_hulls_step);

    PARAM_SETUP_WITH_DEFAULT(nh, total_threads, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, visualize, true);
    PARAM_SETUP_WITH_DEFAULT(nh, short_circuit, true);

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

    _markers_publisher = nh.advertise<visualization_msgs::Marker>("/randup/trajectories/marker", 1);
    _collision_publisher = nh.advertise<std_msgs::Bool>("/randup/collision", 1);
    _convex_hull_publisher = nh.advertise<visualization_msgs::MarkerArray>("/randup/convex_hull", 1);
    _end_points_publisher = nh.advertise<visualization_msgs::MarkerArray>("/randup/end_points", 1);

    std::string output_directory;
    PARAM_SETUP_WITH_DEFAULT(nh, output_directory, "/tmp/");

    const std::string timestamp{ utils::timestamp() };
    _ofs.open(output_directory + "/volumes_" + timestamp);
  }

  ~randup_t()
  {
  }

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
      }
      else
      {
        query = _queries.back();
        _queries.pop_back();
      }
    }

    bool collision{ false };
    // Check only the states inside the convex hull
    // The original Randup paper is unclear about this... But the RRT-randup paper seems to only check the convex hull
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

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& x0,
               const Covariance& w, const std::chrono::time_point<std::chrono::steady_clock>& limit)
  {
    copy(_controller, plan_in);
    copy(_state, x_hat);

    _x0_sampler.set(x0);
    _w_sampler.set(w);

    _collision_found = false;

    _marker_convex_hull.markers.clear();
    _marker_pts.markers.clear();

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
    compute_convex_hulls();
    _collision_msg.data = _collision_found;
    _collision_publisher.publish(_collision_msg);
    _pool.detach_task([&] { this->trajectories_to_marker(); });
    return _collision_found;
  }

  // Get the area of a convex hull (Area~=Volume) for 2D system.
  // If the system is 3D, use CGAL::Polygon_mesh_processing::volume(poly) ?
  double convex_hull_volume(const Polyhedron& poly)
  {
    // for (auto&& v_pt : poly.points().begin() + 3)
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Point_2<cgal_bridge::CgalEpicKernel> Point2;
    typedef CGAL::Polygon_2<K> Polygon_2;

    Polygon_2 poly2;
    for (auto&& v : poly.points())
    {
      poly2.push_back({ v.x(), v.y() });
    }
    return poly2.area();
    // auto A = *(poly.points().begin());
    // auto B = *(poly.points().begin() + 1);

    // for (auto iter = poly.points().begin() + 2; iter != poly.points().end(); iter++)
    // {

    //   CGAL::area(const CGAL::Point_2<Kernel>& p, const CGAL::Point_2<Kernel>& q, const CGAL::Point_2<Kernel>& r);
    // }
  }

  double convex_hull(std::vector<cgal_bridge::CgalEpicKernel::Point_3>& points, const std::string idx)
  {
    Polyhedron poly;
    CGAL::convex_hull_3(points.begin(), points.end(), poly);
    const double volume{ convex_hull_volume(poly) };
    // CGAL::Polygon_mesh_processing::volume(poly);
    // DEBUG_VARS(idx, volume)

    _marker_convex_hull.markers.push_back(ml4kp_bridge::create_marker(0.01, { 1, 0, 1, 0 }));
    visualization_msgs::Marker& marker_ch{ _marker_convex_hull.markers.back() };
    marker_ch.type = visualization_msgs::Marker::LINE_LIST;
    marker_ch.ns = "convex_hull_" + idx;

    bool first{ true };
    cgal_bridge::CgalEpicKernel::Point_3 pt_prev{ *(poly.points().begin()) };
    for (auto&& v_pt : poly.points())
    {
      marker_ch.points.emplace_back();

      marker_ch.points.back().x = v_pt.x();
      marker_ch.points.back().y = v_pt.y();
      marker_ch.points.back().z = v_pt.z();
      if (not first)
      {
        marker_ch.points.push_back(marker_ch.points.back());
      }
      first = false;
      pt_prev = v_pt;
    }
    if (not first)
    {
      marker_ch.points.push_back(marker_ch.points.back());
      marker_ch.points.push_back(marker_ch.points.back());
      marker_ch.points.push_back(marker_ch.points.front());
    }
    return volume;
  }

  void divide_convex_hulls(std::vector<cgal_bridge::CgalEpicKernel::Point_3>& pts0,
                           std::vector<cgal_bridge::CgalEpicKernel::Point_3>& pts1)
  {
    auto prev = *(pts0.begin());
    std::vector<cgal_bridge::CgalEpicKernel::Point_3> pts_aux;
    for (auto pt : pts0)
    {
      const double diff{ std::fabs(pt.x() - prev.x()) };
      if (diff > 3.)
      {
        pts1.push_back(pt);
      }
      else
      {
        pts_aux.push_back(pt);
      }

      // prev = pt;
    }
    std::swap(pts_aux, pts0);
  }

  void compute_convex_hulls()
  {
    PRINT_MSG("Computing convex hull")
    // typedef K::Point_3                                Point_3;

    using Polyhedron_3 = CGAL::Polyhedron_3<cgal_bridge::CgalEpicKernel>;

    bool are_new_states{ true };
    int i{ 0 };
    int state_idx{ 0 };
    std::vector<Trajectory> _checked_trajectories_aux;

    const std::size_t total_trajectories{ _checked_trajectories.size() };
    while (are_new_states)
    {
      DEBUG_VARS(i, state_idx)
      const std::string idx{ prx::utilities::convert_to<std::string>(i) };
      are_new_states = false;
      std::vector<cgal_bridge::CgalEpicKernel::Point_3> cgal_points_0, cgal_points_1;

      visualization_msgs::Marker marker_pt{ ml4kp_bridge::create_marker(0.1, { 1, 0, 1, 0 }) };
      marker_pt.type = visualization_msgs::Marker::POINTS;
      marker_pt.ns = "pts_" + idx;
      for (int traj_idx = 0; traj_idx < total_trajectories; ++traj_idx)
      {
        if (_checked_trajectories[traj_idx].size() > state_idx)
        {
          const State& state{ _checked_trajectories[traj_idx][state_idx] };
          cgal_points_0.push_back(cgal_bridge::cgal_create(state));

          marker_pt.points.emplace_back();
          marker_pt.points.back().x = cgal_points_0.back().x();
          marker_pt.points.back().y = cgal_points_0.back().y();
          marker_pt.points.back().z = cgal_points_0.back().z();

          are_new_states = true;
        }
      }
      if (cgal_points_0.size() > 0)
      {
        double convex_hull_volume{ 0. };
        divide_convex_hulls(cgal_points_0, cgal_points_1);
        convex_hull_volume += convex_hull(cgal_points_0, idx);
        if (cgal_points_1.size() > 0)
        {
          convex_hull_volume += convex_hull(cgal_points_1, idx + "'");
        }
        _marker_pts.markers.push_back(marker_pt);
        VARS_TO_STREAM(_ofs, idx, convex_hull_volume, total_trajectories);
      }
      i++;
      state_idx += _convex_hulls_step;
    }
    // DEBUG_VARS(points)

    // CGAL::convex_hull_3(points.begin(), points.end(), poly);
    // DEBUG_VARS(poly.points().size())

    _convex_hull_publisher.publish(_marker_convex_hull);
    _end_points_publisher.publish(_marker_pts);
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
               const Covariance& cov_w, const int total_trajectories)
  {
    ml4kp_bridge::copy(_controller, plan_in);
    ml4kp_bridge::copy(_state, x_hat);

    _x0_sampler.set(cov_x0);
    _w_sampler.set(cov_w);

    _marker_convex_hull.markers.clear();
    _marker_pts.markers.clear();
    _collision_found = false;
    for (int i = 0; i < total_trajectories; ++i)
    {
      _pool.detach_task([&] { this->propagate(); });
    }
    _pool.wait();

    compute_convex_hulls();

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
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::DELETEALL;
      // _trajectory_markers.markers.push_back(marker);
      _markers_publisher.publish(marker);

      // _trajectory_markers.markers.clear();
      marker.action = visualization_msgs::Marker::ADD;
      std::scoped_lock lock(_checked_trajectories_mutex);

      DEBUG_VARS(_checked_trajectories.size())
      while (_checked_trajectories.size() > 0)
      {
        ml4kp_bridge::update_marker(marker, _checked_trajectories.back(), 0, 1, 0.0);
        _checked_trajectories.pop_back();
        // _trajectory_markers.markers.push_back(marker);
      }

      _markers_publisher.publish(marker);
      // _trajectory_markers.markers.clear();
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

  visualization_msgs::MarkerArray _marker_convex_hull, _marker_pts;

  Controller _controller;

  std_msgs::Bool _collision_msg;
  ros::Publisher _markers_publisher, _collision_publisher;
  ros::Publisher _end_points_publisher, _convex_hull_publisher;

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
  StateSampler _w_sampler;

  bool _short_circuit;
  int _convex_hulls_step;

  std::ofstream _ofs;
};
}  // namespace motion_planning
