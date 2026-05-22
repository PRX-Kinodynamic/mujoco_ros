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

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>

namespace motion_planning
{
template <typename State>
struct mg_cell_t
{
  mg_cell_t() : added_idx(0), safe(false) {};

  std::mutex mutex;
  bool safe;
  std::size_t added_idx;
  State state;
};

template <typename DynamicalSystem, typename Controller>
class morse_graph_reachability_t
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

  using CellPtr = std::shared_ptr<mg_cell_t<State>>;
  using ImplicitGrid = prx::implicit_grid_t<State, CellPtr>;
  using Tangent = typename ImplicitGrid::TangentElement;

  morse_graph_reachability_t(ros::NodeHandle nh) : _iter_idx(0), _short_circuit(true)
  {
    // PRX FILES
    std::string environment;
    std::string plant_parameters;

    // PRX PARAM LOADERS FOR PRX FILES
    prx::param_loader plant_params, env_params;

    using prx::simulation_step;

    int& total_threads{ _total_threads };
    bool& visualize{ _visualize };

    double& cell_size{ _cell_size };
    bool& short_circuit{ _short_circuit };

    PARAM_SETUP(nh, cell_size);
    PARAM_SETUP_WITH_DEFAULT(nh, total_threads, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, short_circuit, true);
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

    _markers_publisher = nh.advertise<visualization_msgs::Marker>("/mg/trajectories/marker", 1);
    _collision_publisher = nh.advertise<std_msgs::Bool>("/mg/collision", 1);
    _cubes_algebra_publisher = nh.advertise<visualization_msgs::Marker>("/mg/cubes/lie_algebra", 1);
    _cubes_state_publisher = nh.advertise<visualization_msgs::Marker>("/mg/cubes/state_space", 1);
  }

  ~morse_graph_reachability_t()
  {
  }

  void collion_check(const State state)
  {
    if (_short_circuit and _collision_found)  // short-circuit
      return;

    CellPtr cellptr{ init_cell(state) };
    // _grid.cell(state) };

    {
      std::scoped_lock lock(cellptr->mutex);
      if (cellptr->added_idx == _iter_idx)  // This cell's safety has been checked
        return;
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
    const Tangent center_tg{ _grid.center(state) };
    const State center{ _grid.state(center_tg) };
    query->plant_configurations = _plant->configuration(center);

    const bool collision{ prx::collision_checking::pqp::collision(*query, _obstacles_bodies) };
    std::scoped_lock lock(cellptr->mutex);
    if (collision)
    {
      cellptr->safe = false;
      _collision_found = true;
      // DEBUG_VARS(collision)
    }
    cellptr->state = state;
    cellptr->added_idx = _iter_idx;  // This cell's safety has been checked
  }

  CellPtr init_cell(const State& state)
  {
    std::scoped_lock lock(_new_cell_mutex);
    if (_grid.cell(state) == nullptr)
    {
      // Need a second one in case the cell was init before acquiring the lock
      if (_cells_buffer.empty())
      {
        for (int i = 0; i < 100; ++i)
        {
          _cells_buffer.push_back(std::make_shared<mg_cell_t<State>>());
        }
      }
      _grid.cell(state) = _cells_buffer.back();
      _used_cells.push_back(_cells_buffer.back());
      _cells_buffer.pop_back();
    }
    return _grid.cell(state);
  }

  void propagate(const State state)
  {
    Trajectory traj;
    FwdProp::propagate(traj, state, _controller, _plant);
    std::set<std::size_t> hashes;
    for (auto state : traj)
    {
      const std::size_t h{ _grid.hash(state) };
      if (hashes.count(h) == 0)
      {
        // LOG_VARS(state, h);
        hashes.insert(h);
        _pool.detach_task([state, this] { this->collion_check(state); });
      }
    }

    std::scoped_lock lock(_trajectories_mutex);
    _trajectories.push_back(traj);
    // _unchecked_trajectories++;
  }

  void propagate_cube(const State& state)
  {
    // if (_collision_found)  // short-circuit
    //   return;
    // const State x0_noise{ _x0_sampler(_state) };

    auto vertices = _grid.vertices(state);
    for (auto v : vertices)
    {
      const State xv{ _grid.state(v) };
      // const State xv{ gtsam::traits<State>::Expmap(v) };
      // DEBUG_VARS(v, xv)
      _pool.detach_task([xv, this] { this->propagate(xv); });
    }
    // {
    //   std::scoped_lock lock(_trajectories_mutex);
    //   _trajectories.push_back(traj);
    //   _unchecked_trajectories++;
    // }

    // _pool.detach_task([&] { this->collion_check(); }, BS::pr::highest);
  }

  const Tangent lipschitz(const State& x0, const State& x0p, const State& xF, const State& xFp)
  {
    const State xBtw_0{ gtsam::traits<State>::Between(x0, x0p) };
    const State xBtw_F{ gtsam::traits<State>::Between(xF, xFp) };

    const Tangent tg_0{ gtsam::traits<State>::Logmap(xBtw_0) };
    const Tangent tg_F{ gtsam::traits<State>::Logmap(xBtw_F) };

    const Tangent L{ tg_F.cwiseQuotient(tg_0).cwiseAbs() };
    // DEBUG_VARS(x0, xF)
    // DEBUG_VARS(x0p, xFp)
    // DEBUG_VARS(xBtw_0, xBtw_F)
    // DEBUG_VARS(tg_0, tg_F)
    return L;
  }

  Tangent grid_cell_size(const State& x0, const Controller u)
  {
    Trajectory traj0, traj1;

    const State x0p{ _x0_sampler(x0) };
    // DEBUG_VARS(x0, x0p)

    // Assuming cov of x0_noise != 0. Need to enforce small eps if it is zero
    FwdProp::propagate(traj0, x0, _controller, _plant);
    FwdProp::propagate(traj1, x0p, _controller, _plant);

    Tangent lip_greater{ Tangent::Zero() };
    for (int i = 0; i < traj0.size() - 1; ++i)
    {
      const State& x0{ traj0[i] };
      const State& xF{ traj0[i + 1] };
      const State& x0p{ traj1[i] };
      const State& xFp{ traj1[i + 1] };
      const Tangent li{ lipschitz(x0, x0p, xF, xFp) };

      if (li.norm() > lip_greater.norm())
      {
        // DEBUG_VARS(li, lip_greater)
        lip_greater = li;
      }
    }
    // DEBUG_VARS(lip_greater)
    // Tangent cell_size{ Tangent::Ones() * _cell_size };
    return Tangent::Ones() * _cell_size;
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& x0_noise,
               const Covariance& w, const std::chrono::time_point<std::chrono::steady_clock>& limit)
  {
    copy(_controller, plan_in);
    copy(_state, x_hat);

    _x0_sampler.set(x0_noise);
    _w_sampler.set(w);

    std::swap(_cells_buffer, _used_cells);

    const Tangent cell_size{ grid_cell_size(_state, _controller) };
    _grid.reset(_state, cell_size);

    _collision_found = false;
    _iter_idx++;

    const std::size_t total_threads{ _pool.get_thread_count() };

    // while (std::chrono::steady_clock::now() < limit)
    // {
    //   if (_collision_found)
    //   {
    //     break;
    //   }

    _pool.detach_task([&] { this->propagate_cube(_state); });

    // }
    _pool.wait();

    _collision_msg.data = _collision_found;
    _collision_publisher.publish(_collision_msg);
    _pool.detach_task([&] { this->grid_to_markers(); });
    _pool.detach_task([&] { this->trajectories_to_marker(); });

    return _collision_found;
  }

  void grid_to_markers()
  {
    visualization_msgs::Marker marker_lie{ ml4kp_bridge::create_marker(0.01, /*color*/ { 0.3, 1, 0, 1 }) };
    visualization_msgs::Marker marker_state{ ml4kp_bridge::create_marker(0.01, /*color*/ { 0.1, 0.1, 0.5, 1 }) };
    marker_lie.type = visualization_msgs::Marker::CUBE_LIST;
    marker_state.type = visualization_msgs::Marker::CUBE_LIST;

    auto cell_size = _grid.cell_sizes();
    marker_lie.scale.x = cell_size[0] * 0.96;
    marker_lie.scale.y = cell_size[1] * 0.96;
    marker_lie.scale.z = 0.1;
    marker_state.scale.x = cell_size[0] * 0.96;
    marker_state.scale.y = cell_size[1] * 0.96;
    marker_state.scale.z = 0.1;

    auto plant_config = _plant->configuration(_grid.x0());
    marker_lie.pose.position.x = plant_config[0].second[0];
    marker_lie.pose.position.y = plant_config[0].second[1];

    marker_state.pose.position.x = 0.;
    marker_state.pose.position.y = 0.;

    const Eigen::Quaterniond q{ plant_config[0].first };
    marker_lie.pose.orientation.w = q.w();
    marker_lie.pose.orientation.x = q.x();
    marker_lie.pose.orientation.y = q.y();
    marker_lie.pose.orientation.z = q.z();

    DEBUG_VARS(cell_size.transpose(), _grid.size())
    // const bool x_sign{ plant_config[0].second[0] > 0 };
    // const bool y_sign{ plant_config[0].second[1] > 0 };
    for (auto cell : _grid)
    {
      const State state{ cell.second->state };
      const Tangent center_tg{ _grid.center(state) };
      const State center{ _grid.state(center_tg) };
      // LOG_VARS(center);
      // DEBUG_VARS(state, center)
      marker_lie.points.emplace_back();
      marker_state.points.emplace_back();
      // marker.points.back().x = center[0];  //- (x_sign ? 0. : cell_size[0]);
      // marker.points.back().y = center[1];  //- (y_sign ? 0. : cell_size[1]);
      // marker.points.back().z = -0.101;
      ml4kp_bridge::update_point(marker_lie.points.back(), center_tg, 0, 1, -0.101);
      ml4kp_bridge::update_point(marker_state.points.back(), center, 0, 1, -0.101);
    }
    _cubes_algebra_publisher.publish(marker_lie);
    _cubes_state_publisher.publish(marker_state);
    // _cubes_publisher.publish(marker);
    _grid.clear();
    // DEBUG_PRINT
  }

  void trajectories_to_marker()
  {
    if (_visualize)
    {
      visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.01, /*color*/ { 1, 1, 0, 0 }) };
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::DELETEALL;
      // _trajectory_markers.markers.push_back(marker);
      _markers_publisher.publish(marker);

      marker.action = visualization_msgs::Marker::ADD;
      // DEBUG_VARS(_trajectories.size())
      std::scoped_lock lock(_trajectories_mutex);
      while (_trajectories.size() > 0)
      {
        ml4kp_bridge::update_marker(marker, _trajectories.back(), 0, 1, 0.0, visualization_msgs::Marker::LINE_LIST);
        _trajectories.pop_back();
      }

      _markers_publisher.publish(marker);
      // _trajectory_markers.markers.clear();
    }

    std::scoped_lock lock(_trajectories_mutex);
    _trajectories.clear();
  }

private:
  bool _visualize;
  std::atomic<int> _trajectories_to_viz;

  std::atomic<bool> _collision_found;
  std::atomic<int> _total_checked_trajectories;
  std::atomic<int> _unchecked_trajectories, _collisions_in_check;

  std::mutex _new_cell_mutex;
  std::mutex _trajectories_mutex, _queries_mutex;

  std::vector<Trajectory> _trajectories;
  // std::vector<Trajectory> _checked_trajectories;

  Controller _controller;

  std_msgs::Bool _collision_msg;
  ros::Publisher _markers_publisher, _cubes_algebra_publisher, _cubes_state_publisher, _collision_publisher;

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

  double _cell_size;

  std::size_t _iter_idx;
  std::vector<CellPtr> _cells_buffer, _used_cells;
  ImplicitGrid _grid;

  bool _short_circuit;
};
}  // namespace motion_planning
