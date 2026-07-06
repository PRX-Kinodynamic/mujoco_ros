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
// #include "prx/external/thread_pool/BS_thread_pool.hpp"

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>
#include <prx/utilities/data_structures/implicit_grid.hpp>
#include <array>
namespace motion_planning
{

template <typename State>
class gt_cell_t : public std::enable_shared_from_this<gt_cell_t<State>>
{
  using Cell = gt_cell_t<State>;
  struct Private
  {
    explicit Private() = default;
  };

  inline static std::size_t idx = 0;

public:
  gt_cell_t(const Private) : safe(false), total_states(0), step_idx(200, false)
  {
    // DEBUG_PRINT
    // step_idx.fill(false);
    // DEBUG_VARS(idx)
    idx++;
  }

  static std::shared_ptr<Cell> create()
  {
    // PRINT_MSG("Create")
    const Private p{};
    std::shared_ptr<Cell> ptr;
    ptr.reset(new Cell(p));
    return ptr;
    // return std::make_shared<Cell>(p);
  }

  bool safe;
  // std::array<bool, 200> step_idx;
  std::vector<bool> step_idx;
  std::size_t total_states;
  State state;
};

template <typename DynamicalSystem, typename Controller>
class reachability_gt_t
{
public:
  using TrajectoryMsg = std::vector<ml4kp_bridge::SpacePointStamped>;
  using PlanMsg = ml4kp_bridge::PlanStepStampedArray;
  using State = typename DynamicalSystem::State;
  using StateDot = typename DynamicalSystem::StateDot;
  using Control = typename DynamicalSystem::Control;
  using Trajectory = std::vector<State>;

  using FwdProp = prx::forward_propagation_t<DynamicalSystem, Trajectory, Controller>;

  static constexpr int DimX{ prx::dynamical_system_traits<DynamicalSystem>::StateDimension };

  using StateSampler = prx::lie_group_gaussian_noise_t<State>;
  using StateDotSampler = prx::multivariate_gaussian_t<DimX>;
  using Covariance = typename StateDotSampler::Covariance;

  using CollisionQuery = prx::collision_checking::pqp::query_t;

  using CellPtr = std::shared_ptr<gt_cell_t<State>>;
  using ImplicitGrid = prx::implicit_grid_t<State, CellPtr>;
  using Tangent = typename ImplicitGrid::TangentElement;

  reachability_gt_t(ros::NodeHandle nh)
    : _max_step_idx(0), _x0_sampler(true, 3.841), _w_sampler(true, 3.841), _trajs_markers(0)
  {
    // PRX FILES
    std::string environment;
    std::string plant_parameters;

    // PRX PARAM LOADERS FOR PRX FILES
    prx::param_loader env_params;

    using prx::simulation_step;

    // int& total_threads{ _total_threads };
    bool& visualize{ _visualize };
    _short_circuit = false;

    double& cell_size{ _cell_size };
    int& mg_step{ _mg_step };
    int& convex_hulls_step{ _convex_hulls_step };
    int random_seed;

    PARAM_SETUP(nh, cell_size);

    PARAM_SETUP_WITH_DEFAULT(nh, convex_hulls_step, 10);
    PARAM_SETUP_WITH_DEFAULT(nh, mg_step, 1);
    // PARAM_SETUP_WITH_DEFAULT(nh, total_threads, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, visualize, true);
    // PARAM_SETUP_WITH_DEFAULT(nh, short_circuit, true);

    GLOBAL_PARAM_BLOCKER(environment);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(simulation_step);
    GLOBAL_PARAM_BLOCKER(random_seed);

    prx::init_random(random_seed);
    // _pool.reset(_total_threads);
    // _half_threads = std::max(static_cast<int>(_total_threads / 2.0), 1);

    // DEBUG_VARS(_pool.get_thread_count())

    PRINT_MSG("Setting env");
    env_params.from_string(environment);
    // plant_params.from_string(plant_parameters);

    // _plant = std::make_shared<DynamicalSystem>(plant_parameters);
    PRINT_MSG("Setting plant");
    _plant = DynamicalSystem::create(plant_parameters);

    PRINT_MSG("Setting obstacles");
    _obstacles_bodies = prx::collision_checking::pqp::create_obstacles(env_params);

    _system_geoms = _plant->geometries();

    _markers_publisher = nh.advertise<visualization_msgs::Marker>("/GT/trajectories/marker", 1);
    _collision_publisher = nh.advertise<std_msgs::Bool>("/GT/collision", 1);
    _end_points_publisher = nh.advertise<visualization_msgs::MarkerArray>("/GT/cells", 1);
    _collision_markers_publisher = nh.advertise<visualization_msgs::Marker>("/GT/collisions/marker", 1);
    _trajectories_publisher = nh.advertise<visualization_msgs::Marker>("/GT/trajectories/marker", 1);

    std::string output_directory, file_prefix;
    PARAM_SETUP_WITH_DEFAULT(nh, output_directory, "/tmp/");
    PARAM_SETUP_WITH_DEFAULT(nh, file_prefix, "gt");

    // const std::string OUTPUT_FILE{ output_directory + "/" + file_prefix + "_volumes_" + timestamp + ".txt" };
    _output_file_prefix = output_directory + "/" + file_prefix + "_volumes_";
    // DEBUG_VARS(OUTPUT_FILE)
    _traj_marker = ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 });
    _traj_marker.type = visualization_msgs::Marker::LINE_LIST;
    _traj_marker.action = visualization_msgs::Marker::ADD;
  }

  ~reachability_gt_t()
  {
  }

  void collion_check()
  {
    // if (_short_circuit and _collision_found)  // short-circuit
    //   return;
    Trajectory traj;
    {
      // std::scoped_lock lock(_trajectories_mutex);
      traj = _trajectories.back();
      _trajectories.pop_back();
      if (_visualize)
      {
        if (_trajs_markers > 1000)
        {
          ml4kp_bridge::update_marker(_traj_marker, traj, 0, 1, -0.01);
          _trajs_markers++;
        }
      }
    }

    std::shared_ptr<CollisionQuery> query;
    {
      // std::scoped_lock lock(_queries_mutex);
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
    for (int state_idx = 0; state_idx < traj.size(); state_idx += _convex_hulls_step)
    {
      const State& state{ traj[state_idx] };

      const Tangent center_tg{ _grid.center(state) };
      const State center{ _grid.state(center_tg) };

      CellPtr cellptr{ init_cell(state) };

      query->plant_configurations = _plant->configuration(state);

      collision = prx::collision_checking::pqp::collision(*query, _obstacles_bodies);

      cellptr->safe = true;
      cellptr->state = center;
      if (cellptr->step_idx.size() <= state_idx)
      {
        cellptr->step_idx.insert(cellptr->step_idx.end(), 200, false);
      }
      // DEBUG_VARS(cellptr->step_idx.size())
      cellptr->step_idx[state_idx] = true;
      cellptr->total_states++;
      _max_step_idx = std::max(_max_step_idx, state_idx);

      if (collision)
      {
        cellptr->safe = false;
        _colliding_states.push_back(state);
        // break;
      }
    }

    // {
    // std::scoped_lock lock(_queries_mutex);
    _queries.push_back(query);
    // }

    _collision_found = _collision_found or collision;
    // std::scoped_lock lock(_checked_trajectories_mutex);
    _checked_trajectories.push_back(traj);
  }

  CellPtr init_cell(const State& state)
  {
    // PRINT_MSG("----------")
    // CellPtr new_ptr;  //{ _grid.cell(state) };

    // if (not _grid.exists(state))
    if (_grid.cell(state) == nullptr)
    {
      // CellPtr new_ptr{ std::make_shared<gt_cell_t<State>>() };
      CellPtr new_ptr{ gt_cell_t<State>::create() };
      new_ptr->state = state;

      _grid.cell(state) = new_ptr;

      // prx_assert(_grid.cell(state) != nullptr, "Grid ptr is null!");
    }
    return _grid.cell(state);
    // return new_ptr;
  }

  void propagate()
  {
    // if (_short_circuit and _collision_found)  // short-circuit
    //   return;
    const State x0_noise{ _x0_sampler(_state) };
    Trajectory traj;
    FwdProp::propagate(traj, x0_noise, _controller, _plant, _w_sampler);

    // {
    // std::scoped_lock lock(_trajectories_mutex);
    _trajectories.push_back(traj);
    _unchecked_trajectories++;
    // }
    collion_check();
    // _pool.detach_task([&] { this->collion_check(); }, BS::pr::highest);
  }

  void init_query(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
                  const Covariance& cov_w)
  {
    _colliding_states.clear();

    _trajectories.clear();
    _traj_marker.points.clear();
    // copy(_controller, plan_in);
    // copy(_state, x_hat);
    ml4kp_bridge::copy(_controller, plan_in);
    ml4kp_bridge::copy(_state, x_hat);

    DEBUG_VARS(_controller.size())
    // DEBUG_VARS(_controller[0])

    _x0_sampler.set(cov_x0);
    _w_sampler.set(cov_w);

    _collision_found = false;

    _marker_pts.markers.clear();

    const Tangent cell_size{ Tangent::Ones() * _cell_size };
    _grid.reset(_state, cell_size);
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in, const Covariance& cov_x0,
               const Covariance& cov_w, const int total_trajectories)
  {
    init_query(x_hat, plan_in, cov_x0, cov_w);

    DEBUG_VARS(total_trajectories)
    ros::Time start{ ros::Time::now() };
    for (int traj_idx = 0; traj_idx < total_trajectories; ++traj_idx)
    {
      propagate();
      if (traj_idx % 1000 == 0)
      {
        DEBUG_VARS(traj_idx)
      }
    }
    // _pool.wait();

    const ros::Time end{ ros::Time::now() };
    const double randup_time{ (end - start).toSec() };
    const bool collision{ _collision_found };

    // _convex_hull_volumes.back().push_back(convex_hull_volume);

    // _ofs << total_trajectories << " ";
    // _ofs << collision << " ";
    // _ofs << randup_time << " ";
    // for (auto&& vol : _convex_hull_volumes)
    // {
    //   _ofs << vol << " ";
    // }
    // _ofs << "\n";
    // VARS_TO_STREAM(_ofs, randup_time, collision);

    // DEBUG_VARS(_grid.size())
    _collision_msg.data = _collision_found;
    _collision_publisher.publish(_collision_msg);
    trajectories_to_marker();
    compute_gt_info();

    return _collision_found;
  }

  void compute_gt_info()
  {
    using prx::utilities::convert_to;
    DEBUG_VARS(_max_step_idx)

    const std::string timestamp{ utils::timestamp() };

    for (int state_idx = 0; state_idx <= _max_step_idx; state_idx += _convex_hulls_step)
    {
      const std::string s_idx{ convert_to<std::string>(state_idx) };
      const std::string filename_i{ _output_file_prefix + "_" + timestamp + "_" + s_idx + ".txt" };
      DEBUG_VARS(filename_i);
      _ofs.open(filename_i.c_str());
      _ofs << "# First line: 'id x0 cell_size' of grid (the id of this reachable set, x0 is the x0 of the grid  ";
      _ofs << "and the size of each cell). Then empty line and then N lines with 'states safe total_states' ";
      _ofs << "corresponding to the reachable set (on the grid).\n";
      prx::to_stream(_ofs, state_idx);
      prx::to_stream(_ofs, _grid.x0());
      prx::to_stream(_ofs, _grid.cell_sizes());
      _ofs << "\n\n";

      for (auto cell : _grid)
      {
        if (cell.second->step_idx[state_idx])
        {
          prx::to_stream(_ofs, cell.second->state);
          prx::to_stream(_ofs, cell.second->safe);
          prx::to_stream(_ofs, cell.second->total_states);
          _ofs << "\n";
        }
      }
      _ofs.close();
    }
  }

  void trajectories_to_marker()
  {
    if (_visualize)
    {
      _trajectories_publisher.publish(_traj_marker);
      visualization_msgs::MarkerArray cell_markers;
      visualization_msgs::Marker marker_free{ ml4kp_bridge::create_marker(0.01, { 1, 0, 1, 0 }) };
      visualization_msgs::Marker marker_coll{ ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 }) };

      marker_free.ns = "Free";
      marker_coll.ns = "Collision";

      marker_free.type = visualization_msgs::Marker::CUBE_LIST;
      marker_free.scale.x = _cell_size;
      marker_free.scale.y = _cell_size;
      marker_free.scale.z = 0.1;

      marker_coll.type = visualization_msgs::Marker::CUBE_LIST;
      marker_coll.scale.x = _cell_size;
      marker_coll.scale.y = _cell_size;
      marker_coll.scale.z = 0.1;

      for (auto cell : _grid)
      {
        // DEBUG_VARS(cell.first, cell.second)
        // prx_assert(cell.second != nullptr, "Nullptr!");
        if (cell.second->safe)
        {
          marker_free.points.emplace_back();
          ml4kp_bridge::update_point(marker_free.points.back(), cell.second->state, 0, 1, 0.0);
        }
        else
        {
          marker_coll.points.emplace_back();
          ml4kp_bridge::update_point(marker_coll.points.back(), cell.second->state, 0, 1, 0.0);
        }
      }
      cell_markers.markers.push_back(marker_free);
      cell_markers.markers.push_back(marker_coll);
      // marker.action = visualization_msgs::Marker::DELETEALL;
      // // _trajectory_markers.markers.push_back(marker);
      // _markers_publisher.publish(marker);

      // // _trajectory_markers.markers.clear();
      // marker.action = visualization_msgs::Marker::ADD;

      // {  // _checked_trajectories_mutex lock
      //   std::scoped_lock lock(_checked_trajectories_mutex);

      //   DEBUG_VARS(_checked_trajectories.size())
      //   while (_checked_trajectories.size() > 0)
      //   {
      //     ml4kp_bridge::update_marker(marker, _checked_trajectories.back(), 0, 1, -0.1);
      //     _checked_trajectories.pop_back();
      //   }
      // }

      // visualization_msgs::Marker collision_marker{ ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 }) };
      // collision_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      // collision_marker.scale.x = 0.25;
      // collision_marker.scale.y = 0.25;
      // collision_marker.scale.z = 0.01;

      // collision_marker.color.a = 0.5;
      // collision_marker.color.r = 0.92;
      // collision_marker.color.g = 0.91;
      // collision_marker.color.b = 0.1;

      // for (auto&& state : _colliding_states)
      // {
      //   collision_marker.points.emplace_back();
      //   ml4kp_bridge::update_point(collision_marker.points.back(), state, 0, 1, 0.0);
      // }

      _end_points_publisher.publish(cell_markers);
      // _collision_markers_publisher.publish(collision_marker);
      // _trajectory_markers.markers.clear();
    }

    // std::scoped_lock lock(_checked_trajectories_mutex);
    _checked_trajectories.clear();
  }

private:
  bool _visualize;
  std::atomic<int> _trajectories_to_viz;

  std::atomic<bool> _collision_found;
  std::atomic<int> _total_checked_trajectories;
  std::atomic<int> _unchecked_trajectories, _collisions_in_check;
  // std::mutex _trajectories_mutex, _checked_trajectories_mutex, _queries_mutex;
  std::vector<Trajectory> _trajectories;
  std::vector<Trajectory> _checked_trajectories;

  visualization_msgs::Marker _traj_marker;
  visualization_msgs::MarkerArray _marker_convex_hull, _marker_pts;

  Controller _controller;

  std_msgs::Bool _collision_msg;
  ros::Publisher _markers_publisher, _collision_publisher, _collision_markers_publisher, _trajectories_publisher;
  ros::Publisher _end_points_publisher;

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

  // int _total_threads, _half_threads;
  // BS::thread_pool<BS::tp::pause | BS::tp::priority> _pool;

  StateSampler _x0_sampler;
  StateDotSampler _w_sampler;

  bool _short_circuit;

  std::ofstream _ofs;

  ImplicitGrid _grid;

  std::vector<State> _colliding_states;
  // std::vector<double> _convex_hull_volumes;

  double _cell_size;
  int _mg_step;

  int _max_step_idx;

  int _convex_hulls_step;
  int _trajs_markers;

  std::string _output_file_prefix;
};
}  // namespace motion_planning
