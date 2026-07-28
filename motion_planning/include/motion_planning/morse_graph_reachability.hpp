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
#include <prx/external/thread_pool/BS_thread_pool.hpp>
#include <prx/utilities/data_structures/implicit_grid.hpp>

#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>
#include <ml4kp_bridge/controller_bridge.hpp>
#include "interface/gaussian_to_ellipse_marker.hpp"

namespace motion_planning
{
template <typename State, typename Vertex>
struct mg_cell_t
{
  mg_cell_t() : added_idx(0), propagated_idx(0), visited_idx(0), safe(true) {};

  std::mutex mutex;
  bool safe;
  std::size_t added_idx;
  std::size_t propagated_idx;
  std::size_t visited_idx;
  // std::size_t _idx;
  State state;
  Vertex vertex;
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

  static constexpr int DimX{ gtsam::traits<State>::dimension };

  using FwdProp = prx::forward_propagation_t<DynamicalSystem, Trajectory, Controller>;

  using StateSampler = prx::lie_group_gaussian_noise_t<State>;
  using Covariance = typename StateSampler::Covariance;

  using CollisionQuery = prx::collision_checking::pqp::query_t;

  // using Tangent = typename ImplicitGrid::TangentElement;
  using Vertex = Eigen::Vector<int, DimX>;
  using Tangent = Eigen::Vector<double, DimX>;
  using Cell = mg_cell_t<State, Vertex>;
  using CellPtr = std::shared_ptr<Cell>;
  using ImplicitGrid = prx::implicit_grid_t<State, CellPtr>;

  morse_graph_reachability_t(ros::NodeHandle nh)
    : _iter_idx(0), _short_circuit(true), _propagated_idx(0), _visited_idx(0), _identity(Covariance::Identity())
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

    // MG step, how many states to step when iterating over each trajectory
    // Needed for randup comparison
    int& mg_step{ _mg_step };
    double& Chi2_alpha{ _Chi2_alpha };
    double& split_time{ _split_time };
    PARAM_SETUP(nh, cell_size);
    PARAM_SETUP_WITH_DEFAULT(nh, Chi2_alpha, 0.05)
    PARAM_SETUP_WITH_DEFAULT(nh, mg_step, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, total_threads, 1);
    PARAM_SETUP_WITH_DEFAULT(nh, short_circuit, true);
    PARAM_SETUP_WITH_DEFAULT(nh, visualize, true);
    PARAM_SETUP_WITH_DEFAULT(nh, split_time, 0.5);

    // DEBUG_VARS(mg_step, cell_size, split_time)

    GLOBAL_PARAM_BLOCKER(environment);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(simulation_step);

    _pool.reset(_total_threads);
    _half_threads = std::max(static_cast<int>(_total_threads / 2.0), 1);

    env_params.from_string(environment);
    plant_params.from_string(plant_parameters);

    _chi2 = std::make_shared<prx::chi_squared>();
    _plant = std::make_shared<DynamicalSystem>(plant_params);

    _obstacles_bodies = prx::collision_checking::pqp::create_obstacles(env_params);

    _system_geoms = _plant->geometries();

    _markers_publisher = nh.advertise<visualization_msgs::Marker>("/mg/trajectories/marker", 1);
    _markers_x0s_publisher = nh.advertise<visualization_msgs::Marker>("/mg/trajectories/start_states/marker", 1);
    _traj_nominal_publisher = nh.advertise<visualization_msgs::Marker>("/mg/trajectories/nominal/marker", 1);
    _collision_publisher = nh.advertise<std_msgs::Bool>("/mg/collision", 1);
    _cubes_algebra_publisher = nh.advertise<visualization_msgs::Marker>("/mg/cubes/lie_algebra", 1);
    _cubes_state_publisher = nh.advertise<visualization_msgs::Marker>("/mg/cubes/state_space", 1);
    _radii_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/mg/cubes/balls", 1);
    _x0_noise_publisher = nh.advertise<visualization_msgs::Marker>("/mg/x0/noise", 1);

    std::string output_directory, file_prefix;
    PARAM_SETUP_WITH_DEFAULT(nh, output_directory, "/tmp/");
    PARAM_SETUP_WITH_DEFAULT(nh, file_prefix, "mg");

    _timestamp = utils::timestamp();
    _prefix = output_directory + "/" + file_prefix;
    const std::string MG_OUTPUT_FILE{ _prefix + "_balls_" + _timestamp + ".txt" };
    const std::string MG_GRID_FILE{ _prefix + "_grid_" + _timestamp + ".txt" };
    const std::string MG_TRAJS_FILE{ _prefix + "_trajs_" + _timestamp + ".txt" };
    const std::string MG_NOMINAL_TRAJS_FILE{ _prefix + "_nominal_trajs_" + _timestamp + ".txt" };
    const std::string MG_STATS_FILE{ _prefix + "_stats_" + _timestamp + ".txt" };
    DEBUG_VARS(MG_OUTPUT_FILE)
    DEBUG_VARS(MG_TRAJS_FILE)
    DEBUG_VARS(MG_NOMINAL_TRAJS_FILE)
    _ofs_balls.open(MG_OUTPUT_FILE);
    _ofs_grid.open(MG_GRID_FILE);
    _ofs_stats.open(MG_STATS_FILE);
    // _ofs_nominal_trajs.open(MG_NOMINAL_TRAJS_FILE);
  }

  ~morse_graph_reachability_t()
  {
  }

  void collision_check(const State state, const double safe_distance, const int step_idx)
  {
    if (_short_circuit and _collision_found)  // short-circuit
      return;

    CellPtr cellptr{ init_cell(state) };

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

    const double min_distance{ prx::collision_checking::pqp::minimum_distance(*query, _obstacles_bodies) };

    std::scoped_lock lock(cellptr->mutex);
    if (min_distance < safe_distance)
    {
      cellptr->safe = false;
      _collision_found = true;

      // DEBUG_VARS(state, min_distance, safe_distance)
      PRINT_MSG("Collision!")
      // DEBUG_VARS(collision)
    }
    else
    {
      // DEBUG_VARS(state, min_distance, query->P1, safe_distance)

      // auto config = query->plant_configurations;
      query->plant_configurations[0].second += query->P1.normalized() * safe_distance;
      const bool collision{ prx::collision_checking::pqp::collision(*query, _obstacles_bodies) };
      // DEBUG_VARS(query->plant_configurations[0].second, collision)
      if (collision)
      {
        cellptr->safe = false;
        _collision_found = true;
      }
    }
    // DEBUG_VARS(step_idx, state, safe_distance)
    _safe_radii.push_back({ step_idx, state, safe_distance });
    cellptr->state = state;
    cellptr->vertex = _grid.vertex(state);
    cellptr->added_idx = _iter_idx;  // This cell's safety has been checked
  }

  template <typename StateOrVertex>
  CellPtr init_cell(const StateOrVertex& x_in)
  {
    std::scoped_lock lock(_new_cell_mutex);
    if (_grid.cell(x_in) == nullptr)
    {
      if (_cells_buffer.empty())
      {
        for (int i = 0; i < 100; ++i)
        {
          _cells_buffer.push_back(std::make_shared<Cell>());
        }
      }
      _grid.cell(x_in) = _cells_buffer.back();
      _used_cells.push_back(_cells_buffer.back());
      _cells_buffer.pop_back();
    }
    return _grid.cell(x_in);
  }

  double compute_safe_distance(const double K_tau, const State& x0p, const State& xp_tau, const double tau,
                               const Control& u0, const Control& ubar, const Trajectory& traj_nominal)
  {
    const double& d{ _cell_size };
    if (tau < prx::simulation_step)
    {
      // return 0.;
      return d / 2.;
    }
    // const double d2{ _cell_size * _cell_size / 4. };
    // // const double K_tau{ 0. };

    const double tau_steps{ tau / prx::simulation_step };

    const State& x0{ traj_nominal.front() };
    const State& x_tau{ traj_nominal[tau_steps] };

    // // DEBUG_VARS(x0, x0p, x_tau, xp_tau, u0, ubar)
    // const double Lf{ lipschitz(x0, x0p, x_tau, xp_tau) };
    // const double Lf2{ Lf * Lf };

    // const double Lu{ lipschitz(x0, x0p, u0, ubar) };
    // const double Lu2{ Lu * Lu };

    // const double Lerr{ prx::TangentBetween(x_tau, xp_tau).norm() };

    // const double P{ 4. * tau * _C2_w };  //+ 4. * tau * Lf * _C2_x0 };

    // const double d2_P_Ktau{ d / 2. + P + K_tau };
    // const double sqrt_d2_P_Ktau{ std::sqrt(d2_P_Ktau) };
    // const double L_tau{ (2. / d) * std::sqrt(d / 2. + P + K_tau) };
    // DEBUG_VARS(P, K_tau, d2_P_Ktau, sqrt_d2_P_Ktau)
    // DEBUG_VARS(tau, tau_steps, P, K_tau, L_tau, _C2_w)

    // DEBUG_VARS(_C2_w, _C2_u)
    // const double noise{ _C2_w + 4 * tau * Lf2 * Lu2 * _C2_u };

    // const double tau_K_tau{ 4 * tau * Lf2 * (1. + Lu2) * K_tau };
    // const double tau_noise{ 4 * tau * noise };
    // const double K{ d2 + tau_noise + 4 * tau * Lf2 * (1. + Lu2) * K_tau };
    // const double L{ std::sqrt(1. + Lerr / d2) };

    // const double L{ 2 * std::sqrt(Lerr) / d };
    const double L{ lipschitz(x0, x0p, x_tau, xp_tau) };
    const double wK{ _C2_w * tau };
    // DEBUG_VARS(tau, L, wK, _C2_w)

    // DEBUG_VARS(d2, tau, tau_K_tau, tau_noise, K_tau, Lf, Lu, noise, K, L)
    // DEBUG_VARS(d2, tau, K_tau, Lf, Lu, noise, K, L)
    return (L + wK) * d / 2.;
  }

  void propagate(Trajectory& traj, const State state, const Controller controller) const
  {
    // DEBUG_VARS(controller)
    FwdProp::propagate(traj, state, controller, _plant);
    // DEBUG_VARS(controller.size())
    // DEBUG_VARS(traj)
  }

  double compute_state_square_diff(const State& xbar, const State& x, const double dt)
  {
    const State xbtw{ gtsam::traits<State>::Between(xbar, x) };
    const Tangent tg{ gtsam::traits<State>::Logmap(xbtw) };
    const double dKdt{ tg.squaredNorm() };
    // DEBUG_VARS(xbar, x)
    // DEBUG_VARS(xbtw, tg)
    // DEBUG_VARS(dKdt)
    return dKdt * dt;
  }

  void propagate_and_check(const State state, const Controller ctrl_head, const Controller controller,
                           const Control& u0_nominal, const Trajectory traj_nominal, const int split_idx)
  {
    Trajectory traj;

    // DEBUG_VARS(controller.size())
    // const Controller current_ctrllr{ ml4kp_bridge::split(controller, _split_time) };

    propagate(traj, state, ctrl_head);

    const Control ubar{ prx::controller_view_t<DynamicalSystem, Controller>::front(ctrl_head, state, _plant) };
    std::set<std::size_t> hashes;

    const State x0V{ traj.front() };

    double K_tau{ 0. };
    double tau{ 0. };
    double tau_prev{ 0. };
    double safe_distance{ 0. };
    for (int i = 0; i < traj.size(); i += _mg_step)
    {
      tau = static_cast<double>(i) * prx::simulation_step;
      const State& xbar{ traj[i] };
      const State& x{ traj_nominal[i] };

      // <<<<<<< HEAD
      // K_tau += compute_state_square_diff(xbar, x, tau - tau_prev);
      tau_prev = tau;
      const std::size_t h{ _grid.hash(xbar) };
      // DEBUG_VARS(i, x, xbar, h)
      // DEBUG_VARS(hashes.size(), hashes.count(h))
      if (hashes.count(h) == 0)
      {
        hashes.insert(h);
        safe_distance = compute_safe_distance(K_tau, x0V, xbar, tau, u0_nominal, ubar, traj_nominal);
        // DEBUG_VARS(safe_distance)
        _pool.detach_task(
            [xbar, safe_distance, split_idx, this] { this->collision_check(xbar, safe_distance, split_idx); });

        if (safe_distance > _cell_size / 4.)
        {
          const double r2{ safe_distance * safe_distance };
          _pool.detach_task([xbar, r2, this] { add_cells_inside_ellipse(xbar, xbar, _identity, r2); });
        }
      }
    }

    {
      std::scoped_lock lock(_trajectories_mutex);
      _trajectories[split_idx].push_back(traj);
    }

    // if (controller.size() > 0)
    // {
    // DEBUG_VARS(traj.back())
    const State xT{ traj.back() };

    // _pool.detach_task([xT, controller, split_idx, this] { this->propagate_cube(xT, controller, split_idx); });

    propagate_neighbors(xT, xT, _identity, safe_distance * safe_distance, controller, split_idx);
    // }
    // _unchecked_trajectories++;
  }

  Trajectory get_nominal_trajectory(const State state, const Controller controller, const int split_idx)
  {
    Trajectory traj;

    const Tangent center_tg{ _grid.center(state) };
    const State xc{ _grid.state(center_tg) };

    // const Controller current_ctrllr{ ml4kp_bridge::split(controller, _split_time) };
    propagate(traj, xc, controller);

    if (_visualize)
    {
      std::scoped_lock lock{ _nominal_trajectories_mutex };
      _nominal_trajs[split_idx].push_back(traj);
    }
    return traj;
  }

  void propagate_cube(const Vertex vx, Controller controller, const int split_idx)
  {
    if (controller.size() == 0)
      return;
    // DEBUG_VARS(state)
    // if (_collision_found)  // short-circuit
    //   return;
    CellPtr cellptr{ init_cell(vx) };
    if (cellptr->propagated_idx == _propagated_idx)
    {
      // DEBUG_VARS(_propagated_idx, cellptr->)
      return;
    }

    cellptr->propagated_idx = _propagated_idx;
    auto vertices = _grid.vertices(vx);
    const State x_center{ _grid.center_state(vx) };
    const Controller controller_head{ ml4kp_bridge::split(controller, _split_time) };
    const Trajectory traj_nominal{ get_nominal_trajectory(x_center, controller_head, split_idx) };

    const Control u_nominal{ prx::controller_view_t<DynamicalSystem, Controller>::front(controller_head, x_center,
                                                                                        _plant) };

    // DEBUG_VARS(state)
    // DEBUG_VARS(controller_head, controller, split_idx)
    for (auto v : vertices)
    {
      const State xv{ _grid.state_from_vertex(v) };
      // DEBUG_VARS(v)

      _pool.detach_task([xv, controller_head, controller, traj_nominal, split_idx, u_nominal, this] {
        this->propagate_and_check(xv, controller_head, controller, u_nominal, traj_nominal, split_idx + 1);
      });
    }
  }

  const double distribution_bound(const Covariance& cov)
  {
    const double chi2_critical_value{ _chi2->critical_value(DimX, _Chi2_alpha) };

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
    const Tangent D_marginal{ es.eigenvalues().cwiseSqrt() * chi2_critical_value };
    const double C_bound{ D_marginal.maxCoeff() };
    return std::pow(C_bound, 2);
  }

  template <typename LieTypeStart, typename LieTypeEnd>
  double lipschitz(const LieTypeStart& x0, const LieTypeStart& x0p, const LieTypeEnd& xF, const LieTypeEnd& xFp) const
  {
    const LieTypeStart xBtw_0{ gtsam::traits<LieTypeStart>::Between(x0, x0p) };
    const LieTypeEnd xBtw_F{ gtsam::traits<LieTypeEnd>::Between(xF, xFp) };

    const auto tg_0 = gtsam::traits<LieTypeStart>::Logmap(xBtw_0);
    const auto tg_F = gtsam::traits<LieTypeEnd>::Logmap(xBtw_F);

    const double tg_0_norm{ tg_0.norm() };
    const double tg_F_norm{ tg_F.norm() };

    const double L{ tg_F_norm / tg_0_norm };

    return L;
  }

  // Given the ellipse (defined by epsilon_inv) centered at x0, check if vertex is inside given chi_confidence.
  // If the ellipse is a circle/ball (epsilon_inv=I) then, chi_confidence can be seen as the squared radius (r^2), where
  // this function will return true if the distance of vertex to x0 is less than chi_confidence. NOTE: this function
  // assumes the chi_confidence is squared, if it is used a radius, the input must be the radius squared.
  bool cell_intersects_with_ellipse(const State& x0, const Vertex& vertex, const Covariance& epsilon_inv,
                                    const double chi_confidence) const
  {
    const State xv{ _grid.state_from_vertex(vertex) };
    std::vector<Vertex> vertices{ _grid.vertices(xv) };
    bool inside{ false };
    for (auto v : vertices)
    {
      const State xv_i{ _grid.state_from_vertex(v) };
      const Tangent tg_v{ prx::TangentBetween(x0, xv_i) };
      const double err{ tg_v.transpose() * epsilon_inv * tg_v };
      if (err < chi_confidence)
      {
        // DEBUG_VARS(xv_i, err, chi_confidence)
        return true;
      }
    }
    return false;
  }

  bool is_cell_unvisited(const Vertex vx)
  {
    CellPtr cellptr{ init_cell(vx) };

    if (cellptr->visited_idx < _visited_idx)
    {
      cellptr->vertex = vx;
      cellptr->visited_idx = _visited_idx;
      return true;
    }
    return false;
  }

  void propagate_neighbors(const State x0, const State state, const Covariance epsilon_inv, const double chi_confidence,
                           const Controller controller, const std::size_t split_idx)
  {
    // std::scoped_lock lock{ _log_mutex };
    std::vector<Vertex> vertices_q;
    std::set<Vertex, typename ImplicitGrid::state_compare_t> local_hashes;

    const Vertex v0{ _grid.vertex(state) };
    vertices_q.push_back(v0);
    // LOG_VARS(x0, v0, state, chi_confidence)
    int total_vertices{ 0 };
    while (not vertices_q.empty())
    {
      Vertex v_next{ vertices_q.back() };
      vertices_q.pop_back();
      // LOG_VARS(v_next, vertices_q.size());
      total_vertices++;

      const bool vx_inside{ cell_intersects_with_ellipse(x0, v_next, epsilon_inv, chi_confidence) };
      if (vx_inside)
      {
        const State xv_i{ _grid.state_from_vertex(v_next) };
        _pool.detach_task(
            [v_next, controller, split_idx, this] { this->propagate_cube(v_next, controller, split_idx); });
        // LOG_VARS(total_vertices, v_next, vx_inside, xv_i)
        for (int i = 0; i < DimX; ++i)
        {
          v_next[i] += 1;

          // const std::size_t hp{ _grid.hash(v_next) };
          const bool hp_new{ local_hashes.count(v_next) == 0 };
          // LOG_VARS(total_vertices, i, v_next, hp_new)
          if (hp_new)
          {
            local_hashes.insert(v_next);
            vertices_q.push_back(v_next);
          }

          v_next[i] -= 2;
          // const std::size_t hm{ _grid.hash(v_next) };
          const bool hm_new{ local_hashes.count(v_next) == 0 };
          // LOG_VARS(total_vertices, i, v_next, hm_new)
          if (hm_new)
          {
            local_hashes.insert(v_next);
            vertices_q.push_back(v_next);
          }
          // LOG_VARS(i, v_next, hp)
          // if (local_hashes.count(hp) == 0)
          // {
          //   local_hashes.insert(hp);
          //   const bool inside_p{ cell_intersects_with_ellipse(x0, v_next, epsilon_inv, chi_confidence) };
          //   const State xv_p{ _grid.state_from_vertex(v_next) };
          //   LOG_VARS(i, xv_p, v_next, inside_p);
          //   if (inside_p)
          //   {
          //     // if (prop_p)
          //     // {
          //     vertices_q.push_back(v_next);
          //     _pool.detach_task(
          //         [xv_p, controller, split_idx, this] { this->propagate_cube(xv_p, controller, split_idx); });
          //     // }
          //     // propagate_neighbors(x0, v_next, epsilon_inv, chi_confidence, controller, split_idx);
          //   }
          // }
          // // As v_next[i] has already a +1, we need to remove it and do -1
          // v_next[i] -= 2;
          // const std::size_t hm{ _grid.hash(v_next) };
          // LOG_VARS(i, v_next, hm)
          // if (local_hashes.count(hm) == 0)
          // {
          //   local_hashes.insert(hm);
          //   const bool inside_m{ cell_intersects_with_ellipse(x0, v_next, epsilon_inv, chi_confidence) };
          //   // const bool prop_m{ is_cell_unvisited(v_next) };
          //   const State xv_m{ _grid.state_from_vertex(v_next) };
          //   LOG_VARS(i, xv_m, v_next, inside_m);
          //   if (inside_m)
          //   {
          //     // if (prop_m)
          //     // {
          //     vertices_q.push_back(v_next);
          //     _pool.detach_task(
          //         [xv_m, controller, split_idx, this] { this->propagate_cube(xv_m, controller, split_idx); });
          //     // }
          //     // is_cell_unvisited(x0, v_next, epsilon_inv, chi_confidence, controller, split_idx);
          //     // propagate_neighbors(x0, v_next, epsilon_inv, chi_confidence, controller, split_idx);
          //   }
          // }
          // Reset it...
          v_next[i] += 1;
          // LOG_VARS(v_next);
        }
      }
      else
      {
        // LOG_VARS(total_vertices, v_next, vx_inside)
      }
      // const State xv{ _grid.state_from_vertex(vertex) };

      // Go over every vertex neighbor: +1 and -1 per dimension

      // DEBUG_VARS(states_q.size())
    }
    // LOG_VARS(total_vertices, local_hashes)
    // return true;
  }

  void add_cells_inside_ellipse(const State x0, const State state, const Covariance epsilon_inv,
                                const double chi_confidence)
  {
    std::vector<Vertex> vertices_q;
    std::set<Vertex, typename ImplicitGrid::state_compare_t> visited;
    const Vertex v0{ _grid.vertex(state) };

    // std::set<std::size_t> local_hashes;
    vertices_q.push_back(v0);
    // DEBUG_VARS(x0, state, chi_confidence)
    while (not vertices_q.empty())
    {
      Vertex v_next{ vertices_q.back() };
      vertices_q.pop_back();

      const bool vx_inside{ cell_intersects_with_ellipse(x0, v_next, epsilon_inv, chi_confidence) };
      if (vx_inside)
      {
        CellPtr cellptr{ init_cell(v_next) };
        cellptr->state = _grid.center_state(v_next);
        cellptr->vertex = v_next;
        for (int i = 0; i < DimX; ++i)
        {
          v_next[i] += 1;
          const bool hp_new{ visited.count(v_next) == 0 };
          if (hp_new)
          {
            visited.insert(v_next);
            vertices_q.push_back(v_next);
          }

          v_next[i] -= 2;
          const bool hm_new{ visited.count(v_next) == 0 };
          if (hm_new)
          {
            visited.insert(v_next);
            vertices_q.push_back(v_next);
          }
          v_next[i] += 1;
        }
      }

      // for (int i = 0; i < DimX; ++i)
      // {
      //   // Tangent v_next{ vertex };
      //   v_next[i] += 1;

      //   const State xv_p{ _grid.state_from_vertex(v_next) };
      //   // const Tangent center_tg_p{ _grid.center(xv_p) };
      //   // const State center_p{ _grid.state(center_tg_p) };
      //   const std::size_t hp{ _grid.hash(v_next) };
      //   const bool inside_p{ cell_intersects_with_ellipse(x0, v_next, epsilon_inv, chi_confidence) };
      //   if (_cells_hashes.count(hp) == 0)
      //   {
      //     _cells_hashes.insert(hp);
      //     if (inside_p)
      //     {
      //       CellPtr cellptr{ init_cell(v_next) };
      //       cellptr->state = xv_p;
      //       cellptr->vertex = v_next;
      //       // vertices_q.push_back(xv_p);
      //     }
      //   }
      //   if (local_hashes.count(hp) == 0 and inside_p)
      //   {
      //     local_hashes.insert(hp);
      //     vertices_q.push_back(v_next);
      //   }

      //   // As v_next[i] has already a +1, we need to remove it and do -1
      //   v_next[i] -= 2;
      //   const State xv_m{ _grid.state_from_vertex(v_next) };
      //   // const Tangent center_tg{ _grid.center(xv_m) };
      //   // const State center{ _grid.state(center_tg) };
      //   const std::size_t hm{ _grid.hash(v_next) };
      //   const bool inside_m{ cell_intersects_with_ellipse(x0, v_next, epsilon_inv, chi_confidence) };
      //   if (_cells_hashes.count(hm) == 0)
      //   {
      //     _cells_hashes.insert(hm);
      //     if (inside_m)
      //     {
      //       CellPtr cellptr{ init_cell(v_next) };
      //       cellptr->state = xv_m;
      //       cellptr->vertex = v_next;
      //       // vertices_q.push_back(center);
      //     }
      //   }
      //   if (local_hashes.count(hm) == 0 and inside_m)
      //   {
      //     local_hashes.insert(hm);
      //     vertices_q.push_back(v_next);
      //   }
      //   v_next[i] += 1;
      // }

      // DEBUG_VARS(states_q.size())
    }

    // return true;
  }

  // template<typename>
  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const PlanMsg& plan_in,  // no-lint
               const Covariance& x0_noise, const Covariance& w_noise, const Covariance& u_noise,
               const std::chrono::time_point<std::chrono::steady_clock>& limit)
  {
    ml4kp_bridge::copy(_controller, plan_in);
    ml4kp_bridge::copy(_state, x_hat);

    // Assuming the "controller" is really a container of controller, aka a set {u_0(\cdot), u_1(\cdot), \dots}
    // Where u_0 could be valid from [0,t_0], t_0 > simulation_step.
    // The Split function divides u_0 into as many u_0^j such that each u_0^j is in [t_i, t_j],
    // where t_j - t_i = simulation_step
    // ml4kp_bridge::split(_controller, _split_time);

    _w_sampler.set(w_noise);

    std::swap(_cells_buffer, _used_cells);

    const Tangent cell_size{ Tangent::Ones() * _cell_size };

    const State x_d2{ gtsam::traits<State>::Expmap(-cell_size / 2.0) };
    const State x0_center{ gtsam::traits<State>::Compose(_state, x_d2) };

    // DEBUG_VARS(_state, x0_center, cell_size)
    _grid.reset(_state, _cell_size / 2.);

    DEBUG_VARS(_grid.cell_sizes())
    _safe_radii.clear();

    _collision_found = false;
    _iter_idx++;

    // Get a nominal trajectory from x0
    // _traj_nominal.clear();
    // propagate(_traj_nominal, _state, _controller);

    _C2_x0 = distribution_bound(x0_noise);
    _C2_w = distribution_bound(2. * w_noise);
    _C2_u = distribution_bound(u_noise);

    // DEBUG_VARS(_cell_size, _C2_x0, _C2_w)

    const std::size_t total_threads{ _pool.get_thread_count() };

    _propagated_idx++;
    // Propagate \bar{x0} \in V(\xi)
    // _pool.detach_task([&] { this->propagate_cube(_state, _controller, 0); });
    _visited_idx++;
    const double chi2_critical_value{ _chi2->critical_value(DimX, _Chi2_alpha) };
    // DEBUG_VARS(chi2_critical_value, DimX, _Chi2_alpha)
    propagate_neighbors(_state, _state, x0_noise.inverse(), chi2_critical_value, _controller, 0);

    _pool.wait();

    if (_visualize)
    {
      visualization_msgs::Marker x0_noise_marker{ ml4kp_bridge::create_marker(0.01, /*color*/ { 0.3, 1, 0, 1 }) };
      x0_noise_marker.type = visualization_msgs::Marker::SPHERE;

      interface::gaussian_params_t gparams;
      gparams.cov_to_3Dellipse(x0_noise, true);
      gparams.confidence = chi2_critical_value;
      // DEBUG_VARS(chi2_critical_value, gparams.axis)

      interface::gaussian_to_ellipse_marker(x0_noise_marker, gparams);
      ml4kp_bridge::update_pose(x0_noise_marker.pose, _state, 0, 1, 0.01);
      _x0_noise_publisher.publish(x0_noise_marker);
      if (DimX == 2)
      {
        x0_noise_marker.scale.z = 0.01;
      }
    }

    _collision_msg.data = _collision_found;
    _collision_publisher.publish(_collision_msg);
    // _pool.detach_task([&] { this->grid_to_markers(); });
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
    // DEBUG_VARS(cell_size)

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

    _ofs_grid << "# First line: 'x0 cell_size' of grid (x0 is the x0 of the grid and the size of each cell).";
    _ofs_grid << "Then empty line and then N lines with 'center_state center_tangent vertex safe' ";
    _ofs_grid << "corresponding to the reachable set on the grid.\n";
    prx::to_stream(_ofs_grid, _grid.x0());
    prx::to_stream(_ofs_grid, _grid.cell_sizes());
    _ofs_grid << "\n\n";

    // prx::to_stream(_ofs, cell.second->state);
    // prx::to_stream(_ofs, cell.second->safe);
    // prx::to_stream(_ofs, cell.second->total_states);

    // DEBUG_VARS(cell_size.transpose(), _grid.size())
    // const bool x_sign{ plant_config[0].second[0] > 0 };
    // const bool y_sign{ plant_config[0].second[1] > 0 };
    for (auto cell : _grid)
    {
      // const State state{ cell.second->state };
      const State xv{ _grid.state_from_vertex(cell.second->vertex) };
      const Tangent center_tg{ _grid.center(xv) };
      const State center{ _grid.state(center_tg) };
      // const State center{ _grid.state(center_tg) };
      // LOG_VARS(center);
      // DEBUG_VARS(state, center)
      marker_lie.points.emplace_back();
      marker_state.points.emplace_back();
      // marker.points.back().x = center[0];  //- (x_sign ? 0. : cell_size[0]);
      // marker.points.back().y = center[1];  //- (y_sign ? 0. : cell_size[1]);
      // marker.points.back().z = -0.101;
      ml4kp_bridge::update_point(marker_lie.points.back(), center_tg, 0, 1, -0.051);
      ml4kp_bridge::update_point(marker_state.points.back(), center, 0, 1, -0.051);

      // prx::to_stream(_ofs_grid, cell.second->state);
      prx::to_stream(_ofs_grid, center);
      prx::to_stream(_ofs_grid, center_tg);
      prx::to_stream(_ofs_grid, cell.second->vertex);
      prx::to_stream(_ofs_grid, cell.second->safe);
      _ofs_grid << "\n";
      // DEBUG_VARS(cell.second->added_idx, cell.second->propagated_idx, cell.second->visited_idx)
    }
    // DEBUG_VARS(_grid.size(), marker_lie.points.back(), mar)
    // const Covariance identity{ Covariance::Identity() };
    // for (auto&& [idx, state, radius] : _safe_radii)
    // {
    //   auto vertex = _grid.vertex(state);
    //   // Go over every vertex neighbor: +1 and -1 per dimension
    //   for (int i = 0; i < DimX; ++i)
    //   {
    //     Tangent v_next{ vertex };
    //     v_next[i] += 1;
    //     const bool inside_p{ cell_intersects_with_ellipse(state, v_next, identity, radius) };
    //     ml4kp_bridge::update_point(marker_state.points.back(), center, 0, 1, -0.101);

    //     // As v_next[i] has already a +1, we need to remove it and do -1
    //     v_next[i] -= 2;
    //     cell_intersects_with_ellipse(state, v_next, identity, radius);
    //   }
    // }

    _cubes_algebra_publisher.publish(marker_lie);
    _cubes_state_publisher.publish(marker_state);
    // _cubes_publisher.publish(marker);
    _grid.clear();
    _ofs_grid.close();
    // DEBUG_PRINT
  }

  void radii_to_markers()
  {
    visualization_msgs::MarkerArray all_markers;
    int id{ 0 };
    // DEBUG_VARS(_safe_radii.size())
    for (auto&& [idx, state, radius] : _safe_radii)
    {
      visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.01, /*color*/ { 0.5, 1, 0, 0 }) };
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      ml4kp_bridge::update_pose(marker.pose, state, 0, 1, 0.0);
      marker.scale.x = marker.scale.y = marker.scale.z = 2 * radius;
      marker.id = id;
      id++;

      // DEBUG_VARS(idx, state, radius)
      prx::to_stream(_ofs_balls, idx);
      prx::to_stream(_ofs_balls, state);
      prx::to_stream(_ofs_balls, radius);
      // prx::to_stream(_ofs_balls, _trajectories.size());
      _ofs_balls << "\n";

      all_markers.markers.push_back(marker);
    }
    _ofs_balls.close();

    _radii_markers_publisher.publish(all_markers);
  }

  void trajectories_to_marker()
  {
    if (_visualize)
    {
      radii_to_markers();
      visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.001, /*color*/ { 1, 1, 0, 0 }) };
      visualization_msgs::Marker nominal_trajs_marker{ ml4kp_bridge::create_marker(0.005, { 1, 0.2, 0.6, 0 }) };
      visualization_msgs::Marker markers_x0s{ ml4kp_bridge::create_marker(0.01, { 1, 0.5, 0.0, 0.5 }) };

      markers_x0s.type = visualization_msgs::Marker::POINTS;
      marker.type = nominal_trajs_marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = nominal_trajs_marker.action = markers_x0s.action = visualization_msgs::Marker::DELETEALL;
      // _trajectory_markers.markers.push_back(marker);
      _markers_publisher.publish(marker);
      _markers_x0s_publisher.publish(markers_x0s);
      _traj_nominal_publisher.publish(nominal_trajs_marker);

      marker.action = nominal_trajs_marker.action = markers_x0s.action = visualization_msgs::Marker::ADD;
      // DEBUG_VARS(_trajectories.size())
      std::scoped_lock lock(_trajectories_mutex);
      int total_states{ 0 };
      for (auto traj_set : _trajectories)
      {
        std::stringstream strstr;
        strstr << _prefix << "_trajs_";
        strstr << _timestamp + "_";
        strstr << std::setfill('0') << std::setw(5) << traj_set.first;
        strstr << ".txt";

        std::ofstream ofs_traj(strstr.str());

        while (traj_set.second.size() > 0)
        {
          const Trajectory& traj{ traj_set.second.back() };
          total_states += traj.size();
          prx::to_stream(ofs_traj, traj);
          ofs_traj << "\n\n";

          ml4kp_bridge::update_marker(marker, traj, 0, 1, 0.0, visualization_msgs::Marker::LINE_LIST);

          markers_x0s.points.emplace_back();
          ml4kp_bridge::update_point(markers_x0s.points.back(), traj.front(), 0, 1, 0.0);

          traj_set.second.pop_back();
        }
      }

      DEBUG_VARS(total_states)
      for (auto traj_set : _nominal_trajs)
      {
        std::stringstream strstr;
        strstr << _prefix << "_nominal_trajs_";
        strstr << _timestamp + "_";
        strstr << std::setfill('0') << std::setw(5) << traj_set.first;
        strstr << ".txt";

        std::ofstream ofs_nominal_traj(strstr.str());
        while (traj_set.second.size() > 0)
        {
          const Trajectory& traj{ traj_set.second.back() };
          total_states += traj.size();
          prx::to_stream(ofs_nominal_traj, traj);
          ofs_nominal_traj << "\n\n";
          ml4kp_bridge::update_marker(nominal_trajs_marker, traj, 0, 1, 0.0, visualization_msgs::Marker::LINE_LIST);

          traj_set.second.pop_back();
        }
      }
      DEBUG_VARS(total_states)
      _ofs_stats << "total_states: " << total_states << "\n";
      _ofs_stats << "total_cells: " << _grid.size() << "\n";
      _ofs_stats << "cell_size: ";
      prx::to_stream(_ofs_stats, _grid.cell_sizes());
      _ofs_stats << "\n";
      _ofs_stats.close();
      // DEBUG_VARS(marker)
      _traj_nominal_publisher.publish(nominal_trajs_marker);
      _markers_publisher.publish(marker);
      _markers_x0s_publisher.publish(markers_x0s);
      // _trajectory_markers.markers.clear();
    }

    std::scoped_lock lock(_trajectories_mutex);
    _trajectories.clear();
    _nominal_trajs.clear();
    grid_to_markers();
  }

private:
  bool _visualize;
  std::atomic<int> _trajectories_to_viz;

  std::atomic<bool> _collision_found;
  std::atomic<int> _total_checked_trajectories;
  std::atomic<int> _unchecked_trajectories, _collisions_in_check;

  std::mutex _log_mutex;
  std::mutex _new_cell_mutex;
  std::mutex _trajectories_mutex, _queries_mutex, _nominal_trajectories_mutex;

  std::map<int, std::vector<Trajectory>> _trajectories, _nominal_trajs;
  // std::vector<Trajectory> _checked_trajectories;

  Controller _controller;

  std_msgs::Bool _collision_msg;
  ros::Publisher _markers_publisher, _cubes_algebra_publisher, _cubes_state_publisher, _collision_publisher;
  ros::Publisher _radii_markers_publisher, _traj_nominal_publisher, _markers_x0s_publisher, _x0_noise_publisher;

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

  // StateSampler _x0_sampler;
  StateSampler _w_sampler;

  double _cell_size;

  int _mg_step;
  std::size_t _iter_idx;
  std::vector<CellPtr> _cells_buffer, _used_cells;
  ImplicitGrid _grid;

  bool _short_circuit;

  std::ofstream _ofs_balls, _ofs_stats, _ofs_nominal_trajs, _ofs_grid;
  std::vector<std::tuple<int, State, double>> _safe_radii;

  // Trajectory _traj_nominal, _nominal_trajs;

  double _Chi2_alpha;
  double _C2_x0, _C2_w, _C2_u;

  double _split_time;
  std::size_t _propagated_idx, _visited_idx;

  std::shared_ptr<prx::chi_squared> _chi2;

  const Covariance _identity;

  std::string _timestamp, _prefix;

  std::set<std::size_t> _cells_hashes;
};
}  // namespace motion_planning
