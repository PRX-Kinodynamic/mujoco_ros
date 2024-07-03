#pragma once

// Ros
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/defs.h>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/factors/obstacle_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/simulation/system_factory.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

namespace prx
{
namespace fg
{

// LTV from CS-BRM: A Probabilistic RoadMap for Consistent Belief Space Planning With Reachability Guarantees
// https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10404055
// Eq. 50
// Setting h = 0
class ltv_sde_utils_t
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using MatrixG = Eigen::Matrix<double, 2, 2>;  // Dims could be parametric

  using State = Eigen::Vector<double, 2>;
  using StateDot = Eigen::Vector<double, 2>;
  using Control = Eigen::Vector<double, 2>;
  using Noise = Eigen::Vector<double, 2>;
  using Observation = Eigen::Vector<double, 2>;

  using StateKeys = std::array<gtsam::Key, 2>;
  using ControlKeys = std::array<gtsam::Key, 1>;

  using StateEstimates = std::tuple<State, StateDot>;
  using ControlEstimates = std::tuple<Control>;

  static constexpr std::string_view plant_name = "fg_ltv_sde";

  using EulerFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
  using DynamicsFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
  using ObservationFactor = gtsam::PriorFactor<State>;

  // KeyU = U^{level}_{step}
  static gtsam::Key keyU(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("U^{", level, "}_{", step, "}");
  }

  // KeyX = X^{level}_{step}
  static gtsam::Key keyX(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("X^{", level, "}_{", step, "}");
  }

  // KeyXdot = X^{level}_{step}
  static gtsam::Key keyXdot(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("\\dot{X}^{", level, "}_{", step, "}");
  }

  static StateKeys keyState(const int& level, const int& step)
  {
    return { keyX(level, step), keyXdot(level, step) };
  }

  static StateKeys keyControl(const int& level, const int& step)
  {
    return { keyU(level, step) };
  }

  static void state(State& x, const ml4kp_bridge::SpacePoint& pt)
  {
    x[0] = pt.point[0];
    x[1] = pt.point[1];
  }

  static void copy(Control& u, const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    u[0] = msg->point[0];
    u[1] = msg->point[1];
  }

  static void copy(Observation& z, const geometry_msgs::TransformStamped& tf)
  {
    z[0] = tf.transform.translation.x;
    z[1] = tf.transform.translation.y;
  }

  template <typename StateIn>
  static void copy(geometry_msgs::Transform& tf, const StateIn& x)
  {
    tf.translation.x = x[0];
    tf.translation.y = x[1];
    tf.translation.z = 0.0;
    tf.rotation.x = 0.0;
    tf.rotation.y = 0.0;
    tf.rotation.z = 0.0;
    tf.rotation.w = 1.0;
  }
  static void copy(ml4kp_bridge::SpacePoint& pt, const StateEstimates& estimates)
  {
    const State& x{ std::get<0>(estimates) };
    const StateDot& xdot{ std::get<1>(estimates) };

    pt.point.resize(4);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = xdot[0];
    pt.point[3] = xdot[1];
  }

  static void copy(ml4kp_bridge::SpacePoint& pt, const State& x, const StateDot& xdot)
  {
    pt.point.resize(4);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = xdot[0];
    pt.point[3] = xdot[1];
  }

  static void copy(geometry_msgs::PoseWithCovariance& pose_cov, const StateEstimates& estimates,
                   std::vector<Eigen::MatrixXd>& covs)
  {
    const State& x{ std::get<0>(estimates) };

    pose_cov.pose.position.x = x[0];
    pose_cov.pose.position.y = x[1];
    pose_cov.pose.position.z = 0.0;
    pose_cov.pose.orientation.w = 1.0;
    pose_cov.pose.orientation.x = 0.0;
    pose_cov.pose.orientation.y = 0.0;
    pose_cov.pose.orientation.z = 0.0;

    for (int i = 0; i < 36; ++i)
    {
      pose_cov.covariance[i] = i;
    }

    const Eigen::Matrix2d cov{ covs[0] };
    for (int i = 0; i < 2; ++i)
    {
      for (int j = 0; j < 2; ++j)
      {
        pose_cov.covariance[6 * i + j] = cov(i, j);
      }
    }
  }

  static void control_vizualization(geometry_msgs::Point& endpoint, const ml4kp_bridge::SpacePoint& msg)
  {
    Control u(msg.point[0], msg.point[1]);
    u.normalize();

    endpoint.x = u[0];
    endpoint.y = u[1];
    endpoint.z = 0.0;
  }

  static std::shared_ptr<prx::fg::collision_info_t> collision_geometry()
  {
    const std::string name{ plant_name };

    std::shared_ptr<prx::plant_t> plant{ prx::system_factory_t::create_system_as<prx::plant_t>(name, name) };
    prx_assert(plant != nullptr, "Plant " << plant_name << " couldn't be constructed!");

    prx::movable_object_t::Geometries geometries{ plant->get_geometries() };
    prx::movable_object_t::Configurations configurations{ plant->get_configurations() };

    const std::size_t total_geoms{ geometries.size() };
    // prx_assert(total_geoms == 1, "More than 1 geometry not supported");
    std::shared_ptr<prx::geometry_t> g{ geometries[0].second };
    std::shared_ptr<prx::transform_t> tf{ configurations[0].second };

    const prx::geometry_type_t g_type{ g->get_geometry_type() };
    const std::vector<double> g_params{ g->get_geometry_params() };

    const Eigen::Matrix3d rot{ tf->rotation() };
    const Eigen::Vector3d t{ tf->translation() };

    return std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t);
  }

  struct ConfigFromState
  {
    void operator()(Eigen::Matrix3d& rotation, Eigen::Vector3d& translation, const State& state)
    {
      rotation = Eigen::Matrix3d::Identity();
      translation[0] = state[0];
      translation[1] = state[1];
      translation[2] = 0;
    }

    void operator()(Eigen::MatrixXd& H, const Eigen::Vector3d& translation)
    {
      H = Eigen::Matrix2d::Identity();
      H.diagonal() = translation.head(2);
    }
  };
  /**
   * @brief compute the noise for the state from the matrix and a vector of sigmas
   * @details Compute the noise at state k given sigmas. The input sigmas is vector that is used as: w\sim N(0,\sigma_i)
   *
   * @param G Matrix that maps vector w\sim N(0,\sigma s) to the state. Likely a diagonal matrix
   * @param sigmas The variance per dimension.
   *
   * @return Computes $G * w$ where $w \sim N(0,\sigma)$
   */
  inline static Noise noise(const MatrixG& G, const Noise& sigmas)
  {
    const Noise w{ prx::gaussian_random(0.0, sigmas[0]), prx::gaussian_random(0.0, sigmas[1]) };
    return G * w;
  }

  static double DistanceFunction(const ml4kp_bridge::SpacePoint& a, const ml4kp_bridge::SpacePoint& b)
  {
    const double diff0{ a.point[0] - b.point[0] };
    const double diff1{ a.point[1] - b.point[1] };
    return std::sqrt(std::pow(diff0, 2) + std::pow(diff1, 2));
  }
  // class trajectory_estimation
  // {
  /**
    Factor graph at some time t:
                                 (Integration)
                           (x_t) ----- * ----- (x_{t+1})
                                       |
                     (Dynamic)         |
    (\dot{x_{t}}) ----- * ----- (\dot{x_{t+1}})
                        |
                        |
                (\ddot{x_t} = u_t)
   */

  /**
   * @brief Creates the small factor graph above: between t and t+1
   * @details Create a small factor graph x_{t+1} <- x_t + xdot * dt AND \dot{x}_{t+1} <- \dot{x}_t + \ddot{x} * dt  AND
   * \hat{x}_t <- x_t + noise ~= z intended to be an update to isam2, accounting for a new observation of a state.
   *
   * @param idx index of the observation. First observation is always 0
   * @param dt The time between x[idx] and x[idx+1]
   * @param keyX A function that produces a key for X taking as input idx
   * @param keyXdot A function that produces a key for Xdot taking as input idx
   * @param integration_noise Noise model to use for integration
   * @param observation_noise Noise model to use for observation
   * @tparam IntegrationFactor Integration class that does x_{t+1} <- x_t + xdot * dt in that order: (x(idx+1), x(idx) +
   * xdot(idx))
   * @return A factor graph
   */

  // The necessary info to create the FG from X_t to X_{t+1}
  struct local_update_t
  {
    local_update_t()
      : estimate(false), integration_noise(nullptr), dynamic_noise(nullptr), observation_noise(nullptr){};

    void init(prx::param_loader& params)
    {
      // const std::vector<double> integration_noise_vec{ params["integration_noise"].as<std::vector<double>>() };
      // const std::vector<double> dynamic_noise_vec{ params["dynamic_noise"].as<std::vector<double>>() };
      // const std::vector<double> observation_noise_vec{ params["observation_noise"].as<std::vector<double>>() };

      // prx_assert(integration_noise_vec.size() == 2,
      //            "Wrong size of integration noise, expected 2 but got " << integration_noise_vec.size());
      // prx_assert(dynamic_noise_vec.size() == 2,
      //            "Wrong size of integration noise, expected 2 but got " << dynamic_noise_vec.size());
      // prx_assert(observation_noise_vec.size() == 2,
      //            "Wrong size of integration noise, expected 2 but got " << observation_noise_vec.size());

      // integration_noise = gtsam::noiseModel::Diagonal::Sigmas(integration_noise_vec);
      // dynamic_noise = gtsam::noiseModel::Diagonal::Sigmas(dynamic_noise_vec);
      // observation_noise = gtsam::noiseModel::Diagonal::Sigmas(observation_noise_vec);
    }

    NoiseModel integration_noise;
    NoiseModel dynamic_noise;
    NoiseModel observation_noise;

    std::size_t parent{};
    // std::size_t edge{};
    std::size_t child{};
    double dt{};
    // level:
    // If (-\inf,-1] ==> estimation (branch is in the past)
    // If [0,\inf) ==> plan (branch is in the future)
    int level{};
    bool estimate{};

    Observation observation;
    Control control;
    StateDot xdot;
    State x;
  };

  static void to_file(const std::string filename, const FactorGraph& factor_graph, const Values& values,
                      const std::ios_base::openmode _mode = std::ofstream::trunc)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    std::ofstream ofs(filename, _mode);

    for (auto factor_ptr : factor_graph)
    {
      auto euler_factor = boost::dynamic_pointer_cast<EulerStateStateDotFactor>(factor_ptr);
      if (euler_factor)
      {
        euler_factor->to_stream(ofs, values);
      }
    }
  }

  static GraphValues root_factor_graph(const local_update_t& update)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    GraphValues graph_values;

    int estimation{ update.estimate ? -1 : +1 };
    const gtsam::Key x0{ keyX(estimation, update.parent) };
    const gtsam::Key xdot0{ keyXdot(estimation, update.parent) };

    graph_values.first.addPrior(x0, update.x);
    graph_values.first.addPrior(xdot0, update.xdot);

    graph_values.second.insert(x0, update.x);
    graph_values.second.insert(xdot0, update.xdot);
    return graph_values;
  }

  static GraphValues local_factor_graph(const local_update_t& update)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(1, update.parent) };
    const gtsam::Key x1{ keyX(1, update.child) };
    const gtsam::Key xdot0{ keyXdot(1, update.parent) };
    const gtsam::Key xdot1{ keyXdot(1, update.child) };
    const gtsam::Key u01{ keyU(update.parent, update.child) };

    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, update.integration_noise, update.dt,
                                                                "EulerX");
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u01, update.dynamic_noise, update.dt,
                                                                  "EulerXdot");
    graph_values.first.addPrior(x1, update.x);
    graph_values.first.addPrior(xdot1, update.xdot);
    graph_values.first.addPrior(u01, update.control);

    graph_values.second.insert(x1, update.x);
    graph_values.second.insert(xdot1, update.xdot);
    graph_values.second.insert(u01, update.control);
    return graph_values;
  }

  static GraphValues estimation_local_factor_graph(const local_update_t& update)
  {
    using ObservationFactor = gtsam::PriorFactor<State>;
    prx_assert(update.level < 0, "Positive branch on estimation FG");

    const gtsam::Key x0{ keyX(update.parent, update.child) };
    GraphValues graph_values{ local_factor_graph(update) };
    graph_values.first.emplace_shared<ObservationFactor>(x0, update.observation, update.observation_noise);

    return graph_values;
  }

  static GraphValues add_observation_factor(const std::size_t prev_id, const std::size_t curr_id,
                                            const StateEstimates& estimates, const Control u_prev, const Observation& z,
                                            const double dt, const double z_noise)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    using ObservationFactor = gtsam::PriorFactor<State>;

    const State& x0_value{ std::get<0>(estimates) };
    const StateDot& xdot0_value{ std::get<1>(estimates) };
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(-1, prev_id) };
    const gtsam::Key x1{ keyX(-1, curr_id) };
    const gtsam::Key xdot0{ keyXdot(-1, prev_id) };
    const gtsam::Key xdot1{ keyXdot(-1, curr_id) };
    const gtsam::Key u01{ keyU(-1 * prev_id, -1 * curr_id) };

    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };
    NoiseModel dynamic_noise{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };
    NoiseModel observation_noise{ gtsam::noiseModel::Isotropic::Sigma(2, z_noise) };

    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, integration_noise, dt);
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u01, dynamic_noise, dt);
    graph_values.first.emplace_shared<ObservationFactor>(x0, z, observation_noise);
    graph_values.first.addPrior(u01, u_prev);

    const StateDot p_xdot1{ EulerStateDotControlFactor::predict(xdot0_value, u_prev, dt) };
    const StateDot p_x1{ EulerStateStateDotFactor::predict(x0_value, xdot0_value, dt) };

    graph_values.second.insert(u01, u_prev);
    graph_values.second.insert(xdot1, p_xdot1);
    graph_values.second.insert(x1, p_x1);
    return graph_values;
  }

  static GraphValues root_to_fg(const std::size_t root, const ml4kp_bridge::SpacePoint& node_state,
                                const bool estimation = false)
  {
    local_update_t update{};

    update.parent = root;
    update.estimate = estimation;

    update.x[0] = node_state.point[0];
    update.x[1] = node_state.point[1];

    update.xdot[0] = node_state.point[2];
    update.xdot[1] = node_state.point[3];

    return root_factor_graph(update);
  }
  // Take a SBMP edge-node and convert it to FG:
  // SBMP:  N0 ----E01---- N1 which is: N0=traj[0]; E01=plan
  // FG:    X0 ----F01---- X1
  static GraphValues node_edge_to_fg(const std::size_t parent, const std::size_t child,
                                     const ml4kp_bridge::SpacePoint& node_state, const ml4kp_bridge::Plan& edge_plan)
  {
    const ml4kp_bridge::SpacePoint& edge_control{ edge_plan.steps[0].control };
    const double duration{ edge_plan.steps[0].duration.data.toSec() };

    local_update_t update{};

    update.dt = duration;

    const double numerical_error{ update.dt };
    update.integration_noise = gtsam::noiseModel::Isotropic::Sigma(2, numerical_error);
    update.dynamic_noise = gtsam::noiseModel::Isotropic::Sigma(2, numerical_error);

    update.parent = parent;
    update.child = child;

    // const Eigen::Vector4d state{ Vec(traj.front()) };
    update.control[0] = edge_control.point[0];
    update.control[1] = edge_control.point[1];

    update.x[0] = node_state.point[0];
    update.x[1] = node_state.point[1];

    update.xdot[0] = node_state.point[2];
    update.xdot[1] = node_state.point[3];
    // update.xdot = node_state.tail(2);

    return local_factor_graph(update);
  };

  static GraphValues trajectory_to_fg(const std::size_t parent, const std::size_t child,
                                      const ml4kp_bridge::SpacePoint& node_state, const ml4kp_bridge::Plan& plan)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;

    const ml4kp_bridge::SpacePoint& edge_control{ plan.steps[0].control };

    GraphValues graph_values;

    const gtsam::Key x0{ keyX(1, parent) };
    const gtsam::Key x1{ keyX(1, child) };
    const gtsam::Key xdot0{ keyXdot(1, parent) };
    const gtsam::Key xdot1{ keyXdot(1, child) };
    const gtsam::Key u01{ keyU(parent, child) };

    // DEBUG_VARS(SF::formatter(x0), x0);
    // DEBUG_VARS(SF::formatter(x1), x1);
    // DEBUG_VARS(SF::formatter(xdot0), xdot0);
    // DEBUG_VARS(SF::formatter(xdot1), xdot1);
    // DEBUG_VARS(SF::formatter(u01), u01);

    const Control control(edge_control.point[0], edge_control.point[1]);
    const State x(node_state.point[0], node_state.point[1]);
    // const StateDot xdot(node_state.point[2], node_state.point[3]);
    const StateDot xdot(0, 0);

    // DEBUG_VARS(control.transpose());
    // DEBUG_VARS(x.transpose());
    // DEBUG_VARS(xdot.transpose());

    const double duration{ plan.steps[0].duration.data.toSec() };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(2, duration) };
    NoiseModel dynamic_noise{ gtsam::noiseModel::Isotropic::Sigma(2, duration) };
    NoiseModel quadratic_cost_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1) };

    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, integration_noise, duration, "EulerX");
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u01, dynamic_noise, duration,
                                                                  "EulerXdot");

    // quadratic_cost_factor_t(const gtsam::Key& key, const Matrix cost, const NoiseModel& cost_model)

    const Eigen::Matrix2d qcost{ Eigen::Matrix2d::Identity() };
    const Eigen::Vector2d goal(20, 20);
    // graph_values.first.emplace_shared<prx::fg::quadratic_cost_factor_t<State>>(x1, qcost, goal, nullptr);
    graph_values.first.emplace_shared<prx::fg::quadratic_cost_factor_t<StateDot>>(xdot1, qcost, quadratic_cost_noise);
    graph_values.first.emplace_shared<prx::fg::quadratic_cost_factor_t<Control>>(u01, qcost, quadratic_cost_noise);
    // graph_values.first.addPrior(xdot1, xdot);
    // graph_values.first.addPrior(u01, control);

    graph_values.second.insert(x1, x);
    graph_values.second.insert(xdot1, xdot);
    graph_values.second.insert(u01, control);
    return graph_values;
  }

  static GraphValues add_fix_cost(const std::size_t goal_id, const ml4kp_bridge::SpacePoint& node_state)
  {
    const gtsam::Key xF{ keyX(1, goal_id) };
    const gtsam::Key xdotF{ keyXdot(1, goal_id) };

    const State x(node_state.point[0], node_state.point[1]);
    const StateDot xdot(node_state.point[2], node_state.point[3]);
    // const StateDot xdot(0.5, 0.5);

    // DEBUG_VARS(SF::formatter(xF), xF);
    // DEBUG_VARS(SF::formatter(xdotF), xdotF);
    gtsam::noiseModel::Base::shared_ptr goal_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e-5) };
    GraphValues graph_values;

    const Eigen::Matrix2d qcost{ Eigen::Matrix2d::Identity() * 100 };
    graph_values.first.emplace_shared<prx::fg::quadratic_cost_factor_t<State>>(xF, qcost, x, nullptr);
    graph_values.first.emplace_shared<prx::fg::quadratic_cost_factor_t<State>>(xdotF, qcost, xdot, nullptr);
    // graph_values.first.addPrior(xF, x, goal_noise);
    // graph_values.first.addPrior(xdotF, xdot, goal_noise);
    return graph_values;
  }

  // Local adaptation: Given that we are somewhere in between nodes X0 and X1, connect to X1 given Z
  static GraphValues local_adaptation(const std::size_t current, const std::size_t next, const Control control,
                                      const double dt)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(-1, current) };
    const gtsam::Key xdot0{ keyXdot(-1, current) };

    const gtsam::Key x1{ keyX(1, next) };
    const gtsam::Key xdot1{ keyXdot(1, next) };

    const gtsam::Key u01{ keyU(-1 * current, next) };

    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };
    NoiseModel dynamic_noise{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };

    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, integration_noise, dt, "EulerX");
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u01, dynamic_noise, dt, "EulerXdot");
    graph_values.first.addPrior(u01, control);

    graph_values.second.insert(u01, control);

    return graph_values;
  }

  static bool goal_check(const ml4kp_bridge::SpacePoint& xi, const ml4kp_bridge::SpacePoint& goal)
  {
    const State x(xi.point[0], xi.point[1]);
    const State xg(goal.point[0], goal.point[1]);
    return (x - xg).norm() < 0.2;
  }

  static GraphValues graph_edge_to_factor_graph(const std::size_t id_A, const std::size_t id_B,
                                                // const ml4kp_bridge::SpacePoint& node_state,
                                                const ml4kp_bridge::Plan& edge_plan)
  {
    using Matrix = Eigen::Matrix<double, 2, 2>;
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    using QuadraticCostFactor = prx::fg::quadratic_cost_factor_t<Control>;

    GraphValues graph_values;

    const ml4kp_bridge::SpacePoint& edge_control{ edge_plan.steps[0].control };
    const double duration{ edge_plan.steps[0].duration.data.toSec() };
    const Control control{ edge_control.point[0], edge_control.point[1] };

    const gtsam::Key x0{ keyX(1, id_A) };
    const gtsam::Key x1{ keyX(1, id_B) };
    const gtsam::Key xdot0{ keyXdot(1, id_A) };
    const gtsam::Key xdot1{ keyXdot(1, id_B) };
    const gtsam::Key u01{ keyU(id_A, id_B) };

    const NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(2, duration) };
    const NoiseModel dynamic_noise{ gtsam::noiseModel::Isotropic::Sigma(2, duration) };
    const NoiseModel cost_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1) };

    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, integration_noise, duration, "EulerX");
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u01, dynamic_noise, duration,
                                                                  "EulerXdot");

    graph_values.first.emplace_shared<QuadraticCostFactor>(u01, Matrix::Identity(), cost_noise);
    graph_values.second.insert(u01, control);

    return graph_values;
  }

  static GraphValues graph_node_to_factor_graph(const std::size_t node_id, const ml4kp_bridge::SpacePoint& node_state)
  {
    using Matrix = Eigen::Matrix<double, 2, 2>;
    using QuadraticCostFactor = prx::fg::quadratic_cost_factor_t<State>;

    GraphValues graph_values;

    const gtsam::Key key_x{ keyX(1, node_id) };
    const gtsam::Key key_xdot{ keyXdot(1, node_id) };

    const State x(node_state.point[0], node_state.point[1]);
    const StateDot xdot{ node_state.point[2], node_state.point[3] };

    const NoiseModel cost_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1) };
    graph_values.first.addPrior(key_x, x);
    graph_values.first.emplace_shared<QuadraticCostFactor>(key_xdot, Matrix::Identity(), cost_noise);
    // graph_values.first.addPrior(xdot1, xdot);
    // graph_values.first.addPrior(u01, update.control);

    graph_values.second.insert(key_x, x);
    graph_values.second.insert(key_xdot, xdot);

    return graph_values;
  }
};

class fg_ltv_sde_t : public plant_t
{
  using State = fg::ltv_sde_utils_t::State;
  using StateDot = fg::ltv_sde_utils_t::StateDot;
  using Control = fg::ltv_sde_utils_t::Control;
  using Noise = fg::ltv_sde_utils_t::Noise;

  using EulerFactor = prx::fg::euler_integration_factor_t<State, StateDot>;

public:
  fg_ltv_sde_t(const std::string& path)
    : plant_t(path)
    , _x(State::Zero())
    , _u(Control::Zero())
    , _Gx(Eigen::DiagonalMatrix<double, 2>(5, 8) * 0.01)
    , _Gxdot(Eigen::DiagonalMatrix<double, 2>(5, 5) * 0.01)
    , _wx_sigmas(Noise::Ones())
    , _wxdot_sigmas(Noise::Ones())
  {
    state_memory = { &_x[0], &_x[1], &_xdot[0], &_xdot[1] };
    state_space = new space_t("EEEE", state_memory, "state_space");

    control_memory = { &_u[0], &_u[1] };
    input_control_space = new space_t("EE", control_memory, "control_space");

    derivative_memory = { &_xdot[0], &_xdot[1], &_u[0], &_u[1] };
    derivative_space = new space_t("EEEE", derivative_memory, "deriv_space");

    parameter_memory = { &_wx_sigmas[0], &_wx_sigmas[1], &_wxdot_sigmas[0], &_wxdot_sigmas[1],
                         &_Gx(0, 0),     &_Gx(1, 1),     &_Gxdot(0, 0),     &_Gxdot(1, 1) };
    parameter_space = new space_t("EEEEEEEE", parameter_memory, "params");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
    geometries["body"]->initialize_geometry({ 0.5 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();
  }

  virtual ~fg_ltv_sde_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    compute_derivative();
    _x = EulerFactor::integrate(_x, _xdot, prx::simulation_step);
    _x += fg::ltv_sde_utils_t::noise(_Gx, _wx_sigmas);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->setIdentity();
    body->translation() = Eigen::Vector3d(_x[0], _x[1], 0.5);
  }

protected:
  virtual void compute_derivative() override final
  {
    if (not _u.isZero(1e-3))
    {
      _xdot = EulerFactor::integrate(_xdot, _u, prx::simulation_step);
      _xdot += fg::ltv_sde_utils_t::noise(_Gxdot, _wxdot_sigmas);
    }
    // _xdot = xdot_next + noise;
    // DEBUG_VARS(_xdot.transpose(), "=", xdot_next.transpose(), noise.transpose());
  }

  State _x;
  StateDot _xdot;
  Control _u;
  Noise _wx_sigmas;
  Noise _wxdot_sigmas;
  fg::ltv_sde_utils_t::MatrixG _Gx;
  fg::ltv_sde_utils_t::MatrixG _Gxdot;
};

}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(fg::fg_ltv_sde_t, fg_ltv_sde)
