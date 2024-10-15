#pragma once

// Ros
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

namespace prx_models
{
class mushrFG_t;

class mushr_utils_t
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

  using XVelFactor = prx_models::mushr_x_xdot_t;
  using VelUbarFactor = prx_models::mushr_xdot_ub_t;
  using CtrlUbarFactor = prx_models::mushr_ub_u_xdot_t;

public:
  static constexpr std::string_view plant_name = "mushrFG";
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = prx_models::mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using StateDotDot = mushr_types::StateDot::type;

  using Ubar = mushr_types::Ubar::type;
  using Control = mushr_types::Control::type;
  using Noise = Eigen::Vector<double, 2>;
  using Observation = State;

  using StateKeys = std::array<gtsam::Key, 3>;
  using ControlKeys = std::array<gtsam::Key, 1>;

  using StateEstimates = std::tuple<State, StateDot>;
  using ControlEstimates = std::tuple<Control>;

  using Parameters = mushr_types::Control::params;
  using PrxPlant = mushrFG_t;

  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };

  static gtsam::Key keyU(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("U^{", level, "}_{", step, "}");
  }

  static gtsam::Key keyT(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("t^{", level, "}_{", step, "}");
  }

  static gtsam::Key keyUbar(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("\\bar{U}^{", level, "}_{", step, "}");
  }

  static gtsam::Key keyParams(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("p^{", level, "}_{", step, "}");
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

  static gtsam::Key keyXdotdot(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("\\ddot{X}^{", level, "}_{", step, "}");
  }

  static StateKeys keyState(const int& level, const int& step)
  {
    return { keyX(level, step), keyXdot(level, step), keyUbar(level, step) };
  }

  static StateKeys keyControl(const int& level, const int& step)
  {
    return { keyU(level, step) };
  }
  static void copy(Control& u, const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    u[velocity_idx] = msg->point[velocity_idx];
    u[steering_idx] = msg->point[steering_idx];
  }

  static void copy(Observation& z, const geometry_msgs::TransformStamped& tf)
  {
    z[0] = tf.transform.translation.x;
    z[1] = tf.transform.translation.y;
    const Eigen::Quaterniond q{ Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                                                   tf.transform.rotation.y, tf.transform.rotation.z) };
    z[2] = prx::quaternion_to_euler(q)[2];
    // DEBUG_VARS(q)
  }

  template <typename StateIn>
  static void copy(geometry_msgs::Transform& tf, const StateIn& x)
  {
    // z[0] = tf.transform.translation.x;
    // z[1] = tf.transform.translation.y;
    // const Eigen::Quaterniond q{ Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
    //                                                tf.transform.rotation.y, tf.transform.rotation.z) };
    // z[2] = prx::quaternion_to_euler(q)[2];

    const Eigen::Quaterniond q{ Eigen::AngleAxisd(x[2], Eigen::Vector3d::UnitZ()) };

    tf.translation.x = x[0];
    tf.translation.y = x[1];
    tf.translation.z = 0.0;
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();
  }

  static void copy(ml4kp_bridge::SpacePoint& pt, const StateEstimates& estimates)
  {
    const State& x{ std::get<0>(estimates) };
    const StateDot& xdot{ std::get<1>(estimates) };
    // const Ubar& ubar{ std::get<2>(estimates) };

    pt.point.resize(6);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = x[2];
    pt.point[3] = xdot[0];
    pt.point[4] = xdot[1];
    pt.point[5] = xdot[2];
  }

  static void copy(Control& u, const ml4kp_bridge::SpacePoint& msg)
  {
    u[mushr_types::Control::vel_desired] = msg.point[mushr_types::Control::vel_desired];
    u[mushr_types::Control::steering] = msg.point[mushr_types::Control::steering];
  }

  static void state(State& x, const ml4kp_bridge::SpacePoint& pt)
  {
    x[0] = pt.point[0];
    x[1] = pt.point[1];
    x[2] = pt.point[2];
  }

  static void stateDot(StateDot& xd, const ml4kp_bridge::SpacePoint& pt)
  {
    xd[0] = pt.point[3];
    xd[1] = pt.point[4];
    xd[2] = pt.point[5];
  }

  static void stateDotDot(StateDotDot& xdd, const ml4kp_bridge::SpacePoint& pt)
  {
    xdd[0] = pt.point[6];
    xdd[1] = pt.point[7];
    xdd[2] = pt.point[8];
  }

  static double node_distance(const ml4kp_bridge::SpacePoint& pt, const Observation& z)
  {
    State x{};
    state(x, pt);

    const State between{ x.between(z) };
    const Eigen::VectorXd error{ State::Logmap(between) };
    return error.norm();
  }

  static void control_vizualization(Eigen::Vector3d& endpoint, const ml4kp_bridge::SpacePoint& msg)
  {
    // Control u(msg.point[0], msg.point[1]);
    // u.normalize();
    // DEBUG_VARS(msg.point);
    const Eigen::Rotation2D<double> R(msg.point[mushr_types::Control::steering]);
    const Eigen::Vector2d u{ R * Eigen::Vector2d(0.25 + mushr_types::Control::vel_desired, 0) };
    // DEBUG_VARS(R.toRotationMatrix());
    // DEBUG_VARS(u.transpose());
    endpoint[0] = u[0];
    endpoint[1] = u[1];
    endpoint[2] = 0.0;
  }

  struct ConfigFromState
  {
    void operator()(Eigen::Matrix3d& rotation, Eigen::Vector3d& translation, const State& state)
    {
      const Eigen::Vector<double, 1> vec{ state.angle() };
      rotation = prx::euler_to_rotation<Eigen::Matrix3d>(vec, "Z");
      translation[0] = state[0];
      translation[1] = state[1];
      translation[2] = 0;
    }

    // void operator()(const Eigen::Vector3d& translation, const State& state, Eigen::MatrixXd& H)
    // void operator()(const State& state, const Eigen::Vector3d& translation, Eigen::MatrixXd& H)
    // {
    //   const Eigen::Vector2d normalized{ translation.head(2).normalized() };
    //   H = Eigen::Matrix<double, 1, 3>::Zero();
    //   H(0, 0) = normalized[0];
    //   H(0, 1) = normalized[1];
    // }

    void operator()(const bool collision, const State& state, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                    Eigen::MatrixXd& H)
    {
      // H = Eigen::Matrix<double, 1, 2>::Zero();
      H = Eigen::Matrix<double, 1, 3>::Zero();
      Eigen::Vector2d vec{ (p2 - p1).head(2) };
      if (collision)
        vec = p1.head(2) - state.translation();
      H(0, 0) = -vec[0];
      H(0, 1) = -vec[1];
    }
  };

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

  static GraphValues add_observation_factor(const std::size_t prev_id, const std::size_t curr_id,
                                            const StateEstimates& estimates, const Control u_prev,
                                            const Observation& zx, const double dt, const double z_noise)
  {
    using ObservationFactor = prx::fg::lie_ode_observation_factor_t<State, StateDot>;
    // using ControlObservationFactor = prx::fg::lie_ode_observation_factor_t<Ubar, Control>;
    // using MushrObservationFactor = prx_models::mushr_observation_factor_t;
    const State& x0_value{ std::get<0>(estimates) };
    const StateDot& xdot0_value{ std::get<1>(estimates) };
    // const Ubar& ubar0_value{ std::get<2>(estimates) };
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(1, prev_id) };
    const gtsam::Key x1{ keyX(1, curr_id) };
    const gtsam::Key xdot0{ keyXdot(1, prev_id) };
    const gtsam::Key xdot1{ keyXdot(1, curr_id) };
    const gtsam::Key ubar0{ keyUbar(1, prev_id) };
    const gtsam::Key ubar1{ keyUbar(1, curr_id) };
    const gtsam::Key u01{ keyU(prev_id, curr_id) };

    NoiseModel observation_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1.0e-0) };
    NoiseModel control_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1.0e-0) };

    graph_values.first.emplace_shared<ObservationFactor>(x0, xdot0, observation_noise, zx, dt);
    // graph_values.first.emplace_shared<MushrObservationFactor>(x0, ubar0, zx, u_prev, dt, default_params,
    // observation_noise);
    if (first == prev_id)
    {
      // first = false;
      graph_values.first.addPrior(u01, u_prev, control_noise);
    }
    else
    {
      first = prev_id;
    }

    return graph_values;
  }

  // Create a FG that goes from N0 to N1 with plan P01
  static GraphValues node_edge_to_fg(const std::size_t parent, const std::size_t child,
                                     const ml4kp_bridge::SpacePoint& node_state, const ml4kp_bridge::Plan& edge_plan)
  {
    // using StateStateDotFactor = prx::fg::lie_integration_factor_t<State, StateDot, double>;
    using StateStateDotFactor = prx_models::mushr_x_xdot_t;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
    using XdotIntegrationFactor = prx_models::mushr_CtrlAccel_t;
    using NHCFactor = prx_models::mushr_NHC_t;

    const ml4kp_bridge::SpacePoint& edge_control{ edge_plan.steps[0].control };
    const double dt{ edge_plan.steps[0].duration.data.toSec() };
    // using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control, double>;
    State x1;
    StateDot xdot1;
    StateDotDot xdot0;
    Control u01;
    // Ubar ubar0, ubar1;

    mushr_utils_t::state(x1, node_state);
    mushr_utils_t::stateDot(xdot1, node_state);
    // mushr_utils_t::stateDotDot(xdot0, node_state);
    u01[0] = edge_control.point[0];
    u01[1] = edge_control.point[1];

    GraphValues graph_values{ aux_graph };
    aux_graph.first.erase(aux_graph.first.begin(), aux_graph.first.end());
    aux_graph.second.clear();

    const gtsam::Key k_x0{ keyX(1, parent) };
    const gtsam::Key k_x1{ keyX(1, child) };

    const gtsam::Key k_xdot0{ keyXdot(1, parent) };
    const gtsam::Key k_xdot1{ keyXdot(1, child) };

    // const gtsam::Key k_xdotdot0{ keyXdotdot(1, parent) };
    // const gtsam::Key k_ubar0{ keyUbar(1, parent) };
    // const gtsam::Key k_ubar1{ keyUbar(1, child) };

    const gtsam::Key k_u01{ keyU(parent, child) };
    const gtsam::Key k_t01{ keyT(parent, child) };

    NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 5e0) };
    NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-0) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };

    graph_values.first.emplace_shared<StateStateDotFactor>(k_x1, k_x0, k_xdot0, k_t01, integration_noise, "MushrXXdot");
    graph_values.first.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_noise);
    aux_graph.first.emplace_shared<XdotIntegrationFactor>(k_xdot1, k_xdot0, k_u01, k_t01, integration_noise,
                                                          default_params);
    aux_graph.first.emplace_shared<NHCFactor>(k_xdot1, k_u01, nullptr, default_params);
    // graph_values.first.emplace_shared<mushr_u_xddot01_t>(k_xdot1, k_xdot0, k_t01, k_u01, default_params, nullptr);

    // _state_dot = mushr_u_xddot01_t::predict(_state_dot, simulation_step, _ctrl, _params_ubar_u);
    // aux_graph.first.emplace_shared<mushr_u_xddot01_t>(k_xdot1, k_xdot0, k_t01, k_u01, default_params, nullptr);
    // aux_graph.first.emplace_shared<mushr_ub_u_xdot_t>(k_ubar1, k_u01, k_ubar0, k_t01, default_params, nullptr);
    // aux_graph.first.emplace_shared<mushr_xdot_ub_t>(k_xdot1, k_ubar1, nullptr);

    graph_values.first.addPrior(k_t01, dt, dt_noise);
    graph_values.first.addPrior(k_x1, x1, prior_noise);
    // graph_values.first.addPrior(k_xdot1, xdot1, xdot_prior_noise);
    // graph_values.first.addPrior(k_ubar1, ubar1);
    graph_values.second.insert(k_x1, x1);
    graph_values.second.insert(k_t01, dt);

    aux_graph.first.addPrior(k_u01, u01, u_prior_noise);
    aux_graph.second.insert(k_xdot1, xdot1);
    aux_graph.second.insert(k_u01, u01);

    // graph_values.second.insert(k_xdot1, xdot1);
    // graph_values.second.insert(k_u01, u01);

    // aux_graph.first.addPrior(k_xdot1, xdot1);

    // aux_graph.second.insert(k_xdot1, xdot1);
    // aux_graph.second.insert(k_u01, u01);

    return graph_values;
  };

  static GraphValues leaf_to_fg(const std::size_t parent, const std::size_t leaf,
                                const ml4kp_bridge::SpacePoint& node_state, const ml4kp_bridge::Plan& edge_plan)
  {
    using StateStateDotFactor = prx_models::mushr_x_xdot_t;
    using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
    using XdotIntegrationFactor = prx_models::mushr_CtrlAccel_t;

    const ml4kp_bridge::SpacePoint& edge_control{ edge_plan.steps[0].control };
    const double dt{ edge_plan.steps[0].duration.data.toSec() };

    State x1;
    StateDot xdot1;
    StateDotDot xdot0;
    Control u01;

    mushr_utils_t::state(x1, node_state);
    mushr_utils_t::stateDot(xdot1, node_state);
    mushr_utils_t::stateDotDot(xdot0, node_state);
    u01[0] = edge_control.point[0];
    u01[1] = edge_control.point[1];

    GraphValues graph_values{ aux_graph };
    aux_graph.first.erase(aux_graph.first.begin(), aux_graph.first.end());
    aux_graph.second.clear();

    const gtsam::Key k_x0{ keyX(1, parent) };
    const gtsam::Key k_x1{ keyX(1, leaf) };

    const gtsam::Key k_xdot0{ keyXdot(1, parent) };
    const gtsam::Key k_xdot1{ keyXdot(1, leaf) };

    // const gtsam::Key k_xdotdot0{ keyXdotdot(1, parent) };
    // const gtsam::Key k_ubar0{ keyUbar(1, parent) };
    // const gtsam::Key k_ubar1{ keyUbar(1, leaf) };

    const gtsam::Key k_u01{ keyU(parent, leaf) };
    const gtsam::Key k_t01{ keyT(parent, leaf) };

    NoiseModel x_leaf_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-0) };
    NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };

    graph_values.first.emplace_shared<StateStateDotFactor>(k_x1, k_x0, k_xdot0, k_t01, integration_noise, "MushrXXdot");
    graph_values.first.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_noise);
    graph_values.first.emplace_shared<XdotIntegrationFactor>(k_xdot1, k_xdot0, k_u01, k_t01, integration_noise,
                                                             default_params);
    // graph_values.first.emplace_shared<mushr_u_xddot01_t>(k_xdot1, k_xdot0, k_t01, k_u01, default_params, nullptr);
    // graph_values.first.emplace_shared<mushr_ub_u_xdot_t>(k_ubar1, k_u01, k_ubar0, k_t01, default_params, nullptr);
    // graph_values.first.emplace_shared<mushr_xdot_ub_t>(k_xdot1, k_ubar1, nullptr);

    graph_values.first.addPrior(k_t01, dt, dt_noise);
    graph_values.first.addPrior(k_x1, x1, x_leaf_noise);
    graph_values.first.addPrior(k_xdot1, xdot1);
    // graph_values.first.addPrior(k_ubar1, ubar1);

    // graph_values.second.insert(k_xdotdot0, xdot0);
    graph_values.second.insert(k_x1, x1);
    graph_values.second.insert(k_t01, dt);
    graph_values.second.insert(k_xdot1, xdot1);
    // graph_values.second.insert(k_ubar1, ubar1);
    graph_values.second.insert(k_u01, u01);

    return graph_values;
  }

  static GraphValues root_to_fg(const std::size_t root, const ml4kp_bridge::SpacePoint& node_state,
                                const bool estimation = false)
  {
    GraphValues graph_values;
    State x{ State::Zero() };
    StateDot xdot{ StateDot::Zero() };
    // Ubar ubar{ Ubar::Zero() };

    mushr_utils_t::state(x, node_state);
    mushr_utils_t::stateDot(xdot, node_state);
    // mushr_utils_t::StateDotDot(xdotdot, node_state);

    const gtsam::Key k_x{ keyX(1, root) };
    const gtsam::Key k_xdot{ keyXdot(1, root) };
    const gtsam::Key k_ubar{ keyUbar(1, root) };

    NoiseModel x_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    // NoiseModel ubar_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(prx_models::mushr_types::Ubar::Dim, 1e0) };

    graph_values.first.addPrior(k_x, x, x_prior_noise);
    // graph_values.first.addPrior(k_xdot, xdot, x_prior_noise);
    // graph_values.first.addPrior(k_xdot, xdot, x_prior_noise);
    // graph_values.first.addPrior(k_ubar, ubar, ubar_prior_noise);

    graph_values.second.insert(k_x, x);
    graph_values.second.insert(k_xdot, xdot);
    // graph_values.second.insert(k_xdotdot, xdotdot);
    // graph_values.second.insert(k_ubar, ubar);

    return graph_values;
  }

  template <typename Params>
  static void set_params(const Params& params)
  {
    prx_assert(params.size() == default_params.size(), "Wrong number of parameters!");
    for (int i = 0; i < params.size(); ++i)
    {
      default_params[i] = params[i];
    }
  }

  static void print_params()
  {
    const Eigen::RowVectorXd mushr_parameters{ default_params.transpose() };
    DEBUG_VARS(mushr_parameters);
  }

  // static inline mushr_types::Ubar::params default_params{ 0.929102, 0.752216, 0.398495 };
  static inline Parameters default_params{ 0.010, 0.52 };

private:
  static inline GraphValues aux_graph;
  static inline std::size_t first{ std::numeric_limits<std::size_t>::max() };
};
// mushr_types::Ubar::params mushr_types::default_params = mushr_types::Ubar::params(0.9898, 0.4203, 0.6228);

class mushrFG_t : public prx::plant_t
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;

public:
  mushrFG_t(const std::string& path)
    : plant_t(path)
    , _params_u(mushr_utils_t::default_params)
    , _ubar(mushr_types::Ubar::type::Zero())
    , _state_dot_noise(mushr_types::StateDot::type::Zero())
  {
    // state_memory = { &_state[0], &_state[1], &_state[2], &_ubar[0], &_ubar[1] };
    state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
                     &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    state_space = new prx::space_t("EEREEE", state_memory, "mushr_state");
    state_space->set_bounds({ -100, -100, -prx::constants::pi, -10, -10, -10 },
                            { 100, 100, prx::constants::pi, 10, 10, 10 });

    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -prx::constants::pi / 2.0, -10 }, { prx::constants::pi / 2.0, 10 });

    derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");

    parameter_memory = { &_params_u[0], &_params_u[1],  // no-lint
                         &_state_dot_noise[0], &_state_dot_noise[1], &_state_dot_noise[2] };
    parameter_space = new prx::space_t("EEEEE", parameter_memory, "mushr_params");

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    // geometries["body"]->initialize_geometry({ 0.01, 0.01, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }
  ~mushrFG_t() {};

  virtual void propagate(const double simulation_step) override final
  {
    // DEBUG_VARS("--------------")
    // DEBUG_VARS(_state, _state_dot.transpose(), _ubar.transpose(), _ctrl.transpose(), simulation_step);
    // _ubar = mushr_ub_u_xdot_param_t::dynamics(_ctrl, _ubar, _params_ubar_u, simulation_step);
    // _state_dot = mushr_xdot_ub_t::dynamics(_ubar);
    // _state_dot_dot[0] = _ctrl[mushr_types::Control::vel_desired];
    // _state_dot_dot[1] = 0;
    // _state_dot_dot[2] = _ctrl[mushr_types::Control::steering];
    // _state_dot = EulerFactor::integrate(_state_dot, _state_dot_dot, prx::simulation_step);
    // _state_dot = mushr_u_xddot01_t::predict(_state_dot, simulation_step, _ctrl, _params_ubar_u);
    _state_dot = mushr_CtrlAccel_t::predict(_state_dot, _ctrl, prx::simulation_step, _params_u);
    const Eigen::Vector3d w{ prx::gaussian_random(0.0, _state_dot_noise[0]),
                             prx::gaussian_random(0.0, _state_dot_noise[1]),
                             prx::gaussian_random(0.0, _state_dot_noise[2]) };
    _state_dot += w;

    _state = mushr_x_xdot_t::predict(_state, _state_dot, simulation_step);
    // DEBUG_VARS(_state, _state_dot.transpose(), _ubar.transpose(), _ctrl.transpose(), simulation_step);
    // state_space->enforce_bounds();
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
    body->translation()[0] = _state[0];
    body->translation()[1] = _state[1];
    body->translation()[2] = 0.0;
  }
  virtual void compute_derivative() override final
  {
  }

protected:
  mushr_types::State::type _state;
  mushr_types::StateDot::type _state_dot, _state_dot_dot;
  mushr_types::Control::type _ctrl;
  mushr_types::Ubar::type _ubar;
  mushr_types::Control::params _params_u;
  mushr_types::StateDot::type _state_dot_noise;

  double _idle;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushrFG_t, mushrFG)
