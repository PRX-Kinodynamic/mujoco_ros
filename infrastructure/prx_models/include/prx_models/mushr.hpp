#pragma once

// Ros
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

namespace prx_models
{
class mushr_utils_t
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

  using XVelFactor = prx_models::mushr_x_xdot_t;
  using VelUbarFactor = prx_models::mushr_xdot_ub_t;
  using CtrlUbarFactor = prx_models::mushr_ub_u_xdot_t;
  using ObservationFactor = gtsam::PriorFactor<prx_models::mushr_types::State::type>;

public:
  static constexpr std::string_view plant_name = "mushrFG";
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = prx_models::mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using Ubar = mushr_types::Ubar::type;
  using Control = mushr_types::Control::type;
  using Noise = Eigen::Vector<double, 2>;
  using Observation = State;

  using StateKeys = std::array<gtsam::Key, 3>;
  using ControlKeys = std::array<gtsam::Key, 1>;

  using StateEstimates = std::tuple<State, StateDot, Ubar>;

  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };

  static gtsam::Key keyU(const int& level, const int& step)
  {
    return SF::create_hashed_symbol("U^{", level, "}_{", step, "}");
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
    z[3] = prx::quaternion_to_euler(q)[2];
  }

  static void copy(ml4kp_bridge::SpacePoint& pt, const StateEstimates& estimates)
  {
    const State& x{ std::get<0>(estimates) };
    const StateDot& xdot{ std::get<1>(estimates) };
    const Ubar& ubar{ std::get<2>(estimates) };

    pt.point.resize(4);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = x[2];
    pt.point[3] = ubar[mushr_types::Ubar::velocity];
  }

  struct ConfigFromState
  {
    void operator()(Eigen::Matrix3d& rotation, Eigen::Vector3d& translation, const State& state)
    {
      translation[0] = state[0];
      translation[1] = state[1];
      translation[2] = 0;
      rotation = Eigen::AngleAxisd(state[2], Eigen::Vector3d::UnitZ());
    }

    void operator()(Eigen::MatrixXd& H, const Eigen::Vector3d& translation)
    {
      H = Eigen::Matrix2d::Identity();
      H.diagonal() = translation.head(2);
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
                                            const StateEstimates& estimates, const Control u_prev, const Observation& z,
                                            const double dt, const double z_noise)
  {
    //     XVelFactor
    // VelUbarFactor
    // CtrlUbarFactor
    // ObservationFactor
    const State& x0_value{ std::get<0>(estimates) };
    const StateDot& xdot0_value{ std::get<1>(estimates) };
    const Ubar& ubar0_value{ std::get<2>(estimates) };
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(-1, prev_id) };
    const gtsam::Key x1{ keyX(-1, curr_id) };
    const gtsam::Key xdot0{ keyXdot(-1, prev_id) };
    const gtsam::Key xdot1{ keyXdot(-1, curr_id) };
    const gtsam::Key ubar0{ keyUbar(-1, prev_id) };
    const gtsam::Key ubar1{ keyUbar(-1, curr_id) };
    const gtsam::Key u01{ keyU(-1 * prev_id, -1 * curr_id) };

    NoiseModel ubar_noise{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };
    NoiseModel qdot_noise{ gtsam::noiseModel::Isotropic::Sigma(3, dt) };
    NoiseModel q_noise{ gtsam::noiseModel::Isotropic::Sigma(3, z_noise) };
    NoiseModel observation_noise{ gtsam::noiseModel::Isotropic::Sigma(3, z_noise) };

    graph_values.first.emplace_shared<CtrlUbarFactor>(ubar1, u01, ubar0, mushr_utils_t::default_params, ubar_noise);
    graph_values.first.emplace_shared<VelUbarFactor>(xdot1, ubar1, qdot_noise);
    graph_values.first.emplace_shared<XVelFactor>(x1, x0, xdot0, q_noise, dt);
    graph_values.first.emplace_shared<ObservationFactor>(x0, z, observation_noise);
    graph_values.first.addPrior(u01, u_prev);

    const Ubar val_ubar1{ CtrlUbarFactor::dynamics(u_prev, ubar0_value, mushr_utils_t::default_params) };
    const StateDot val_xdot1{ VelUbarFactor::dynamics(val_ubar1) };
    const State val_x1{ XVelFactor::predict(x0_value, val_xdot1, dt) };

    graph_values.second.insert(u01, u_prev);
    graph_values.second.insert(xdot1, val_xdot1);
    graph_values.second.insert(x1, val_x1);

    graph_values.second.insert(ubar1, val_ubar1);
    return graph_values;
  }

private:
  static inline mushr_types::Ubar::params default_params{ 0.9898, 0.4203, 0.6228 };
};
// mushr_types::Ubar::params mushr_types::default_params = mushr_types::Ubar::params(0.9898, 0.4203, 0.6228);

class mushrFG_t : public prx::plant_t
{
public:
  mushrFG_t(const std::string& path)
    : plant_t(path), _params_ubar_u(0.9898, 0.4203, 0.6228), _ubar(mushr_types::Ubar::type::Zero())
  {
    // state_memory = { &_state[0], &_state[1], &_state[2], &_ubar[0], &_ubar[1] };
    state_memory = { &_state[0], &_state[1], &_state[2], &_ubar[mushr_types::Ubar::velocity] };
    state_space = new prx::space_t("EERE", state_memory, "mushr_state");
    state_space->set_bounds({ -100, -100, -prx::constants::pi, -100 }, { 100, 100, prx::constants::pi, 100 });

    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -prx::constants::pi / 2.0, -10 }, { prx::constants::pi / 2.0, 10 });

    derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2], &_idle };
    derivative_space = new prx::space_t("EEEI", derivative_memory, "mushr_deriv");

    parameter_memory = { &_params_ubar_u[0], &_params_ubar_u[1], &_params_ubar_u[2] };
    parameter_space = new prx::space_t("EEE", parameter_memory, "mushr_params");

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.4, 0.28, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }
  ~mushrFG_t(){};

  virtual void propagate(const double simulation_step) override final
  {
    // DEBUG_VARS("--------------")
    // DEBUG_VARS(_state, _state_dot.transpose(), _ubar.transpose(), _ctrl.transpose(), simulation_step);
    _ubar = mushr_ub_u_xdot_param_t::dynamics(_ctrl, _ubar, _params_ubar_u);
    _state_dot = mushr_xdot_ub_t::dynamics(_ubar);
    // _state = mushr_x_xdot_t::dynamics(_state, _state_dot, simulation_step);
    // mushr_x_xdot_ub_t();
    _state = mushr_x_xdot_t::predict(_state, _state_dot, simulation_step);
    // DEBUG_VARS(_state, _state_dot.transpose(), _ubar.transpose(), _ctrl.transpose(), simulation_step);
    state_space->enforce_bounds();
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
  mushr_types::StateDot::type _state_dot;
  mushr_types::Control::type _ctrl;
  mushr_types::Ubar::type _ubar;
  mushr_types::Ubar::params _params_ubar_u;

  double _idle;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushrFG_t, mushrFG)
