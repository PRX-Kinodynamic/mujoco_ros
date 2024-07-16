#pragma once

// Ros
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/mj_mushr.hpp>

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

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = Eigen::Vector<double, 3>;
  using StateDot = Eigen::Vector<double, 3>;
  using Ubar = Eigen::Vector<double, 2>;
  using Control = Eigen::Vector<double, 2>;
  using Noise = Eigen::Vector<double, 2>;
  using Observation = Eigen::Vector<double, 3>;

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

  static void copy(ml4kp_bridge::SpacePoint& pt, const State& x, const StateDot& xdot)
  {
    pt.point.resize(4);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = x[0];
    // _ubar = mushr_ub_u_xdot_param_t::predict(_ctrl, _ubar, _params_ubar_u);
    pt.point[3] = xdot[1];
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
};

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
    _ubar = mushr_ub_u_xdot_param_t::dynamics(_ctrl, _ubar, _params_ubar_u);
    _state_dot = mushr_x_xdot_ub_t::dynamics(_state, _ubar);
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
