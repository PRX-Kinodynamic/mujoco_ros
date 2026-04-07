#pragma once

#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <string>

// Ros
#include <Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/tree_msg_wrapper.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/stela_robot_interface.hpp>
#include <prx_models/Edge.h>
#include <prx_models/tree_utils.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <interface/SensorDataStamped.h>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>

namespace prx_models
{

class mushr_first_order_t : public prx::plant_t
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;
  using MushrMjFactor = mushr_mj_factor_t<>;

public:
  mushr_first_order_t(const std::string& path)
    : plant_t(path), _state(0., 0., 0.), _state_dot(StateDot::Zero()), _ctrl(mushr_types::Control::type::Zero())
  {
    state_memory = { &_state[0], &_state[1], &_state[2] };
    state_space = new prx::space_t("EER", state_memory, "mushr_state");
    state_space->set_bounds({ -100, -100, -prx::constants::pi }, { 100, 100, prx::constants::pi });

    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -0.5, -1. }, { 0.5, 1. });

    parameter_memory = { &_delta_poly[0], &_delta_poly[1], &_delta_poly[2], &_delta_poly[3] };
    const std::string param_topology{ std::string(parameter_memory.size(), 'E') };
    parameter_space = new prx::space_t(param_topology, parameter_memory, "mushr_params");

    _sensor_memory = { &_sensor_position[0],    &_sensor_position[1],    &_sensor_position[2],  // no-lint
                       &_sensor_quaternion.w(), &_sensor_quaternion.x(), &_sensor_quaternion.y(),
                       &_sensor_quaternion.z() };
    _sensor_space = new prx::space_t("EEEEEEE", _sensor_memory, "mushr_sensors");
    // _sensor_space->set_bounds({ -100, -100, -100, -100, -100, -100, -100 },
    // { +100, +100, +100, +100, +100, +100, +100 });

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }
  ~mushr_first_order_t() {};

  virtual void environment_bounds(const std::pair<Eigen::Vector3d, Eigen::Vector3d> bounds) override
  {
    state_space->set_bounds({ bounds.first[0], bounds.first[1], -prx::constants::pi },
                            { bounds.second[0], bounds.second[1], prx::constants::pi });
  }
  // virtual prx::param_loader initialization_parameters() override
  // {
  //   prx::param_loader params{ prx::plant_t::initialization_parameters() };

  //   return params;
  // }

  // static prx::param_loader init()
  // {
  //   prx::param_loader params{ prx::plant_t::init() };
  //   params["state_space"] = space_t::init();
  //   params["control_space"] = space_t::init();
  //   params["parameter_space"] = space_t::init();
  //   params["sensor_space"] = space_t::init();
  // }

  virtual void sense() override
  {
    _sensor_position << _state[0], _state[1], 0.125;
    _sensor_quaternion = Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ());
  }

  virtual void propagate(const double simulation_step) override final
  {
    const double& V{ _ctrl[mushr_types::Control::vel_desired] };
    const double& steering{ _ctrl[mushr_types::Control::steering] };
    const double& L{ prx_models::mushr_types::Parameters::L };

    const double delta{ mushr_types::Control::evaluate_polynomial(_delta_poly, steering) };
    const double beta{ mushr_types::Control::beta(delta) };
    const double omega{ 2.0 * std::sin(beta) / L };

    _state_dot[0] = V * std::cos(delta);
    _state_dot[1] = V * std::sin(delta);
    _state_dot[2] = V * omega;  //(V / L) * std::tan(steering);

    _state = mushr_x_xdot_t::predict(_state, _state_dot, prx::simulation_step);
    // _state_dot = mushr_CtrlAccel_t<>::predict(_state_dot, _ctrl, prx::simulation_step, _params_u, _delta_poly);
    // DEBUG_VARS(_state, _state_dot.transpose());
    // DEBUG_VARS(_state.matrix())
    // DEBUG_VARS(_state, _state_dot.transpose(), _ubar.transpose(), _ctrl.transpose(), simulation_step);
    // state_space->enforce_bounds();
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
    body->translation()[0] = _state[0];
    body->translation()[1] = _state[1];
    body->translation()[2] = 0.125;
  }

  virtual void compute_derivative() override final
  {
  }

protected:
  // State space
  mushr_types::State::type _state;
  mushr_types::StateDot::type _state_dot;

  // Control space
  mushr_types::Control::type _ctrl;

  // Sensor space
  Eigen::Vector3d _sensor_position;
  Eigen::Quaterniond _sensor_quaternion;

  mushr_types::Control::Poly _delta_poly;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushr_first_order_t, mushr_first_order)