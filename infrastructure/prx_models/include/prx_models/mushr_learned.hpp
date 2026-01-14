#pragma once

#include <string>

// Ros
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/mushr_factors.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <interface/SensorDataStamped.h>
#include <torch_bridge/query_utils.hpp>
// torch_bridge/query_utils.hpp
// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/quadratic_cost_factor.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include "torch_bridge/TorchQuery.h"

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <ros/node_handle.h>

namespace prx_models
{
class mushr_learned_msg_passing_t : public prx::plant_t
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;
  using MushrMjFactor = mushr_mj_factor_t<>;

public:
  mushr_learned_msg_passing_t(const std::string& path) : plant_t(path), _nh("/mushr_learned/")
  {
    // state_memory = { &_state[0], &_state[1], &_state[2], &_ubar[0], &_ubar[1] };
    state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
                     &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    state_space = new prx::space_t("EEREEE", state_memory, "mushr_state");
    state_space->set_bounds({ -100, -100, -prx::constants::pi, -10, -10, -10 },
                            { 100, 100, prx::constants::pi, 10, 10, 10 });

    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -100, -100 }, { 100, 100 });

    derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");

    // parameter_memory = { };
    // const std::string param_topology{ std::string(parameter_memory.size(), 'E') };
    // parameter_space = new prx::space_t(param_topology, parameter_memory, "mushr_params");

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();

    _service_client = _nh.serviceClient<torch_bridge::TorchQuery>(_nh.getNamespace() + "/service", true);
    _torch_service_call.request.inputs = 2;
    _torch_service_call.request.input_dimensions = { 3, 2 };
    _torch_service_call.request.compute_jacobians = false;
    // DEBUG_PRINT
    // DEBUG_VARS(_propagation_factor)
    // prx_assert(std::abs(prx::simulation_step - 0.1) < 0.0001, "Simulation step is not 0.1 (model was trained for
    // 0.1)")
  }

  ~mushr_learned_msg_passing_t() {};

  virtual void propagate(const double simulation_step) override final
  {
    _torch_service_call.request.data.clear();

    torch_bridge::update_request(_torch_service_call, _state_dot, _ctrl);

    const bool ack{ _service_client.call(_torch_service_call) };
    if (not ack)
    {
      prx_throw("[mushr_learned_msg_passing] Call to service failed!")
    }
    torch_bridge::get_result(_torch_service_call, _state_dot);

    // _state_dot = EulerFactor::predict(_state_dot, _state_dot_dot, prx::simulation_step);
    // _state_dot = mushr_CtrlAccel_t<>::predict(_state_dot, _ctrl, prx::simulation_step, _params_u, _delta_poly);

    _state = mushr_x_xdot_t::predict(_state, _state_dot, prx::simulation_step);
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

  ros::NodeHandle _nh;
  ros::ServiceClient _service_client;

  torch_bridge::TorchQuery _torch_service_call;

  // std::shared_ptr<MushrMjFactor> _mj_factor;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushr_learned_msg_passing_t, mushrLearnedMsgPassing)
