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

#include <memory>
#include <gtsam/geometry/Pose2.h>
#include <prx/utilities/spaces/space_v2.hpp>
#include <prx_models/mushr_stela_interface.hpp>
#include <prx/simulation/dynamical_system.hpp>

class mushrPolynomial_t;
template <>
struct prx::dynamical_system_traits<mushrPolynomial_t>
{
  // clang-format off
  enum {StateDimension = 6}; 
  enum {ControlDimension = 2}; 
  enum {ParametersDimension = prx_models::mushr_types::Control::ParamsDim + prx_models::mushr_types::Control::PolyDeg+1}; 
  enum {ObservationDimension = 3};
  // clang-format on

  using State = gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d>;
  using Control = Eigen::Vector<double, ControlDimension>;
  using Parameters = Eigen::Vector<double, ParametersDimension>;
  using Observation = Eigen::Vector<double, ObservationDimension>;
};

class mushrPolynomial_t : public prx::dynamical_system_t<mushrPolynomial_t>
{
public:
  using Base = prx::dynamical_system_t<mushrPolynomial_t>;
  using Derived = mushrPolynomial_t;

  // using State = typename Base::State;
  // using Control = typename Base::Control;
  // using Parameters = typename Base::Parameters;
  // using Observation = typename Base::Observation;

  // using StateSpace = typename Base::StateSpace;
  // using ControlSpace = typename Base::ControlSpace;
  // using ParametersSpace = typename Base::ParametersSpace;
  // using ObservationSpace = typename Base::ObservationSpace;

  // using StateSpacePtr = std::shared_ptr<StateSpace>;
  // using ControlSpacePtr = std::shared_ptr<ControlSpace>;
  // using ParametersSpacePtr = std::shared_ptr<ParametersSpace>;
  // using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;

  // mushrPolynomial_t() : Base("mushrPolynomial_t") {};
  // mushrPolynomial_t(const std::string name) : Base(name) {};
  // mushrPolynomial_t(prx::param_loader params) : Base(params)
  // {
  //   Parameters mushr_params{ params["parameter_space/value"].as<Parameters>() };

  //   _params_u[mushr_types::Control::vel_desired] = mushr_params[mushr_types::Control::vel_desired];
  //   _params_u[mushr_types::Control::steering] = mushr_params[mushr_types::Control::steering];
  //   _params_u[mushr_types::Control::friction] = mushr_params[mushr_types::Control::friction];
  //   _params_u[mushr_types::Control::delta_offset] = mushr_params[mushr_types::Control::delta_offset];
  //   _params_u[mushr_types::Control::delta_gain] = mushr_params[mushr_types::Control::delta_gain];
  //   _delta_poly = mushr_params.tail(mushr_types::Control::PolyDeg + 1);
  // }

  // static prx::param_loader default_params()
  // {
  //   prx::param_loader params;
  //   const std::string state_space_bounds_yaml =
  //       "bounds:\n"
  //       "  -\n"
  //       "    min: [-10, -10, -3.14159]\n"
  //       "    max: [+10, +10, +3.14159]\n"
  //       "  -\n"
  //       "    min: [-0.5, -0.5, -0.1]\n"
  //       "    max: [+0.5, +0.5, +0.1]\n";
  //   const std::string control_space_bounds_yaml =
  //       "bounds:\n"
  //       "    min: [-1, -1]\n"
  //       "    max: [+1, +1]\n";
  //   const std::string observation_space_bounds_yaml =
  //       "bounds:\n"
  //       "    min: [-10, -10, -3.14159]\n"
  //       "    max: [+10, +10, +3.14159]\n";
  //   // const std::string parameter_space_bounds_yaml =
  //   //     "bounds:\n"
  //   //     "    min: [-1, -1, -1]\n"
  //   //     "    max: [+1, +1, +1]\n";

  //   params["state_space"].from_string(state_space_bounds_yaml);
  //   params["control_space"].from_string(control_space_bounds_yaml);
  //   params["observation_space"].from_string(observation_space_bounds_yaml);

  //   return params;
  // }

  // void initialize_geometries()
  // {
  //   _geometries.push_back(std::make_shared<prx::geometry_t>(geometry_type_t::SPHERE));
  //   _geometries.back()->initialize_geometry({ 0.5 });
  //   _geometries.back()->generate_collision_geometry();
  //   _geometries.back()->set_visualization_color("0x00ff00");
  // }

  // void initialize()
  // {
  //   prx::param_loader params{ default_params() };

  //   _state_space = StateSpace::create(params["state_space"]);
  //   _control_space = ControlSpace::create(params["control_space"]);
  //   _sensor_space = ObservationSpace::create(params["observation_space"]);
  //   _parameter_space = ParametersSpace::create(params["parameter_space"]);
  // }

  // virtual ~mushrPolynomial_t() {};

  // static double distance(const State& a, const State& b)
  // {
  //   using ErrorVector = Eigen::Vector<double, prx::dynamical_system_traits<mushrPolynomial_t>::StateDimension>;
  //   const State between{ a.between(b) };
  //   const ErrorVector error{ State::Logmap(between) };
  //   return error.norm();
  // }

  // State propagate(const State& x0, const Control& u0, const double& dt)
  // {
  //   _state = mushr_x_xdot_t::predict(_state, _state_dot, prx::simulation_step);
  //   _state_dot = mushr_CtrlAccel_t<>::predict(_state_dot, _ctrl, prx::simulation_step, _params_u, _delta_poly);
  //   // Eigen::Vector<double, 6> xdot;
  //   // xdot << x0.second, u0 * dt;
  //   // const State x01{ State::Expmap(xdot) };
  //   // const State x1{ gtsam::traits<State>::Compose(x0, x01) };
  //   // return x1;
  // }

  // std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  // {
  //   const Eigen::Matrix3d R{ prx::axis_to_rotation_matrix({ state.first.theta() }, 'Z') };
  //   const Eigen::Vector3d t(state.first.x(), state.first.y(), 0.0);
  //   return { { R, t } };
  // }

  // void environment(const prx::obstacle_loader_t& loader)
  // {
  //   auto bounds = _state_space->sampler.bounds();
  //   auto env_min_bounds = loader.min_bounds();
  //   auto env_max_bounds = loader.max_bounds();
  //   bounds.first.first.head(2) = env_min_bounds.head(2);
  //   bounds.first.second.head(2) = env_max_bounds.head(2);
  //   _state_space->sampler.bounds(bounds.first.first, bounds.first.second, bounds.second.first, bounds.second.second);
  // }
  // virtual void sense(const State& x0, const Control& u0, const double& dt, const Parameters& params) = 0;

protected:
  prx_models::mushr_types::Control::params _params_u;
  prx_models::mushr_types::Control::Poly _delta_poly;

  StateSpacePtr _state_space;
  ControlSpacePtr _control_space;
  ParametersSpacePtr _parameter_space;
  ObservationSpacePtr _sensor_space;
};

namespace prx_models
{

class mushrFG_t : public prx::plant_t
{
  using State = mushr_types::State::type;
  using StateDot = mushr_types::StateDot::type;
  using EulerFactor = prx::fg::euler_integration_factor_t<StateDot, StateDot>;
  using MushrMjFactor = mushr_mj_factor_t<>;

public:
  mushrFG_t(const std::string& path)
    : plant_t(path)
    , _params_u(mushr_stela_t::default_params)
    , _delta_poly(mushr_stela_t::default_poly)
    , _state(0., 0., 0.)
    , _state_dot(StateDot::Zero())
    , _ctrl(mushr_types::Control::type::Zero())
    , _sensor_position(Eigen::Vector3d::Zero())
    , _sensor_quaternion(1., 0., 0., 0.)
  {
    state_memory = { &_state[0],     &_state[1],     &_state[2],  // no-lint
                     &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    state_space = new prx::space_t("EEREEE", state_memory, "mushr_state");
    environment_bounds({ { -100, -100, -prx::constants::pi }, { 100, 100, prx::constants::pi } });
    // state_space->set_bounds({ -100, -100, -prx::constants::pi, -10, -10, -10 },
    //                         { 100, 100, prx::constants::pi, 10, 10, 10 });

    control_memory = { &_ctrl[mushr_types::Control::vel_desired], &_ctrl[mushr_types::Control::steering] };
    input_control_space = new prx::space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -1., -1. }, { 1., 1. });

    // derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2] };
    // derivative_space = new prx::space_t("EEE", derivative_memory, "mushr_deriv");

    parameter_memory = { &_params_u[mushr_types::Control::vel_desired],
                         &_params_u[mushr_types::Control::steering],      // no-lint
                         &_params_u[mushr_types::Control::friction],      // no-lint
                         &_params_u[mushr_types::Control::delta_offset],  // no-lint
                         &_params_u[mushr_types::Control::delta_gain],    // no-lint
                         &_delta_poly[0],
                         &_delta_poly[1],
                         &_delta_poly[2],
                         &_delta_poly[3] };
    const std::string param_topology{ std::string(parameter_memory.size(), 'E') };
    parameter_space = new prx::space_t(param_topology, parameter_memory, "mushr_params");

    _sensor_memory = { &_sensor_position[0],    &_sensor_position[1],    &_sensor_position[2],  // no-lint
                       &_sensor_quaternion.w(), &_sensor_quaternion.x(), &_sensor_quaternion.y(),
                       &_sensor_quaternion.z() };
    _sensor_space = new prx::space_t("EEEEEEE", _sensor_memory, "mushr_sensors");
    _sensor_space->set_bounds({ -100, -100, -100, -100, -100, -100, -100 },
                              { +100, +100, +100, +100, +100, +100, +100 });

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();

    // DEBUG_PRINT
    // DEBUG_VARS(_propagation_factor)
  }
  ~mushrFG_t() {};

  virtual prx::param_loader initialization_parameters() override
  {
    prx::param_loader params{ prx::plant_t::initialization_parameters() };

    return params;
  }

  virtual void environment_bounds(const std::pair<Eigen::Vector3d, Eigen::Vector3d> bounds) override
  {
    state_space->set_bounds({ bounds.first[0], bounds.first[1], -prx::constants::pi, -5, -5, -5 },
                            { bounds.second[0], bounds.second[1], prx::constants::pi, 5, 5, 5 });
  }
  virtual void propagate(const double simulation_step) override final
  {
    _state = mushr_x_xdot_t::predict(_state, _state_dot, prx::simulation_step);
    // _state_dot = mushr_accel_t<>::predict(_state_dot, _ctrl, prx::simulation_step, _params_u, _delta_poly);

    _state_dot = mushr_CtrlAccel_t<>::predict(_state_dot, _ctrl, prx::simulation_step, _params_u, _delta_poly);
  }

  virtual void sense() override
  {
    _sensor_position << _state[0], _state[1], 0.125;
    _sensor_quaternion = Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ());
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
  // mushr_types::Ubar::type _ubar;

  // Parameter space
  mushr_types::Control::params _params_u;
  mushr_types::Control::Poly _delta_poly;

  // Sensor space
  Eigen::Vector3d _sensor_position;
  Eigen::Quaterniond _sensor_quaternion;

  // double _propagate_id;       // If mj prop using, it needs to reset if curr_propid != _propagate_id
  // double _curr_propagate_id;  // If mj prop using, it needs to reset if curr_propid != _propagate_id

  // std::shared_ptr<MushrMjFactor> _mj_factor;
};
}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushrFG_t, mushrFG)