#pragma once

#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <string>

// Ros
#include <Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/sampler_bridge.hpp>

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
// #include <prx_models/mushr_stela_interface.hpp>
#include <prx/simulation/dynamical_system.hpp>

namespace prx
{

class SO2_system_t;
template <>
struct dynamical_system_traits<SO2_system_t>
{
  // clang-format off
  enum {StateDimension = 2}; 
  enum {ControlDimension = 1}; 
  enum {ParametersDimension = 1}; 
  enum {ObservationDimension = 2};
  // clang-format on

  using State = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Control = double;
  using Parameters = Eigen::Vector<double, ParametersDimension>;
  using Observation = State;

  using StateDot = Eigen::Vector<double, StateDimension>;
};

class SO2_system_t : public prx::dynamical_system_t<SO2_system_t>
{
public:
  using Base = prx::dynamical_system_t<SO2_system_t>;
  using Derived = SO2_system_t;

  const std::string Name = "S02DynamicalSystem";
  using State = typename Base::State;
  using Control = typename Base::Control;
  using Parameters = typename Base::Parameters;
  using Observation = typename Base::Observation;

  using StateSpace = typename Base::StateSpace;
  using ControlSpace = typename Base::ControlSpace;
  using ParametersSpace = typename Base::ParametersSpace;
  using ObservationSpace = typename Base::ObservationSpace;

  using StateSpacePtr = std::shared_ptr<StateSpace>;
  using ControlSpacePtr = std::shared_ptr<ControlSpace>;
  using ParametersSpacePtr = std::shared_ptr<ParametersSpace>;
  using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;

  SO2_system_t(prx::param_loader params)
    : Base(params)
    , _gravity(9.8)
    , _friction(0.1)
    , _length(0.5)
    , _mass(0.15)
    , _inertia(_mass * _length * _length)
    , _u_max(0.6371781908344007)
  {
    DEBUG_VARS(_friction, _length, _mass, _inertia);
  }
  SO2_system_t() : SO2_system_t(default_params()) {};

  SO2_system_t(const std::string params) : SO2_system_t(prx::param_loader::create(params))
  {
  }

  static prx::param_loader default_params()
  {
    prx::param_loader params;
    const std::string state_space_bounds_yaml =
        "bounds:\n"
        "  -\n"
        "    min: [-3.14159]\n"
        "    max: [+3.14159]\n"
        "  -\n"
        "    min: [-6.28318]\n"
        "    max: [+6.28318]\n";
    const std::string control_space_bounds_yaml =
        "bounds:\n"
        "    min: [-1, -1]\n"
        "    max: [+1, +1]\n";
    const std::string observation_space_bounds_yaml =
        "bounds:\n"
        "    min: [-3.14159,-6.28318]\n"
        "    max: [+3.14159,+6.28318]\n";
    // const std::string parameter_space_bounds_yaml =
    //     "bounds:\n"
    //     "    min: [-1, -1, -1]\n"
    //     "    max: [+1, +1, +1]\n";

    params["state_space"].from_string(state_space_bounds_yaml);
    params["control_space"].from_string(control_space_bounds_yaml);
    params["observation_space"].from_string(observation_space_bounds_yaml);

    return params;
  }

  void initialize_geometries()
  {
    _geometries.push_back(std::make_shared<prx::geometry_t>(prx::geometry_type_t::SPHERE));
    _geometries.back()->initialize_geometry({ 0.1 });
    _geometries.back()->generate_collision_geometry();
    _geometries.back()->set_visualization_color("0x00ff00");
  }

  void initialize()
  {
    prx::param_loader params{ default_params() };

    _state_space = StateSpace::create(params["state_space"]);
    _control_space = ControlSpace::create(params["control_space"]);
    _sensor_space = ObservationSpace::create(params["observation_space"]);
    _parameter_space = ParametersSpace::create(params["parameter_space"]);
  }

  // virtual ~SO2_system_t() {};

  // static double distance(const State& a, const State& b)
  // {
  //   using ErrorVector = Eigen::Vector<double, prx::dynamical_system_traits<SO2_system_t>::StateDimension>;
  //   const State between{ a.between(b) };
  //   const ErrorVector error{ State::Logmap(between) };
  //   return error.norm();
  // }

  Observation sense(const State& x)
  {
    return x;
  }

  StateDot ode(const State& x0, const Control& u0, OptJacX Hx = nullptr, OptJacU Hu = nullptr)
  {
    const Control u_eff{ std::min(std::max(-_u_max, u0), _u_max) };
    const double& sth{ x0.first.s() };  // sin(theta)
    const double& thdot{ x0.second };
    const double thddot{ _gravity / _length * sth + u_eff / _inertia - (_friction / _inertia) * thdot };
    const StateDot dot(thdot, thddot);

    return dot;
  }

  State integrate(const State& x0, const StateDot& xd0, const double& dt, OptJacX Hx = nullptr, OptJacX Hxd = nullptr,
                  OptJacDT Hdt = nullptr)
  {
    using LieIntegrator = prx::fg::lie_integrator_t<State, Eigen::Vector2d>;
    const State x1{ LieIntegrator::integrate(x0, xd0, dt, Hx, Hxd, Hdt) };
    return x1;
  }

  State propagate(const State& x0, const Control& u0, const double& dt,  // no-lint
                  OptJacX Hx = nullptr, OptJacU Hu = nullptr, OptJacDT Hdt = nullptr)
  {
    const bool jacs{ Hx or Hu or Hdt };
    JacX x1_H_x0, x1_H_xd, xd_H_x0;
    JacU x1_H_dt, xd_Hu_u0;

    const StateDot xdot{ ode(x0, u0, jacs ? &xd_H_x0 : nullptr, jacs ? &xd_Hu_u0 : nullptr) };
    const State x1{ integrate(x0, xdot, dt, jacs ? &x1_H_x0 : nullptr, jacs ? &x1_H_xd : nullptr,
                              jacs ? &x1_H_dt : nullptr) };
    if (Hx)
    {
      *Hx = x1_H_x0 + x1_H_xd * xd_H_x0;
    }
    if (Hu)
    {
      *Hu = x1_H_xd * xd_Hu_u0;
    }
    if (Hdt)
    {
      *Hdt = x1_H_dt;
    }
    return x1;
    // using LieIntegrator = prx::fg::lie_integrator_t<State, Eigen::Vector2d>;

    // const Control u_eff{ std::min(std::max(-_u_max, u0), _u_max) };
    // const double& sth{ x0.first.s() };  // sin(theta)
    // const double& thdot{ x0.second };
    // const double thddot{ _gravity / _length * sth + u_eff / _inertia - (_friction / _inertia) * thdot };
    // const Eigen::Vector2d dot(thdot, thddot);

    // Eigen::Matrix<double, 2, 2> x1_H_x0{ Eigen::Matrix<double, 2, 2>::Identity() };
    // Eigen::Matrix<double, 2, 2> x1_H_dot{ Eigen::Matrix<double, 2, 2>::Identity() };
    // const State x1{ LieIntegrator::integrate(x0, dot, dt, x1_H_x0, x1_H_dot) };

    // if (Hx)
    // {
    //   *Hx = x1_H_x0;
    // }
    // if (Hu)
    // {
    //   PRX_WARNING("Wrong derivatives");
    //   // dthddot/du0 = 1. / inertia
    //   *Hu = x1_H_dot * Eigen::Vector2d(0., 1. / _inertia);
    // }

    // return x1;
  }

  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    const Eigen::Matrix3d R{ Eigen::Matrix3d::Identity() };
    const Eigen::Vector3d t(state.first.theta(), state.second, 0.0);
    return { { R, t } };
  }

  void environment(const prx::obstacle_loader_t& loader)
  {
    auto bounds = _state_space->sampler.bounds();
    auto env_min_bounds = loader.min_bounds();
    auto env_max_bounds = loader.max_bounds();
    bounds.first.first = env_min_bounds[0];
    bounds.first.second = env_max_bounds[1];
    _state_space->sampler.bounds(bounds.first.first, bounds.first.second, bounds.second.first, bounds.second.second);
  }
  // virtual void sense(const State& x0, const Control& u0, const double& dt, const Parameters& params) = 0;

protected:
  const double _friction;  //{ 0.1 };
  const double _length;    //{ 0.5 };
  const double _mass;      //{ 0.15 };
  const double _inertia;   //{ mass * length * length };
  const double _gravity;
  const double _u_max;
};

};  // namespace prx