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
#include <ml4kp_bridge/sampler_bridge.hpp>
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

namespace prx
{
class unicycle_model_t;
template <>
struct dynamical_system_traits<unicycle_model_t>
{
  // clang-format off
  enum {StateDimension = 3}; 
  enum {ControlDimension = 2}; 
  enum {ParametersDimension = 1}; 
  enum {ObservationDimension = 3};
  // clang-format on

  using State = gtsam::Pose2;
  using Control = Eigen::Vector<double, ControlDimension>;
  using Parameters = Eigen::Vector<double, ParametersDimension>;
  using Observation = gtsam::Pose2;

  using StateDot = Eigen::Vector<double, StateDimension>;
};

class unicycle_model_t : public prx::dynamical_system_t<unicycle_model_t>
{
public:
  using Base = prx::dynamical_system_t<unicycle_model_t>;
  using Derived = unicycle_model_t;

  const std::string Name = "unicycle";
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

  unicycle_model_t() : Base(default_params()) {};
  unicycle_model_t(const std::string params) : unicycle_model_t(prx::param_loader::create(params)) {};
  // unicycle_model_t(const std::string name) : Base(name) {};
  unicycle_model_t(prx::param_loader params) : Base(params)
  {
  }

  static prx::param_loader default_params()
  {
    prx::param_loader params;
    const std::string state_space_bounds_yaml =
        "bounds:\n"
        "    min: [-10, -10, -3.14159]\n"
        "    max: [+10, +10, +3.14159]\n";
    const std::string control_space_bounds_yaml =
        "bounds:\n"
        "    min: [-1, -1]\n"
        "    max: [+1, +1]\n";
    const std::string observation_space_bounds_yaml = state_space_bounds_yaml;
    const std::string parameter_space_bounds_yaml = "values: [0.001]\n";

    params["state_space"].from_string(state_space_bounds_yaml);
    params["control_space"].from_string(control_space_bounds_yaml);
    params["observation_space"].from_string(observation_space_bounds_yaml);
    params["parameter_space"].from_string(parameter_space_bounds_yaml);

    return params;
  }

  void initialize_geometries()
  {
    _geometries.push_back(std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX));
    _geometries.back()->initialize_geometry({ 0.42, 0.25, 0.25 });
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

  virtual ~unicycle_model_t() {};

  Observation sense(const State& x)
  {
    return x;
  }

  StateDot ode(const State& x0, const Control& u0, OptJacX Hx = nullptr, OptJacU Hu = nullptr)
  {
    using Mx = Eigen::Matrix<double, 3, 2>;
    const double& th{ x0.theta() };
    const double cth{ std::cos(th) };
    const double sth{ std::sin(th) };
    // const Mx M{ (Mx() << cth, 0., sth, 0., 0., 1.).finished() };
    const Mx M{ (Mx() << 1, 0., 0, 0., 0., 1.).finished() };
    const Eigen::Vector3d xdot{ M * u0 };
    return xdot;
  }

  State integrate(const State& x0, const StateDot& xd0, const double& dt, OptJacX Hx = nullptr, OptJacX Hxd = nullptr,
                  OptJacDT Hdt = nullptr)
  {
    using LieIntegrator = prx::fg::lie_integrator_t<gtsam::Pose2, Eigen::Vector3d, double>;
    const bool jacs{ Hx or Hxd or Hdt };

    Eigen::Matrix<double, 3, 3> x1_H_x0, x1_H_xdot;
    Eigen::Matrix<double, 3, 1> x1_H_dt;

    const gtsam::Pose2 x1{ LieIntegrator::integrate(x0, xd0, dt,                  // no-lint
                                                    jacs ? &x1_H_x0 : nullptr,    // no-lint
                                                    jacs ? &x1_H_xdot : nullptr,  // no-lint
                                                    jacs ? &x1_H_dt : nullptr) };
    return x1;
  }

  State propagate(const State& x0, const Control& u0, const double& dt, OptJacX Hx = nullptr, OptJacU Hu = nullptr,
                  OptJacDT Hdt = nullptr)
  {
    const bool jacs{ Hx or Hu or Hdt };
    JacX x1_H_x0, x1_H_xd, xd_H_x0;
    JacU xd_Hu_u0;
    JacDT x1_H_dt;

    const StateDot xdot{ ode(x0, u0, jacs ? &xd_H_x0 : nullptr, jacs ? &xd_Hu_u0 : nullptr) };
    const State x1{ integrate(x0, xdot, dt, jacs ? &x1_H_x0 : nullptr, jacs ? &x1_H_xd : nullptr,
                              jacs ? &x1_H_dt : nullptr) };

    // const Eigen::Vector3d xdot_H_x0{ (Eigen::Vector3d() << -u0[0] * sth, u0[0] * cth, 0.).finished() };
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
    return std::move(x1);
  }

  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    const Eigen::Matrix3d R{ prx::axis_to_rotation_matrix({ state.theta() }, 'Z') };
    const Eigen::Vector3d t(state.x(), state.y(), 0.0);
    return { { R, t } };
  }

  void environment(const prx::obstacle_loader_t& loader)
  {
    auto bounds = _state_space->sampler.bounds();
    auto env_min_bounds = loader.min_bounds();
    auto env_max_bounds = loader.max_bounds();
    bounds.first.head(2) = env_min_bounds.head(2);
    bounds.second.head(2) = env_max_bounds.head(2);
    _state_space->sampler.bounds(bounds.first, bounds.second);
  }
  // virtual void sense(const State& x0, const Control& u0, const double& dt, const Parameters& params) = 0;

protected:
  StateSpacePtr _state_space;
  ControlSpacePtr _control_space;
  ParametersSpacePtr _parameter_space;
  ObservationSpacePtr _sensor_space;
};
}  // namespace prx
