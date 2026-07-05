#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <ml4kp_bridge/PlanStepStampedArray.h>
#include <ml4kp_bridge/SpacePointStampedArray.h>
#include <prx/simulation/system.hpp>
#include <prx/simulation/forward_propagation.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

namespace prx
{

template <typename DynamicalSystem>
class forward_propagation_t<DynamicalSystem,                               // no-lint
                            std::vector<typename DynamicalSystem::State>,  // no-lint
                            prx::piecewise_step_t<typename DynamicalSystem::Control, double>>
{
public:
  // using DynamicalSystem = dynamical_system_t<DerivedSystemType>;
  using State = typename DynamicalSystem::State;
  using StateDot = typename DynamicalSystem::StateDot;
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;

  using Trajectory = std::vector<State>;
  using Controller = prx::piecewise_step_t<typename DynamicalSystem::Control, double>;

  static void propagate(Trajectory& trajectory, const Controller& plan_step, DynamicalSystemPtr f)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
    for (double ti = 0.; ti < plan_step.duration; ti += prx::simulation_step)
    {
      const State xi{ f->propagate(trajectory.back(), plan_step.control, prx::simulation_step) };
      trajectory.push_back(std::move(xi));
    }
  }

  // \dot{x} = f(x,u) + w;
  template <typename NoiseSampler>
  static void propagate(Trajectory& trajectory, const Controller& plan_step, DynamicalSystemPtr f, NoiseSampler& noise)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
    for (double ti = 0.; ti < plan_step.duration; ti += prx::simulation_step)
    {
      const StateDot xd{ f->ode(trajectory.back(), plan_step.control) };
      const StateDot xd_w{ noise(xd) };
      const State xi{ f->integrate(trajectory.back(), xd_w, prx::simulation_step) };
      trajectory.push_back(std::move(xi));
      // trajectory.push_back(std::move(noise(xi)));
    }
  }

protected:
};

template <typename DynamicalSystem>
class forward_propagation_t<DynamicalSystem,                               // no-lint
                            std::vector<typename DynamicalSystem::State>,  // no-lint
                            std::vector<prx::piecewise_step_t<typename DynamicalSystem::Control, double>>>
{
public:
  // using DynamicalSystem = dynamical_system_t<DerivedSystemType>;
  using State = typename DynamicalSystem::State;
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;

  using Step = prx::piecewise_step_t<typename DynamicalSystem::Control, double>;
  using Controller = std::vector<Step>;

  using Trajectory = std::vector<State>;
  using FwdPropStep = forward_propagation_t<DynamicalSystem, Trajectory, Step>;

  template <typename... Args>
  static void propagate(Trajectory& trajectory, const Controller& plan, DynamicalSystemPtr f, Args&... args)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
    for (auto step : plan)
    {
      FwdPropStep::propagate(trajectory, step, f, args...);
    }
  }

  // \dot{x} = f(x,u) + w;
  // template <typename NoiseSampler>
  // static void propagate(Trajectory& trajectory, const Controller& plan, DynamicalSystemPtr f, NoiseSampler& noise)
  // {
  //   prx_assert(trajectory.size() > 0,
  //              "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
  //   for (auto step : plan)
  //   {
  //     FwdPropStep::propagate(trajectory, plan, f, noise);
  //   }
  // }

  // Start from x0 instead from the back of the trajectory
  template <typename... Args>
  static void propagate(Trajectory& trajectory, const State x0, const Controller& plan, DynamicalSystemPtr f,
                        Args&... args)
  {
    trajectory.clear();
    trajectory.push_back(x0);
    propagate(trajectory, plan, f, args...);
  }

  // \dot{x} = f(x,u) + w;
  // template <typename NoiseSampler>
  // static void propagate(Trajectory& trajectory, const State x0, const Controller& plan, DynamicalSystemPtr f,
  //                       NoiseSampler& noise)
  // {
  //   trajectory.clear();
  //   trajectory.push_back(x0);
  //   propagate(trajectory, plan, f, noise);
  // }

protected:
};

template <typename DynamicalSystem>
class forward_propagation_t<
    DynamicalSystem,                               // no-lint
    std::vector<typename DynamicalSystem::State>,  // no-lint
    std::tuple<gtsam::NonlinearFactorGraph, gtsam::Values, gtsam::LevenbergMarquardtParams, int>>
{
public:
  // using DynamicalSystem = dynamical_system_t<DerivedSystemType>;
  using State = typename DynamicalSystem::State;
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;

  using Trajectory = std::vector<State>;
  using Controller = std::tuple<gtsam::NonlinearFactorGraph, gtsam::Values, gtsam::LevenbergMarquardtParams, int>;

  // static void propagate(Trajectory& trajectory, const Controller& graph_values, DynamicalSystemPtr f)
  static void propagate(Trajectory& trajectory, const Controller& graph_values, DynamicalSystemPtr f)
  {
    const gtsam::NonlinearFactorGraph& graph{ std::get<gtsam::NonlinearFactorGraph>(graph_values) };
    const gtsam::Values& values{ std::get<gtsam::Values>(graph_values) };
    const gtsam::LevenbergMarquardtParams& lm_params{ std::get<gtsam::LevenbergMarquardtParams>(graph_values) };
    const int& N{ std::get<int>(graph_values) };
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

    for (int i = 0; i < N; ++i)
    {
      const gtsam::Key kxi{ gtsam::Symbol('X', i) };
      trajectory.push_back(values.at<State>());
    }
  }

  static void propagate(Trajectory& trajectory, const State x0, const Controller& graph_values, DynamicalSystemPtr f)
  {
    const gtsam::NonlinearFactorGraph& graph{ std::get<gtsam::NonlinearFactorGraph>(graph_values) };
    const gtsam::Values& values{ std::get<gtsam::Values>(graph_values) };
    const gtsam::LevenbergMarquardtParams& lm_params{ std::get<gtsam::LevenbergMarquardtParams>(graph_values) };
    const int& N{ std::get<int>(graph_values) };

    values.update(gtsam::Symbol('X', 0), x0);

    propagate(trajectory, graph_values, f);
  }

  // \dot{x} = f(x,u) + w;
  template <typename NoiseSampler>
  static void propagate(Trajectory& trajectory, const State x0, const Controller& graph_values, DynamicalSystemPtr f,
                        NoiseSampler& noise)
  {
    Controller graph_values_p{ graph_values };

    const int& N{ std::get<int>(graph_values) };
    const gtsam::Values& values{ std::get<gtsam::Values>(graph_values) };
    for (int i = 1; i < N; ++i)
    {
      const gtsam::Key kxi{ gtsam::Symbol('X', i) };
      const State xi{ values.at<State>(kxi) };
      std::get<gtsam::Values>(graph_values_p).update(kxi, std::move(noise(xi)));
    }

    propagate(trajectory, x0, graph_values_p, f);
  }

protected:
};

// Basic LQR:= u=-K*dx
template <typename DynamicalSystem>
class forward_propagation_t<
    DynamicalSystem,                               // no-lint
    std::vector<typename DynamicalSystem::State>,  // no-lint
    std::vector<std::pair<Eigen::Matrix<double, prx::dynamical_system_traits<DynamicalSystem>::ControlDimension,
                                        prx::dynamical_system_traits<DynamicalSystem>::StateDimension>,
                          double>>>
{
  static constexpr int DimX{ prx::dynamical_system_traits<DynamicalSystem>::StateDimension };
  static constexpr int DimU{ prx::dynamical_system_traits<DynamicalSystem>::ControlDimension };

public:
  // using DynamicalSystem = dynamical_system_t<DerivedSystemType>;
  using State = typename DynamicalSystem::State;
  using StateDot = typename DynamicalSystem::StateDot;
  using Control = typename DynamicalSystem::Control;
  using ControlVec = Eigen::Vector<double, DimU>;
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;

  using Trajectory = std::vector<State>;
  using Gain = Eigen::Matrix<double, DimU, DimX>;
  using Controller = std::vector<std::pair<Eigen::Matrix<double, DimU, DimX>, double>>;

  // template<typename >
  // template <typename NoiseSampler>
  static StateDot xdot(const State& x0, const Gain& K, DynamicalSystemPtr f)
  {
    const ControlVec ui{ -K * gtsam::traits<State>::Logmap(x0) };
    StateDot xdot;

    if constexpr (std::is_same_v<Control, ControlVec>)
    {
      xdot = f->ode(x0, ui);
      // return f->propagate(x0, ui, prx::simulation_step);
    }
    else if constexpr (std::is_same_v<Control, double>)
    {
      xdot = f->ode(x0, ui[0]);
    }
    return xdot;
    // return f->propagate(x0, xdot, prx::simulation_step);
    // prx_throw("[forward_propagation_t] Unknown control type!");
    // return State();
  }

  template <typename... Args>
  static void propagate(Trajectory& trajectory, const State& x0, const Controller& ctrls, DynamicalSystemPtr f,
                        Args&... args)
  {
    trajectory.push_back(x0);
    propagate(trajectory, ctrls, f, args...);
  }

  static void propagate(Trajectory& trajectory, const Controller& ctrls, DynamicalSystemPtr f)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
    // for (double ti = 0.; ti < plan_step.duration; ti += prx::simulation_step)
    for (auto& [K, dt] : ctrls)
    {
      for (double ti = 0.; ti < dt; ti += prx::simulation_step)
      {
        // const State x1{ propagate(trajectory.back(), K, f) };
        // trajectory.push_back(std::move(x1));
        const StateDot xd{ xdot(trajectory.back(), K, f) };
        const State x1{ f->integrate(trajectory.back(), xd, prx::simulation_step) };
        trajectory.push_back(std::move(x1));
      }
    }
  }

  // \dot{x} = f(x,u) + w;
  template <typename NoiseSampler>
  static void propagate(Trajectory& trajectory, const Controller& ctrls, DynamicalSystemPtr f, NoiseSampler& noise)
  {
    // trajectory.back() = std::move(noise(trajectory.back()));
    for (auto& [K, dt] : ctrls)
    {
      for (double ti = 0.; ti < dt; ti += prx::simulation_step)
      {
        // const State x1{ propagate(trajectory.back(), K, f) };
        // const StateDot xdot{ ode(x0, u0, jacs ? &xd_H_x0 : nullptr, jacs ? &xd_Hu_u0 : nullptr) };
        const StateDot xd{ xdot(trajectory.back(), K, f) };
        const StateDot xd_w{ noise(xd) };
        const State x1{ f->integrate(trajectory.back(), xd_w, prx::simulation_step) };
        trajectory.push_back(std::move(x1));
      }
    }
  }

protected:
};

}  // namespace prx
