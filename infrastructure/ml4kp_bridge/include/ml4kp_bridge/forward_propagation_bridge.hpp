#pragma once

#include <ml4kp_bridge/PlanStepStampedArray.h>
#include <ml4kp_bridge/SpacePointStampedArray.h>
#include <prx/simulation/system.hpp>
#include <prx/simulation/forward_propagation.hpp>

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
      const State xi{ f->propagate(trajectory.back(), plan_step.control, prx::simulation_step) };
      trajectory.push_back(std::move(noise(xi)));
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

}  // namespace prx
