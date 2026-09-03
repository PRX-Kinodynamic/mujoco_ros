#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <ml4kp_bridge/PlanStepStampedArray.h>
#include <ml4kp_bridge/SpacePointStampedArray.h>
#include <prx/simulation/system.hpp>
#include <prx/simulation/forward_propagation.hpp>
#include <prx/factor_graphs/factors/constraint_factor.hpp>
#include <prx/utilities/math/lie_utils.hpp>

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
      // const State xi0{ f->integrate(trajectory.back(), xd, prx::simulation_step) };
      const State xi{ f->integrate(trajectory.back(), xd_w, prx::simulation_step) };
      // PRX_DBG_VARS(xd, xd_w)
      // PRX_DBG_VARS(xi0, xi)
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
struct fg_trajectory_tracking_controller_t
{
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Plan = std::vector<prx::piecewise_step_t<Control, double>>;
  using Trajectory = std::vector<State>;
  using FgValues = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>;

  Plan plan;
  Trajectory traj_nominal;
  std::size_t steps_to_propagate;
  // Control u_min;
  // Control u_max;
  gtsam::LevenbergMarquardtParams lm_params;
  // std::function<Control(const State&, const Trajectory&, const Plan&)> fg_control;

  virtual Control fg_control(const State&, const Trajectory&, const Plan&) const
  {
    PRX_NOT_IMPLEMENTED;
  };

  virtual std::size_t size() const
  {
    return plan.size();
  }
};

template <typename DynamicalSystem>
class forward_propagation_t<DynamicalSystem,                               // no-lint
                            std::vector<typename DynamicalSystem::State>,  // no-lint
                            fg_trajectory_tracking_controller_t<DynamicalSystem>>
{
public:
  // using DynamicalSystem = dynamical_system_t<DerivedSystemType>;
  using State = typename DynamicalSystem::State;
  using StateDot = typename DynamicalSystem::StateDot;

  using Control = typename DynamicalSystem::Control;

  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;

  using PlanStep = prx::piecewise_step_t<Control, double>;
  using Plan = std::vector<PlanStep>;
  using Trajectory = std::vector<State>;
  using Controller = fg_trajectory_tracking_controller_t<DynamicalSystem>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  // static void propagate(Trajectory& trajectory, const Controller& graph_values, DynamicalSystemPtr f)
  static void propagate(Trajectory& trajectory, const Controller& ctrls, DynamicalSystemPtr f)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
    Plan plan{ ctrls.plan };
    Trajectory traj{ ctrls.traj_nominal };

    double t0{ 0. };
    // for (int j = 0; j < ctrls.traj_nominal.size() - 1; ++j)
    for (std::size_t i = 0; i < ctrls.steps_to_propagate; ++i)
    {
      // traj.erase(traj.begin());

      const PlanStep& u0{ plan[i] };

      for (double ti = 0.; ti < u0.duration; ti += prx::simulation_step, t0 += prx::simulation_step)
      {
        const State& x0{ trajectory.back() };
        const Control u{ ctrls.fg_control(x0, traj, plan, t0) };
        const StateDot xd{ f->ode(x0, u) };

        // const StateDot xd_w{ xd };
        const State x1{ f->integrate(x0, xd, prx::simulation_step) };

        trajectory.push_back(std::move(x1));
      }

      // plan.erase(plan.begin());
    }
  }
  template <typename... Args>
  static void propagate(Trajectory& trajectory, const State& x0, const Controller& ctrls, DynamicalSystemPtr f,
                        Args&... args)
  {
    trajectory.push_back(x0);
    propagate(trajectory, ctrls, f, args...);
  }

  // \dot{x} = f(x,u) + w;
  template <typename NoiseSampler>
  static void propagate(Trajectory& trajectory, const Controller& ctrls, DynamicalSystemPtr f, NoiseSampler& noise)
  {
    // prx_assert(trajectory.size() > 0,
    //            "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");
    // for (int i = 0; i < ctrls.size(); ++i)
    // {
    Plan plan{ ctrls.plan };
    Trajectory traj{ ctrls.traj_nominal };

    double t0{ 0. };
    // for (int j = 0; j < ctrls.traj_nominal.size() - 1; ++j)
    for (std::size_t i = 0; i < ctrls.steps_to_propagate; ++i)
    {
      // traj.erase(traj.begin());

      const PlanStep& u0{ plan[i] };

      for (double ti = 0.; ti < u0.duration; ti += prx::simulation_step, t0 += prx::simulation_step)
      {
        const State& x0{ trajectory.back() };
        const Control u{ ctrls.fg_control(x0, traj, plan, t0) };
        const StateDot xd{ f->ode(x0, u) };

        // const StateDot xd_w{ xd };
        const StateDot xd_w{ noise(xd) };
        const State x1{ f->integrate(x0, xd_w, prx::simulation_step) };

        trajectory.push_back(std::move(x1));
      }

      // plan.erase(plan.begin());
    }
    // }
    // Controller graph_values_p{ graph_values };

    // const int& N{ std::get<int>(graph_values) };
    // const gtsam::Values& values{ std::get<gtsam::Values>(graph_values) };
    // for (int i = 1; i < N; ++i)
    // {
    //   const gtsam::Key kxi{ gtsam::Symbol('X', i) };
    //   const State xi{ values.at<State>(kxi) };
    //   std::get<gtsam::Values>(graph_values_p).update(kxi, std::move(noise(xi)));
    // }

    // propagate(trajectory, x0, graph_values_p, f);
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

template <typename DynamicalSystem>
class forward_propagation_t<DynamicalSystem,                               // no-lint
                            std::vector<typename DynamicalSystem::State>,  // no-lint
                            std::vector<std::tuple<std::vector<typename DynamicalSystem::State>,
                                                   std::vector<typename DynamicalSystem::Control>, Eigen::MatrixXd>>>
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
  using NominalControls = std::vector<ControlVec>;
  using NominalTrajectory = std::vector<State>;
  using K_SSL = Eigen::MatrixXd;
  using Controller = std::vector<std::tuple<NominalTrajectory, NominalControls, K_SSL>>;

  template <typename... Args>
  static void propagate(Trajectory& trajectory, const State& x0, const Controller& ctrls, DynamicalSystemPtr f,
                        Args&... args)
  {
    trajectory.push_back(x0);
    propagate(trajectory, ctrls, f, args...);
  }

  static void propagate(Trajectory& trajectory, const Controller& ctrl, DynamicalSystemPtr f)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");

    for (auto& [nominal_traj, us, K] : ctrl)
    {
      // const K_SSL K{ std::get<K_SSL>(ctrl) };
      // const NominalControls us{ std::get<NominalControls>(ctrl) };
      // const NominalTrajectory nominal_traj{ std::get<NominalTrajectory>(ctrl) };

      Eigen::VectorXd w{ Eigen::VectorXd::Zero(K.cols()) };

      std::size_t idx{ 0 };
      std::size_t u_idx{ 0 };
      for (auto xi : nominal_traj)
      {
        const State& x0{ trajectory.back() };
        w.segment<DimX>(idx) = prx::TangentBetween(xi, x0);

        // const Eigen::MatrixXd dU{ K * w };
        const Eigen::VectorXd dU{ K * w };  //
        const ControlVec du_t{ dU.segment<DimU>(u_idx) };
        const StateDot xd{ f->ode(x0, du_t) };
        const State x1{ f->integrate(trajectory.back(), xd, prx::simulation_step) };
        trajectory.push_back(std::move(x1));
        idx += DimX;
        u_idx += DimU;
      }
    }
  }

  // \dot{x} = f(x,u) + w;
  template <typename NoiseSampler>
  static void propagate(Trajectory& trajectory, const Controller& ctrl, DynamicalSystemPtr f, NoiseSampler& noise)
  {
    prx_assert(trajectory.size() > 0,
               "forward_propagation_t::propagate] trajectory needs to contain at least the initial state");

    PRX_DBG_VARS(ctrl.size());
    for (auto& [nominal_traj, nominal_plan, K] : ctrl)
    {
      // const K_SSL K{ std::get<K_SSL>(ctrl) };
      // const NominalControls us{ std::get<NominalControls>(ctrl) };
      // const NominalTrajectory nominal_traj{ std::get<NominalTrajectory>(ctrl) };

      Eigen::VectorXd w{ Eigen::VectorXd::Zero(K.cols()) };

      std::size_t idx{ 0 };
      std::size_t u_idx{ 0 };
      std::size_t un_idx{ 0 };
      // PRX_DBG_VARS(K);
      for (auto xi : nominal_traj)
      {
        const State& x0{ trajectory.back() };
        // if (idx == 0)
        // {
        //   // w.segment<DimX>(idx) = gtsam::traits<State>::Logmap(x0);
        //   w[idx] = x0.x();
        //   w[idx + 1] = x0.y();
        //   w[idx + 2] = x0.theta();
        // }
        // else
        // {
        w[idx] = xi.x() - x0.x();
        w[idx + 1] = xi.y() - x0.y();
        w[idx + 2] = xi.theta() - x0.theta();
        // w.segment<DimX>(idx) = gtsam::traits<State>::Logmap(xi) - gtsam::traits<State>::Logmap(x0);
        // prx::TangentBetween(x0, xi);
        // }

        const Eigen::VectorXd dU{ K * w };  //
        const ControlVec u_t{ nominal_plan[un_idx] };
        const ControlVec du_t{ dU.segment<DimU>(u_idx) };
        const ControlVec u_eff{ u_t + du_t };
        const StateDot xd{ f->ode(x0, u_eff) };
        // const StateDot xd{ f->ode(x0, u_t) };
        const StateDot xd_w{ xd };
        // PRX_DBG_VARS(w)
        // PRX_DBG_VARS(dU)

        // const StateDot xd_w{ noise(xd) };
        const State x1{ f->integrate(x0, xd_w, prx::simulation_step) };
        trajectory.push_back(std::move(x1));
        idx += DimX;
        u_idx += DimU;
        un_idx++;
      }
    }

    /////
    // trajectory.back() = std::move(noise(trajectory.back()));
    // for (auto& [K, dt] : ctrls)
    // {
    //   for (double ti = 0.; ti < dt; ti += prx::simulation_step)
    //   {
    //     // const State x1{ propagate(trajectory.back(), K, f) };
    //     // const StateDot xdot{ ode(x0, u0, jacs ? &xd_H_x0 : nullptr, jacs ? &xd_Hu_u0 : nullptr) };
    //     const StateDot xd{ xdot(trajectory.back(), K, f) };
    //     const StateDot xd_w{ noise(xd) };
    //     const State x1{ f->integrate(trajectory.back(), xd_w, prx::simulation_step) };
    //     trajectory.push_back(std::move(x1));
    //   }
    // }
  }

protected:
};

}  // namespace prx
