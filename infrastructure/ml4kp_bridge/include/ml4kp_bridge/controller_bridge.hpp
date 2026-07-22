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
template <typename DynamicalSystem, typename Controller>
struct controller_view_t
{
};

template <typename DynamicalSystem>
struct controller_view_t<DynamicalSystem, prx::piecewise_step_t<typename DynamicalSystem::Control, double>>
{
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using Controller = prx::piecewise_step_t<typename DynamicalSystem::Control, double>;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  static Control front(const Controller& ctrl, const State& x, DynamicalSystemPtr f)
  {
    const Control u{ ctrl.control };
    return f->bound(u);
  }
};

template <typename DynamicalSystem>
struct controller_view_t<DynamicalSystem, std::vector<prx::piecewise_step_t<typename DynamicalSystem::Control, double>>>
{
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using ControllerElement = prx::piecewise_step_t<typename DynamicalSystem::Control, double>;
  using Controller = std::vector<ControllerElement>;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  static Control front(const Controller& ctrl, const State& x, DynamicalSystemPtr f)
  {
    return controller_view_t<DynamicalSystem, ControllerElement>::front(ctrl.front(), x, f);
  }
};

template <typename DynamicalSystem>
struct controller_view_t<DynamicalSystem,
                         Eigen::Matrix<double, gtsam::traits<typename DynamicalSystem::Control>::dimension,
                                       gtsam::traits<typename DynamicalSystem::State>::dimension>>
{
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using Controller = Eigen::Matrix<double, gtsam::traits<typename DynamicalSystem::Control>::dimension,
                                   gtsam::traits<typename DynamicalSystem::State>::dimension>;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  static Control front(const Controller& ctrl, const State& x, DynamicalSystemPtr f)
  {
    const Control u{ -ctrl * gtsam::traits<State>::Logmap(x) };
    return f->bound(u);
  }
};

template <typename DynamicalSystem>
struct controller_view_t<
    DynamicalSystem,
    std::vector<std::pair<Eigen::Matrix<double, prx::dynamical_system_traits<DynamicalSystem>::ControlDimension,
                                        prx::dynamical_system_traits<DynamicalSystem>::StateDimension>,
                          double>>>
{
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;

  using ControllerElement = Eigen::Matrix<double, prx::dynamical_system_traits<DynamicalSystem>::ControlDimension,
                                          prx::dynamical_system_traits<DynamicalSystem>::StateDimension>;
  using Controller = std::vector<std::pair<ControllerElement, double>>;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  static Control front(const Controller& ctrl, const State& x, DynamicalSystemPtr f)
  {
    return controller_view_t<DynamicalSystem, ControllerElement>::front(ctrl.front().first, x, f);
  }
};

}  // namespace prx