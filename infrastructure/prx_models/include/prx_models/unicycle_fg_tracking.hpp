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
#include <interface/levenberg_marquardt_interface.hpp>

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

#include <prx_models/unicycle_model.hpp>

namespace prx
{

class unicycle_factor_t : public gtsam::NoiseModelFactorN<prx::unicycle_model_t::State, prx::unicycle_model_t::State,
                                                          prx::unicycle_model_t::Control>
{
public:
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using State = prx::unicycle_model_t::State;
  using Control = prx::unicycle_model_t::Control;
  using Base = gtsam::NoiseModelFactorN<State, State, Control>;

  unicycle_factor_t(const gtsam::Key& x1, const gtsam::Key& x0, const gtsam::Key& u01,  // no-lint
                    std::shared_ptr<prx::unicycle_model_t> plant, const NoiseModel& cost_model = nullptr)
    : _plant(plant), Base(cost_model, x1, x0, u01)
  {
  }

  virtual Eigen::VectorXd evaluateError(const State& x1, const State& x0, const Control& u01,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> Hx1 = boost::none,   // no-lint
                                        boost::optional<Eigen::MatrixXd&> Hx0 = boost::none,   // no-lint
                                        boost::optional<Eigen::MatrixXd&> Hu01 = boost::none) const override
  {
    Eigen::Matrix3d x1p_H_x0;
    Eigen::Matrix<double, 3, 2> x1p_H_u;
    Eigen::Matrix3d err_H_x1p, err_H_x1;

    const State x1p{ _plant->propagate(x0, u01, prx::simulation_step, &x1p_H_x0, &x1p_H_u) };

    const Eigen::Vector3d error{ prx::TangentBetween(x1p, x1, &err_H_x1p, &err_H_x1) };

    if (Hx1)
    {
      *Hx1 = err_H_x1;
    }
    if (Hx0)
    {
      *Hx0 = err_H_x1p * x1p_H_x0;
    }
    if (Hu01)
    {
      *Hu01 = err_H_x1p * x1p_H_u;
    }

    return error;
  }

protected:
  std::shared_ptr<prx::unicycle_model_t> _plant;
};

template <>
struct fg_trajectory_tracking_controller_t<prx::unicycle_model_t>
{
  using Base = fg_trajectory_tracking_controller_t<prx::unicycle_model_t>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using State = prx::unicycle_model_t::State;
  using Control = prx::unicycle_model_t::Control;

  using Plan = std::vector<prx::piecewise_step_t<Control, double>>;
  using Trajectory = std::vector<State>;

  using LessThanFn = prx::fg::VectorLessThanCmp<Control>;
  using GreaterThanFn = prx::fg::VectorGreaterThanCmp<Control>;
  using LessThanFactor = prx::fg::constraint_factor_t<Control, LessThanFn>;
  using GreaterThanFactor = prx::fg::constraint_factor_t<Control, GreaterThanFn>;

  std::shared_ptr<prx::unicycle_model_t> plant;
  std::string plant_parameters;
  Control u_min;
  Control u_max;

  Plan plan;
  Trajectory traj_nominal;
  gtsam::LevenbergMarquardtParams lm_params;

  fg_trajectory_tracking_controller_t() : u_max(1.1, 1.1), u_min(-1.1, -1.1)
  {
    ros::NodeHandle nh("~");
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    plant = prx::unicycle_model_t::create(plant_parameters);
    // gtsam::LevenbergMarquardtParams lm_params;
    interface::initialize(lm_params, ros::NodeHandle(nh, "lm"));
    lm_params.print();
  }

  static fg_trajectory_tracking_controller_t init(fg_trajectory_tracking_controller_t& other)
  {
    // : u_max(other.u_max), u_min(other.u_min), plant(prx::unicycle_model_t::create(other.plant_parameters))
    fg_trajectory_tracking_controller_t new_obj;
    new_obj.plant = prx::unicycle_model_t::create(other.plant_parameters);
    return new_obj;
    // plant = prx::unicycle_model_t::create(other.plant_parameters);
  }

  std::size_t size() const
  {
    return plan.size();
  }
  // Control fg_control(std::shared_ptr<prx::unicycle_model_t> plant, const Trajectory traj_gt, const Plan plan,
  //                      const State& xt, const Control u_min, const Control u_max,
  //                      gtsam::LevenbergMarquardtParams& lm_params)
  // std::function<Control(const State&, const Trajectory&, const Plan&)> fg_control =

  virtual Control fg_control(const State& xt, const Trajectory& traj_gt, const Plan& plan) const
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    // PRINT_MSG("Building FG")
    const NoiseModel f_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };
    const NoiseModel x0_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-3) };
    const NoiseModel xT_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };

    // DEBUG_VARS(traj_gt.size())
    // for (int i = 0; i < traj_gt.size() - 1; ++i)
    std::size_t i{ 0 };

    // DEBUG_VARS(plan.size());
    for (auto&& step : plan)
    {
      // double ti{ 0. };
      // DEBUG_VARS(step.control, step.duration);
      // while (ti < step.duration)
      // {
      const gtsam::Key xk0{ gtsam::Symbol('X', i) };
      const gtsam::Key xk1{ gtsam::Symbol('X', i + 1) };
      const gtsam::Key uk01{ gtsam::Symbol('U', i) };

      // const State xi_w{ traj_w[i] };
      const State xi_gt{ traj_gt[i] };

      values.insert(xk0, xi_gt);
      values.insert(uk01, step.control);
      // values.insert(uk01, plan[i].control);

      // if (i != 0)
      // {
      //   graph.addPrior(xk0, xi_gt);
      // }
      graph.emplace_shared<LessThanFactor>(uk01, u_min);
      graph.emplace_shared<GreaterThanFactor>(uk01, u_max);
      graph.emplace_shared<prx::unicycle_factor_t>(xk1, xk0, uk01, plant, f_nm);

      i++;
      // ti += prx::simulation_step;
      // }
    }

    const gtsam::Key xk0{ gtsam::Symbol('X', 0) };
    const gtsam::Key xkT{ gtsam::Symbol('X', traj_gt.size() - 1) };

    graph.addPrior(xk0, xt, x0_nm);
    graph.addPrior(xkT, traj_gt.back(), xT_nm);

    values.insert(xkT, traj_gt.back());

    std::vector<State> traj;
    std::vector<Control> ctrls;

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

    gtsam::Values result{ optimizer.optimize() };

    // const double err_prev{ graph.error(values) };
    // const double err_after{ graph.error(result) };
    // DEBUG_VARS(err_prev, err_after, optimizer.iterations())

    const gtsam::Key ku0{ gtsam::Symbol('U', 0) };
    const Control u0{ result.at<Control>(ku0) };

    // DEBUG_VARS(u0)
    return u0;
  };
};
}  // namespace prx

namespace ml4kp_bridge
{

// inline prx::unicycle_fg_controller_t split(prx::unicycle_fg_controller_t& ctrl, const double split_time)
// {
//   return prx::unicycle_fg_controller_t();
// }

// inline void copy(prx::unicycle_fg_controller_t& ctrl, const ml4kp_bridge::SlsGain& msg)
// {
// }

}  // namespace ml4kp_bridge
