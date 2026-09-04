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

  unicycle_factor_t(const gtsam::Key& x1, const gtsam::Key& x0, const gtsam::Key& u01,                     // no-lint
                    std::shared_ptr<prx::unicycle_model_t> plant, const double dt = prx::simulation_step,  // no-lint
                    const NoiseModel& cost_model = nullptr)
    : _plant(plant), Base(cost_model, x1, x0, u01), _dt(dt)
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

    const State x1p{ _plant->propagate(x0, u01, _dt, &x1p_H_x0, &x1p_H_u) };

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
  const double _dt;
};

template <>
struct fg_trajectory_tracking_controller_t<prx::unicycle_model_t>
{
  using Base = fg_trajectory_tracking_controller_t<prx::unicycle_model_t>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using State = prx::unicycle_model_t::State;
  using Control = prx::unicycle_model_t::Control;

  using PlanStep = prx::piecewise_step_t<Control, double>;
  using Plan = std::vector<PlanStep>;
  using Trajectory = std::vector<State>;

  using LessThanFn = prx::fg::VectorLessThanCmp<Control>;
  using GreaterThanFn = prx::fg::VectorGreaterThanCmp<Control>;
  using LessThanFactor = prx::fg::constraint_factor_t<Control, LessThanFn>;
  using GreaterThanFactor = prx::fg::constraint_factor_t<Control, GreaterThanFn>;

  std::shared_ptr<prx::unicycle_model_t> plant;
  std::string plant_parameters;
  Control u_min;
  Control u_max;
  double fg_dt;

  Plan plan;
  Trajectory traj_nominal;
  std::size_t steps_to_propagate;
  gtsam::LevenbergMarquardtParams lm_params;

  fg_trajectory_tracking_controller_t() : u_max(1.2, 1.2), u_min(-1.2, -1.2)
  {
    ros::NodeHandle nh("~");
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    plant = prx::unicycle_model_t::create(plant_parameters);
    // gtsam::LevenbergMarquardtParams lm_params;
    interface::initialize(lm_params, ros::NodeHandle(nh, "lm"));
    // lm_params.print();
  }

  static fg_trajectory_tracking_controller_t init(fg_trajectory_tracking_controller_t& other)
  {
    fg_trajectory_tracking_controller_t new_obj;
    new_obj.plant = prx::unicycle_model_t::create(other.plant_parameters);
    new_obj.plant_parameters = other.plant_parameters;
    new_obj.u_min = other.u_min;
    new_obj.u_max = other.u_max;
    new_obj.fg_dt = other.fg_dt;

    new_obj.plan = other.plan;
    new_obj.traj_nominal = other.traj_nominal;
    new_obj.lm_params = other.lm_params;
    new_obj.steps_to_propagate = other.steps_to_propagate;

    return new_obj;
  }

  std::size_t size() const
  {
    return plan.size();
  }
  // Control fg_control(std::shared_ptr<prx::unicycle_model_t> plant, const Trajectory traj_gt, const Plan plan,
  //                      const State& xt, const Control u_min, const Control u_max,
  //                      gtsam::LevenbergMarquardtParams& lm_params)
  // std::function<Control(const State&, const Trajectory&, const Plan&)> fg_control =

  virtual Control fg_control(const State& zt, const Trajectory& traj_gt, const Plan& plan_in, const double t0) const
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    // PRINT_MSG("Building FG")
    const NoiseModel f_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    const NoiseModel x0_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-3) };
    const NoiseModel xi_nm{ gtsam::noiseModel::Isotropic::Sigmas(Eigen::Vector3d(0.5, 0.5, 1.0)) };
    const NoiseModel xT_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };

    const double epsilon{ 0.0001 };
    // DEBUG_VARS(traj_gt.size())
    // for (int i = 0; i < traj_gt.size() - 1; ++i)
    std::size_t idx{ 0 };

    // DEBUG_VARS(ti)
    double t_accum{ 0. };
    while (t_accum + plan[idx].duration < t0)
    {
      // DEBUG_VARS(t_accum)
      idx++;
      t_accum += plan[idx].duration;
    }
    double ti{ t0 - t_accum };
    std::size_t traj_idx{ static_cast<std::size_t>((t0 + epsilon) / fg_dt) + 1 };
    // const std::size_t traj_step{ static_cast<std::size_t>((fg_dt + epsilon) / prx::simulation_step) };
    const std::size_t traj_step{ 1 };
    // DEBUG_VARS(traj_nominal.size(), plan.size())
    // DEBUG_VARS(t0, t_accum, idx, traj_idx, ti, traj_step)

    int i{ 0 };
    // Add the first
    const gtsam::Key xk0{ gtsam::Symbol('X', i) };
    graph.addPrior(xk0, zt, x0_nm);
    values.insert(xk0, zt);

    double dt{ std::fmod(ti, fg_dt) };
    for (; idx < plan.size(); ++idx)
    {
      // DEBUG_VARS(idx)
      for (; ti < plan[idx].duration; ti += fg_dt, ++i, traj_idx += traj_step)
      {
        const gtsam::Key xk0{ gtsam::Symbol('X', i) };
        const gtsam::Key xk1{ gtsam::Symbol('X', i + 1) };
        const gtsam::Key uk01{ gtsam::Symbol('U', i) };

        const State xi_gt{ traj_nominal[traj_idx] };
        const Control ui{ plan[idx].control };

        values.insert(uk01, ui);
        values.insert(xk1, xi_gt);

        // graph.addPrior(xk1, xi_gt, xi_nm);
        // graph.emplace_shared<LessThanFactor>(uk01, u_min);
        // graph.emplace_shared<GreaterThanFactor>(uk01, u_max);
        graph.emplace_shared<prx::unicycle_factor_t>(xk1, xk0, uk01, plant, dt, f_nm);

        dt = fg_dt;
        // DEBUG_VARS(i, t_accum, ti, traj_idx, idx, xi_gt, ui)
      }
      ti = 0;
    }
    // DEBUG_VARS(i, t_accum, ti, traj_idx, idx)
    const gtsam::Key xkT{ gtsam::Symbol('X', i) };
    graph.addPrior(xkT, traj_gt.back(), xT_nm);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

    gtsam::Values result{ optimizer.optimize() };
    // graph.printErrors(result);
    // result.print();

    const double err_prev{ graph.error(values) };
    const double err_after{ graph.error(result) };
    // DEBUG_VARS(err_prev, err_after, optimizer.iterations())

    const gtsam::Key ku0{ gtsam::Symbol('U', 0) };
    const Control u0{ result.at<Control>(ku0) };

    // DEBUG_VARS(u0)
    return u0;
  };
};

template <>  // primary template
struct stream_specialization<fg_trajectory_tracking_controller_t<prx::unicycle_model_t>> : std::true_type
{
};

template <>
struct streamer_t<fg_trajectory_tracking_controller_t<prx::unicycle_model_t>> : std::true_type
{
  using Ctrl = fg_trajectory_tracking_controller_t<prx::unicycle_model_t>;

public:
  static void to_stream(std::ostream& os, const Ctrl& ctrl)
  {
    os << "Plan:\n";
    prx::to_stream(os, ctrl.plan);
    os << "Nominal Trajectory:\n";
    prx::to_stream(os, ctrl.traj_nominal);
    os << "Steps to Propagate: ";
    prx::to_stream(os, ctrl.steps_to_propagate);
    os << "\n";
  }
};
//
}  // namespace prx
