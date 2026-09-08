#pragma once

#include <ros/assert.h>
#include <ml4kp_bridge/gtsam_bridge.hpp>

#include <ml4kp_bridge/forward_propagation_bridge.hpp>
#include <prx/simulation/playback/piecewise_plan.hpp>
#include <prx/simulation/playback/plan.hpp>
#include <ml4kp_bridge/SlsGain.h>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/space_bridge.hpp>
#include <prx/simulation/system.hpp>

#include <ml4kp_bridge/FgTrajectoryTracking.h>
#include <ml4kp_bridge/PlanStepStamped.h>
#include <ml4kp_bridge/PlanStepStampedArray.h>

namespace ml4kp_bridge
{
inline void copy(ml4kp_bridge::PlanStep& msg, const prx::plan_step_t& plan_step)
{
  copy(msg.control, plan_step.control);
  msg.duration.data = ros::Duration(plan_step.duration);
}

inline void copy(ml4kp_bridge::PlanStepStamped& msg, const prx::plan_step_t& plan_step)
{
  msg.header.seq++;
  msg.header.stamp = ros::Time::now();
  copy(msg.plan_step, plan_step);
}

inline void copy(prx::plan_step_t& plan_step, const ml4kp_bridge::PlanStep& msg)
{
  copy(plan_step.control, msg.control);
  plan_step.duration = msg.duration.data.toSec();
}

inline void copy(prx::plan_step_t& plan_step, const ml4kp_bridge::PlanStepStamped& msg)
{
  copy(plan_step, msg.plan_step);
}

template <typename ControlType>
inline void copy(prx::piecewise_step_t<ControlType, double>& plan_step, const ml4kp_bridge::PlanStep& msg)
{
  copy(plan_step.control, msg.control);
  plan_step.duration = msg.duration.data.toSec();
}

template <typename ControlType>
inline void copy(prx::piecewise_step_t<ControlType, double>& plan_step, const ml4kp_bridge::PlanStepStamped& msg)
{
  copy(plan_step, msg.plan_step);
}

template <typename ControlType>
inline void copy(std::vector<prx::piecewise_step_t<ControlType, double>>& plan,
                 const std::vector<ml4kp_bridge::PlanStepStamped>& msg)
{
  plan.resize(msg.size());
  for (int i = 0; i < msg.size(); ++i)
  {
    copy(plan[i], msg[i]);
  }
}
template <typename ControlType>
inline void copy(std::vector<prx::piecewise_step_t<ControlType, double>>& plan,
                 const ml4kp_bridge::PlanStepStampedArray& msg)
{
  ml4kp_bridge::copy(plan, msg.data);
}

template <typename ControlType>
inline std::vector<prx::piecewise_step_t<ControlType, double>>
split(std::vector<prx::piecewise_step_t<ControlType, double>>& plan, const double split_time)
{
  std::vector<prx::piecewise_step_t<ControlType, double>> split_plan;

  double remaining_time{ split_time };
  // std::size_t idx{ 0 };
  while (remaining_time > 0.)
  {
    if (plan.size() == 0)
    {
      break;
    }
    else if (plan.front().duration < prx::simulation_step)
    {
      plan.erase(plan.begin());
    }
    else if (plan.front().duration < remaining_time)
    {
      split_plan.push_back(plan.front());
      remaining_time -= plan.front().duration;
      plan.erase(plan.begin());
    }
    else
    {
      // need to split
      // const double dt{ std::min(remaining_time, split_time) };
      split_plan.emplace_back(plan.front().control, remaining_time);
      plan.front().duration -= remaining_time;
      remaining_time = 0.;
    }
  }
  if (plan.front().duration < prx::simulation_step)
  {
    plan.erase(plan.begin());
  }

  return split_plan;
  // std::swap(split_plan, plan);
}

template <int DimX, int DimU>
inline void copy(std::vector<std::pair<Eigen::Matrix<double, DimU, DimX>, double>>& gains,
                 const ml4kp_bridge::PlanStepStampedArray& msg)
{
  using Gain = Eigen::Matrix<double, DimU, DimX>;
  gains.clear();
  for (auto& p : msg.data)
  {
    const double secs{ p.plan_step.duration.data.toSec() };
    // const int total_gains{ static_cast<int>(std::ceil(secs / prx::simulation_step)) };
    const Eigen::Vector<double, DimX * DimU> k_vec{ Eigen::Map<const Eigen::Vector<double, DimX * DimU>>(
        p.plan_step.control.point.data(), DimX * DimU) };
    const Gain k{ k_vec.reshaped(DimU, DimX) };
    gains.push_back({ k, secs });
    // gains.insert(gains.end(), total_gains, k);  // std::vector<Gain>(total_gains, k);
  }
}

template <int DimX, int DimU>
inline std::vector<std::pair<Eigen::Matrix<double, DimU, DimX>, double>>
split(std::vector<std::pair<Eigen::Matrix<double, DimU, DimX>, double>>& gains, const double split_time)
{
  std::vector<std::pair<Eigen::Matrix<double, DimU, DimX>, double>> split_gains;
  double remaining_time{ split_time };
  // std::size_t idx{ 0 };
  while (remaining_time > 0.)
  {
    if (gains.size() == 0)
    {
      break;
    }
    else if (gains.front().second < remaining_time)
    {
      split_gains.push_back(gains.front());
      remaining_time -= gains.front().second;
      gains.erase(gains.begin());
    }
    else
    {
      // need to split
      // const double dt{ std::min(remaining_time, split_time) };
      split_gains.push_back({ gains.front().first, remaining_time });
      gains.front().second -= remaining_time;
      remaining_time = 0.;
    }
  }

  return split_gains;
}

template <typename State, typename Control>
inline std::vector<std::tuple<std::vector<State>, std::vector<Control>, Eigen::MatrixXd>>
split(std::vector<std::tuple<std::vector<State>, std::vector<Control>, Eigen::MatrixXd>>& ctrl, const double split_time)
{
  std::vector<std::tuple<std::vector<State>, std::vector<Control>, Eigen::MatrixXd>> split_ctrl;

  // double remaining_time{ split_time };
  // while (remaining_time > 0.)
  // {
  //   if (ctrl.size() == 0)
  //   {
  //     break;
  //   }
  //   else
  //   {
  //     std::cout << "TODO" << std::endl;
  //     // for (auto& [traj, ctrl, K] : ctrl)
  //     // {
  //     // }
  //   }
  // }
  std::vector<State> traj_split;
  std::vector<Control> ctrl_split;

  double ti{ 0.0 };
  // return split_ctrl;
  for (auto& [traj, ctrl, K] : ctrl)
  {
    std::size_t idx{ 0 };
    // std::vector<Eigen::VectorXd> K_split;
    while (ti < split_time)
    {
      traj_split.push_back(traj[idx]);
      ctrl_split.push_back(ctrl[idx]);
      // K_split.append(K())

      traj.erase(traj.begin());
      ctrl.erase(ctrl.begin());

      ti += prx::simulation_step;
    }
  }

  const Eigen::MatrixXd old_K{ std::get<Eigen::MatrixXd>(ctrl.front()) };

  const std::size_t car_rows{ ctrl_split.size() };
  const std::size_t cdr_rows{ old_K.rows() - car_rows };

  Eigen::MatrixXd K_car{ Eigen::MatrixXd::Zero(car_rows, old_K.cols()) };
  Eigen::MatrixXd K_cdr{ Eigen::MatrixXd::Zero(cdr_rows, old_K.cols()) };
  for (int i = 0; i < car_rows; ++i)
  {
    K_car.row(i) = old_K.row(i);
  }
  for (int i = 0; i < cdr_rows; ++i)
  {
    K_cdr.row(i) = old_K.row(car_rows + i);
  }
  std::get<Eigen::MatrixXd>(ctrl.front()) = K_cdr;
  // for (double ti = 0.; ti < split_ctrl; ti += prx::simulation_step)
  // {

  split_ctrl.push_back(std::make_tuple(traj_split, ctrl_split, K_car));
  return split_ctrl;
}

template <typename State, typename Control>
inline void copy(std::vector<std::tuple<std::vector<State>, std::vector<Control>, Eigen::MatrixXd>>& ctrl,
                 const ml4kp_bridge::SlsGain& msg)
{
  std::vector<State> trajectory;
  std::vector<Control> controls;
  Eigen::MatrixXd K{ Eigen::MatrixXd::Zero(msg.Krows, msg.Kcols) };

  Control ut;
  for (const auto& u_msg : msg.controls)
  {
    ml4kp_bridge::copy(ut, u_msg);
    controls.push_back(ut);
  }

  State xt;
  for (const auto& x_msg : msg.trajectory)
  {
    ml4kp_bridge::copy(xt, x_msg);
    trajectory.push_back(xt);
  }

  std::size_t idx{ 0 };
  for (int i = 0; i < msg.Krows; ++i)
  {
    for (int j = 0; j < msg.Kcols; ++j)
    {
      K(i, j) = msg.K[idx];
      idx++;
    }
  }

  ctrl.push_back(std::make_tuple(trajectory, controls, K));
}

template <typename DynamicalSystem>
inline prx::fg_trajectory_tracking_controller_t<DynamicalSystem>
split(prx::fg_trajectory_tracking_controller_t<DynamicalSystem>& ctrl, const double split_time_in)
{
  using Controller = prx::fg_trajectory_tracking_controller_t<DynamicalSystem>;
  Controller head{ Controller::init(ctrl) };
  const double epsilon{ 0.001 };

  double split_time{ split_time_in };
  double ti{ 0. };
  // PRX_DBG_VARS(ctrl.traj_nominal.size())
  while (ti <= split_time)
  {
    // PRX_DBG_VARS(ti)
    if (not ctrl.traj_nominal.empty())
    {
      ti += head.fg_dt;
      ctrl.traj_nominal.erase(ctrl.traj_nominal.begin());
    }
    else
    {
      split_time = ti;
      break;
    }
  }
  // PRX_DBG_VARS(ctrl.traj_nominal.size())

  // PRX_DBG_VARS(ti)
  ti = 0;
  head.steps_to_propagate = 0;

  std::size_t idx{ 0 };
  while (ti < split_time)
  {
    if (idx >= ctrl.plan.size())
    {
      idx++;
      break;
    }
    else
    {
      ti += ctrl.plan[idx].duration;
    }
    idx++;
    // ctrl.plan.erase(ctrl.plan.begin());
    // if (ctrl.plan.size() == 0)
    // {
    //   break;
    // }
    // ti = 1.5
    // split = 1.3
    // ctrl.plan = 0.5
  }

  const double excess{ ti - split_time };
  const double head_dt{ ctrl.plan[idx - 1].duration - excess };
  for (int i = 0; i < idx - 1; ++i)
  {
    ctrl.plan.erase(ctrl.plan.begin());
  }
  // if (ti > split_time)
  if (ti + epsilon >= split_time)
  {
    ctrl.plan.front().duration = excess;
  }
  if (ctrl.plan.size() > 0 and ctrl.plan.front().duration < ctrl.fg_dt / 2.)
  {
    ctrl.plan.erase(ctrl.plan.begin());
  }
  ///

  // while()
  head.steps_to_propagate = std::min(idx, head.plan.size());
  if (ti > split_time)
  {
    // const double t_cdr{ ti + ctrl.plan[0].duration - split_time };
    // const double t_car{ ctrl.plan[0].duration - t_cdr };

    // if (t_car > head.fg_dt)
    // {
    head.plan.insert(head.plan.begin() + idx - 1, head.plan[idx - 1]);
    head.plan[idx - 1].duration = head_dt;
    head.plan[idx].duration = excess;
    // head.steps_to_propagate++;
    // }

    // ctrl.plan[0].duration = t_cdr;
  }
  else
  {
    // head.plan.insert(head.plan.begin() + idx, head.plan[idx]);
    head.plan[idx].duration = ti;
    // head.plan[idx + 1].duration = ti - split_time;
    // head.steps_to_propagate++;
  }

  ctrl.steps_to_propagate = ctrl.plan.size();
  // PRX_DBG_VARS(head.plan);

  return head;
}

template <typename DynamicalSystem>
inline void copy(prx::fg_trajectory_tracking_controller_t<DynamicalSystem>& ctrl,
                 const ml4kp_bridge::FgTrajectoryTracking& msg)
{
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Trajectory = typename prx::fg_trajectory_tracking_controller_t<DynamicalSystem>::Trajectory;
  using Plan = typename prx::fg_trajectory_tracking_controller_t<DynamicalSystem>::Plan;

  using FwdPropOpenLoop = prx::forward_propagation_t<DynamicalSystem, Trajectory, Plan>;
  // std::vector<State> trajectory;
  // std::vector<Control> controls;

  // Plan plan;
  ml4kp_bridge::copy(ctrl.plan, msg.plan);
  ctrl.steps_to_propagate = msg.steps_to_propagate;
  ctrl.fg_dt = msg.fg_dt;
  // Control ut;
  // PRX_DBG_VARS(plan);
  // for (const auto& u_dt : plan)
  // {
  //   // ml4kp_bridge::copy(ut, u_msg);
  //   double ti{ 0. };
  //   while (ti < u_dt.duration)
  //   {
  //     // ml4kp_bridge::copy();
  //     ctrl.plan.emplace_back(u_dt.control, prx::simulation_step);
  //     ti += prx::simulation_step;
  //   }
  // }
  // PRX_DBG_VARS(ctrl.plan);

  State x0;
  ml4kp_bridge::copy(x0, msg.x0);

  std::swap(ctrl.fg_dt, prx::simulation_step);
  FwdPropOpenLoop::propagate(ctrl.traj_nominal, x0, ctrl.plan, ctrl.plant);
  std::swap(ctrl.fg_dt, prx::simulation_step);
  // for (const auto& x_msg : msg.trajectory)
  // {
  //   ml4kp_bridge::copy(xt, x_msg);
  //   ctrl.traj_nominal.push_back(xt);
  // }

  // PRX_DBG_VARS(msg)
  // PRX_DBG_VARS(x0)
  // PRX_DBG_VARS(ctrl.traj_nominal)
}

inline void to_file(const ml4kp_bridge::PlanStep& msg, std::ofstream& ofs)
{
  to_file(msg.control, ofs);
  ofs << msg.duration.data.toSec() << " ";
}

}  // namespace ml4kp_bridge