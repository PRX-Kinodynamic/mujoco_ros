#pragma once

#include <ros/assert.h>

#include <prx/simulation/playback/piecewise_plan.hpp>
#include <prx/simulation/playback/plan.hpp>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/space_bridge.hpp>
#include <prx/simulation/system.hpp>

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
                 const ml4kp_bridge::PlanStepStampedArray& msg)
{
  plan.resize(msg.data.size());
  for (int i = 0; i < msg.data.size(); ++i)
  {
    copy(plan[i], msg.data[i]);
  }
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
  // for (auto& step : plan)
  // {
  //   if (step.duration > split_time)
  //   {
  //     double remaining_time{ step.duration };
  //     while (remaining_time > 0.)
  //     {
  //       const double dt{ std::min(remaining_time, split_time) };
  //       split_plan.emplace_back(step.control, dt);
  //       remaining_time -= dt;
  //     }
  //   }
  // }
  // DEBUG_VARS(split_plan);
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

inline void to_file(const ml4kp_bridge::PlanStep& msg, std::ofstream& ofs)
{
  to_file(msg.control, ofs);
  ofs << msg.duration.data.toSec() << " ";
}

}  // namespace ml4kp_bridge