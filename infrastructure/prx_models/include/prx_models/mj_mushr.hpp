#pragma once
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"

#include <ml4kp_bridge/defs.h>
#include "prx_models/mj_copy.hpp"

#include <prx_models/MushrControl.h>
#include <prx_models/MushrPlan.h>
#include <prx_models/MushrObservation.h>
#include <prx_models/MushrFeedback.h>
#include <prx_models/MushrPlanner.h>

namespace prx_models
{

namespace mushr_t
{
static constexpr std::size_t u_dim{ 2 };
namespace control
{
static constexpr std::size_t steering_idx{ 0 };
static constexpr std::size_t velocity_idx{ 1 };
};  // namespace control
namespace sensors_t
{
static constexpr std::size_t PosX{ 0 };
static constexpr std::size_t PosY{ 1 };
static constexpr std::size_t PosZ{ 2 };
static constexpr std::size_t QuatW{ 3 };
static constexpr std::size_t QuatX{ 4 };
static constexpr std::size_t QuatY{ 5 };
static constexpr std::size_t QuatZ{ 6 };
};  // namespace sensors_t
};  // namespace mushr_t

template <typename Ctrl>
inline void copy(Ctrl ctrl_out, const prx_models::MushrControl& msg)
{
  ctrl_out[mushr_t::control::steering_idx] = msg.steering_angle.data;
  ctrl_out[mushr_t::control::velocity_idx] = msg.velocity.data;
}

template <typename Ctrl>
inline void copy(prx_models::MushrControl& msg, const Ctrl& ctrl)
{
  msg.steering_angle.data = ctrl[mushr_t::control::steering_idx];
  msg.velocity.data = ctrl[mushr_t::control::velocity_idx];
}

template <typename SensorData>
inline void get_observation(prx_models::MushrObservation& msg, const SensorData& sensordata)
{
  msg.pose.position.x = sensordata[mushr_t::sensors_t::PosX].data;
  msg.pose.position.y = sensordata[mushr_t::sensors_t::PosY].data;
  msg.pose.position.z = sensordata[mushr_t::sensors_t::PosZ].data;
  msg.pose.orientation.x = sensordata[mushr_t::sensors_t::QuatX].data;
  msg.pose.orientation.y = sensordata[mushr_t::sensors_t::QuatY].data;
  msg.pose.orientation.z = sensordata[mushr_t::sensors_t::QuatZ].data;
  msg.pose.orientation.w = sensordata[mushr_t::sensors_t::QuatW].data;
}

template <typename StateSpacePoint>
inline void copy(StateSpacePoint& state, const prx_models::MushrObservation& msg)
{
  state->at(0) = msg.pose.position.x;
  state->at(1) = msg.pose.position.y;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                               msg.pose.orientation.z);
  // Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  Eigen::Vector3d euler = prx::quaternion_to_euler(quat);
  state->at(2) = euler[2];
  // ROS_WARN("Copying observation: %f, %f, %f", state->at(0), state->at(1), state->at(2));
}

template <typename StateSpacePoint>
inline void copy(StateSpacePoint& state, const geometry_msgs::Pose2D& msg)
{
  state->at(0) = msg.x;
  state->at(1) = msg.y;
  state->at(2) = msg.theta;
  state->at(3) = 0.0;
  ROS_WARN("Setting current velocity to 0.0");
}
}  // namespace prx_models