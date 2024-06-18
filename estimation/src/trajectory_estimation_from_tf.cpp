#include <thread>

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <estimation/TrajectoryEstimation.h>
#include <estimation/StateEstimation.h>
#include <estimation/fg_trajectory_estimation.hpp>
#include <analytical/fg_ltv_sde.hpp>

// ML4KP
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"

// GTSAM
#include <gtsam/nonlinear/ISAM2.h>

int main(int argc, char** argv)
{
  const std::string node_name{ "TrajectoryEstimationFromTf" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // using EulerFactor = prx::fg::euler_integration_factor_t<ltv_sde::State, ltv_sde::StateDot>;
  // using DynamicsFactor = prx::fg::euler_integration_factor_t<ltv_sde::StateDot, ltv_sde::Control>;
  // using ObservationFactor = gtsam::PriorFactor<ltv_sde::State>;
  // trajectory_estimator<prx::fg::ltv_sde_utils_t, EulerFactor, DynamicsFactor, ObservationFactor> estimator(nh);

  // ros::spin();

  return 0;
}