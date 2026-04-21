#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

#include <iterator>
#include <memory>
#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/std_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <interface/node_status.hpp>
#include <interface/SensorDataStamped.h>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/mushr.hpp>
#include <prx_models/PlannerStats.h>
#include <utils/dbg_utils.hpp>
#include <prx_models/planner_utils.hpp>

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrExperiments" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  double dt;
  std::vector<double> params, poly, pose, velocity, control;

  PARAM_SETUP(nh, dt)
  PARAM_SETUP(nh, pose)
  PARAM_SETUP(nh, poly)
  PARAM_SETUP(nh, params)
  PARAM_SETUP(nh, control)
  PARAM_SETUP(nh, velocity)

  Eigen::MatrixXd q1p_H_q0, q1p_H_qd0;
  Eigen::MatrixXd qd1p_H_qd0;

  Eigen::VectorXd _params(params.size()), _poly(poly.size());
  for (int i = 0; i < params.size(); ++i)
  {
    _params[i] = params[i];
  }
  for (int i = 0; i < poly.size(); ++i)
  {
    _poly[i] = poly[i];
  }

  gtsam::Pose2 q0(pose[0], pose[1], pose[2]);
  Eigen::Vector3d qdot0(velocity[0], velocity[1], velocity[2]);
  Eigen::Vector2d ctrl(control[0], control[1]);

  const gtsam::Pose2 pose_prediction{ prx_models::mushr_x_xdot_t::predict(q0, qdot0, dt, q1p_H_q0, q1p_H_qd0) };
  const Eigen::Vector3d vel_prediction{ prx_models::mushr_CtrlAccel_t<>::predict(qdot0, ctrl, dt, _params, _poly,
                                                                                 qd1p_H_qd0) };

  DEBUG_VARS(pose_prediction)
  DEBUG_VARS(vel_prediction)

  return 0;
}