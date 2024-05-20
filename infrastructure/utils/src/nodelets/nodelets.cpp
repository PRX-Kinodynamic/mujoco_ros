#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/defs.h>

#include <utils/environment_publisher.hpp>
#include <utils/trajectory_viz_publisher.hpp>
#include <utils/plan_to_trajectory_publisher.hpp>
#include <utils/topic_to_file.hpp>

namespace utils
{

using EnvironmentPublisher = environment_publisher_t<nodelet::Nodelet>;
using TrajectoryVizPublisher = trajectory_viz_publisher_t<nodelet::Nodelet>;
using PlanToTrajectoryPublisher = plan_to_trajectory_publisher_t<nodelet::Nodelet>;
using TrajectoryToFile = topic_to_file_t<ml4kp_bridge::Trajectory, ml4kp_bridge::to_file, nodelet::Nodelet>;
using PlanStampedToFile = topic_to_file_t<ml4kp_bridge::PlanStamped, ml4kp_bridge::to_file, nodelet::Nodelet>;
using PlanToFile = topic_to_file_t<ml4kp_bridge::Plan, ml4kp_bridge::to_file, nodelet::Nodelet>;
using StateToFile = topic_to_file_t<ml4kp_bridge::SpacePoint, ml4kp_bridge::to_file, nodelet::Nodelet>;
using StateStampedToFile = topic_to_file_t<ml4kp_bridge::SpacePointStamped, ml4kp_bridge::to_file, nodelet::Nodelet>;

}  // namespace utils
PLUGINLIB_EXPORT_CLASS(utils::EnvironmentPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::TrajectoryVizPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::PlanToTrajectoryPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::TrajectoryToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::PlanStampedToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::PlanToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::StateToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::StateStampedToFile, nodelet::Nodelet);
