#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/defs.h>

#include <utils/environment_publisher.hpp>
#include <utils/trajectory_viz_publisher.hpp>
#include <utils/plan_to_trajectory_publisher.hpp>
#include <utils/topic_to_file.hpp>
#include <utils/graph_viz_publisher.hpp>

namespace utils
{
inline void collision_to_file(const std_msgs::Bool& msg, std::ofstream& ofs)
{
  const std::string str{ msg.data ? "true" : "false" };
  ofs << "Collision: " << str << "\n";
}

using EnvironmentPublisher = environment_publisher_t<nodelet::Nodelet>;
using TrajectoryVizPublisher = trajectory_viz_publisher_t<nodelet::Nodelet>;
using PlanToTrajectoryPublisher = plan_to_trajectory_publisher_t<nodelet::Nodelet>;
using TrajectoryToFile = topic_to_file_t<ml4kp_bridge::Trajectory, ml4kp_bridge::to_file, nodelet::Nodelet>;
using PlanStampedToFile = topic_to_file_t<ml4kp_bridge::PlanStamped, ml4kp_bridge::to_file, nodelet::Nodelet>;
using PlanToFile = topic_to_file_t<ml4kp_bridge::Plan, ml4kp_bridge::to_file, nodelet::Nodelet>;
using StateToFile = topic_to_file_t<ml4kp_bridge::SpacePoint, ml4kp_bridge::to_file, nodelet::Nodelet>;
using StateStampedToFile = topic_to_file_t<ml4kp_bridge::SpacePointStamped, ml4kp_bridge::to_file, nodelet::Nodelet>;
using CollisionToFile = topic_to_file_t<std_msgs::Bool, collision_to_file, nodelet::Nodelet>;
using GraphVizPublisher = graph_viz_publisher_t<nodelet::Nodelet>;

}  // namespace utils
PLUGINLIB_EXPORT_CLASS(utils::EnvironmentPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::TrajectoryVizPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::PlanToTrajectoryPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::TrajectoryToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::PlanStampedToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::PlanToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::StateToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::StateStampedToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::GraphVizPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(utils::CollisionToFile, nodelet::Nodelet);
