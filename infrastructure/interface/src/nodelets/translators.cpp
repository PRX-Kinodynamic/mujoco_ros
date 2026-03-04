#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/defs.h>
#include <ml4kp_bridge/fg_ltv_sde.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <interface/ackermann_msg.hpp>
#include <interface/mushr_translation.hpp>
#include <interface/nodelets/msg_translator.hpp>
#include <interface/control_viz_publisher.hpp>
#include <interface/ackermann_msg.hpp>
#include <interface/mushr_translation.hpp>

#include <prx_models/mushr.hpp>
#include <prx_models/mushr_factors.hpp>

#include <utils/topic_to_file.hpp>

#include <interface/SensorDataStamped.h>
#include <interface/graph_viz_publisher.hpp>
#include <interface/plan_tree_publisher.hpp>
#include <interface/mushr_observation_to_marker.hpp>
#include <interface/sensor_to_marker.hpp>

namespace interface
{
inline void sensor_data_stamped_to_file(const interface::SensorDataStamped& msg, std::ofstream& ofs)
{
  ml4kp_bridge::to_file(msg.header, ofs);
  for (auto value : msg.raw_sensor_data)
  {
    ofs << value << " ";
  }
  // ofs << "\n";
}

using ControlVisualizationNodelet = interface::control_vizualizer_t<prx::fg::ltv_sde_utils_t, nodelet::Nodelet>;
using MushrFromSpacePointNodelet = interface::msg_translator_t<prx_models::MushrControl, ml4kp_bridge::SpacePoint>;
using MushrFromSpacePointStampedNodelet =
    interface::msg_translator_t<prx_models::MushrControl, ml4kp_bridge::SpacePointStamped>;
using GroundTruthPoseNodelet = interface::msg_translator_t<prx_models::MushrObservation, interface::SensorDataStamped>;
// using MushrControlVisualizationNodelet = interface::control_vizualizer_t<prx_models::mushr_utils_t,
// nodelet::Nodelet>;
using SensorDataStampedToFile =
    utils::topic_to_file_t<interface::SensorDataStamped, sensor_data_stamped_to_file, nodelet::Nodelet>;

using AckermannFromSpacePointNodelet =
    interface::msg_translator_t<ackermann_msgs::AckermannDriveStamped, ml4kp_bridge::SpacePoint>;
using AckermannFromSpacePointStampedNodelet =
    interface::msg_translator_t<ackermann_msgs::AckermannDriveStamped, ml4kp_bridge::SpacePointStamped>;
using GroundTruthPoseNodelet = interface::msg_translator_t<prx_models::MushrObservation, interface::SensorDataStamped>;
using MushrObservationToMarkerNodelet = interface::mushr_observation_to_marker_t<nodelet::Nodelet>;
using GraphVizPublisher = graph_viz_publisher_t<nodelet::Nodelet>;
using PlanToTreePublisher = plan_to_tree_publisher_t<nodelet::Nodelet>;
using SensorToMarkerPublisher = sensor_to_marker_t<nodelet::Nodelet>;

}  // namespace interface
PLUGINLIB_EXPORT_CLASS(interface::PlanToTreePublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::MushrFromSpacePointNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::AckermannFromSpacePointNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::AckermannFromSpacePointStampedNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::GroundTruthPoseNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::ControlVisualizationNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::MushrFromSpacePointStampedNodelet, nodelet::Nodelet);
// PLUGINLIB_EXPORT_CLASS(interface::MushrControlVisualizationNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::SensorDataStampedToFile, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::MushrObservationToMarkerNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::GraphVizPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::SensorToMarkerPublisher, nodelet::Nodelet);
