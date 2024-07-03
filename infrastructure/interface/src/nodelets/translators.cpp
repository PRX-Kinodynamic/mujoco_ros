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

namespace interface
{

using ControlVisualizationNodelet = interface::control_vizualizer_t<prx::fg::ltv_sde_utils_t, nodelet::Nodelet>;
using MushrFromSpacePointNodelet = interface::msg_translator_t<prx_models::MushrControl, ml4kp_bridge::SpacePoint>;
using AckermannFromSpacePointNodelet =
    interface::msg_translator_t<ackermann_msgs::AckermannDriveStamped, ml4kp_bridge::SpacePoint>;
using GroundTruthPoseNodelet = interface::msg_translator_t<prx_models::MushrObservation, interface::SensorDataStamped>;
}  // namespace interface
PLUGINLIB_EXPORT_CLASS(interface::MushrFromSpacePointNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::AckermannFromSpacePointNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::GroundTruthPoseNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::ControlVisualizationNodelet, nodelet::Nodelet);
