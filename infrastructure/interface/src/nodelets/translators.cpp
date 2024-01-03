#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/defs.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "interface/ackermann_msg.hpp"
#include "interface/mushr_translation.hpp"
#include "interface/nodelets/msg_translator.hpp"
namespace interface
{

using MushrFromSpacePointNodelet = interface::msg_translator_t<prx_models::MushrControl, ml4kp_bridge::SpacePoint>;
using AckermannFromSpacePointNodelet =
    interface::msg_translator_t<ackermann_msgs::AckermannDriveStamped, ml4kp_bridge::SpacePoint>;
using GroundTruthPoseNodelet = 
    interface::msg_translator_t<prx_models::MushrObservation, mujoco_ros::SensorDataStamped>;
}  // namespace interface
PLUGINLIB_EXPORT_CLASS(interface::MushrFromSpacePointNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::AckermannFromSpacePointNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(interface::GroundTruthPoseNodelet, nodelet::Nodelet);
