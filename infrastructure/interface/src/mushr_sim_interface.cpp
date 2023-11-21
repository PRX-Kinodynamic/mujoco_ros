#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_mushr.hpp>

#include <interface/mushr_translation.hpp>
#include <interface/msg_translator.hpp>
#include <interface/utils.hpp>

int main(int argc, char** argv)
{
  const std::string node_name{ "msgs_interface" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  const std::string root{ ros::this_node::getName() };

  std::string perception_subscriber_topic{};
  std::string perception_publisher_topic{};
  std::string control_subscriber_topic{};
  std::string control_publisher_topic{};

  interface::get_param_and_check(nh, root + "/perception_from_topic", perception_subscriber_topic);
  interface::get_param_and_check(nh, root + "/perception_to_topic", perception_publisher_topic);
  interface::get_param_and_check(nh, root + "/control_from_topic", control_subscriber_topic);
  interface::get_param_and_check(nh, root + "/control_to_topic", control_publisher_topic);

  interface::msg_translator_t<prx_models::MushrObservation, mujoco_ros::SensorDataStamped> observation_translator(
      nh, perception_subscriber_topic, perception_publisher_topic);

  interface::msg_translator_t<prx_models::MushrPlan, ml4kp_bridge::Plan> control_translator(
      nh, control_subscriber_topic, control_publisher_topic);

  ros::spin();
}