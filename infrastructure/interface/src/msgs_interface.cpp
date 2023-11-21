#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_mushr.hpp>

#include <interface/msg_translator.hpp>
#include <interface/msg_translation.hpp>
#include <interface/utils.hpp>

int main(int argc, char** argv)
{
  const std::string node_name{ "msgs_interface" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  const std::string root{ ros::this_node::getNamespace() };

  std::string subscriber_topic{};
  std::string publisher_topic{};

  interface::get_param_and_check(nh, root + "/from_topic", subscriber_topic);
  interface::get_param_and_check(nh, root + "/to_topic", publisher_topic);

  interface::msg_translator_t<prx_models::MushrPlan, ml4kp_bridge::Plan> msg_translator(nh, subscriber_topic,
                                                                                        publisher_topic);

  ros::spin();
}