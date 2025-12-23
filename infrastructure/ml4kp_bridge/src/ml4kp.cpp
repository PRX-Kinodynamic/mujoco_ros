#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

int main(int argc, char** argv)
{
  const std::string node_name{ "msgs_interface" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  const std::string root{ ros::this_node::getName() };
  std::cout << "ML4KP loaded!" << std::endl;
  ros::spin();
}
