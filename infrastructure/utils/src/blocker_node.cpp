#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <tf2_ros/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.cpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "BlockerNode");
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };

  std::string stop_topic;

  PARAM_SETUP(nh, stop_topic);

  utils::execution_status_t status(nh, stop_topic);

  ros::Rate rate(5);
  while (status.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  ros::shutdown();

  return 0;
}