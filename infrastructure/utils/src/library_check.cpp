#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

int main(int argc, char** argv)
{
  const std::string node_name{ "utils_lib" };
  ros::init(argc, argv, node_name);
  std::cout << "mujoco_ros/utils library running!" << std::endl;
  return 0;
}
