#include <ros/node_handle.h>
#include <prx/utilities/general/param_loader.hpp>
#include <thread>
#include <utils/rosparams_utils.hpp>
#include <utils/nodelet_as_node.hpp>

#include <ml4kp_bridge/defs.h>
#include <analytical/simulator.hpp>
#include <utils/dbg_utils.hpp>

int main(int argc, char** argv)
{
  const std::string node_name{ "AnalyticalSimulator" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  analytical::simulator_t sim(nh);

  ros::spin();

  return 0;
}