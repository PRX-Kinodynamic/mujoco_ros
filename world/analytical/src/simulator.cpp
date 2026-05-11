#include <ros/node_handle.h>
#include <prx/utilities/general/param_loader.hpp>
#include <thread>
#include <utils/rosparams_utils.hpp>
#include <utils/nodelet_as_node.hpp>

#include <ml4kp_bridge/defs.h>
#include <analytical/simulator.hpp>
#include <utils/dbg_utils.hpp>
#include <prx_models/SO2_system.hpp>

int main(int argc, char** argv)
{
  const std::string node_name{ "AnalyticalSimulator" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string plant_name;
  PARAM_SETUP(nh, plant_name);

  if (plant_name == "prx_system")
  {
    analytical::simulator_t<prx::system_t> sim(nh);
    ros::spin();
  }
  else if (plant_name == "SO2_system")
  {
    analytical::simulator_t<prx::SO2_system_t> sim(nh);
    ros::spin();
  }

  return 0;
}