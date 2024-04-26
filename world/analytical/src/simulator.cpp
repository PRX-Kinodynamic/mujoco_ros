#include <thread>
#include <utils/rosparams_utils.hpp>
#include <utils/nodelet_as_node.hpp>

#include <analytical/ltv_sde.hpp>
#include <analytical/simulator.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulator_t");

  std::unique_ptr<utils::nodelet_as_node_t> node;

  PRX_DEBUG_PRINT;
  node = std::make_unique<analytical::simulator_t<utils::nodelet_as_node_t>>();
  PRX_DEBUG_PRINT;

  ROS_ASSERT_MSG(node != nullptr, "Node not initialized.");
  PRX_DEBUG_PRINT;
  node->init();
  PRX_DEBUG_PRINT;
  ros::spin();

  return 0;
}