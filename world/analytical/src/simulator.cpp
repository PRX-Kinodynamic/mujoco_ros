#include <thread>
#include <utils/rosparams_utils.hpp>
#include <utils/nodelet_as_node.hpp>

#include <analytical/ltv_sde.hpp>
#include <analytical/simulator.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulator_t");

  std::unique_ptr<utils::nodelet_as_node_t> node;

  node = std::make_unique<analytical::simulator_t<utils::nodelet_as_node_t>>();

  ROS_ASSERT_MSG(node != nullptr, "Node not initialized.");
  node->init();
  ros::spin();

  return 0;
}