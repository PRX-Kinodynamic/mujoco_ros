#include <utils/nodelet_as_node.hpp>
#include "nodelets/aruco_wTc.cpp"
#include "nodelets/aruco_cTw.cpp"
#include "nodelets/plant_estimator.cpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nodelets_as_nodes");
  ros::NodeHandle nh;

  std::string node_name{};
  DEBUG_VARS(node_name);
  GLOBAL_PARAM_SETUP(nh, node_name);

  DEBUG_VARS(node_name);
  std::unique_ptr<utils::nodelet_as_node_t> node;
  if (node_name == "aruco_wTc")
  {
    node = std::make_unique<estimation::aruco_wTc_t<utils::nodelet_as_node_t>>();
  }

  ROS_ASSERT_MSG(node != nullptr, "Node not initialized. Node name %s not valid.", node_name.c_str());
  node->init();
  ros::spin();

  return 0;
}