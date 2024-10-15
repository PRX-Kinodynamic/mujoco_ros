#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <utils/nodelet_as_node.hpp>
#include <utils/dbg_utils.hpp>

#include <motion_planning/motion_planning_tree.hpp>
#include <motion_planning/tree_viz_publisher.hpp>
#include <motion_planning/sbmp_publisher.hpp>
#include <motion_planning/tree_to_trajectories.hpp>
#include <motion_planning/stela.hpp>
#include <motion_planning/scate.hpp>
#include <motion_planning/stela_stepper.hpp>

#include <ml4kp_bridge/fg_ltv_sde.hpp>

void test_sub(const std_msgs::String::ConstPtr& msg)
{
  DEBUG_VARS(msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mp_nodelet_debug");
  ros::NodeHandle nh("~");

  std::string node_name{};
  ROS_PARAM_SETUP(nh, node_name);
  DEBUG_VARS(node_name);

  // ros::Subscriber test_subscriber = nh.subscribe("/test_global", 100, test_sub);

  std::unique_ptr<utils::nodelet_as_node_t> node;
  if (node_name == "MotionPlanningTreeVizPublisher")
  {
    node = std::make_unique<motion_planning::mp_tree_viz_publisher_t<utils::nodelet_as_node_t>>();
  }
  else if (node_name == "MotionPlanningTree")
  {
    node = std::make_unique<motion_planning::mp_tree_t<utils::nodelet_as_node_t>>();
  }
  else if (node_name == "DirtPublisher")
  {
    node = std::make_unique<motion_planning::sbmp_publisher_t<prx::dirt_t, prx::dirt_specification_t, prx::dirt_query_t,
                                                              utils::nodelet_as_node_t>>();
  }
  else if (node_name == "AORRTPublisher")
  {
    node = std::make_unique<motion_planning::sbmp_publisher_t<prx::aorrt_t, prx::aorrt_specification_t,
                                                              prx::aorrt_query_t, utils::nodelet_as_node_t>>();
  }
  else if (node_name == "TreeToTrajectories")
  {
    node = std::make_unique<motion_planning::tree_to_trajectories_t<utils::nodelet_as_node_t>>();
  }
  else if (node_name == "Stela")
  {
    node = std::make_unique<motion_planning::stela_t<prx::fg::ltv_sde_utils_t, utils::nodelet_as_node_t>>();
  }
  else if (node_name == "StelaStepper")
  {
    node = std::make_unique<motion_planning::stela_stepper_t<utils::nodelet_as_node_t>>();
  }
  else if (node_name == "Scate")
  {
    node = std::make_unique<motion_planning::scate_t<prx::fg::ltv_sde_utils_t, utils::nodelet_as_node_t>>();
  }
  ROS_ASSERT_MSG(node != nullptr, "Node not initialized. Node name %s not valid.", node_name.c_str());
  // ros::AsyncSpinner spinner(4);  // Use 4 threads
  // spinner.start();
  DEBUG_VARS("Successful node");
  node->init();
  DEBUG_VARS("Spinning");
  ros::spin();

  // ros::waitForShutdown();

  return 0;
}