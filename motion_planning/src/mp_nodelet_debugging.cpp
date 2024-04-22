#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <utils/nodelet_as_node.hpp>
#include <utils/dbg_utils.hpp>

#include <motion_planning/motion_planning_tree.hpp>
#include <motion_planning/tree_viz_publisher.hpp>

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

  ROS_ASSERT_MSG(node != nullptr, "Node not initialized. Node name %s not valid.", node_name.c_str());
  // ros::AsyncSpinner spinner(4);  // Use 4 threads
  // spinner.start();
  node->init();
  DEBUG_VARS("Spinning");
  ros::spin();

  // ros::waitForShutdown();

  return 0;
}