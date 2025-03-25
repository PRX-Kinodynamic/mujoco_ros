#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <motion_planning/tree_viz_publisher.hpp>

namespace motion_planning
{

using MotionPlanningTreeVizPublisher = mp_tree_viz_publisher_t<nodelet::Nodelet>;

}  // namespace motion_planning

PLUGINLIB_EXPORT_CLASS(motion_planning::MotionPlanningTreeVizPublisher, nodelet::Nodelet);
