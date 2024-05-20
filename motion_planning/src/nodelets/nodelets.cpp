#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <motion_planning/motion_planning_tree.hpp>
#include <motion_planning/tree_viz_publisher.hpp>
#include <motion_planning/sbmp_publisher.hpp>
#include <motion_planning/tree_to_trajectories.hpp>

#include <prx/planning/planners/planner.hpp>
#include <prx/planning/planners/rrt.hpp>
#include <prx/planning/planners/dirt.hpp>
#include <prx/planning/planners/aorrt.hpp>

namespace motion_planning
{

using MotionPlanningTree = mp_tree_t<nodelet::Nodelet>;
using MotionPlanningTreeVizPublisher = mp_tree_viz_publisher_t<nodelet::Nodelet>;
using TreeToTrajectories = tree_to_trajectories_t<nodelet::Nodelet>;
using DirtPublisher = sbmp_publisher_t<prx::dirt_t, prx::dirt_specification_t, prx::dirt_query_t, nodelet::Nodelet>;
using RRTPublisher = sbmp_publisher_t<prx::rrt_t, prx::rrt_specification_t, prx::rrt_query_t, nodelet::Nodelet>;
using AORRTPublisher = sbmp_publisher_t<prx::aorrt_t, prx::aorrt_specification_t, prx::aorrt_query_t, nodelet::Nodelet>;

}  // namespace motion_planning
PLUGINLIB_EXPORT_CLASS(motion_planning::TreeToTrajectories, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::MotionPlanningTree, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::MotionPlanningTreeVizPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::DirtPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::RRTPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::AORRTPublisher, nodelet::Nodelet);
