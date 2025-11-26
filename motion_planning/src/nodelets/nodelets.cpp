#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <motion_planning/tree_viz_publisher.hpp>
#include <prx_models/mushr.hpp>
#include <motion_planning/stela_sliding_window.hpp>
#include <motion_planning/clock_sync.hpp>
#include <motion_planning/tree_to_plan.hpp>

namespace motion_planning
{

using MotionPlanningTreeVizPublisher = mp_tree_viz_publisher_t<nodelet::Nodelet>;
using StelaWindowedMushr = stela_windowed_t<prx_models::mushr_stela_t, nodelet::Nodelet>;
using PlannerClockSync = clock_sync_t<nodelet::Nodelet>;
using TreeToPlanNodelet = tree_to_plan_t<nodelet::Nodelet>;

}  // namespace motion_planning

PLUGINLIB_EXPORT_CLASS(motion_planning::MotionPlanningTreeVizPublisher, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::StelaWindowedMushr, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::PlannerClockSync, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(motion_planning::TreeToPlanNodelet, nodelet::Nodelet);
