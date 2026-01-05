#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <estimation/aruco_camera_to_world.hpp>

namespace estimation
{

using ArucoCameraToWorld = aruco_camera_to_world_t<nodelet::Nodelet>;

}  // namespace estimation
PLUGINLIB_EXPORT_CLASS(estimation::ArucoCameraToWorld, nodelet::Nodelet);
