#pragma once
#include <visualization_msgs/MarkerArray.h>
#include <ml4kp_bridge/defs.h>

namespace interface
{

struct gaussian_params_t
{
  gaussian_params_t() : idx(0), frame_id("world"), color(1, 0, 0, 0), confidence(7.815)
  {
  }
  int idx;
  std::string frame_id;
  Eigen::Vector<double, 3> position;
  Eigen::Quaterniond orientation;
  Eigen::Vector<double, 4> color;  // alpha first: ARGB
  double confidence;
  Eigen::Vector<double, 3> axis;
};

static visualization_msgs::Marker gaussian_to_ellipse_marker(const gaussian_params_t& input)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = input.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "confidence_ellipse";
  marker.id = input.idx;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = input.position[0];
  marker.pose.position.y = input.position[1];
  marker.pose.position.z = input.position[2];
  marker.pose.orientation.x = input.orientation.x();
  marker.pose.orientation.y = input.orientation.y();
  marker.pose.orientation.z = input.orientation.z();
  marker.pose.orientation.w = input.orientation.w();
  marker.color.a = input.color[0];
  marker.color.r = input.color[1];
  marker.color.g = input.color[2];
  marker.color.b = input.color[3];
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = input.confidence * std::sqrt(input.axis[0]);  // 7.815 * std::sqrt(D[0]);
  marker.scale.y = input.confidence * std::sqrt(input.axis[1]);  // 7.815 * std::sqrt(D[1]);
  marker.scale.z = input.confidence * std::sqrt(input.axis[2]);  // 7.815 * std::sqrt(D[2]);
  return marker;
}

}  // namespace interface
