#pragma once
#include <gtsam/geometry/Rot2.h>
#include <visualization_msgs/Marker.h>
#include <ml4kp_bridge/SpacePointStamped.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx/utilities/general/type_conversions.hpp>
#include "ml4kp_bridge/product_lie_group.hpp"

namespace ml4kp_bridge
{
inline visualization_msgs::Marker create_marker(const double scale = 0.01,
                                                const std::vector<double> color = { 1, 1, 0, 1 })
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  if (color.size() == 4)
  {
    marker.color.a = color[0];  // Don't forget to set the alpha!
    marker.color.r = color[1];
    marker.color.g = color[2];
    marker.color.b = color[3];
  }

  return marker;
}

template <typename Value, typename IndexOrValue, typename VectorOfValues>
inline double value_or_index(const IndexOrValue index, const VectorOfValues& vector, const Value value)
{
  if constexpr (std::is_integral_v<IndexOrValue>)
  {
    return vector[index];
  }
  else if constexpr (std::is_floating_point_v<IndexOrValue>)
  {
    return value;
  }
  prx_throw("[marker_utils::value_or_index] incorrect type");
}

template <typename XValue, typename YValue, typename ZValue>
inline void update_point(geometry_msgs::Point& pt, const ml4kp_bridge::SpacePointStamped& state,  // no-lint
                         const XValue x_value, const YValue y_value, const ZValue z_value)
{
  pt.x = value_or_index(x_value, state.space_point.point, x_value);
  pt.y = value_or_index(y_value, state.space_point.point, y_value);
  pt.z = value_or_index(z_value, state.space_point.point, z_value);
}

template <typename XValue, typename YValue, typename ZValue>
inline void update_point(geometry_msgs::Point& pt,
                         const gtsam::ProductLieGroupV43<gtsam::Rot2, double>& state,  // no-lint
                         const XValue x_value, const YValue y_value, const ZValue z_value)
{
  pt.x = state.first.theta();
  pt.y = state.second;
  pt.z = 0.0;
}

template <typename State, typename XValue, typename YValue, typename ZValue>
inline void update_marker(visualization_msgs::Marker& marker,
                          const std::vector<State>& traj,  // no-lint
                          const XValue x_value, const YValue y_value, const ZValue z_value)
{
  marker.id++;
  prx_assert(marker.type == visualization_msgs::Marker::LINE_STRIP,
             "[marker_utils::vector<SpacePointStamped>] Marker expected to be LINE_STRIP, got " +
                 prx::utilities::convert_to<std::string>(marker.type));
  marker.points.clear();

  for (auto&& state : traj)
  {
    marker.points.emplace_back();
    update_point(marker.points.back(), state, x_value, y_value, z_value);

    // marker.points.back().x = value_or_index(x_value, state.space_point.point, x_value);
    // marker.points.back().y = value_or_index(y_value, state.space_point.point, y_value);
    // marker.points.back().z = value_or_index(z_value, state.space_point.point, z_value);
  }
}

}  // namespace ml4kp_bridge