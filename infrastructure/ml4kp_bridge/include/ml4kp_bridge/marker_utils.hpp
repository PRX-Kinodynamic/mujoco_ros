#pragma once
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <visualization_msgs/Marker.h>
#include <ml4kp_bridge/SpacePointStamped.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx/utilities/general/transforms.hpp>
#include <prx/utilities/general/type_conversions.hpp>
#include "ml4kp_bridge/product_lie_group.hpp"
// #include "utils/dbg_utils.hpp"

namespace ml4kp_bridge
{
inline visualization_msgs::Marker create_marker(const double scale = 0.01,
                                                const std::vector<double> color = { 1, 1, 0, 1 }, const int id = 0,
                                                const std::string ns = "marker", const std::string frame_id = "world")
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = id;
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

template <typename XValue, typename YValue, typename ZValue>
inline void update_point(geometry_msgs::Point& pt,
                         const gtsam::ProductLieGroupV43<gtsam::Pose2, Eigen::Vector3d>& state,  // no-lint
                         const XValue x_value, const YValue y_value, const ZValue z_value)
{
  pt.x = state.first.x();
  pt.y = state.first.y();
  if constexpr (std::is_integral_v<ZValue>)
  {
    pt.z = 0.0;
  }
  else if constexpr (std::is_floating_point_v<ZValue>)
  {
    pt.z = z_value;
  }
}

template <typename XValue, typename YValue, typename ZValue>
inline void update_point(geometry_msgs::Point& pt,
                         const gtsam::Pose2& state,  // no-lint
                         const XValue x_value, const YValue y_value, const ZValue z_value)
{
  pt.x = state.x();
  pt.y = state.y();
  if constexpr (std::is_integral_v<ZValue>)
  {
    pt.z = state.theta();
  }
  else if constexpr (std::is_floating_point_v<ZValue>)
  {
    pt.z = z_value;
  }
}

template <typename XValue, typename YValue, typename ZValue, int Dim>
inline void update_point(geometry_msgs::Point& pt, const Eigen::Vector<double, Dim>& state,  // no-lint
                         const XValue x_value, const YValue y_value, const ZValue z_value)
{
  pt.x = value_or_index(x_value, state, x_value);
  pt.y = value_or_index(y_value, state, y_value);
  pt.z = value_or_index(z_value, state, z_value);
}

template <typename XValue, typename YValue, typename ZValue>
inline void update_pose(geometry_msgs::Pose& pose,
                        const gtsam::ProductLieGroupV43<gtsam::Rot2, double>& state,  // no-lint
                        const XValue x_value, const YValue y_value, const ZValue z_value)
{
  update_point(pose.position, state, x_value, y_value, z_value);
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
}

template <typename XValue, typename YValue, typename ZValue>
inline void update_pose(geometry_msgs::Pose& pose,
                        const gtsam::ProductLieGroupV43<gtsam::Pose2, Eigen::Vector3d>& state,  // no-lint
                        const XValue x_value, const YValue y_value, const ZValue z_value)
{
  update_point(pose.position, state, x_value, y_value, 0.);
  const Eigen::Quaterniond q{ prx::euler_to_rotation<Eigen::Quaterniond>(std::vector<double>({ state.first.theta() }),
                                                                         "z") };
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

template <typename XValue, typename YValue, typename ZValue>
inline void update_pose(geometry_msgs::Pose& pose,
                        const gtsam::Pose2& state,  // no-lint
                        const XValue x_value, const YValue y_value, const ZValue z_value)
{
  update_point(pose.position, state, x_value, y_value, z_value);
  // const Eigen::Quaterniond q{ prx::euler_to_rotation<Eigen::Quaterniond>(std::vector<double>({ state.theta() }), "z")
  // };

  const Eigen::Quaterniond q{ prx::axis_to_rotation_matrix(state.theta(), 'Z') };

  if constexpr (std::is_integral_v<ZValue>)
  {
    pose.position.z = state.theta();
  }

  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

inline double norm(geometry_msgs::Point& pt0, geometry_msgs::Point& pt1)
{
  const double& dx{ pt0.x - pt1.x };
  const double& dy{ pt0.y - pt1.y };
  const double& dz{ pt0.z - pt1.z };
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

template <typename State, typename XValue, typename YValue, typename ZValue>
inline void update_marker(visualization_msgs::Marker& marker,
                          const std::vector<State>& traj,  // no-lint
                          const XValue x_value, const YValue y_value, const ZValue z_value,
                          const int LineType = visualization_msgs::Marker::LINE_STRIP)
{
  marker.id++;
  prx_assert(marker.type == visualization_msgs::Marker::LINE_STRIP or
                 marker.type == visualization_msgs::Marker::LINE_LIST,
             "[marker_utils::vector<SpacePointStamped>] Marker expected to be LINE_STRIP or LINE_LIST, got " +
                 prx::utilities::convert_to<std::string>(marker.type));

  if (marker.type == visualization_msgs::Marker::LINE_STRIP)
  {
    marker.points.clear();
    for (auto&& state : traj)
    {
      marker.points.emplace_back();
      update_point(marker.points.back(), state, x_value, y_value, z_value);
    }
  }
  else  // visualization_msgs::Marker::LINE_LIST
  {
    bool first{ true };
    int idx{ 0 };
    for (auto&& state : traj)
    {
      marker.points.emplace_back();
      update_point(marker.points.back(), state, x_value, y_value, z_value);
      const std::size_t& tot{ marker.points.size() };
      const double diff{ norm(marker.points[tot - 2], marker.points[tot - 1]) };
      if (not first)
      {
        // PRX_DBG_VARS(idx, diff, pt.x, pt.y, pt.z);
        if (diff > 1.)
        {
          auto pt = marker.points.back();
          // PRX_DBG_VARS(pt.x, pt.y, pt.z)
          marker.points.pop_back();
          marker.points.pop_back();
          // marker.points.push_back(marker.points.back());
          marker.points.push_back(pt);
        }
        else
        {
          marker.points.push_back(marker.points.back());
        }
      }
      first = false;
    }
    if (not first)
    {
      marker.points.push_back(marker.points.back());
    }
    idx++;
  }
}

}  // namespace ml4kp_bridge