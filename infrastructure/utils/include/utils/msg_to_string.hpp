#pragma once

#include <tf/transform_datatypes.h>
#include <ml4kp_bridge/defs.h>
#include <ml4kp_bridge/template_utils.hpp>

namespace utils
{

inline std::string to_string(const std_msgs::Header& header)
{
  std::stringstream strstr{};
  strstr << header.stamp << " ";
  return strstr.str();
}

inline std::string to_string(const geometry_msgs::TransformStamped& tf)
{
  std::stringstream strstr{};
  const geometry_msgs::Vector3& vec{ tf.transform.translation };
  const geometry_msgs::Quaternion& quat{ tf.transform.rotation };

  strstr << to_string(tf.header);
  strstr << vec.x << " ";
  strstr << vec.y << " ";
  strstr << vec.z << " ";
  strstr << quat.w << " ";
  strstr << quat.x << " ";
  strstr << quat.y << " ";
  strstr << quat.z << " ";

  return strstr.str();
}

inline std::string to_string(const ml4kp_bridge::SpacePoint& msg)
{
  std::stringstream strstr{};
  for (auto e : msg.point)
  {
    strstr << e << " ";
  }
  return strstr.str();
}
inline std::string to_string(const ml4kp_bridge::SpacePointStamped& msg)
{
  std::stringstream strstr{};
  strstr << to_string(msg.header);
  strstr << to_string(msg.space_point);
  return strstr.str();
}

inline std::string to_string(const ml4kp_bridge::PlanStamped& message)
{
  std::stringstream strstr{};
  for (auto step : message.plan.steps)
  {
    strstr << to_string(message.header);
    strstr << to_string(step.control);
    strstr << step.duration.data << "\n";
  }
  return strstr.str();
}

template <typename MsgPtr, std::enable_if_t<ml4kp_bridge::is_any_ptr<MsgPtr>::value, bool> = true>
inline std::string to_string(const MsgPtr& msg)
{
  return to_string(*msg);
}

}  // namespace utils