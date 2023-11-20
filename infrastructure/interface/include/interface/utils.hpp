#pragma once

namespace interface
{
template <typename T>
void get_param_and_check(ros::NodeHandle& nh, const std::string var_name, T& var)
{
  if (!nh.getParam(var_name, var))
  {
    ROS_FATAL_STREAM(var_name << " parameter is needed.");
    exit(-1);
  }
}

}  // namespace interface