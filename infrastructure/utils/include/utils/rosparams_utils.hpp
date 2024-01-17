#pragma once

#define GET_VARIABLE_NAME(Variable) (#Variable)

#define ROS_PARAM_SETUP(nh, var) (interface::get_param_and_check(nh, GET_VARIABLE_NAME(var), var))

namespace utils
{

template <typename T>
void get_param_and_check(ros::NodeHandle& nh, const std::string var_name, T& var)
{
  if (!nh.getParam(var_name, var))
  {
    ROS_FATAL_STREAM("Parameter " << var_name << " is needed.");
    exit(-1);
  }
}

#define NODELET_PARAM_SETUP(nh, var)                                                                                   \
  if (!nh.getParam(GET_VARIABLE_NAME(var), var))                                                                       \
  {                                                                                                                    \
    NODELET_ERROR_STREAM("Parameter " << GET_VARIABLE_NAME(var) << " is needed.");                                     \
  }

#define NODELET_PARAM_SETUP_WITH_DEFAULT(nh, var, default_value)                                                       \
  if (!nh.getParam(GET_VARIABLE_NAME(var), var))                                                                       \
  {                                                                                                                    \
    var = default_value;                                                                                               \
  }

}  // namespace utils