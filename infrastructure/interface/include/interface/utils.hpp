#pragma once

#define GET_VARIABLE_NAME(Variable) (#Variable)

#define ROS_PARAM_SETUP(nh, var) (interface::get_param_and_check(nh, GET_VARIABLE_NAME(var), var))

namespace interface
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

// (interface::nodelet_get_param_and_check(nh, GET_VARIABLE_NAME(var), var))
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

template <typename T>
void print_container(const std::string& name, const T& container)
{
  std::cout << name << ": ";
  for (auto e : container)
  {
    std::cout << e << ", ";
  }
  std::cout << std::endl;
}
}  // namespace interface