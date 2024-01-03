#pragma once

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <strstream>
#include <iomanip>
#include <regex>

#include "GLFW/glfw3.h"
#include "ros/ros.h"
#include "mujoco/mujoco.h"
#include "std_msgs/Empty.h"
#include "mujoco_ros/SensorDataStamped.h"

#define PRX_DEBUG_PRINT std::cout << __PRETTY_FUNCTION__ << ": " << __LINE__ << std::endl;

template <typename T>
void get_param_and_check(ros::NodeHandle& nh, const std::string var_name, T& var)
{
  if (!nh.getParam(var_name, var))
  {
    ROS_FATAL_STREAM(var_name << " parameter is needed.");
    exit(-1);
  }
}

namespace dbg
{

inline void print_variables(std::string name)
{
  std::cout << std::endl;
}

template <typename Var0, class... Vars>
inline void print_variables(std::string name, Var0& var, Vars&... vars)
{
  const std::regex regex(",(\\s*)+");
  std::string var_name{ name };
  std::string other_names{ "" };
  std::smatch match;  // <-- need a match object
  // std::cout << "name: " << name << std::endl;
  if (std::regex_search(name, match, regex))  // <-- use it here to get the match
  {
    const int split_on = match.position();  // <-- use the match position
    var_name = name.substr(0, split_on);
    other_names = name.substr(split_on + match.length());  // <-- also, skip the whole math
  }

  std::cout << var_name << ": ";
  std::cout << var << " ";
  print_variables(other_names, vars...);
}

}  // namespace dbg
#define PRX_DEBUG_VARS(...) dbg::print_variables(#__VA_ARGS__, __VA_ARGS__);
