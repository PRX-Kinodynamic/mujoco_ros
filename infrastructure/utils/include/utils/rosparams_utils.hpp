#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <ros/node_handle.h>
#include <prx/utilities/general/param_loader.hpp>
#include <utils/dbg_utils.hpp>

#define GET_VARIABLE_NAME(Variable) (#Variable)

#define ROS_PARAM_SETUP(nh, var) (utils::get_param_and_check(nh, GET_VARIABLE_NAME(var), var))
#define PARAM_SETUP(nh, var) PARAM_NAME_SETUP(nh, GET_VARIABLE_NAME(var), var)
#define NODELET_PARAM_SETUP(nh, var) PARAM_NAME_SETUP(nh, GET_VARIABLE_NAME(var), var)
#define PARAM_SETUP_WITH_DEFAULT(nh, var, default_value) NODELET_PARAM_SETUP_WITH_DEFAULT(nh, var, default_value)
#define GLOBAL_PARAM_SETUP(var) (utils::get_global_param_and_check(GET_VARIABLE_NAME(var), var))

namespace utils
{

template <typename T>
void get_param_and_check(ros::NodeHandle& nh, const std::string var_name, T& var)
{
  if (!nh.getParam(var_name, var))
  {
    const std::string ros_namespace{ nh.getNamespace() };
    std::vector<std::string> available_parameters;
    nh.getParamNames(available_parameters);
    // DEBUG_VARS(namespace);
    DEBUG_VARS(ros_namespace);
    DEBUG_VARS(available_parameters);
    ROS_FATAL_STREAM(ros_namespace << ": Parameter " << var_name << " is needed.");
    exit(-1);
  }
}

template <typename T>
void get_global_param_and_check(const std::string var_name, T& var)
{
  ros::NodeHandle nh("");
  get_param_and_check(nh, var_name, var);
}

inline void get_value(int& value, const XmlRpc::XmlRpcValue& input)
{
  if (input.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    value = input;
  }
  else if (input.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    prx_throw("Invalid input type: expected double");
  }
}

inline void get_value(double& value, const XmlRpc::XmlRpcValue& input)
{
  if (input.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    value = (int)input;
  }
  else if (input.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    prx_throw("Invalid input type: expected double");
  }
  else
  {
    value = input;
  }
}

template <typename Type>
inline void get_value(std::vector<Type>& value, const XmlRpc::XmlRpcValue& input)
{
  for (int i = 0; i < input.size(); ++i)
  {
    value.emplace_back();
    get_value(value.back(), input[i]);
  }
}

inline void get_value(gtsam::Rot3& value, const XmlRpc::XmlRpcValue& input)
{
  std::vector<double> quaternion;
  get_value(quaternion, input);
  value = gtsam::Rot3(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
}

template <int Dim>
inline void get_value(Eigen::Vector<double, Dim>& value, const XmlRpc::XmlRpcValue& input)
{
  std::vector<double> position;
  get_value(position, input);
  value = Eigen::Vector<double, Dim>(position.data());

  // value = Eigen::Ve(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
}

inline void get_value(gtsam::Pose3& value, const XmlRpc::XmlRpcValue& input)
{
  gtsam::Rot3 rot;
  Eigen::Vector3d position;
  get_value(rot, input["quaternion"]);
  get_value(position, input["position"]);
  value = gtsam::Pose3(rot, position);
}

#define PARAM_NAME_SETUP(nh, name, var)                                                                                \
  if (!nh.getParam(name, var))                                                                                         \
  {                                                                                                                    \
    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Parameter " << GET_VARIABLE_NAME(var) << " is needed.");        \
    exit(-1);                                                                                                          \
  }

#define NODELET_PARAM_SETUP_WITH_DEFAULT(nh, var, default_value)                                                       \
  if (!nh.getParam(GET_VARIABLE_NAME(var), var))                                                                       \
  {                                                                                                                    \
    var = default_value;                                                                                               \
  }

#define GLOBAL_PARAM_SETUP_DEFAULT(var, default_value)                                                                 \
  if (not(ros::param::has(GET_VARIABLE_NAME(var)) and ros::param::get(GET_VARIABLE_NAME(var), environment)))           \
  {                                                                                                                    \
    var = default_value;                                                                                               \
  }

}  // namespace utils
