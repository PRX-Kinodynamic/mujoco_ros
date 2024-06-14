#pragma once

#include <prx/utilities/general/param_loader.hpp>

namespace ml4kp_bridge
{

void check_for_ros_params(prx::param_loader& param, ros::NodeHandle& nh)
{
  std::vector<std::string> keys{};
  nh.getParamNames(keys);
  // PRX_DEBUG_VAR_1(ros::this_node::getNamespace());
  const std::string current_node{ ros::this_node::getName() };
  std::string parameter;
  for (auto k : keys)
  {
    // const std::string ros_namespace{ ros::names::parentNamespace(k) };
    if (k.size() <= current_node.size())
      continue;
    const std::string param_namespace{ k.substr(0, current_node.size()) };
    const std::string parameter_name{ k.substr(current_node.size() + 1) };
    // PRX_DEBUG_VAR_1("---------------------");
    // PRX_DEBUG_VAR_2(ros_namespace, param_namespace);
    if (current_node == param_namespace)
    {
      // PRX_DEBUG_VAR_2(parameter_name, param.exists(parameter_name));
      if (param.exists(parameter_name))
      {
        // PRX_DEBUG_VAR_1(parameter_name);
        nh.getParam(parameter_name, parameter);
        // PRX_DEBUG_VAR_1(parameter);
        param[parameter_name] = YAML::Load(parameter);
        // PRX_DEBUG_VAR_2(parameter_name, param[parameter_name].as<std::string>());
      }
      // inline bool exists(const std::string& key) const
    }
  }
}
}  // namespace ml4kp_bridge