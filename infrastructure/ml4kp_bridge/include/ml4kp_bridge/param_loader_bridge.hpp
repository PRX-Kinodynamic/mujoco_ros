#pragma once

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>

namespace ml4kp_bridge
{
// template <std::size_t I, typename... Tp, std::enable_if_t<(I == sizeof...(Tp) - 1), bool> = true>

// template<typename ...Args>
//, std::enable_if_t<(0 < sizeof...(OtherTypes)), bool> = true>  // no-lint
// template <typename Type>
template <typename Type, typename... OtherTypes, std::enable_if_t<(0 == sizeof...(OtherTypes)), bool> = true>
inline bool param_loder_set(prx::param_loader& pl, const std::string& key, const ros::NodeHandle& nh)
{
  Type type;
  if (nh.getParam(key, type))
  {
    pl[key].set(type);
    return true;
  }
  return false;
}
// template <std::size_t I, typename... Tp, std::enable_if_t<(I < sizeof...(Tp) - 1), bool> = true>
template <typename Type, typename... OtherTypes, std::enable_if_t<(0 < sizeof...(OtherTypes)), bool> = true>
inline bool param_loder_set(prx::param_loader& pl, const std::string& key, const ros::NodeHandle& nh)
{
  Type type;
  if (nh.getParam(key, type))
  {
    pl[key].set(type);
    return true;
  }

  return param_loder_set<OtherTypes...>(pl, key, nh);
  // return false;
}

inline void copy(prx::param_loader& pl, const ros::NodeHandle& nh)
{
  std::vector<std::string> keys;
  nh.getParamNames(keys);

  prx::param_loader pl_all;
  for (auto& k : keys)
  {
    bool found{ false };
    // Short circuit operation to make compilation faster
    found = found or param_loder_set<double, std::vector<double>>(pl_all, k, nh);            // no-lint
    found = found or param_loder_set<float, std::vector<float>>(pl_all, k, nh);              // no-lint
    found = found or param_loder_set<int, std::vector<int>>(pl_all, k, nh);                  // no-lint
    found = found or param_loder_set<bool>(pl_all, k, nh);                                   // no-lint
    found = found or param_loder_set<std::string, std::vector<std::string>>(pl_all, k, nh);  // no-lint

    if (not found)
    {
      prx_warn("[ml4kp_bridge::copy(param_loader, node_handle)] Key " << k << " not found ")
    }
    // found =     ;
    // param_loder_set<bool, double, float, int, std::string>(pl, k, nh);
  }
  pl.merge(pl_all[nh.getNamespace()]);
}
// inline void check_for_ros_params(prx::param_loader& param, ros::NodeHandle& nh)
// {
//   std::vector<std::string> keys{};
//   nh.getParamNames(keys);
//   // PRX_DEBUG_VAR_1(ros::this_node::getNamespace());
//   const std::string current_node{ ros::this_node::getName() };
//   std::string parameter;
//   for (auto k : keys)
//   {
//     // const std::string ros_namespace{ ros::names::parentNamespace(k) };
//     if (k.size() <= current_node.size())
//       continue;
//     const std::string param_namespace{ k.substr(0, current_node.size()) };
//     const std::string parameter_name{ k.substr(current_node.size() + 1) };
//     // PRX_DEBUG_VAR_1("---------------------");
//     // PRX_DEBUG_VAR_2(ros_namespace, param_namespace);
//     if (current_node == param_namespace)
//     {
//       // PRX_DEBUG_VAR_2(parameter_name, param.exists(parameter_name));
//       if (param.exists(parameter_name))
//       {
//         // PRX_DEBUG_VAR_1(parameter_name);
//         nh.getParam(parameter_name, parameter);
//         // PRX_DEBUG_VAR_1(parameter);
//         param[parameter_name] = YAML::Load(parameter);
//         // PRX_DEBUG_VAR_2(parameter_name, param[parameter_name].as<std::string>());
//       }
//       // inline bool exists(const std::string& key) const
//     }
//   }
// }
}  // namespace ml4kp_bridge
