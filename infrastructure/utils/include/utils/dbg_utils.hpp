#pragma once

#include <ros/ros.h>
#include <regex>
#define DEBUG_PRINT std::cout << __PRETTY_FUNCTION__ << ": " << __LINE__ << std::endl;

namespace dbg
{

inline void print_variables(std::string name)
{
  std::cout << std::endl;
}

template <typename Var0, class... Vars>
inline void print_variables(std::string name, Var0 var, Vars... vars)
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
#define DEBUG_VARS(...) dbg::print_variables(#__VA_ARGS__, __VA_ARGS__);
