#pragma once

#include <fstream>
#include <regex>

#include <ros/ros.h>

#include <prx/utilities/general/constants.hpp>
#include <prx/utilities/general/template_utils.hpp>

#define DEBUG_PRINT std::cout << __PRETTY_FUNCTION__ << ": " << __LINE__ << std::endl;

namespace dbg
{
namespace variables
{

inline static const std::string lib_path{ prx::lib_path_safe("ML4KP_ROS") };
inline static std::ofstream ofs_log;
}  // namespace variables

template <typename Value, std::enable_if_t<prx::utilities::is_streamable<Value>::value, bool> = true>
inline void print_value(std::ostream& stream, const Value& value)
{
  stream << value << " ";
}

template <typename Value, std::enable_if_t<prx::utilities::is_iterable<Value>::value and
                                               not prx::utilities::is_streamable<Value>::value,
                                           bool> = true>
inline void print_value(std::ostream& stream, const Value& value)
{
  for (auto e : value)
  {
    print_value(stream, e);
  }
  // stream << "\n";
}

inline void print_variables(std::ostream& stream, bool color, std::string name)
{
  stream << std::endl;
}

template <typename Var0, class... Vars>
inline void print_variables(std::ostream& stream, bool color, std::string name, Var0 var, Vars... vars)
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
  if (color)
  {
    stream << prx::constants::color::yellow;
    stream << var_name << ": ";
    stream << prx::constants::color::normal;
  }
  else
  {
    stream << var_name << ": ";
  }

  print_value(stream, var);
  print_variables(stream, color, other_names, vars...);
}

template <class... Vars>
inline void log_variables(std::string name, Vars... vars)
{
  using dbg::variables::ofs_log;
  if (not ofs_log.is_open())
  {
    ofs_log.open(dbg::variables::lib_path + "/log.txt");
  }
  dbg::print_variables(ofs_log, false, name, vars...);
}

}  // namespace dbg
#define DEBUG_VARS(...) dbg::print_variables(std::cout, true, #__VA_ARGS__, __VA_ARGS__);
#define LOG_VARS(...) dbg::log_variables(#__VA_ARGS__, __VA_ARGS__);
#define PRINT_MSG(MSG)                                                                                                 \
  {                                                                                                                    \
    const std::string msg{ MSG };                                                                                      \
    DEBUG_VARS(msg)                                                                                                    \
  };

#define PRINT_MSG_VARS(MSG, ...)                                                                                       \
  {                                                                                                                    \
    const std::string msg{ MSG };                                                                                      \
    dbg::print_variables(std::cout, true, "msg", msg, #__VA_ARGS__, __VA_ARGS__);                                      \
  };

#define PRINT_KEY(KEY)                                                                                                 \
  {                                                                                                                    \
    const std::string key{ SF::formatter(KEY) };                                                                       \
    dbg::print_variables(std::cout, true, #KEY, key);                                                                  \
  };
#define PRINT_KEYS(KEYS)                                                                                               \
  {                                                                                                                    \
    std::cout << prx::constants::color::yellow << #KEYS << ": " << prx::constants::color::normal;                      \
    for (auto key : KEYS)                                                                                              \
    {                                                                                                                  \
      const std::string key_str{ SF::formatter(key) };                                                                 \
      dbg::print_value(std::cout, key_str);                                                                            \
    }                                                                                                                  \
    dbg::print_variables(std::cout, true, "");                                                                         \
  };

#define PRINT_MSG_ONCE(MSG)                                                                                            \
  static bool deprecated_print_once = []() {                                                                           \
    const std::string msg{ MSG };                                                                                      \
    DEBUG_VARS(msg)                                                                                                    \
    return true;                                                                                                       \
  }();
