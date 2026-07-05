#pragma once

#include <fstream>
#include <ostream>
#include <regex>

#include <ros/ros.h>

#include <prx/utilities/general/constants.hpp>
#include <prx/utilities/general/template_utils.hpp>
#include <prx/utilities/spaces/streamer.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <ml4kp_bridge/template_utils.hpp>
// #include <ml4kp_bridge/product_lie_group.hpp>
#include <gtsam/base/types.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <ml4kp_bridge/eigen_utils.hpp>
#include <ml4kp_bridge/gtsam_traits.hpp>
// #include <ml4kp_bridge/streamer_bridge.hpp>

#define DEBUG_PRINT std::cout << __PRETTY_FUNCTION__ << ": " << __LINE__ << std::endl;

namespace dbg
{
namespace variables
{

inline static const std::string lib_path{ prx::lib_path_safe("ML4KP_ROS") };
inline static std::ofstream ofs_log;
inline static std::string log_filename = "log.txt";
}  // namespace variables

inline void set_log_filename(const std::string filename)
{
  variables::log_filename = filename;
}
inline void close_log()
{
  if (variables::ofs_log.is_open())
  {
    variables::ofs_log.flush();
    variables::ofs_log.close();
  }
}

// template <typename Value,                                   // no-lint
// std::enable_if_t<                                 // no-lint
// prx::utilities::is_streamable<Value>::value,  // no-lint
// bool> = true>
template <typename Value>  // no-lint
inline void print_value(std::ostream& stream, const Value& value)
{
  prx::streamer_t<Value>::to_stream(stream, value);
}

// template <typename Value,                                                 // no-lint
//           std::enable_if_t<                                               // no-lint
//               ml4kp_bridge::is_eigen_vector<Value>::value, bool> = true>  // no-lint
// inline void print_value(std::ostream& stream, const Value& value)
// {
//   stream << value.transpose() << " ";
// }

template <typename Value, std::enable_if_t<std::is_same<Value, gtsam::NonlinearFactorGraph>::value, bool> = true>
inline void print_value(std::ostream& stream, const Value& graph)
{
  stream << "FactorGraph (size: " << graph.size() << ")\n";
  for (size_t i = 0; i < graph.size(); i++)
  {
    // std::stringstream ss;
    if (graph.at(i))
    {
      stream << "\t--Factor " << i << ": ";
      const gtsam::KeyVector keys{ graph.at(i)->keys() };
      for (auto k : keys)
      {
        stream << prx::fg::symbol_factory_t::formatter(k) << " ";
      }
      stream << "\n";
      // graph.at(i)->print(ss.str(), prx::fg::symbol_factory_t::formatter);
    }
  }
}

// std::enable_if_t<std::is_same<Value, gtsam::ProductLieGroupV43<First, Second>>::value, bool> = true>
// template <typename First, typename Second>
// inline void print_value(std::ostream& stream, const gtsam::ProductLieGroupV43<First, Second>& value)
// {
//   // stream << value.first << " " << value.second << " ";
// }

// inline void print_value(std::ostream& stream, const gtsam::Rot2& value)
// {
//   stream << value.theta() << " ";
// }
// inline void print_value(std::ostream& stream, const gtsam::Pose2& value)
// {
//   stream << value.x() << " ";
//   stream << value.y() << " ";
//   stream << value.theta() << " ";
// }

// template <int Dim>
// inline void print_value(std::ostream& stream, const Eigen::Vector<double, Dim>& value)
// {
//   stream << value.transpose() << " ";
// }

template <typename Value, std::enable_if_t<std::is_same<Value, gtsam::Values>::value, bool> = true>
inline void print_value(std::ostream& stream, const Value& values)
{
  stream << "FG Values (size: " << values.size() << ")\n";

  for (const auto& key_value : values)
  {
    stream << "\t--Value " << prx::fg::symbol_factory_t::formatter(key_value.key) << ": ";
    key_value.value.print("");
    stream << "\n";
  }
}

template <typename PairValue, std::enable_if_t<ml4kp_bridge::is_pair<PairValue>::value, bool> = true>
inline void print_value(std::ostream& stream, const PairValue& pair)
{
  print_value(stream, "First: ");
  print_value(stream, pair.first);
  print_value(stream, " Second: ");
  print_value(stream, pair.second);
}

// template <typename Value, std::enable_if_t<prx::utilities::is_iterable<Value>::value and
//                                                not prx::utilities::is_streamable<Value>::value and
//                                                not std::is_same<Value, gtsam::NonlinearFactorGraph>::value and
//                                                not std::is_same<Value, gtsam::Values>::value,
//                                            bool> = true>
// inline void print_value(std::ostream& stream, const Value& value)
// {
//   for (auto& e : value)
//   {
//     print_value(stream, e);
//     // print_value(stream, "\n");
//   }
//   // stream << "\n";
// }

template <std::size_t I, typename TupleValue,
          std::enable_if_t<(I == std::tuple_size<TupleValue>{}), bool> = true>  // no-lint
inline void print_tuple(std::ostream& stream, const TupleValue& tuple)
{
}

template <std::size_t I, typename TupleValue,
          std::enable_if_t<(I < std::tuple_size<TupleValue>{}), bool> = true>  // no-lint
inline void print_tuple(std::ostream& stream, const TupleValue& tuple)
{
  print_value(stream, I);
  print_value(stream, ": ");
  print_value(stream, std::get<I>(tuple));
  print_tuple<I + 1>(stream, tuple);
}

template <typename TupleValue, std::enable_if_t<ml4kp_bridge::is_tuple<TupleValue>::value, bool> = true>
inline void print_value(std::ostream& stream, const TupleValue& tuple)
{
  // for (auto e : value)
  // for (int i = 0; i < std::tuple_size<TupleValue>{}; ++i)
  // {
  print_tuple<0>(stream, tuple);
  // }
  // stream << "\n";
}

inline void print_all_variables(std::ostream& stream, bool color, std::string name)
{
  stream << std::endl;
}

template <typename Var0, class... Vars>
inline void print_all_variables(std::ostream& stream, bool color, std::string name, Var0 var, Vars... vars)
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
  print_all_variables(stream, color, other_names, vars...);
}

template <class... Vars>
inline void log_variables(std::ofstream& ofs, const bool time_log, const std::string fn_name, const std::string name,
                          Vars... vars)
{
  // using dbg::variables::ofs_log;
  if (not ofs.is_open())
  {
    const std::string log_filename{ dbg::variables::lib_path + dbg::variables::log_filename };
    ofs.open(log_filename);

    prx_assert(ofs.is_open(), "[log_variables] couldn't open log file: " << log_filename);

    const std::string msg{ "Log set to: " + log_filename };
    dbg::print_all_variables(std::cout, true, "msg", msg);
  }
  std::streambuf* coutbuf = std::cout.rdbuf();  // save old buf
  std::cout.rdbuf(ofs.rdbuf());                 // redirect std::cout to out.txt!

  if (time_log)
    ofs << "[ " << fn_name << " " << ros::Time::now() << " ] ";
  dbg::print_all_variables(ofs, false, name, vars...);
  std::cout.rdbuf(coutbuf);
}

template <typename Key, std::enable_if_t<std::is_same<Key, gtsam::Key>::value, bool> = true>
void print_key(std::ostream& stream, const Key& key)
{
  stream << prx::fg::symbol_factory_t::formatter(key) << " ";
}

template <std::size_t I, typename TupleValue,
          std::enable_if_t<(I == std::tuple_size<TupleValue>{}), bool> = true>  // no-lint
inline void print_key_tuple(std::ostream& stream, const TupleValue& tuple)
{
}

template <std::size_t I, typename TupleValue,
          std::enable_if_t<(I < std::tuple_size<TupleValue>{}), bool> = true>  // no-lint
inline void print_key_tuple(std::ostream& stream, const TupleValue& key_tuple)
{
  print_key(stream, std::get<I>(key_tuple));
  print_key_tuple<I + 1>(stream, key_tuple);
}

template <typename Keys, std::enable_if_t<prx::utilities::is_iterable<Keys>::value, bool> = true>
inline void print_key(std::ostream& stream, const Keys& keys)
{
  for (auto& k : keys)
  {
    print_key(stream, k);
  }
}

template <typename TupleValue, std::enable_if_t<ml4kp_bridge::is_tuple<TupleValue>::value, bool> = true>
inline void print_key(std::ostream& stream, const TupleValue& tuple)
{
  print_key_tuple<0>(stream, tuple);
}

template <class... Keys>
void print_keys(const std::string fn_name, std::ostream& stream, Keys... vars)
{
  stream << "[ " << fn_name << " " << ros::Time::now() << " ] ";
  stream << "Keys: ";
  print_key(stream, vars...);
  stream << "\n";
}

}  // namespace dbg
#define LOG_CLOSE dbg::close_log();
#define LOG_FILENAME(FILENAME) ::dbg::set_log_filename(FILENAME);

#define DEBUG_VARS(...) ::dbg::print_all_variables(std::cout, true, #__VA_ARGS__, __VA_ARGS__);
#define LOG_VARS(...) ::dbg::log_variables(dbg::variables::ofs_log, true, __FUNCTION__, #__VA_ARGS__, __VA_ARGS__);
#define LOG_FILE_VARS(OFS, ...) dbg::log_variables(OFS, false, "", #__VA_ARGS__, __VA_ARGS__);
#define ERROR_VARS(...)                                                                                                \
  {                                                                                                                    \
    std::cout << prx::constants::color::red;                                                                           \
    dbg::print_all_variables(std::cout, false, #__VA_ARGS__, __VA_ARGS__);                                             \
    std::cout << prx::constants::color::normal;                                                                        \
  };
#define PRINT_ERROR(MSG)                                                                                               \
  {                                                                                                                    \
    std::cout << prx::constants::color::red;                                                                           \
    const std::string msg{ MSG };                                                                                      \
    std::cout << msg;                                                                                                  \
    std::cout << prx::constants::color::normal;                                                                        \
  };

#define LOG_MSG(MSG)                                                                                                   \
  {                                                                                                                    \
    const std::string msg{ MSG };                                                                                      \
    LOG_VARS(msg)                                                                                                      \
  };

#define LOG_LINE()                                                                                                     \
  {                                                                                                                    \
    const auto line = __LINE__;                                                                                        \
    LOG_VARS(line)                                                                                                     \
  };

#define PRINT_MSG(MSG)                                                                                                 \
  {                                                                                                                    \
    const std::string msg{ MSG };                                                                                      \
    DEBUG_VARS(msg)                                                                                                    \
  };

#define PRINT_MSG_VARS(MSG, ...)                                                                                       \
  {                                                                                                                    \
    const std::string msg{ MSG };                                                                                      \
    std::string all_names = "msg, " + std::string(#__VA_ARGS__);                                                       \
    dbg::print_all_variables(std::cout, true, all_names, msg, __VA_ARGS__);                                            \
  };

#define PRINT_KEY(KEY)                                                                                                 \
  {                                                                                                                    \
    const std::string _key{ SF::formatter(KEY) };                                                                      \
    dbg::print_all_variables(std::cout, true, #KEY, _key);                                                             \
  };

#define PRINT_KEY_ERROR(KEY)                                                                                           \
  {                                                                                                                    \
    std::cout << prx::constants::color::red;                                                                           \
    const std::string _key{ SF::formatter(KEY) };                                                                      \
    dbg::print_all_variables(std::cout, false, #KEY, _key);                                                            \
    std::cout << prx::constants::color::normal;                                                                        \
  };
#define PRINT_KEYS_CONTAINER(KEYS)                                                                                     \
  {                                                                                                                    \
    std::cout << prx::constants::color::yellow << #KEYS << ": " << prx::constants::color::normal;                      \
    for (auto key : KEYS)                                                                                              \
    {                                                                                                                  \
      const std::string key_str{ SF::formatter(key) };                                                                 \
      dbg::print_value(std::cout, key_str);                                                                            \
    }                                                                                                                  \
    dbg::print_all_variables(std::cout, true, "");                                                                     \
  };
#define LOG_KEY(KEY)                                                                                                   \
  {                                                                                                                    \
    const std::string _key{ SF::formatter(KEY) };                                                                      \
    dbg::log_variables(dbg::variables::ofs_log, true, __FUNCTION__, #KEY, _key);                                       \
  };
#define LOG_KEYS(...) dbg::print_keys(__FUNCTION__, dbg::variables::ofs_log, __VA_ARGS__);
// void print_keys(const std::string fn_name, std::ostream& stream, Keys... vars)

#define PRINT_KEYS_(KEYS) PRINT_KEYS_CONTAINER(KEYS)

#define PRINT_MSG_ONCE(MSG)                                                                                            \
  static bool deprecated_print_once = []() {                                                                           \
    const std::string msg{ MSG };                                                                                      \
    DEBUG_VARS(msg)                                                                                                    \
    return true;                                                                                                       \
  }();

#define VARS_TO_STREAM(STREAM, ...) ::dbg::print_all_variables(STREAM, false, #__VA_ARGS__, __VA_ARGS__);