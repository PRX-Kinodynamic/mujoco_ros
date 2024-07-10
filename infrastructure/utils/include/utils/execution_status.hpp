#pragma once
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

namespace utils
{
class execution_status_t
{
public:
  using FunctionOnReset = std::function<void()>;
  // Status of the node: either running, message receieved to stop exec
  enum Status
  {
    running,
    stop_execution,
  };

  execution_status_t(ros::NodeHandle& nh, const std::string stop_topic_name)
    : execution_status_t(nh, stop_topic_name, "", FunctionOnReset())
  {
  }

  template <typename CallableOnReset>
  execution_status_t(ros::NodeHandle& nh, const std::string stop_topic_name, const std::string reset_topic,
                     const CallableOnReset cor)
    : _status(Status::running), _stop_topic_name(stop_topic_name), _reset_function(cor)
  {
    _stop_topic = nh.subscribe(stop_topic_name, 1, &execution_status_t::callback, this);

    if (reset_topic != "")
      _reset_subscriber = nh.subscribe(reset_topic, 1, &execution_status_t::reset_callback, this);
  }

  inline Status status() const
  {
    return _status;
  }

  void status(const Status status)
  {
    _status = status;
  }

  // Convinient method to do: `while(stop_execution.continue_running()){(...)}
  inline bool ok() const
  {
    return ros::ok() && _status == utils::execution_status_t::Status::running;
  }

  void reset_callback(const std_msgs::Empty& msg)
  {
    _reset_function();
  }

private:
  void callback(const std_msgs::BoolConstPtr& msg)
  {
    if (msg->data)
    {
      _status = utils::execution_status_t::Status::stop_execution;
    }
    // res.success = true;
    // res.message = _stop_topic_name;
    // return true;
  }
  ros::Subscriber _stop_topic;
  ros::Subscriber _reset_subscriber;
  Status _status;
  const std::string _stop_topic_name;
  FunctionOnReset _reset_function;
};
}  // namespace utils