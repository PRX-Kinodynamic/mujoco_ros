#pragma once
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>

namespace utils
{
class execution_status_t
{
public:
  // Status of the node: either running, message receieved to stop exec
  enum Status
  {
    running,
    reset,
    stop_execution,
  };

  execution_status_t(ros::NodeHandle& nh, const std::string topic_name)
    : _status(Status::running), _topic_name(topic_name)
  {
    _subscriber = nh.subscribe<std_msgs::Empty>(
        topic_name + "/stop", 1,
        boost::bind(&execution_status_t::callback, this, _1, utils::execution_status_t::Status::stop_execution));
    _subscriber_reset = nh.subscribe<std_msgs::Empty>(
        topic_name + "/reset", 1,
        boost::bind(&execution_status_t::callback, this, _1, utils::execution_status_t::Status::reset));
  }

  inline Status status() const
  {
    return _status;
  }

  void status(const Status status)
  {
    _status = status;
  }

  // Return true if a reset signal has been received. The status is changed to running
  inline bool reset_received()
  {
    const bool status{ _status == utils::execution_status_t::Status::reset };
    _status = utils::execution_status_t::Status::running;
    return status;
  }
  // Convinient method to do: `while(stop_execution.continue_running()){(...)}
  inline bool ok() const
  {
    return ros::ok() && _status != utils::execution_status_t::Status::stop_execution;
  }

private:
  void callback(const std_msgs::EmptyConstPtr& msg, const utils::execution_status_t::Status status)
  {
    _status = status;
  }
  ros::Subscriber _subscriber, _subscriber_reset;
  Status _status;
  const std::string _topic_name;
};
}  // namespace utils