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
    stop_execution,
  };

  execution_status_t(ros::NodeHandle& nh, const std::string topic_name)
    : _status(Status::running), _topic_name(topic_name)
  {
    _subscriber = nh.subscribe(topic_name + "/stop", 1, &execution_status_t::callback, this);
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

private:
  void callback(const std_msgs::Empty& msg)
  {
    // res.success = true;
    // res.message = _service_name;
    _status = utils::execution_status_t::Status::stop_execution;
    // return true;
  }
  ros::Subscriber _subscriber;
  Status _status;
  const std::string _topic_name;
};
}  // namespace utils