#include <ros/ros.h>
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

  execution_status_t(ros::NodeHandle& nh, const std::string service_name)
    : execution_status_t(nh, service_name, "", FunctionOnReset())
  {
  }

  template <typename CallableOnReset>
  execution_status_t(ros::NodeHandle& nh, const std::string service_name, const std::string reset_topic,
                     const CallableOnReset cor)
    : _status(Status::running), _service_name(service_name), _reset_function(cor)
  {
    _service = nh.advertiseService(service_name + "/stop", &execution_status_t::callback, this);

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
  bool callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    res.success = true;
    res.message = _service_name;
    _status = utils::execution_status_t::Status::stop_execution;
    return true;
  }
  ros::ServiceServer _service;
  ros::Subscriber _reset_subscriber;
  Status _status;
  const std::string _service_name;
  FunctionOnReset _reset_function;
};
}  // namespace utils