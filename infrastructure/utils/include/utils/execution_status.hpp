#include <ros/ros.h>
#include <std_srvs/Trigger.h>

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

  execution_status_t(ros::NodeHandle& nh, const std::string service_name)
    : _status(Status::running), _service_name(service_name)
  {
    _service = nh.advertiseService(service_name + "/stop", &execution_status_t::callback, this);
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
  bool callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    res.success = true;
    res.message = _service_name;
    _status = utils::execution_status_t::Status::stop_execution;
    return true;
  }
  ros::ServiceServer _service;
  Status _status;
  const std::string _service_name;
};
}  // namespace utils