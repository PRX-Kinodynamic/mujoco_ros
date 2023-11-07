#include "ml4kp_defs.h"

#include <ros/ros.h>

namespace mj_ros
{
template <typename PlannerPtr, typename Service>
class planner_service_t
{
private:
  ros::ServiceServer _service_server;
  PlannerPtr _planner;
  Service _service;
public:
  planner_service_t(ros::NodeHandle& nh, PlannerPtr planner) : _planner(planner)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_server = nh.advertiseService(service_name, &planner_service_t::service_callback, this);
  }

  void run()
  {
    ros::spin();
  }

  bool service_callback(typename Service::Request& request, typename Service::Response& response)
  {
    _planner->plan(request, response);
    return true;
  }
};
}  // namespace mj_ros