#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

namespace mj_ros
{
template <typename PlannerQueryPtr, typename Service>
class planner_client_t
{
private:
  ros::ServiceClient _service_client;
  PlannerQueryPtr _planner_query;
  Service _service;

public:
  planner_client_t(ros::NodeHandle& nh, PlannerQueryPtr planner_query) : _planner_query(planner_query)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<Service>(service_name);
  }

  void call_service()
  {
    // Copy from interface/simulator to request
    if (_service_client.call(_service))
    {
      ROS_INFO("Service call successful");
    }
    else
    {
      ROS_ERROR("Service call failed");
    }
  }
};
}  // namespace mj_ros