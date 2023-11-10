#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

namespace mj_ros
{
template <typename Service, typename Observation>
class planner_client_t
{
private:
  ros::ServiceClient _service_client;
  ros::Subscriber _obs_subscriber;
  Service _service;
  Observation _most_recent_observation;

public:
  planner_client_t(ros::NodeHandle& nh) 
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<Service>(service_name);
    _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_client_t::observation_callback, this);
  }

  void observation_callback(const Observation& message)
  {
    _most_recent_observation = message;
  }

  void call_service(const geometry_msgs::Pose& goal_configuration)
  {
    _service.request.observation = _most_recent_observation;
    _service.request.planning_duration.data = 1.0;
    _service.request.goal_configuration = goal_configuration;
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