#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

namespace mj_ros
{
template <typename Service, typename Observation, typename Plan>
class planner_client_t
{
private:
  ros::ServiceClient _service_client;
  ros::Subscriber _obs_subscriber;
  ros::Publisher _plan_publisher;
  Service _service;
  Observation _most_recent_observation;
  ros::Rate _rate{ 10 };

public:
  planner_client_t(ros::NodeHandle& nh)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<Service>(service_name);
    _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_client_t::observation_callback, this);
    _plan_publisher = nh.advertise<Plan>(root + "/plan", 1000);
  }

  void observation_callback(const Observation& message)
  {
    _most_recent_observation = message;
  }

  void call_service(const geometry_msgs::Pose2D& goal_configuration)
  {
    ROS_INFO("Calling planner service");
    _service.request.current_observation = _most_recent_observation;
    _service.request.planning_duration.data = ros::Duration(1.0);
    _service.request.goal_configuration = goal_configuration;
    if (_service_client.call(_service))
    {
      ROS_INFO("Service call successful");
      if (_service.response.planner_output == Service::Response::TYPE_SUCCESS)
      {
        ROS_INFO("Publishing plan");
        _plan_publisher.publish(_service.response.output_plan);
      }
      else
      {
        ROS_WARN("No plan found");
      }
    }
    else
    {
      ROS_ERROR("Service call failed");
    }
  }
};
}  // namespace mj_ros