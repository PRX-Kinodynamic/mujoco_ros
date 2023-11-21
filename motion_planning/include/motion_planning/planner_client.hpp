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
  bool _obs_received{ false };

public:
  planner_client_t(ros::NodeHandle& nh)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<Service>(service_name);
    _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_client_t::observation_callback, this);
    _plan_publisher = nh.advertise<Plan>(root + "/ml4kp_plan", 1000, true);
  }

  void observation_callback(const Observation& message)
  {
    _most_recent_observation = message;
    _obs_received = true;
  }

  void call_service(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius)
  {
    while (!_obs_received)
    {
      ros::Duration(0.1).sleep();
    }
    ROS_INFO("Calling planner service");
    _service.request.current_observation = _most_recent_observation;
    ROS_INFO("Current observation: %f, %f, %f, %f, %f, %f", _service.request.current_observation.pose.position.x,
             _service.request.current_observation.pose.position.y,
             _service.request.current_observation.pose.orientation.w,
             _service.request.current_observation.pose.orientation.x,
             _service.request.current_observation.pose.orientation.y,
             _service.request.current_observation.pose.orientation.z);
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