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
  ros::Publisher _plan_publisher, _traj_publisher;
  Service _service;
  Observation _most_recent_observation;
  bool _obs_received{ false }, _publish_trajectory{ false };
  double _preprocess_start_time, _query_fulfill_end_time;

public:
  planner_client_t(ros::NodeHandle& nh, bool publish_trajectory = false) : _publish_trajectory(publish_trajectory)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<Service>(service_name);
    _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_client_t::observation_callback, this);
    _plan_publisher = nh.advertise<ml4kp_bridge::PlanStamped>(root + "/ml4kp_plan", 1000, true);
    _traj_publisher = nh.advertise<ml4kp_bridge::Trajectory>(root + "/ml4kp_traj", 1000, true);
  }

  double get_preprocess_time() const
  {
    return _preprocess_start_time;
  }

  double get_query_fulfill_time() const
  {
    return _query_fulfill_end_time;
  }

  void observation_callback(const Observation& message)
  {
    _most_recent_observation = message;
    _obs_received = true;
  }

  void call_service(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius, double planning_duration = 1.0,
                    bool first_cycle = true)
  {
    while (!_obs_received)
    {
      ROS_WARN("Waiting for observation");
      ros::Duration(0.1).sleep();
    }
    ROS_DEBUG("Calling planner service");
    _preprocess_start_time = ros::Time::now().toSec();
    _service.request.current_observation = _most_recent_observation;
    ROS_DEBUG("Current observation: %f, %f, %f, %f, %f, %f", _service.request.current_observation.pose.position.x,
             _service.request.current_observation.pose.position.y,
             _service.request.current_observation.pose.orientation.w,
             _service.request.current_observation.pose.orientation.x,
             _service.request.current_observation.pose.orientation.y,
             _service.request.current_observation.pose.orientation.z);
    _service.request.planning_duration.data = ros::Duration(planning_duration);
    _service.request.first_cycle.data = first_cycle;
    _service.request.goal_configuration = goal_configuration;
    _service.request.return_trajectory.data = _publish_trajectory;
    if (_service_client.call(_service))
    {
      ROS_DEBUG("Service call successful");
      if (_service.response.planner_output == Service::Response::TYPE_SUCCESS)
      {
        ROS_DEBUG("Publishing plan");
        _plan_publisher.publish(_service.response.output_plan);
        if (_publish_trajectory)
        {
          _traj_publisher.publish(_service.response.output_trajectory);
        }
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
    _query_fulfill_end_time = ros::Time::now().toSec();
  }
};
}  // namespace mj_ros