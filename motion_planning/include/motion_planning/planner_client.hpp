#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

namespace mj_ros
{
template <typename PlannerService, typename Observation>
class planner_client_t
{
private:
  ros::ServiceClient _service_client;
  ros::Subscriber _obs_subscriber;
  ros::Publisher _plan_publisher, _traj_publisher, _feedback_traj_publisher;
  PlannerService _service;
  Observation _most_recent_observation;
  bool _obs_received{ false };
  double _preprocess_start_time, _query_fulfill_end_time;
  int _control_dim;

  bool _use_contingency;
  int _contingency_steps { 0 };

  std::vector<ml4kp_bridge::SpacePoint> planning_cycle_start_states, planning_cycle_end_states;
  std::vector<Observation> execution_cycle_start_observations;

  ml4kp_bridge::TrajectoryStamped _feedback_traj;

public:
  planner_client_t(ros::NodeHandle& nh, int control_dim) : _control_dim(control_dim)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<PlannerService>(service_name);
    _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_client_t::observation_callback, this);
    _plan_publisher = nh.advertise<ml4kp_bridge::PlanStamped>(root + "/ml4kp_plan", 1000, true);
    _traj_publisher = nh.advertise<ml4kp_bridge::TrajectoryStamped>(root + "/ml4kp_traj", 1000, true);
    _feedback_traj_publisher = nh.advertise<ml4kp_bridge::TrajectoryStamped>(root + "/feedback_traj", 1, true);

    double planning_cycle_duration;
    nh.getParam(ros::this_node::getName() + "/use_contingency", _use_contingency);
    nh.getParam(ros::this_node::getName() + "/planning_cycle_duration", planning_cycle_duration);
    _contingency_steps = 1 + planning_cycle_duration / prx::simulation_step;
    if (_use_contingency)
    {
      ROS_INFO("Using contingency with %d steps", _contingency_steps);
    }
  }

  void publish_feedback_traj(const ml4kp_bridge::TrajectoryStamped& feedback_traj)
  {
    _feedback_traj.trajectory.data.clear();
    if (_use_contingency)
    {
      int size = feedback_traj.trajectory.data.size();
      unsigned end_index = std::min(_contingency_steps, size);
      for (unsigned i = 0; i < end_index; ++i)
      {
        _feedback_traj.trajectory.data.push_back(feedback_traj.trajectory.data[i]);
      }
      _feedback_traj_publisher.publish(_feedback_traj);
    }
    else
    {
      _feedback_traj_publisher.publish(feedback_traj);
    }
  }

  std::tuple<std::vector<ml4kp_bridge::SpacePoint>, std::vector<ml4kp_bridge::SpacePoint>, std::vector<Observation>>
  get_error_data()
  {
    return std::make_tuple(planning_cycle_start_states, planning_cycle_end_states, execution_cycle_start_observations);
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

  // TODO: This is probably not the best place to have this (should be inside mj_mushr.hpp)
  bool is_goal_reached(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius)
  {
    if (!_obs_received)
    {
      return false;
    }
    return std::hypot(goal_configuration.x - _most_recent_observation.pose.position.x,
                      goal_configuration.y - _most_recent_observation.pose.position.y) < goal_radius.data;
  }

  void call_service(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius,
                    double planning_duration = 1.0)
  {
    while (!_obs_received)
    {
      ROS_WARN("Service waiting for observation");
      ros::Duration(0.1).sleep();
    }
    _preprocess_start_time = ros::Time::now().toSec();
    _service.request.current_observation = _most_recent_observation;
    _service.request.planning_duration.data = ros::Duration(planning_duration);
    _service.request.goal_configuration = goal_configuration;
    if (_service_client.call(_service))
    {
      ROS_DEBUG("Service call successful");
      ROS_DEBUG_STREAM("Current obs: " << _most_recent_observation.pose.position.x << ", "
                                       << _most_recent_observation.pose.position.y);
      if (is_goal_reached(goal_configuration, goal_radius))
      {
        ROS_INFO("Goal reached. Not publishing plan");
        _service.response.output_plan.plan.steps.clear();
        ml4kp_bridge::add_zero_plan(_service.response.output_plan, planning_duration, _control_dim);
        _plan_publisher.publish(_service.response.output_plan);
      }
      else if (_service.response.planner_output == PlannerService::Response::TYPE_SUCCESS)
      {
        ROS_DEBUG("Publishing trajectory");
        publish_feedback_traj(_service.response.output_trajectory);
        _traj_publisher.publish(_service.response.output_trajectory);
        planning_cycle_start_states.push_back(_service.response.output_trajectory.trajectory.data[0]);
        planning_cycle_end_states.push_back(
            _service.response.output_trajectory.trajectory.data[planning_duration * prx::simulation_step + 1]);
        execution_cycle_start_observations.push_back(_most_recent_observation);
        _plan_publisher.publish(_service.response.output_plan);
      }
      else
      {
        _feedback_traj_publisher.publish(_service.response.output_trajectory);
        ROS_INFO_STREAM(
            "Planner failure, output traj size: " << _service.response.output_trajectory.trajectory.data.size());
        _plan_publisher.publish(_service.response.output_plan);
        _traj_publisher.publish(_service.response.output_trajectory);
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