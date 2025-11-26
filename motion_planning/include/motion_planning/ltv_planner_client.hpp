#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <utils/dbg_utils.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <ml4kp_bridge/Plan.h>
#include <utils/std_utils.hpp>
#include <std_msgs/Bool.h>

namespace mj_ros
{
template <typename PlannerService, typename Observation>
class ltv_planner_client_t
{
private:
  ros::ServiceClient _service_client;
  ros::Publisher _traj_publisher;
  PlannerService _service;
  Observation _most_recent_observation;

  ros::Time _experiment_start_time;
  double _preprocess_start_time, _query_fulfill_end_time;
  int _control_dim, _cycle_id;
  double _planning_duration;

  std::vector<ml4kp_bridge::SpacePoint> planning_cycle_start_states, planning_cycle_end_states;
  std::vector<Observation> execution_cycle_start_observations;

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_tf_header;
  bool _obs_received{ false };
  bool _collision_detected{ false };
  int _step_idx{ 0 };
  int _controller_cycle_idx{ -1 };
  ros::Time _step_end_time;
  ros::Timer _observation_timer;
  ros::Timer _control_timer;
  ros::Publisher _stamped_control_publisher;
  ros::Subscriber _collision_subscriber;
  ml4kp_bridge::SpacePointStamped _control_stamped;

  std::map<int, ml4kp_bridge::Plan> _plans;

public:
  ltv_planner_client_t(ros::NodeHandle& nh, int control_dim)
    : _control_dim(control_dim)
    , _tf_listener(_tf_buffer)
    , _most_recent_observation()
    , _controller_cycle_idx(-1)
    , _step_idx(0)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_client = nh.serviceClient<PlannerService>(service_name);

    std::string stamped_control_topic, trajectory_topic, collision_topic;
    double observation_frequency, control_frequency;
    nh.getParam(ros::this_node::getName() + "/planning_cycle_duration", _planning_duration);
    nh.getParam(ros::this_node::getName() + "/world_frame", _world_frame);
    nh.getParam(ros::this_node::getName() + "/robot_frame", _robot_frame);
    nh.getParam(ros::this_node::getName() + "/observation_frequency", observation_frequency);
    nh.getParam(ros::this_node::getName() + "/control_frequency", control_frequency);
    nh.getParam(ros::this_node::getName() + "/stamped_control_topic", stamped_control_topic);
    nh.getParam(ros::this_node::getName() + "/trajectory_topic", trajectory_topic);
    nh.getParam(ros::this_node::getName() + "/collision_topic", collision_topic);
    const ros::Duration observation_timer(1.0 / observation_frequency);
    _observation_timer = nh.createTimer(observation_timer, &ltv_planner_client_t::observation_callback, this);
    const ros::Duration control_timer(1.0 / control_frequency);
    _control_timer = nh.createTimer(control_timer, &ltv_planner_client_t::control_callback, this);
    _stamped_control_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);
    _traj_publisher = nh.advertise<ml4kp_bridge::Trajectory>(trajectory_topic, 100, true);
    ROS_INFO_STREAM("Trajectory topic: " << trajectory_topic);
    _collision_subscriber = nh.subscribe(collision_topic, 100, &ltv_planner_client_t::collision_callback, this);
    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "LTV_REPLANNER";
    _collision_detected = false;
  }

  bool query_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  bool is_collision_detected() const
  {
    return _collision_detected;
  }

  void collision_callback(const std_msgs::Bool& collision_msg)
  {
    if (collision_msg.data)
    {
      ROS_WARN("Collision detected");
      _collision_detected = true;
    }
  }
  void observation_callback(const ros::TimerEvent& event)
  {
    _obs_received = true;
    if (query_tf() && _tf.header.stamp > _prev_tf_header.stamp)
    {
      ROS_DEBUG_STREAM("Observation callback");
      _prev_tf_header = _tf.header;
      _most_recent_observation.pose.position.x = _tf.transform.translation.x;
      _most_recent_observation.pose.position.y = _tf.transform.translation.y;
    }
  }

  bool is_goal_reached(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius)
  {
    if (!_obs_received)
    {
      return false;
    }
    return std::hypot(goal_configuration.x - _most_recent_observation.pose.position.x,
                      goal_configuration.y - _most_recent_observation.pose.position.y) < goal_radius.data;
  }

  double get_preprocess_time() const
  {
    return _preprocess_start_time;
  }

  double get_query_fulfill_time() const
  {
    return _query_fulfill_end_time;
  }

  ros::Time get_experiment_start_time() const
  {
    return _experiment_start_time;
  }

  void update_controls(const ml4kp_bridge::PlanStamped& plan, const uint32_t& cycle_idx)
  {
    _plans[cycle_idx + 1] = plan.plan;
  }

  void set_zero_controls(const uint32_t& cycle_idx)
  {
    ml4kp_bridge::PlanStamped plan;
    ml4kp_bridge::add_zero_plan(plan, _planning_duration, _control_dim);
    update_controls(plan, cycle_idx);
  }

  void control_callback(const ros::TimerEvent& event)
  {
    if (_controller_cycle_idx == -1)
    {
      return;
    }

    int current_cycle = floor((event.current_real - _experiment_start_time).toSec() / _planning_duration) + 1;

    if (current_cycle > 0 && current_cycle != _controller_cycle_idx)
    {
      _controller_cycle_idx = current_cycle;
      _step_idx = -1;
    }

    if (_plans.find(_controller_cycle_idx) == _plans.end())
    {
      ROS_WARN("No plan found for cycle index: %d", _controller_cycle_idx);
      return;
    }

    if (_step_idx == -1)
    {
      _step_idx = 0;
      _step_end_time = _experiment_start_time + ros::Duration(_planning_duration * (_controller_cycle_idx - 1)) +
                       _plans[_controller_cycle_idx].steps[_step_idx].duration.data;
      _control_stamped.space_point = _plans[_controller_cycle_idx].steps[_step_idx].control;
    }
    else if (event.current_real >= _step_end_time)
    {
      _step_idx++;
      if (_step_idx < _plans[_controller_cycle_idx].steps.size())
      {
        _control_stamped.space_point = _plans[_controller_cycle_idx].steps[_step_idx].control;
        _step_end_time += _plans[_controller_cycle_idx].steps[_step_idx].duration.data;
      }
    }

    if (_step_idx < _plans[_controller_cycle_idx].steps.size())
    {
      _control_stamped.header.seq++;
      _control_stamped.header.stamp = ros::Time::now();
      _stamped_control_publisher.publish(_control_stamped);
    }
    else
    {
      ROS_WARN("Publishing zero control");
      ml4kp_bridge::SpacePointStamped zero_control;
      zero_control.header.seq = 0;
      zero_control.header.stamp = ros::Time::now();
      zero_control.header.frame_id = "LTV_REPLANNER";

      zero_control.space_point.point.resize(_control_dim, 0.0);

      _stamped_control_publisher.publish(zero_control);
    }
  }

  void call_service(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius,
                    const uint32_t& planning_controller_cycle_idx, double planning_duration = 1.0)
  {
    while (!_obs_received)
    {
      ROS_WARN("Service waiting for observation");
      ros::Duration(0.1).sleep();
    }
    _preprocess_start_time = ros::Time::now().toSec();

    // _service.request.current_observation = _most_recent_observation; // TODO: adapt changes?
    _service.request.planning_duration.data = ros::Duration(planning_duration);
    _service.request.goal_configuration = goal_configuration;
    if (_service_client.call(_service))
    {
      if (planning_controller_cycle_idx == 0)
      {
        _experiment_start_time = ros::Time::now() + ros::Duration(planning_duration);
        _controller_cycle_idx = 0;
        _step_idx = -1;
      }

      if (is_goal_reached(goal_configuration, goal_radius))
      {
        ROS_INFO("Goal reached. Not publishing plan");
        _service.response.output_plan.plan.steps.clear();
        ml4kp_bridge::add_zero_plan(_service.response.output_plan, planning_duration, _control_dim);
        update_controls(_service.response.output_plan, planning_controller_cycle_idx);
      }
      else if (_service.response.planner_output == PlannerService::Response::TYPE_SUCCESS)
      {
        _traj_publisher.publish(_service.response.output_trajectory.trajectory);
        planning_cycle_start_states.push_back(_service.response.output_trajectory.trajectory.data[0]);
        planning_cycle_end_states.push_back(
            _service.response.output_trajectory.trajectory.data[planning_duration * prx::simulation_step + 1]);
        execution_cycle_start_observations.push_back(_most_recent_observation);
        update_controls(_service.response.output_plan, planning_controller_cycle_idx);
      }
      else
      {
        ROS_INFO_STREAM(
            "Planner failure, output traj size: " << _service.response.output_trajectory.trajectory.data.size());
        set_zero_controls(planning_controller_cycle_idx);
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