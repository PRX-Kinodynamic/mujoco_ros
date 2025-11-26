#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

#include <prx_models/Tree.h>
#include <motion_planning/tree_bridge.hpp>

#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>

namespace mj_ros
{
template <typename PlannerService, typename RobotInterface>
class planner_client_t
{
private:
  ros::ServiceClient _service_client;
  ros::Subscriber _obs_subscriber;
  ros::Publisher _plan_publisher, _traj_publisher, _feedback_traj_publisher;
  PlannerService _service;
  prx_models::Tree _received_tree;
  bool _obs_received{ false };
  ros::Time _experiment_start_time;
  double _preprocess_start_time, _query_fulfill_end_time;
  int _control_dim;

  std::size_t _current_node_idx;
  bool _use_contingency;
  int _contingency_steps{ 0 };
  bool _use_complete_traj;
  std::vector<ml4kp_bridge::SpacePoint> planning_cycle_start_states, planning_cycle_end_states;
  std::vector<ml4kp_bridge::SpacePoint> execution_cycle_start_observations;

  ml4kp_bridge::TrajectoryStamped _feedback_traj;
  ml4kp_bridge::SpacePoint _z0;

public:
  planner_client_t(ros::NodeHandle& nh, int control_dim) : _control_dim(control_dim), _use_complete_traj(false)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };

    std::string estimation_tree_topic;
    bool& use_contingency{ _use_contingency };
    bool& use_complete_traj{ _use_complete_traj };

    double planning_cycle_duration;

    PARAM_SETUP(nh, estimation_tree_topic);
    PARAM_SETUP(nh, use_contingency);
    PARAM_SETUP(nh, planning_cycle_duration);
    PARAM_SETUP_WITH_DEFAULT(nh, use_complete_traj, use_complete_traj);

    _service_client = nh.serviceClient<PlannerService>(service_name);
    _obs_subscriber = nh.subscribe(estimation_tree_topic, 10, &planner_client_t::observation_callback, this);
    _plan_publisher = nh.advertise<ml4kp_bridge::PlanStamped>(root + "/ml4kp_plan", 1000, true);
    _traj_publisher = nh.advertise<ml4kp_bridge::TrajectoryStamped>(root + "/ml4kp_traj", 1000, true);
    _feedback_traj_publisher = nh.advertise<ml4kp_bridge::TrajectoryStamped>(root + "/feedback_traj", 1, true);

    // nh.getParam(ros::this_node::getName() + "/use_contingency", _use_contingency);
    // nh.getParam(ros::this_node::getName() + "/use_complete_traj", _use_complete_traj);
    // nh.getParam(ros::this_node::getName() + "/planning_cycle_duration", planning_cycle_duration);
    _contingency_steps = 1 + planning_cycle_duration / prx::simulation_step;
    if (_use_contingency)
    {
      ROS_INFO("Using contingency with %d steps", _contingency_steps);
    }
    if (_use_complete_traj)
    {
      ROS_INFO("Using complete trajectory");
    }

    PRINT_MSG("Planner Client initialized");
  }

  void publish_feedback_traj(const ml4kp_bridge::TrajectoryStamped& feedback_traj, const u_int32_t& cycle_idx,
                             ros::Time& start_time)
  {
    _feedback_traj.trajectory.data.clear();

    if (_use_contingency and !_use_complete_traj and feedback_traj.trajectory.data.size() > 0)
    {
      int size = feedback_traj.trajectory.data.size();
      unsigned end_index = std::min(_contingency_steps, size);
      for (unsigned i = 0; i < end_index; ++i)
      {
        _feedback_traj.trajectory.data.push_back(feedback_traj.trajectory.data[i]);
      }
    }
    else
    {
      _feedback_traj = feedback_traj;
    }

    std::cout << "Publishing feedback traj for cycle " << cycle_idx << std::endl;

    _feedback_traj.header.seq = cycle_idx;
    _feedback_traj.header.stamp = start_time;
    _feedback_traj_publisher.publish(_feedback_traj);
  }

  std::tuple<std::vector<ml4kp_bridge::SpacePoint>, std::vector<ml4kp_bridge::SpacePoint>,
             std::vector<ml4kp_bridge::SpacePoint>>
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

  ros::Time get_experiment_start_time() const
  {
    return _experiment_start_time;
  }

  void get_last_node_of_tree()
  {
    prx_models::Node node{ motion_planning::get_root(_received_tree) };

    while (node.children.size() > 0)
    {
      _current_node_idx = node.children[0];
      node = motion_planning::get_node(_received_tree, _current_node_idx);
      // _obs_received = true;
    }
    DEBUG_VARS(_current_node_idx);
    _z0 = node.point;
  }

  void observation_callback(const prx_models::TreeConstPtr msg)
  {
    // _most_recent_observation = message;
    if (msg->nodes.size() > 0)
    {
      _received_tree = *msg;
      get_last_node_of_tree();
      _obs_received = true;
    }

    // const prx_models::Node root{ motion_planning::get_root(_received_tree) };

    // if (root.children.size() > 0)
    // {
    //   _current_node_idx = root.children[0];
    //   _z0 = motion_planning::get_node(_received_tree, _current_node_idx).point;
    //   _obs_received = true;
    // }
  }

  // TODO: This is probably not the best place to have this (should be inside mj_mushr.hpp)
  bool is_goal_reached(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius)
  {
    if (!_obs_received)
    {
      return false;
    }
    ml4kp_bridge::SpacePoint goal;
    goal.point.push_back(goal_configuration.x);
    goal.point.push_back(goal_configuration.y);
    goal.point.push_back(goal_configuration.theta);
    return RobotInterface::distance(goal, _z0) < goal_radius.data;
  }

  void call_service(const geometry_msgs::Pose2D& goal_configuration, const std_msgs::Float64& goal_radius,
                    const uint32_t& cycle_idx, double planning_duration = 1.0)
  {
    while (!_obs_received)
    {
      ROS_WARN("Service waiting for observation");
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    _preprocess_start_time = ros::Time::now().toSec();
    _service.request.idx = _current_node_idx;
    _service.request.current_observation = _z0;
    _service.request.planning_duration.data = ros::Duration(planning_duration);
    _service.request.goal_configuration = goal_configuration;
    if (_service_client.call(_service))
    {
      if (cycle_idx == 0)
      {
        _experiment_start_time = ros::Time::now() + ros::Duration(planning_duration);
      }

      ROS_DEBUG("Service call successful");
      // ROS_DEBUG_STREAM("Current obs: " << _most_recent_observation.pose.position.x << ", "
      //                                  << _most_recent_observation.pose.position.y);
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
        publish_feedback_traj(_service.response.output_trajectory, cycle_idx, _experiment_start_time);
        _traj_publisher.publish(_service.response.output_trajectory);
        planning_cycle_start_states.push_back(_service.response.output_trajectory.trajectory.data[0]);
        planning_cycle_end_states.push_back(
            _service.response.output_trajectory.trajectory.data[planning_duration * prx::simulation_step + 1]);
        execution_cycle_start_observations.push_back(_z0);
        _plan_publisher.publish(_service.response.output_plan);
      }
      else
      {
        publish_feedback_traj(_service.response.output_trajectory, cycle_idx, _experiment_start_time);
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
    // _obs_received = false;
  }
};
}  // namespace mj_ros