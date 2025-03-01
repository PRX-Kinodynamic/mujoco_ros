#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_copy.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <functional>

namespace mj_ros
{
template <typename PlannerPtr, typename SpecPtr, typename QueryPtr, typename PlannerService, typename Observation>
class planner_service_t
{
private:
  ros::ServiceServer _service_server;
  ros::ServiceClient _controller_client;

  ros::Subscriber _obs_subscriber;
  ros::Publisher _safety_radius_publisher;
  Observation _most_recent_observation;
  ml4kp_bridge::TrajectoryStamped _feedback_traj;

  PlannerPtr _planner;
  QueryPtr _query;
  SpecPtr _spec;
  PlannerService _service;
  double _preprocess_end_time, _query_fulfill_start_time;
  double preprocess_timeout, postprocess_timeout;
  bool _propagate_dynamics, _retain_previous;
  int _consecutive_contingency_failures = 0;

  // Function to calculate safe distance based on speed
  std::function<double(double)> _safe_distance_calculator;

  prx::plan_t* step_plan;
  prx::plan_t* rest_of_plan;
  prx::trajectory_t* step_traj;

public:
  planner_service_t(ros::NodeHandle& nh, PlannerPtr planner, SpecPtr spec, QueryPtr query, bool propagate_dynamics,
                    bool retain_previous)
    : _planner(planner)
    , _query(query)
    , _spec(spec)
    , preprocess_timeout(0.0)
    , postprocess_timeout(0.0)
    , _propagate_dynamics(propagate_dynamics)
    , _retain_previous(retain_previous)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_server = nh.advertiseService(service_name, &planner_service_t::service_callback, this);
    _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_service_t::observation_callback, this);
    _safety_radius_publisher = nh.advertise<std_msgs::Float64>(root + "/safety_radius", 10, true);

    step_plan = new prx::plan_t(_spec->control_space);
    rest_of_plan = new prx::plan_t(_spec->control_space);
    ml4kp_bridge::add_zero_control(*step_plan);
    ml4kp_bridge::add_zero_control(*rest_of_plan);
    step_traj = new prx::trajectory_t(_spec->state_space);

    // Default safe distance calculator (returns 0.0 if not set)
    _safe_distance_calculator = [](double speed) { return 0.0; };
  }

  void set_safe_distance_calculator(std::function<double(double)> calculator)
  {
    _safe_distance_calculator = calculator;
  }

  void observation_callback(const Observation& message)
  {
    _most_recent_observation = message;
    double velocity = _most_recent_observation.float_extra[0].data;
    // Calculate and publish the safe distance
    // if (_safe_distance_calculator)
    // {
    //   std_msgs::Float64 safe_distance_msg;
    //   safe_distance_msg.data = _safe_distance_calculator(velocity);
    //   _safety_radius_publisher.publish(safe_distance_msg);
    // }
  }

  void set_preprocess_timeout(double timeout)
  {
    preprocess_timeout = timeout;
  }

  void set_postprocess_timeout(double timeout)
  {
    postprocess_timeout = timeout;
  }

  double get_preprocess_time() const
  {
    return _preprocess_end_time;
  }

  double get_query_fulfill_time() const
  {
    return _query_fulfill_start_time;
  }

  bool service_callback(typename PlannerService::Request& request, typename PlannerService::Response& response)
  {
    const double time_limit{ request.planning_duration.data.toSec() - preprocess_timeout - postprocess_timeout };
    prx_assert(time_limit > 0, "Time limit is less than 0");
    prx::condition_check_t checker("time", time_limit);

    prx_models::copy(_query->start_state, request.current_observation);
    prx_models::copy(_query->goal_state, request.goal_configuration);

    step_traj->clear();

    if (_propagate_dynamics)
    {
      ROS_INFO_STREAM("Before f: " << _spec->state_space->print_point(_query->start_state, 4));
      _spec->propagate(_query->start_state, *step_plan, *step_traj);
      _spec->state_space->copy(_query->start_state, step_traj->back());
      ROS_INFO_STREAM("After f: " << _spec->state_space->print_point(_query->start_state, 4));

      if (!_spec->valid_state(_query->start_state))
      {
        ROS_WARN("Invalid start state");
        prx_models::copy(_query->start_state, request.current_observation);
      }
    }

    _planner->link_and_setup_spec(_spec);
    _planner->preprocess();
    _planner->link_and_setup_query(_query);
    _preprocess_end_time = ros::Time::now().toSec();
    _planner->resolve_query(&checker);
    _query_fulfill_start_time = ros::Time::now().toSec();
    _query->clear_outputs();
    _planner->fulfill_query();

    prx::space_point_t current_state = _spec->state_space->make_point();
    double execution_time = request.planning_duration.data.toSec();
    if (_query->solution_traj.size() > 0)
    {
      double plan_duration = _query->solution_cost;
      if (plan_duration < execution_time)
      {
        ml4kp_bridge::add_zero_control(_query->solution_plan, execution_time - plan_duration + prx::simulation_step);
      }
      step_plan->clear();
      rest_of_plan->clear();
      _query->solution_plan.copy_to(0, execution_time, *step_plan);
      _query->solution_plan.copy_to(execution_time, _query->solution_plan.duration(), *rest_of_plan);

      if (_retain_previous)
      {
        _query->solution_plan.clear();
        rest_of_plan->copy_to(0, rest_of_plan->duration(), _query->solution_plan);
      }

      bool valid = true;
      if (_spec->use_contingency)
      {
        prx_models::copy(current_state, _most_recent_observation);
        step_traj->clear();
        _spec->propagate(current_state, *step_plan, *step_traj);
        if (_consecutive_contingency_failures == 2)
        {
          valid = _spec->valid_check(*step_traj);
          _consecutive_contingency_failures = 0;
        }
        else
        {
          // valid = _spec->valid_check(*step_traj) && _spec->contingency_check(*step_traj);
          valid = _spec->contingency_check(*step_traj);
        }
      }

      if (valid)
      {
        ml4kp_bridge::copy(response.output_plan, step_plan);
        ml4kp_bridge::copy(response.output_trajectory, _query->solution_traj);
        response.planner_output = PlannerService::Response::TYPE_SUCCESS;
      }
      else
      {
        ROS_WARN("Contingency check failed");
        step_plan->clear();
        ml4kp_bridge::add_zero_control(*step_plan, execution_time);
        ml4kp_bridge::copy(response.output_plan, step_plan);
        response.planner_output = PlannerService::Response::TYPE_FAILURE;
        _consecutive_contingency_failures++;
      }
    }
    else
    {
      ROS_WARN("No solution found");
      step_plan->clear();
      ml4kp_bridge::add_zero_control(*step_plan, execution_time);
      ml4kp_bridge::copy(response.output_plan, step_plan);
      response.planner_output = PlannerService::Response::TYPE_FAILURE;
      _consecutive_contingency_failures++;
    }
    _planner->reset();
    if (!_retain_previous)
      _query->clear_outputs();
    return true;
  }
};
}  // namespace mj_ros