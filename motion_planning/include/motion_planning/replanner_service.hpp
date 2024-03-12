#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_copy.hpp>
#include <ros/ros.h>

namespace mj_ros
{
template <typename PlannerPtr, typename SpecPtr, typename QueryPtr, typename Service>
class planner_service_t
{
private:
  ros::ServiceServer _service_server;
  PlannerPtr _planner;
  QueryPtr _query;
  SpecPtr _spec;
  Service _service;
  double _preprocess_end_time, _query_fulfill_start_time;
  double preprocess_timeout, postprocess_timeout;
  bool _propagate_dynamics, _retain_previous;

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

    step_plan = new prx::plan_t(_spec->control_space);
    rest_of_plan = new prx::plan_t(_spec->control_space);
    ml4kp_bridge::add_zero_control(*step_plan);
    ml4kp_bridge::add_zero_control(*rest_of_plan);
    step_traj = new prx::trajectory_t(_spec->state_space);
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

  bool service_callback(typename Service::Request& request, typename Service::Response& response)
  {
    prx::condition_check_t checker("time",
                                   request.planning_duration.data.toSec() - preprocess_timeout - postprocess_timeout);

    prx_models::copy(_query->start_state, request.current_observation);
    prx_models::copy(_query->goal_state, request.goal_configuration);

    step_traj->clear();

    if (_propagate_dynamics)
    {
      ROS_DEBUG_STREAM("Before f: " << _spec->state_space->print_point(_query->start_state, 4));
      _spec->propagate(_query->start_state, *step_plan, *step_traj);
      _spec->state_space->copy(_query->start_state, step_traj->back());
      ROS_DEBUG_STREAM("After f: " << _spec->state_space->print_point(_query->start_state, 4));

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

      if (_propagate_dynamics)
      {
        _query->solution_plan.clear();
        rest_of_plan->copy_to(0, rest_of_plan->duration(), _query->solution_plan);
      }

      ml4kp_bridge::copy(response.output_plan, step_plan);
      ml4kp_bridge::copy(response.output_trajectory, _query->solution_traj);
      response.planner_output = Service::Response::TYPE_SUCCESS;
    }
    else
    {
      ROS_WARN("No solution found");
      step_plan->clear();
      ml4kp_bridge::add_zero_control(*step_plan, execution_time);
      response.planner_output = Service::Response::TYPE_FAILURE;
    }
    _planner->reset();
    return true;
  }
};
}  // namespace mj_ros