#include <ml4kp_bridge/defs.h>
#include <ml4kp_bridge/plan_bridge.hpp>

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

  prx::plan_t* step_plan;
  prx::trajectory_t* step_traj;

public:
  planner_service_t(ros::NodeHandle& nh, PlannerPtr planner, SpecPtr spec, QueryPtr query)
    : _planner(planner), _query(query), _spec(spec), preprocess_timeout(0.0), postprocess_timeout(0.0)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_server = nh.advertiseService(service_name, &planner_service_t::service_callback, this);

    step_plan = new prx::plan_t(_spec->control_space);
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
    prx::condition_check_t checker("time", request.planning_duration.data.toSec() - preprocess_timeout - postprocess_timeout);

    prx_models::copy(_query->start_state, request.current_observation);
    prx_models::copy(_query->goal_state, request.goal_configuration);

    if (!request.first_cycle.data)
    {
      step_traj->clear();
      _spec->propagate(_query->start_state, *step_plan, *step_traj);
      _spec->state_space->copy_point(_query->start_state, step_traj->back());
    }

    _planner->link_and_setup_spec(_spec);
    _planner->preprocess();
    _planner->link_and_setup_query(_query);
    _preprocess_end_time = ros::Time::now().toSec();
    _planner->resolve_query(&checker);
    _query_fulfill_start_time = ros::Time::now().toSec();
    _query->clear_outputs();
    _planner->fulfill_query();

    if (_query->solution_traj.size() > 0)
    {
      double execution_time = request.planning_duration.data.toSec();
      step_plan->clear();
      _query->solution_plan.copy_to(0, execution_time, *step_plan);
      // ml4kp_bridge::add_zero_control(*step_plan);
      ml4kp_bridge::copy(response.output_plan, step_plan);
      response.planner_output = Service::Response::TYPE_SUCCESS;
    }
    else
    {
      ROS_WARN("No solution found");
      step_plan->clear();
      ml4kp_bridge::add_zero_control(*step_plan, request.planning_duration.data.toSec());
      response.planner_output = Service::Response::TYPE_FAILURE;
    }
    _planner->reset();
    return true;
  }
};
}  // namespace mj_ros