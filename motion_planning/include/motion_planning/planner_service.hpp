#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_mushr.hpp>
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

public:
  planner_service_t(ros::NodeHandle& nh, PlannerPtr planner, SpecPtr spec, QueryPtr query)
    : _planner(planner), _query(query), _spec(spec)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };
    _service_server = nh.advertiseService(service_name, &planner_service_t::service_callback, this);
  }

  bool service_callback(typename Service::Request& request, typename Service::Response& response)
  {
    ROS_INFO("Planner service was called.");
    prx::condition_check_t checker("time", request.planning_duration.data.toSec());

    prx_models::copy(_query->start_state, request.current_observation);
    prx_models::copy(_query->goal_state, request.goal_configuration);

    _planner->link_and_setup_spec(_spec);
    _planner->preprocess();
    _planner->link_and_setup_query(_query);
    _planner->resolve_query(&checker);
    _planner->fulfill_query();

    if (_query->solution_traj.size() > 0)
    {
      prx_models::copy(response.output_plan, _query->solution_plan);
      response.planner_output = Service::Response::TYPE_SUCCESS;
    }
    else
    {
      ROS_WARN("No solution found");
      response.planner_output = Service::Response::TYPE_FAILURE;
    }
    _query->clear_outputs();
    _planner->reset();
    return true;
  }
};
}  // namespace mj_ros