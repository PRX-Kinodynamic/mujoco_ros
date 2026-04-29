#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>

#include <iterator>
#include <memory>
#include <string>
#include <std_msgs/Bool.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>
#include <prx_models/StelaKraft.h>

#include <interface/PlannerClock.h>

#include <motion_planning/utils.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <interface/ReplannerStatus.h>
#include <visualization_msgs/Marker.h>

namespace motion_planning
{

class sbmp_caller_t
{
public:
  using Trajectory = std::vector<ml4kp_bridge::SpacePointStamped>;
  using Plan = std::vector<ml4kp_bridge::PlanStepStamped>;
  using Result = std::pair<Trajectory, Plan>;

  sbmp_caller_t(ros::NodeHandle nh)
  {
    std::string replanner_service, condition;

    bool retain_plan;
    double solution_duration;
    double& postprocessing_rate{ _postprocessing_rate };

    PARAM_SETUP(nh, condition);
    PARAM_SETUP(nh, retain_plan);
    PARAM_SETUP(nh, replanner_service);
    PARAM_SETUP(nh, solution_duration);
    PARAM_SETUP(nh, postprocessing_rate);

    prx_assert(condition == "TIME" or condition == "ITERATIONS",
               "[sbmp_caller_t] Condition must be 'TIME' or 'ITERATIONS'");
    _planner_service_call.request.condition = condition == "TIME" ?
                                                  prx_models::StelaKraft::Request::CONDITION_TIME :
                                                  prx_models::StelaKraft::Request::CONDITION_ITERATIONS;

    if (_planner_service_call.request.condition == prx_models::StelaKraft::Request::CONDITION_ITERATIONS)
    {
      int& iterations{ _planner_service_call.request.iterations };
      PARAM_SETUP(nh, iterations);
    }
    _planner_service_call.request.solution_duration = ros::Duration(solution_duration);
    _planner_service_call.request.retain_plan = retain_plan;

    const ros::Duration timer_duration(0.01);

    _planner_service_client = nh.serviceClient<prx_models::StelaKraft>(replanner_service);
    _planner_status = nh.advertise<interface::ReplannerStatus>(replanner_service + "/status", 1, true);

    _status_timer = nh.createTimer(timer_duration, &sbmp_caller_t::timer_callback, this);

    status(interface::ReplannerStatus::IDLE);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    _status.header.stamp = ros::Time::now();
    _planner_status.publish(_status);
  }

  ~sbmp_caller_t()
  {
  }

  bool valid()
  {
    return _planner_service_client.exists();
  }

  int condition() const
  {
    return _planner_service_call.request.condition;
  }

  Result call(const double planning_time, const ml4kp_bridge::SpacePointStamped& root_state, const Plan& plan)
  {
    status(interface::ReplannerStatus::PREPROCESSING);
    _trajectory.clear();

    if (_planner_service_call.request.condition == prx_models::StelaKraft::Request::CONDITION_TIME)
    {
      // _planner_service_call.request.deadline = deadline;

      _planner_service_call.request.planning_time = planning_time * _postprocessing_rate;
    }

    _planner_service_call.request.root_state = root_state;
    _planner_service_call.request.retainment_plan = plan;

    status(interface::ReplannerStatus::PLANNING);
    if (_planner_service_client.call(_planner_service_call))
    {
      status(interface::ReplannerStatus::POSTPROCESSING);

      if (_planner_service_call.response.planner_output == prx_models::StelaKraft::Response::TYPE_SUCCESS)
      {
        Plan plan{ _planner_service_call.response.piecewise_plan.data };
        _trajectory = _planner_service_call.response.trajectory.data;
        // const double dt_used{ (ros::Time::now() - cycle_start).toSec() };
        // const double dt_remaining{ (_planner_service_call.request.deadline - ros::Time::now()).toSec() };
        // change_status(stela_thread_t::REPLANNING, interface::StelaStatus::VALIDATING);

        // const std::size_t root_idx{ _planner_service_call.response.sln_tree.root };

        // prx_models::tree_msg_wrapper_t wrapped_tree(_planner_service_call.response.sln_tree);

        // if (_validation_plan_feasibility)
        // {
        //   // Change to:  check_new_tree(prx_models::tree_msg_wrapper_t& new_tree, const tree_validation_params_t
        //   // params) ;
        //   PRINT_MSG("Change [check_new_tree] to motion_planning::check_new_tree(tree, params)");
        //   _new_tree_available = check_new_tree(wrapped_tree);
        //   // _new_tree_available = _validate_replanner_sln ? check_new_tree(wrapped_tree) : true;
        // }

        // if (_new_tree_available and _validation_collision_only)
        // {
        //   _new_tree_available =
        //       _robot->propagate_plan(_replanning_root_estimates, wrapped_tree);  // check_new_tree(wrapped_tree);
        //   // bool propagate_plan(_replanning_root_estimates, wrapped_tree);
        // }

        // const double dt_validated{ (ros::Time::now() - cycle_start).toSec() };
        // const double dt_remaining_valid{ (_planner_service_call.request.deadline - ros::Time::now()).toSec() };
        // // change_status(stela_thread_t::REPLANNING, interface::StelaStatus::VALIDATING);

        // if (_new_tree_available)
        // {
        //   _new_tree = wrapped_tree;

        //   const double dt_used_acepted{ (ros::Time::now() - cycle_start).toSec() };
        //   const double dt_remaining_accepted{ (_planner_service_call.request.deadline - ros::Time::now()).toSec()
        //   }; LOG_VARS(_new_tree_available, dt_used_acepted, dt_remaining_accepted)
        // }
        status(interface::ReplannerStatus::SUCCESS);
        return std::make_pair(_trajectory, plan);
      }

      // change_status(stela_thread_t::REPLANNING, interface::StelaStatus::IDLE);
    }
    status(interface::ReplannerStatus::FAILURE);
    return { {}, {} };
  }

protected:
  void status(const int status)
  {
    _status.state = status;
  }

  Trajectory _trajectory;

  double _postprocessing_rate;
  prx_models::StelaKraft _planner_service_call;

  interface::ReplannerStatus _status;

  visualization_msgs::Marker _traj_marker;

  ros::Timer _status_timer;
  ros::Publisher _planner_status;
  ros::ServiceClient _planner_service_client;
};
}  // namespace motion_planning
