#include <functional>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_copy.hpp>
#include <utils/dbg_utils.hpp>
#include <motion_planning/tree_bridge.hpp>
#include <utils/rosparams_utils.hpp>
#include <interface/ReplannerStatus.h>

namespace mj_ros
{
template <typename Planner, typename SpecPtr, typename QueryPtr, typename PlannerService>
class planner_service_t
{
  using PlannerPtr = std::shared_ptr<Planner>;

private:
  ros::ServiceServer _service_server;
  ros::ServiceClient _controller_client;

  ros::Subscriber _obs_subscriber;
  ros::Publisher _safety_radius_publisher, _tree_publisher, _sln_tree_publisher;
  ros::Publisher _status_publisher;
  // Observation _most_recent_observation;
  ml4kp_bridge::TrajectoryStamped _feedback_traj;

  PlannerPtr _planner;
  QueryPtr _query;
  SpecPtr _spec;
  PlannerService _service;
  ros::Time _preprocess_end_time, _query_fulfill_start_time;
  double preprocess_timeout, postprocess_timeout;
  bool _propagate_dynamics, _retain_previous;
  int _consecutive_contingency_failures = 0;

  double _max_edge_duration;
  std::size_t _current_idx;
  // Function to calculate safe distance based on speed
  std::function<double(double)> _safe_distance_calculator;

  prx::plan_t* step_plan;
  prx::plan_t* rest_of_plan;
  prx::trajectory_t* step_traj;

  interface::ReplannerStatus _status;

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
    , _current_idx(0)
    , _max_edge_duration(0.1)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string service_name{ root + "/planner_service" };

    double& max_edge_duration{ _max_edge_duration };

    std::string sbmp_solution_tree_topic, sbmp_full_tree_topic;

    PARAM_SETUP(nh, sbmp_full_tree_topic);
    PARAM_SETUP(nh, sbmp_solution_tree_topic);
    PARAM_SETUP_WITH_DEFAULT(nh, max_edge_duration, max_edge_duration);

    // DEBUG_VARS(root);
    _service_server = nh.advertiseService(service_name, &planner_service_t::service_callback, this);
    // _obs_subscriber = nh.subscribe(root + "/pose", 1000, &planner_service_t::observation_callback, this);
    _safety_radius_publisher = nh.advertise<std_msgs::Float64>(root + "/safety_radius", 10, true);
    _tree_publisher = nh.advertise<prx_models::Tree>(sbmp_full_tree_topic, 10, true);
    _sln_tree_publisher = nh.advertise<prx_models::Tree>(sbmp_solution_tree_topic, 10, true);

    _status_publisher = nh.advertise<interface::ReplannerStatus>("/kraft/status", 1, true);

    step_plan = new prx::plan_t(_spec->control_space);
    rest_of_plan = new prx::plan_t(_spec->control_space);
    ml4kp_bridge::add_zero_control(*step_plan);
    ml4kp_bridge::add_zero_control(*rest_of_plan);
    step_traj = new prx::trajectory_t(_spec->state_space);

    // Default safe distance calculator (returns 0.0 if not set)
    _safe_distance_calculator = [](double speed) { return 0.0; };
    PRINT_MSG("Planner Service initialized");

    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);
  }

  void set_safe_distance_calculator(std::function<double(double)> calculator)
  {
    _safe_distance_calculator = calculator;
  }

  // void observation_callback(const Observation& message)
  // {
  //   _most_recent_observation = message;
  //   double velocity = _most_recent_observation.float_extra[0].data;
  //   // Calculate and publish the safe distance
  //   // if (_safe_distance_calculator)
  //   // {
  //   //   std_msgs::Float64 safe_distance_msg;
  //   //   safe_distance_msg.data = _safe_distance_calculator(velocity);
  //   //   _safety_radius_publisher.publish(safe_distance_msg);
  //   // }
  // }

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
    return _preprocess_end_time.toSec();
  }

  double get_query_fulfill_time() const
  {
    return _query_fulfill_start_time.toSec();
  }

  void setup_tree(prx_models::Tree& sln_tree, prx::plan_t& plan, prx::trajectory_t& traj)
  {
    sln_tree.root = _current_idx;
    double ti{ 0.0 };
    double curr_cost{ 0.0 };

    sln_tree.nodes.emplace_back();

    ml4kp_bridge::copy(sln_tree.nodes.back().point, traj.at(0.0, false));
    sln_tree.nodes.back().cost = curr_cost;
    sln_tree.nodes.back().index = _current_idx;        //                    # id of the current node
    sln_tree.nodes.back().parent = _current_idx;       //              # parent of the current node. Only used for Tree
    sln_tree.nodes.back().parent_edge = _current_idx;  // # Edge id between the parent and this node. Only used for Tree
                                                       // uint64[] children
    _current_idx++;
    // DEBUG_VARS(plan);
    // DEBUG_VARS(traj);

    for (std::size_t i = 0; i < plan.size(); ++i)
    {
      const prx::plan_step_t ps_i{ plan[i] };

      double dt_remaining{ ps_i.duration };
      // for (double ti = 0.0; ti < ps_i.duration; ti += _max_edge_duration)
      while (dt_remaining > 0.0001)  // small epsilon
      {
        motion_planning::EdgeNodePair edge_node{ motion_planning::create_edge_node(sln_tree.nodes.back(),
                                                                                   _current_idx) };

        const double dt_curr{ std::min(_max_edge_duration, dt_remaining) };
        const prx::space_point_t xi{ traj.at(ti + dt_curr, false) };
        edge_node.first.plan.steps.emplace_back();

        ml4kp_bridge::copy(edge_node.first.plan.steps.back(), ps_i);
        edge_node.first.plan.steps.back().duration.data = ros::Duration(dt_curr);

        ml4kp_bridge::copy(edge_node.second.point, xi);

        sln_tree.edges.push_back(edge_node.first);
        sln_tree.nodes.push_back(edge_node.second);

        dt_remaining = dt_remaining - _max_edge_duration;
        // _current_idx++;
        // DEBUG_VARS(ti, ps_i.duration, dt_curr, xi);
      }
      ti += ps_i.duration;

      // sln_tree.edges.emplace_back();
      // sln_tree.edges.back().cost = curr_cost;
      // sln_tree.edges.back().index = _current_idx;
      // sln_tree.edges.back().source = sln_tree.nodes.back().index;
      // sln_tree.edges.back().target = sln_tree.nodes.back().index + 1;
      // sln_tree.edges.back().plan.steps.emplace_back();
      // ml4kp_bridge::copy(sln_tree.edges.back().plan.steps.back(), ps_i);

      // _current_idx++;
      // sln_tree.nodes.back().children.push_back(_current_idx);

      // sln_tree.nodes.emplace_back();
      // ml4kp_bridge::copy(sln_tree.nodes.back().point, xi);
      // sln_tree.nodes.back().cost = curr_cost;
      // sln_tree.nodes.back().index = _current_idx;  //                    # id of the current node
      // sln_tree.nodes.back().parent =
      //     sln_tree.edges[sln_tree.edges.back().index].source;           // # parent of the current node.
      // sln_tree.nodes.back().parent_edge = sln_tree.edges.back().index;  // # Edge id between the parent and this node
      // uint64[] children
    }
    // DEBUG_VARS(sln_tree);
  }

  bool service_callback(typename PlannerService::Request& request, typename PlannerService::Response& response)
  {
    const ros::Time start_time{ ros::Time::now() };
    _status.state = interface::ReplannerStatus::PREPROCESSING;
    _status_publisher.publish(_status);

    _current_idx = request.idx;

    // prx_models::copy(_query->start_state, request.current_observation);
    _spec->state_space->copy(_query->start_state, request.current_observation.point);
    // if (step_traj->size() > 0)
    // {
    //   _query->start_state->at(2) = step_traj->back()->at(2);
    //   _query->start_state->at(3) = step_traj->back()->at(3);
    // }

    // _query->goal_state-, request.goal_configuration);
    prx_models::copy(_query->goal_state, request.goal_configuration);

    step_traj->clear();

    // if (_propagate_dynamics)
    // {
    //   ROS_INFO_STREAM("Before f: " << _spec->state_space->print_point(_query->start_state, 4));
    //   _spec->propagate(_query->start_state, *step_plan, *step_traj);
    //   _spec->state_space->copy(_query->start_state, step_traj->back());
    //   ROS_INFO_STREAM("After f: " << _spec->state_space->print_point(_query->start_state, 4));

    //   if (!_spec->valid_state(_query->start_state))
    //   {
    //     ROS_WARN("Invalid start state");
    //     // prx_models::copy(_query->start_state, request.current_observation);
    //     _spec->state_space->copy(_query->start_state, request.current_observation.point);
    //   }
    // }

    _planner->link_and_setup_spec(_spec);
    _planner->preprocess();
    _planner->link_and_setup_query(_query);

    _status.header.stamp = ros::Time::now();
    _preprocess_end_time = _status.header.stamp;
    _status.state = interface::ReplannerStatus::PLANNING;
    _status_publisher.publish(_status);

    const double preprocess_real_dt{ (_preprocess_end_time - start_time).toSec() };
    const double time_limit{ request.planning_duration.data.toSec() - preprocess_real_dt - postprocess_timeout };
    prx_assert(time_limit > 0, "Time limit is less than 0");
    prx::condition_check_t checker("time", time_limit);

    _planner->resolve_query(&checker);

    _status.header.stamp = ros::Time::now();
    _query_fulfill_start_time = _status.header.stamp;
    _status.state = interface::ReplannerStatus::POSTPROCESSING;
    _status_publisher.publish(_status);

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
        _spec->state_space->copy(current_state, request.current_observation.point);
        // prx_models::copy(current_state, _most_recent_observation);
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
      else
      {
        // _query->solution_traj;
      }

      if (valid)
      {
        ml4kp_bridge::copy(response.output_plan, step_plan);
        ml4kp_bridge::copy(response.output_trajectory, _query->solution_traj);
        setup_tree(response.sln_tree, *step_plan, _query->solution_traj);
        _sln_tree_publisher.publish(response.sln_tree);
        response.planner_output = PlannerService::Response::TYPE_SUCCESS;
        PRINT_MSG("Valid solution published")
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

    prx_models::Tree ros_tree;
    motion_planning::copy<typename Planner::Node, typename Planner::Edge>(ros_tree, _planner->tree());
    _tree_publisher.publish(ros_tree);

    _planner->reset();
    if (!_retain_previous)
      _query->clear_outputs();

    _status.header.stamp = ros::Time::now();
    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);
    return true;
  }
};
}  // namespace mj_ros