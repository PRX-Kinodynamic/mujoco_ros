#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "control/MushrControlPropagation.h"
#include "motion_planning/replanner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "motion_planning/PlanningResult.h"
#include "mujoco_ros/Collision.h"
#include "std_msgs/Empty.h"
#include <utils/std_utils.cpp>
#include <motion_planning/tree_bridge.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <interface/ReplannerStatus.h>
#include <interface/PlannerClock.h>

// Function to calculate safe distance based on speed
template <typename ParamsType>
double calculate_safe_distance(double speed, const ParamsType& params)
{
  return params["safe_min"].template as<double>() + params["safe_mul"].template as<double>() * std::fabs(speed) +
         params["safe_quad_mul"].template as<double>() * std::pow(std::fabs(speed), 2);
}

struct replanner_t
{
  prx::system_ptr_t _plant;
  prx::param_loader params;

  std::shared_ptr<prx::world_model_t> _planning_model;
  prx::space_t* _state_space;
  prx::space_t* _control_space;
  prx::space_t* _param_space;

  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;

  std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;
  std::vector<std::string> _obstacle_names;

  std::shared_ptr<prx::dirt_replan_t> _dirt;
  std::shared_ptr<prx::dirt_replan_specification_t> _dirt_spec;

  std::string _heuristic_map_filename;
  std::shared_ptr<prx::heuristic_map_t> _heuristic_map;

  std::shared_ptr<prx::dirt_replan_query_t> _dirt_query;

  bool _use_contingency, _cycle_update;

  motion_planning::PlanningResult _planning_result_msg;

  ros::Publisher _goal_pos_publisher;
  ros::Publisher _goal_radius_publisher;
  ros::Publisher _safety_radius_publisher;
  ros::Publisher _planning_result_publisher;
  ros::Publisher _reset_publisher;
  ros::Publisher _status_publisher;
  ros::Publisher _tree_publisher;
  ros::Publisher _sln_tree_publisher;

  ros::Subscriber _z_tree_subscriber;
  ros::Subscriber _planner_clock_subscriber;

  ros::Timer _replan_timer;

  int _max_cycles;
  double _preprocess_timeout, _postprocess_timeout;

  double _max_edge_duration;

  prx::space_point_t _future_state;  // start for the planner

  bool _z_received, _start_replanning;

  int _current_cycle;
  ros::Time _cycle_end, _cycle_start, _next_replan_start, _next_replan_end;
  ros::Duration _cycle_duration;

  // bool _propagate_dynamics, retain_previous, use_contingency;
  interface::ReplannerStatus _status;

  std::shared_ptr<prx::trajectory_t> _step_traj;
  std::shared_ptr<prx::plan_t> _step_plan, _rest_of_plan;

  ros::Time _preprocess_end_time, _query_fulfill_start_time;

  replanner_t(ros::NodeHandle& nh)
    : _z_received(false), _next_replan_start(ros::Time::ZERO), _next_replan_end(ros::Time::ZERO)
  {
    std::string params_file, estimation_tree_topic, planner_clock_topic;
    std::string sbmp_solution_tree_topic, sbmp_full_tree_topic;
    std::string& heuristic_map_filename{ _heuristic_map_filename };

    int& max_cycles{ _max_cycles };

    double& preprocess_timeout{ _preprocess_timeout };
    double& postprocess_timeout{ _postprocess_timeout };
    double& max_edge_duration{ _max_edge_duration };

    DEBUG_PRINT
    PARAM_SETUP(nh, params_file);
    PARAM_SETUP(nh, heuristic_map_filename);

    PARAM_SETUP(nh, preprocess_timeout);
    PARAM_SETUP(nh, postprocess_timeout);
    PARAM_SETUP(nh, max_cycles);

    PARAM_SETUP(nh, estimation_tree_topic);
    PARAM_SETUP(nh, planner_clock_topic);

    PARAM_SETUP(nh, sbmp_full_tree_topic);
    PARAM_SETUP(nh, sbmp_solution_tree_topic);
    PARAM_SETUP_WITH_DEFAULT(nh, max_edge_duration, max_edge_duration);

    // planner_service.set_preprocess_timeout(preprocess_timeout);
    // planner_service.set_postprocess_timeout(postprocess_timeout);

    // Publisher
    _goal_pos_publisher = nh.advertise<geometry_msgs::Pose2D>("/kraft/goal_pose", 10, true);
    _goal_radius_publisher = nh.advertise<std_msgs::Float64>("/kraft/goal_radius", 10, true);
    _safety_radius_publisher = nh.advertise<std_msgs::Float64>("/kraft/safety_radius", 10, true);
    _planning_result_publisher = nh.advertise<motion_planning::PlanningResult>("/kraft/planning_result", 1, true);
    _reset_publisher = nh.advertise<std_msgs::Empty>("/kraft/reset", 1, true);
    _status_publisher = nh.advertise<interface::ReplannerStatus>("/kraft/status", 1, true);
    _tree_publisher = nh.advertise<prx_models::Tree>(sbmp_full_tree_topic, 1, true);
    _sln_tree_publisher = nh.advertise<prx_models::Tree>(sbmp_solution_tree_topic, 1, true);

    _status.state = interface::ReplannerStatus::INITIALIZING;
    _status_publisher.publish(_status);

    params = prx::param_loader(params_file, "");

    prx::init_random(params["random_seed"].as<int>());

    prx::simulation_step = params["simulation_step"].as<double>();
    DEBUG_PRINT

    init_planner_spec(nh);
    DEBUG_PRINT
    init_planner_query();
    DEBUG_PRINT
    init_heuristic_map();

    DEBUG_PRINT
    // get an initial plan
    _dirt = std::make_shared<prx::dirt_replan_t>("dirt");
    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());
    DEBUG_PRINT

    // Subscribers
    _z_tree_subscriber = nh.subscribe(estimation_tree_topic, 10, &replanner_t::observation_callback, this);
    _planner_clock_subscriber = nh.subscribe(planner_clock_topic, 1, &replanner_t::clock_callback, this);

    // Timers
    const ros::Duration timer_duration(0.05);
    _replan_timer = nh.createTimer(timer_duration, &replanner_t::timer_callback, this);

    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    // {}
    // const bool start_replanning{_cycle_start};
    // DEBUG_VARS(_start_replanning, _z_received)
    // if (_start_replanning and _z_received)
    if (not _next_replan_start.isZero() and        // Clock has been set
        _next_replan_start < ros::Time::now() and  // Reached end of current cycle
        _z_received)                               // Observation received
    {
      // _start_replanning = false;
      _next_replan_start = _cycle_end;
      replan();
    }
  }

  void clock_callback(const interface::PlannerClockConstPtr msg)
  {
    if (_next_replan_end.isZero())
    {
      _next_replan_start = msg->cycle_end;
      // _next_replan_end = msg->cycle_end;
    }
    if (_current_cycle != msg->cycle)
    {
      _current_cycle = msg->cycle;
      _cycle_end = msg->cycle_end;
      _cycle_start = msg->cycle_start;
      _cycle_duration = msg->cycle_duration;
      // _cycle_update = true;
      // _start_replanning = true;
      DEBUG_VARS(*msg);
    }
  }

  void observation_callback(prx_models::TreePtr msg)
  {
    if (msg->nodes.size() > 0)
    {
      // _received_tree = *msg;
      get_next_prediction(msg);
      _z_received = true;
    }
  }

  void get_next_prediction(prx_models::TreePtr msg)
  {
    prx_models::Node node{ motion_planning::get_root(*msg) };
    prx_models::Edge edge;

    ros::Time curr_time{ msg->header.stamp };
    // double remaining_dt{ _planning_cycle };
    while (_cycle_end > curr_time or node.children.size() > 0)
    {
      // _current_node_idx = node.children[0];
      node = motion_planning::get_node(*msg, node.children[0]);
      edge = motion_planning::get_edge(*msg, node.parent_edge);
      for (auto& step : edge.plan.steps)
      {
        curr_time += ros::Duration(step.duration.data);
      }
      // _obs_received = true;
    }
    // DEBUG_VARS(_current_node_idx);
    // _future_state = node.point;
    ml4kp_bridge::copy(_future_state, node.point);
  }

  void init_planner_spec(ros::NodeHandle& nh)
  {
    const std::string plant_name{ params["/plant/name"].as<std::string>() };
    const std::string plant_path{ params["/plant/path"].as<std::string>() };
    _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(_plant != nullptr, "Failed to create plant");

    auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
    _obstacle_list = obstacles.second;
    _obstacle_names = obstacles.first;

    const std::vector<prx::system_ptr_t> all_systems{ { _plant } };
    const std::vector<std::shared_ptr<prx::movable_object_t>> all_obstacles{ { _obstacle_list } };
    _planning_model = std::make_shared<prx::world_model_t>(all_systems, all_obstacles);
    _planning_model->create_context("planner_context", { plant_name }, { _obstacle_names });
    auto planning_context = _planning_model->get_context("planner_context");

    _system_group = prx::system_group(planning_context);
    _collision_group = prx::collision_group(planning_context);

    _state_space = _system_group->get_state_space();
    _control_space = _system_group->get_control_space();
    _param_space = _system_group->get_parameter_space();

    _state_space->init(params["/plant/state_space"]);
    _control_space->init(params["/plant/control_space"]);
    _param_space->init(params["/plant/parameter_space"]);

    _step_plan = std::make_shared<prx::plan_t>(_control_space);
    _rest_of_plan = std::make_shared<prx::plan_t>(_control_space);
    _step_traj = std::make_shared<prx::trajectory_t>(_state_space);

    _future_state = _state_space->make_point();
    _dirt_spec = std::make_shared<prx::dirt_replan_specification_t>(_system_group, _collision_group);
    // _dirt_spec = new prx::dirt_replan_specification_t(planning_context.first, planning_context.second);

    _dirt_spec->min_control_steps = params["min_time"].as<double>() * 1.0 / prx::simulation_step;
    _dirt_spec->max_control_steps = params["max_time"].as<double>() * 1.0 / prx::simulation_step;
    _dirt_spec->blossom_number = params["blossom_number"].as<int>();
    _dirt_spec->use_pruning = false;

    bool& use_contingency{ _dirt_spec->use_contingency };
    double& planning_cycle_duration{ _dirt_spec->planning_cycle_duration };
    PARAM_SETUP(nh, use_contingency)
    PARAM_SETUP(nh, planning_cycle_duration)

    _dirt_spec->plan_safety_check = [&](prx::trajectory_t& traj) {
      for (auto&& s : traj)
      {
        auto pqp_distance = prx::default_obstacle_distance_function(s, _state_space, _collision_group);
        double min_distance{ std::numeric_limits<double>::max() };
        for (auto&& d : pqp_distance.distances)
        {
          if (d < min_distance)
          {
            min_distance = d;
          }
        }
        double speed = std::fabs(s->at(3));
        double safe_distance = calculate_safe_distance<decltype(params)>(speed, params["plan_safety_params"]);
        if (min_distance < safe_distance)
        {
          return false;
        }
      }
      return true;
    };
  }

  void init_heuristic_map()
  {
    auto heuristic_plant = prx::system_factory_t::create_system("2D_Point", "2D_Point");
    prx_assert(heuristic_plant != nullptr, "Failed to create plant");
    prx::world_model_t heuristic_model({ heuristic_plant }, { _obstacle_list });
    heuristic_model.create_context("heuristic_context", { "2D_Point" }, { _obstacle_names });
    auto heuristic_context = heuristic_model.get_context("heuristic_context");

    // std::vector<double> goal_config = params["goal_state"].as<std::vector<double>>();

    // std::cout << "Goal: " << ss->print_point(goal) << std::endl;

    // prx::heuristic_map_t heuristic_map(-0.5, 2.5, -1.0, 6.0, 0.01, 0.01, heuristic_context);
    _heuristic_map = std::make_shared<prx::heuristic_map_t>(-0.5, 2.5, -1.0, 6.0, 0.01, 0.01, heuristic_context);
    _heuristic_map->set_obstacle_grid();
    _heuristic_map->set_heuristic_grid(_dirt_query->goal_state);
    _heuristic_map->set_u_rep_coeff(params["u_rep"].as<double>());

    std::ofstream file(_heuristic_map_filename.c_str());
    file << *_heuristic_map;
    file.close();

    _dirt_spec->wavefront_h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
      return _heuristic_map->get_cost(s);
    };

    _dirt_spec->h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
      return _dirt_spec->distance_function(s, s2) / 0.62;
    };
  }

  void init_planner_query()
  {
    // _goal = _state_space->make_point(params["goal/state"]);
    // _goal_radius =

    // DEBUG_VARS(_goal, _goal_radius);

    _dirt_query = std::make_shared<prx::dirt_replan_query_t>(_state_space, _control_space);
    _dirt_query->start_state = _state_space->make_point(params["/plant/start_state"]);
    _dirt_query->goal_state = _state_space->make_point(params["/goal/state"]);
    _dirt_query->goal_region_radius = params["goal/radius"].as<double>();
    _dirt_query->get_visualization = false;

    DEBUG_VARS(_dirt_query->start_state)
    DEBUG_VARS(_dirt_query->goal_state)
    DEBUG_VARS(_dirt_query->goal_region_radius)
    // ROS_WARN("Using default goal check");
  }

  void tree_from_plan_traj(prx_models::Tree& sln_tree, prx::plan_t& plan, prx::trajectory_t& traj)
  {
    if (traj.duration() < plan.duration())
    {
      PRINT_MSG("[Replanner::tree_from_plan_traj] Trajectory shorter than plan");
      return;
    }

    std::size_t current_idx{ 0 };
    sln_tree.root = current_idx;
    double ti{ 0.0 };
    double curr_cost{ 0.0 };

    sln_tree.nodes.emplace_back();

    ml4kp_bridge::copy(sln_tree.nodes.back().point, traj.at(0.0, false));
    sln_tree.nodes.back().cost = curr_cost;
    sln_tree.nodes.back().index = current_idx;        //                    # id of the current node
    sln_tree.nodes.back().parent = current_idx;       //              # parent of the current node. Only used for Tree
    sln_tree.nodes.back().parent_edge = current_idx;  // # Edge id between the parent and this node. Only used for Tree
                                                      // uint64[] children
    current_idx++;
    // DEBUG_VARS(plan);
    // DEBUG_VARS(traj);

    for (std::size_t i = 0; i < plan.size(); ++i)
    {
      const prx::plan_step_t ps_i{ plan[i] };

      double dt_remaining{ ps_i.duration };
      while (dt_remaining > 0.0001)  // small epsilon
      {
        motion_planning::EdgeNodePair edge_node{ motion_planning::create_edge_node(sln_tree.nodes.back(),
                                                                                   current_idx) };

        const double dt_curr{ std::min(_max_edge_duration, dt_remaining) };
        const prx::space_point_t xi{ traj.at(ti + dt_curr, false) };
        edge_node.first.plan.steps.emplace_back();

        ml4kp_bridge::copy(edge_node.first.plan.steps.back(), ps_i);
        edge_node.first.plan.steps.back().duration.data = ros::Duration(dt_curr);

        ml4kp_bridge::copy(edge_node.second.point, xi);

        sln_tree.edges.push_back(edge_node.first);
        sln_tree.nodes.push_back(edge_node.second);

        dt_remaining = dt_remaining - _max_edge_duration;
      }
      ti += ps_i.duration;
    }
  }

  void replan()
  {
    // const ros::Time start_time{ ros::Time::now() };
    const double& planning_duration{ _dirt_spec->planning_cycle_duration };
    _status.state = interface::ReplannerStatus::PREPROCESSING;
    _status_publisher.publish(_status);
    DEBUG_VARS(_status.state);

    // _current_idx = request.idx;

    _state_space->copy(_dirt_query->start_state, _future_state);
    // prx_models::copy(_query->start_state, request.current_observation);
    // _spec->state_space->copy(_query->start_state, request.current_observation.point);

    // prx_models::copy(_dirt_query->goal_state, request.goal_configuration);

    _step_traj->clear();

    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());

    const double preprocess_real_dt{ (_preprocess_end_time - _cycle_start).toSec() };
    const double time_limit{ planning_duration - preprocess_real_dt - _postprocess_timeout };
    prx_assert(time_limit > 0, "Time limit is less than 0");
    DEBUG_VARS(time_limit, planning_duration, preprocess_real_dt, _postprocess_timeout);
    prx::condition_check_t checker("time", time_limit);

    _status.header.stamp = ros::Time::now();
    _preprocess_end_time = _status.header.stamp;
    _status.state = interface::ReplannerStatus::PLANNING;
    _status_publisher.publish(_status);
    DEBUG_VARS(_status.state);

    _dirt->resolve_query(&checker);

    _status.header.stamp = ros::Time::now();
    _query_fulfill_start_time = _status.header.stamp;
    _status.state = interface::ReplannerStatus::POSTPROCESSING;
    _status_publisher.publish(_status);
    DEBUG_VARS(_status.state);

    _dirt_query->clear_outputs();
    _dirt->fulfill_query();

    // prx::space_point_t current_state = _spec->state_space->make_point();
    // double execution_time = request.planning_duration.data.toSec();
    if (_dirt_query->solution_traj.size() > 0)
    {
      double plan_duration = _dirt_query->solution_cost;
      if (plan_duration < planning_duration)
      {
        ml4kp_bridge::add_zero_control(_dirt_query->solution_plan,
                                       planning_duration - plan_duration + prx::simulation_step);
      }

      // DEBUG_VARS(_step_plan);
      _step_plan->clear();

      _rest_of_plan->clear();

      // DEBUG_VARS(_dirt_query->solution_plan);
      // DEBUG_VARS(planning_duration);
      _dirt_query->solution_plan.copy_to(0, planning_duration, *_step_plan);
      _dirt_query->solution_plan.copy_to(planning_duration, _dirt_query->solution_plan.duration(), *_rest_of_plan);

      // if (_retain_previous)
      // {
      //   _query->solution_plan.clear();
      //   rest_of_plan->copy_to(0, rest_of_plan->duration(), _query->solution_plan);
      // }

      bool valid{ true };
      // if (_spec->use_contingency)
      // {
      //   // _spec->state_space->copy(current_state, request.current_observation.point);
      //   // prx_models::copy(current_state, _most_recent_observation);
      //   step_traj->clear();
      //   _spec->propagate(current_state, *step_plan, *step_traj);
      //   if (_consecutive_contingency_failures == 2)
      //   {
      //     valid = _spec->valid_check(*step_traj);
      //     _consecutive_contingency_failures = 0;
      //   }
      //   else
      //   {
      //     // valid = _spec->valid_check(*step_traj) && _spec->contingency_check(*step_traj);
      //     valid = _spec->contingency_check(*step_traj);
      //   }
      // }
      // else
      // {
      //   // _query->solution_traj;
      // }

      // if (valid)
      // {
      // ml4kp_bridge::copy(response.output_plan, step_plan);
      // ml4kp_bridge::copy(response.output_trajectory, _query->solution_traj);
      prx_models::Tree sln_tree;

      tree_from_plan_traj(sln_tree, *_step_plan, _dirt_query->solution_traj);
      _sln_tree_publisher.publish(sln_tree);
      // response.planner_output = PlannerService::Response::TYPE_SUCCESS;
      // PRINT_MSG("Valid solution published")
      // }
      // else
      // {
      //   ROS_WARN("Contingency check failed");
      //   step_plan->clear();
      //   ml4kp_bridge::add_zero_control(*step_plan, execution_time);
      //   ml4kp_bridge::copy(response.output_plan, step_plan);
      //   response.planner_output = PlannerService::Response::TYPE_FAILURE;
      //   _consecutive_contingency_failures++;
      // }
    }
    // else
    // {
    //   ROS_WARN("No solution found");
    //   step_plan->clear();
    //   ml4kp_bridge::add_zero_control(*step_plan, execution_time);
    //   ml4kp_bridge::copy(response.output_plan, step_plan);
    //   response.planner_output = PlannerService::Response::TYPE_FAILURE;
    //   _consecutive_contingency_failures++;
    // }

    prx_models::Tree ros_tree;
    motion_planning::copy<prx::dirt_replan_t::Node, prx::dirt_replan_t::Edge>(ros_tree, _dirt->tree());
    _tree_publisher.publish(ros_tree);

    _dirt->reset();
    // if (!_retain_previous)
    //   _dirt_query->clear_outputs();

    _status.header.stamp = ros::Time::now();
    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);
    DEBUG_VARS(_status.state);
    // return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrPlanner_example");
  ros::NodeHandle nh("~");
  // ros::NodeHandle private_nh("~");

  ros::AsyncSpinner spinner(4);

  replanner_t replanner(nh);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  // const std::string root{ ros::this_node::getNamespace() };
  // std::string params_fname;
  // n.getParam(ros::this_node::getName() + "/params_file", params_fname);
  // auto params = prx::param_loader(params_fname, "");

  // // TODO: Does this need to be set inside the client/service for better determinism?
  // prx::init_random(params["random_seed"].as<int>());

  // prx::simulation_step = params["simulation_step"].as<double>();
  // std::string plant_name = params["/plant/name"].as<std::string>();
  // std::string plant_path = params["/plant/path"].as<std::string>();
  // auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  // prx_assert(plant != nullptr, "Failed to create plant");

  // auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
  // std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  // std::vector<std::string> obstacle_names = obstacles.first;

  // prx::world_model_t planning_model({ plant }, { obstacle_list });
  // planning_model.create_context("planner_context", { plant_name }, { obstacle_names });
  // auto planning_context = planning_model.get_context("planner_context");
  // auto ss = planning_context.first->get_state_space();
  // auto cs = planning_context.first->get_control_space();
  // auto ps = planning_context.first->get_parameter_space();
  // std::vector<double> min_control_limits = params["/plant/control_space/lower_bound"].as<std::vector<double>>();
  // std::vector<double> max_control_limits = params["/plant/control_space/upper_bound"].as<std::vector<double>>();
  // std::vector<double> min_state_limits = params["/plant/state_space/lower_bound"].as<std::vector<double>>();
  // std::vector<double> max_state_limits = params["/plant/state_space/upper_bound"].as<std::vector<double>>();
  // ss->set_bounds(min_state_limits, max_state_limits);
  // cs->set_bounds(min_control_limits, max_control_limits);
  // std::vector<double> param_values = params["/plant/parameter_space/values"].as<std::vector<double>>();
  // ps->copy_from(param_values);

  // auto sg = planning_context.first;
  // auto cg = planning_context.second;

  // std::shared_ptr<prx::dirt_replan_t> dirt = std::make_shared<prx::dirt_replan_t>("dirt");
  // prx::dirt_replan_specification_t* dirt_spec =
  //     new prx::dirt_replan_specification_t(planning_context.first, planning_context.second);

  // auto heuristic_plant = prx::system_factory_t::create_system("2D_Point", "2D_Point");
  // prx_assert(heuristic_plant != nullptr, "Failed to create plant");
  // prx::world_model_t heuristic_model({ heuristic_plant }, { obstacle_list });
  // heuristic_model.create_context("heuristic_context", { "2D_Point" }, { obstacle_names });
  // auto heuristic_context = heuristic_model.get_context("heuristic_context");

  // std::vector<double> goal_config = params["goal_state"].as<std::vector<double>>();
  // prx::space_point_t goal = ss->make_point();
  // goal->at(0) = goal_config[0];
  // goal->at(1) = goal_config[1];
  // std::cout << "Goal: " << ss->print_point(goal) << std::endl;
  // auto dirt_spec_ptr = std::shared_ptr<prx::dirt_replan_specification_t>(dirt_spec);
  // prx::heuristic_map_t heuristic_map(-0.5, 2.5, -1.0, 6.0, 0.01, 0.01, heuristic_context);
  // heuristic_map.set_obstacle_grid();
  // heuristic_map.set_heuristic_grid(goal);
  // heuristic_map.set_u_rep_coeff(params["u_rep"].as<double>());

  // std::ofstream file;
  // file.open("/home/aravind/heuristic_map.txt");
  // file << heuristic_map;
  // file.close();

  // dirt_spec->wavefront_h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
  //   return heuristic_map.get_cost(s);
  // };

  // dirt_spec->h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
  //   return dirt_spec->distance_function(s, s2) / 0.62;
  // };

  /*
  prx::plan_t plan(cs);
  plan.append_onto_back(1.0);
  prx::trajectory_t traj(ss);

  std::vector<std::vector<double>> control_list = { { -1.0, 1.0 },  { 0.0, 1.0 },  { 1.0, 1.0 },
                                                    { -1.0, -1.0 }, { 0.0, -1.0 }, { 1.0, -1.0 } };
  dirt_spec->expand = [&](prx::space_point_t& s, std::vector<prx::plan_t*>& plans,
                          std::vector<prx::trajectory_t*>& trajs, int bn, bool blossom_expand) {
    if (blossom_expand)
    {
      for (unsigned i = 0; i < control_list.size(); i++)
      {
        traj.clear();
        cs->copy(plan.back().control, control_list[i]);
        sg->propagate(s, plan, traj);
        plans.push_back(new prx::plan_t(plan));
        trajs.push_back(new prx::trajectory_t(traj));
      }
    }
    else
    {
      prx::default_expand(s, plans, trajs, 1, sg, dirt_spec->sample_plan, dirt_spec->propagate);
    }
  };
  */

  // bool propagate_dynamics, retain_previous, use_contingency;
  // n.getParam(ros::this_node::getName() + "/propagate_dynamics", propagate_dynamics);
  // n.getParam(ros::this_node::getName() + "/retain_previous", retain_previous);
  // n.getParam(ros::this_node::getName() + "/use_contingency", use_contingency);

  // dirt_spec->min_control_steps = params["min_time"].as<double>() * 1.0 / prx::simulation_step;
  // dirt_spec->max_control_steps = params["max_time"].as<double>() * 1.0 / prx::simulation_step;
  // dirt_spec->blossom_number = params["blossom_number"].as<int>();
  // dirt_spec->use_pruning = false;
  // dirt_spec->use_contingency = use_contingency;

  // dirt_spec->plan_safety_check = [&](prx::trajectory_t& traj) {
  //   for (auto&& s : traj)
  //   {
  //     auto pqp_distance = prx::default_obstacle_distance_function(s, ss, cg);
  //     double min_distance = std::numeric_limits<double>::max();
  //     for (auto&& d : pqp_distance.distances)
  //     {
  //       if (d < min_distance)
  //       {
  //         min_distance = d;
  //       }
  //     }
  //     double speed = std::fabs(s->at(3));
  //     double safe_distance = calculate_safe_distance<decltype(params)>(speed, params["plan_safety_params"]);
  //     if (min_distance < safe_distance)
  //     {
  //       return false;
  //     }
  //   }
  //   return true;
  // };

  // dirt_spec->contingency_check = [&](prx::trajectory_t& traj) {
  //   for (auto&& s : traj)
  //   {
  //     auto pqp_distance = prx::default_obstacle_distance_function(s, ss, cg);
  //     double min_distance = std::numeric_limits<double>::max();
  //     for (auto&& d : pqp_distance.distances)
  //     {
  //       if (d < min_distance)
  //       {
  //         min_distance = d;
  //       }
  //     }
  //     double speed = std::fabs(s->at(3));
  //     double safe_distance = calculate_safe_distance<decltype(params)>(speed, params["contingency_safety_params"]);
  //     if (min_distance < safe_distance)
  //     {
  //       return false;
  //     }
  //   }
  //   return true;
  // };

  // geometry_msgs::Pose2D goal_configuration;
  // goal_configuration.x = goal_config[0];
  // goal_configuration.y = goal_config[1];
  // goal_configuration.theta = goal_config[2];

  // std_msgs::Float64 goal_radius;
  // goal_radius.data = params["goal_region_radius"].as<double>();

  // prx::dirt_replan_query_t* dirt_query = new prx::dirt_replan_query_t(ss, cs);
  // dirt_query->start_state = ss->make_point();
  // dirt_query->goal_state = ss->make_point();
  // dirt_query->goal_region_radius = goal_radius.data;
  // dirt_query->get_visualization = false;
  // ROS_WARN("Using default goal check");

  // dirt->link_and_setup_spec(dirt_spec);
  // dirt->preprocess();
  // dirt->link_and_setup_query(dirt_query);

  // ros::AsyncSpinner spinner(4);

  // using PlannerService = mj_ros::planner_service_t<prx::dirt_replan_t, prx::dirt_replan_specification_t*,
  //                                                  prx::dirt_replan_query_t*, prx_models::MushrPlanner>;
  // using PlannerClient = mj_ros::planner_client_t<prx_models::MushrPlanner, prx_models::mushr_stela_t>;

  // PlannerService planner_service(private_nh, dirt, dirt_spec, dirt_query, propagate_dynamics, retain_previous);

  // // Pass the safe distance calculation function to the planner service
  // planner_service.set_safe_distance_calculator([&params](double speed) {
  //   return calculate_safe_distance<decltype(params)>(speed, params["contingency_safety_params"]);
  // });

  // PlannerClient planner_client(private_nh, cs->get_dimension());

  // ros::Publisher goal_pos_publisher = n.advertise<geometry_msgs::Pose2D>(root + "/goal_pose", 10, true);
  // ros::Publisher goal_radius_publisher = n.advertise<std_msgs::Float64>(root + "/goal_radius", 10, true);
  // ros::Publisher safety_radius_publisher = n.advertise<std_msgs::Float64>(root + "/safety_radius", 10, true);
  // ros::Publisher planning_result_publisher =
  //     n.advertise<motion_planning::PlanningResult>(root + "/planning_result", 1, true);
  // ros::Publisher reset_publisher = n.advertise<std_msgs::Empty>(root + "/reset", 1, true);
  // motion_planning::PlanningResult planning_result_msg;

  // ros::ServiceClient collision_client = n.serviceClient<mujoco_ros::Collision>(root + "/collision");
  // mujoco_ros::Collision collision_srv;

  // int max_cycles;
  // double planning_cycle_duration_double, preprocess_timeout, postprocess_timeout;
  // n.getParam(ros::this_node::getName() + "/planning_cycle_duration", planning_cycle_duration_double);
  // n.getParam(ros::this_node::getName() + "/preprocess_timeout", preprocess_timeout);
  // n.getParam(ros::this_node::getName() + "/postprocess_timeout", postprocess_timeout);
  // n.getParam(ros::this_node::getName() + "/max_cycles", max_cycles);
  // planner_service.set_preprocess_timeout(preprocess_timeout);
  // planner_service.set_postprocess_timeout(postprocess_timeout);
  // dirt_spec->planning_cycle_duration = planning_cycle_duration_double;

  // ros::Duration planning_cycle_duration_ros(planning_cycle_duration_double);

  // ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

  /*

  spinner.start();
  goal_pos_publisher.publish(goal_configuration);
  goal_radius_publisher.publish(goal_radius);

  planner_client.call_service(goal_configuration, goal_radius, 0, 2 * planning_cycle_duration_double);

  ros::Time experiment_start_time = planner_client.get_experiment_start_time();

  int current_cycle = 1;
  PRINT_MSG("Replanner initialized");

  while (ros::ok() && current_cycle <= max_cycles)
  {
    if (!planner_client.is_goal_reached(goal_configuration, goal_radius))
    {
      ros::Time current_time = ros::Time::now();
      if (current_time >= experiment_start_time + (planning_cycle_duration_ros * (current_cycle - 1)))
      {
        ROS_INFO("Cycle %d Elapsed Time: %f", current_cycle,
                 (current_time - planner_client.get_experiment_start_time()).toSec());
        planner_client.call_service(goal_configuration, goal_radius, current_cycle, planning_cycle_duration_double);
        current_cycle++;
        ROS_INFO("Preprocess time: %f", planner_service.get_preprocess_time() - planner_client.get_preprocess_time());
        ROS_INFO("Query fulfill time: %f",
                 planner_client.get_query_fulfill_time() - planner_service.get_query_fulfill_time());
      }
    }
    else
    {
      ROS_WARN("Goal reached, not replanning");
      planning_result_msg.goal_reached.data = true;
      planning_result_msg.in_collision.data = false;
      planning_result_msg.total_time.data = current_cycle * planning_cycle_duration_double;
      planning_result_publisher.publish(planning_result_msg);
      break;
    }
    if (collision_client.call(collision_srv))
    {
      if (collision_srv.response.collision_result.data)
      {
        ROS_WARN("Collision detected");
        planning_result_msg.goal_reached.data = false;
        planning_result_msg.in_collision.data = true;
        planning_result_msg.total_time.data = current_cycle * planning_cycle_duration_double;
        planning_result_publisher.publish(planning_result_msg);
        std_msgs::Empty reset_msg;
        reset_publisher.publish(reset_msg);
        break;
      }
    }
    ros::spinOnce();
  }

  if (current_cycle > max_cycles && !planner_client.is_goal_reached(goal_configuration, goal_radius))
  {
    ROS_WARN("Goal not reached");
    planning_result_msg.goal_reached.data = false;
    planning_result_msg.total_time.data = current_cycle * planning_cycle_duration_double;
    planning_result_publisher.publish(planning_result_msg);
  }

  int id;
  n.getParam(ros::this_node::getName() + "/id", id);

  /*
  auto error_data = planner_client.get_error_data();
  prx::space_point_t print_state = ss->make_point();

  std::ofstream error_file;
  error_file.open("/home/aravind/error_data_" + std::to_string(id) + ".txt");
  for (unsigned i = 0; i < std::get<0>(error_data).size(); i++)
  {
    ml4kp_bridge::copy(print_state, std::get<0>(error_data)[i]);
    error_file << ss->print_point(print_state) << ", ";
    ml4kp_bridge::copy(print_state, std::get<1>(error_data)[i]);
    error_file << ss->print_point(print_state) << ", ";
    prx_models::copy(print_state, std::get<2>(error_data)[i]);
    error_file << ss->print_point(print_state) << std::endl;
  }

  spinner.stop();
  */

  return 0;
}