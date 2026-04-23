#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "control/MushrControlPropagation.h"
#include "motion_planning/replanner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "motion_planning/PlanningResult.h"
#include "mujoco_ros/Collision.h"
#include "std_msgs/Empty.h"
#include <fstream>
#include <prx/utilities/general/type_conversions.hpp>
#include <utils/std_utils.hpp>
#include <motion_planning/tree_bridge.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <interface/ReplannerStatus.h>
#include <interface/PlannerClock.h>

#include <prx_models/StelaKraft.h>

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

  ros::Timer _clock_timer, _replan_timer, _tree_timer;

  int _max_cycles;
  double _preprocess_timeout, _postprocess_timeout;

  double _max_edge_duration;

  prx::space_point_t _future_state;  // start for the planner

  bool _z_received, _start_replanning;

  int _current_cycle;
  ros::Time _cycle_end, _cycle_start;
  ros::Duration _cycle_duration;

  // bool _propagate_dynamics, retain_previous, use_contingency;
  interface::ReplannerStatus _status;

  std::shared_ptr<prx::trajectory_t> _step_traj;
  std::shared_ptr<prx::plan_t> _step_plan, _rest_of_plan;

  ros::Time _preprocess_end_time, _query_fulfill_start_time;

  ros::ServiceServer _replanning_service;

  std::string _environment;

  bool _new_tree;
  prx::tree_t _full_tree;

  std::size_t _tot_replans;
  std::string _tree_file_prefix;

  replanner_t(ros::NodeHandle& nh) : _z_received(false), _cycle_start(ros::Time::ZERO), _new_tree(false)
  {
    std::string params_file, estimation_tree_topic, planner_clock_topic;
    std::string sbmp_solution_tree_topic, sbmp_full_tree_topic;
    std::string& heuristic_map_filename{ _heuristic_map_filename };

    int& max_cycles{ _max_cycles };

    double& preprocess_timeout{ _preprocess_timeout };
    double& postprocess_timeout{ _postprocess_timeout };
    double& max_edge_duration{ _max_edge_duration };

    std::string& environment{ _environment };

    // std::string& _tree_file_prefix{};

    PARAM_SETUP(nh, params_file);
    PARAM_SETUP(nh, heuristic_map_filename);

    PARAM_SETUP(nh, preprocess_timeout);
    PARAM_SETUP(nh, postprocess_timeout);
    PARAM_SETUP(nh, max_cycles);

    PARAM_SETUP(nh, estimation_tree_topic);
    PARAM_SETUP(nh, planner_clock_topic);

    PARAM_SETUP(nh, sbmp_full_tree_topic);
    PARAM_SETUP(nh, sbmp_solution_tree_topic);
    PARAM_SETUP(nh, environment);

    PARAM_SETUP_WITH_DEFAULT(nh, max_edge_duration, max_edge_duration);

    DEBUG_VARS(environment)
    // planner_replanning_service.set_preprocess_timeout(preprocess_timeout);
    // planner_replanning_service.set_postprocess_timeout(postprocess_timeout);

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

    init_planner_spec(nh);
    init_planner_query();
    init_heuristic_map();

    // get an initial plan
    _dirt = std::make_shared<prx::dirt_replan_t>("dirt");
    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());

    // Subscribers
    // _z_tree_subscriber = nh.subscribe(estimation_tree_topic, 10, &replanner_t::observation_callback, this);
    // _planner_clock_subscriber = nh.subscribe(planner_clock_topic, 1, &replanner_t::clock_callback, this);

    // Timers
    const ros::Duration timer_duration(0.01);
    // _clock_timer = nh.createTimer(timer_duration, &replanner_t::timer_callback, this);
    _tree_timer = nh.createTimer(timer_duration, &replanner_t::tree_publish_callback, this);

    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);

    _replanning_service = nh.advertiseService("/kraft/replan", &replanner_t::replan, this);

    dbg::set_log_filename("log_replanning.txt");
    PRINT_MSG("Replanner Ready!");
  }

  // void timer_callback(const ros::TimerEvent& event)
  // {
  // if (_cycle_start.isZero())
  // {
  //   return;
  // }
  // else if (_cycle_start <= event.current_real and event.current_real < _cycle_end)
  // {
  //   return;
  // }
  // else if (_cycle_end <= event.current_real)
  // {
  //   _cycle_start = _cycle_end;
  //   _cycle_end += _cycle_duration;
  //   _start_replanning = true;
  //   // DEBUG_VARS(ros::Time::now(), _cycle_start, _cycle_end);
  // }
  // else
  // {
  //   PRINT_MSG("Out of cycle!");
  //   DEBUG_VARS(ros::Time::now(), _cycle_start, _cycle_end);
  // }
  // }
  // {}
  // const bool start_replanning{_cycle_start};
  // DEBUG_VARS(_start_replanning, _z_received)
  // if (_start_replanning and _z_received)
  void tree_publish_callback(const ros::TimerEvent& event)
  {
    if (_new_tree)
    {
      // PRINT_MSG("FULL TREE PUBLISHING!")
      // _new_tree = false;
      // prx_models::Tree ros_tree;
      // motion_planning::copy<prx::dirt_replan_t::Node, prx::dirt_replan_t::Edge>(ros_tree, _full_tree);
      // _tree_publisher.publish(ros_tree);
      // _full_tree.purge();
      // PRINT_MSG("FULL TREE PUBLISHED!!!!!!!")
    }
  }

  // void clock_callback(const interface::PlannerClockConstPtr msg)
  // {
  //   // if (_next_replan_end.isZero())
  //   // {
  //   //   _next_replan_start = msg->cycle_end;
  //   //   // _next_replan_end = msg->cycle_end;
  //   // }
  //   if (_current_cycle != msg->cycle)
  //   {
  //     _current_cycle = msg->cycle;
  //     _cycle_end = msg->cycle_end;
  //     _cycle_start = msg->cycle_start;
  //     _cycle_duration = msg->cycle_duration;

  //     // _cycle_update = true;
  //     // _start_replanning = true;
  //     // DEBUG_VARS(*msg);
  //   }
  // }

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

    // auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
    auto obstacles = prx::load_obstacles(_environment);
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

    _heuristic_map = std::make_shared<prx::heuristic_map_t>(-0.5, 2.5, -1.0, 6.0, 0.01, 0.01, heuristic_context);
    _heuristic_map->set_obstacle_grid();
    _heuristic_map->set_heuristic_grid(_dirt_query->goal_state);
    _heuristic_map->set_u_rep_coeff(params["u_rep"].as<double>());

    std::ofstream file(_heuristic_map_filename.c_str());
    file << *_heuristic_map;
    file.close();

    // _dirt_spec->wavefront_h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
    //   return _heuristic_map->get_cost(s);
    // };

    _dirt_spec->heuristic = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
      return _dirt_spec->distance_function(s, s2) / 0.62;
    };
  }

  void init_planner_query()
  {
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

  void tree_from_plan_traj(prx_models::Tree& sln_tree, const prx::plan_t& plan, const prx::trajectory_t& traj)
  {
    if (traj.duration() < plan.duration())
    {
      PRINT_MSG("[Replanner::tree_from_plan_traj] Trajectory shorter than plan");
      DEBUG_VARS(traj.duration(), plan.duration())
      // DEBUG_VARS(plan)
      // DEBUG_VARS(traj)
      return;
    }

    std::size_t current_idx{ sln_tree.root + 1 };
    // sln_tree.root = current_idx;
    double ti{ 0.0 };
    double curr_cost{ 0.0 };

    for (std::size_t i = 0; i < plan.size(); ++i)
    {
      const prx::plan_step_t ps_i{ plan[i] };

      double dt_remaining{ ps_i.duration };
      while (dt_remaining > 0.0001)  // small epsilon
      {
        // DEBUG_VARS(dt_remaining)
        motion_planning::EdgeNodePair edge_node{ motion_planning::create_edge_node(sln_tree.nodes.back(),
                                                                                   current_idx) };

        const double dt_curr{ std::min(_max_edge_duration, dt_remaining) };
        // DEBUG_VARS(ti, dt_curr)
        const prx::space_point_t xi{ traj.at(ti + dt_curr, false) };
        edge_node.first.plan.steps.emplace_back();

        ml4kp_bridge::copy(edge_node.first.plan.steps.back(), ps_i);
        edge_node.first.plan.steps.back().duration.data = ros::Duration(dt_curr);

        ml4kp_bridge::copy(edge_node.second.point, xi);

        sln_tree.edges.push_back(edge_node.first);
        sln_tree.nodes.push_back(edge_node.second);

        dt_remaining = dt_remaining - _max_edge_duration;
        ti += dt_curr;
      }
      // ti += ps_i.duration;
    }
  }

  void change_status(const int16_t new_status)
  {
    _status.header.stamp = ros::Time::now();
    _status.state = new_status;
    _status_publisher.publish(_status);
    // DEBUG_VARS(_status.state);
  }

  void plan_to_file(const prx::plan_t& plan)
  {
    const std::string filename{ _tree_file_prefix + prx::utilities::convert_to<std::string>(_tot_replans) };
    std::ofstream ofs(filename);
  }

  // void replan()
  bool replan(prx_models::StelaKraft::Request& request, prx_models::StelaKraft::Response& response)
  {
    LOG_MSG("START REPLANNING");
    response.planner_output = prx_models::StelaKraft::Response::TYPE_FAILURE;

    LOG_MSG("PREPROCESSING");
    // DEBUG_VARS(ros::Time::now(), request);
    change_status(interface::ReplannerStatus::PREPROCESSING);
    _dirt_query->clear_outputs();
    // const double& planning_duration{ _dirt_spec->planning_cycle_duration };

    prx_assert(_dirt_query->start_state->size() == request.root.point.point.size(),
               "[mushr_replanning] Size "
               "mismatch ");
    // if (request.root.point.point.size() != _dirt_query->start_state->size())
    // {
    //   prx_warn("Start state of ");
    //   return false;
    // }
    ml4kp_bridge::copy(_dirt_query->start_state, request.root.point);
    // _state_space->copy(_dirt_query->start_state, _future_state);
    // LOG_VARS(_dirt_query->start_state);
    // LOG_VARS(*_dirt_spec)
    // LOG_VARS(*_dirt_query)
    _step_traj->clear();

    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());

    // const double preprocess_real_dt{ (ros::Time::now() - _cycle_start).toSec() };
    // const double time_limit{ planning_duration - preprocess_real_dt - _postprocess_timeout };
    // DEBUG_VARS(time_limit, planning_duration, preprocess_real_dt, _postprocess_timeout);
    const ros::Time start_plan_stamp{ ros::Time::now() };
    const ros::Duration dt_available{ request.deadline - start_plan_stamp };
    const double time_limit{ dt_available.toSec() - _postprocess_timeout };

    LOG_VARS(request.deadline, dt_available, time_limit);
    if (time_limit <= 0)
    {
      change_status(interface::ReplannerStatus::IDLE);
      return true;
    }
    // prx_assert(time_limit > 0, "Time limit is less than 0");
    prx::condition_check_t checker("time", time_limit);

    change_status(interface::ReplannerStatus::PLANNING);
    LOG_MSG("PLANNING");

    _dirt->resolve_query(&checker);

    const ros::Time end{ ros::Time::now() };
    const double real_plan_dt{ (end - start_plan_stamp).toSec() };
    const double dt_diff{ time_limit - real_plan_dt };
    change_status(interface::ReplannerStatus::POSTPROCESSING);

    LOG_VARS(real_plan_dt, dt_diff);
    LOG_MSG("POSTPROCESSING");

    _dirt->fulfill_query();

    // prx::space_point_t current_state = _spec->state_space->make_point();
    // double execution_time = request.planning_duration.data.toSec();
    if (_dirt_query->solution_traj.size() > 0)
    {
      // PRINT_MSG("Solution found");
      // double plan_duration = _dirt_query->solution_cost;
      _step_plan->clear();
      _rest_of_plan->clear();
      const double plan_duration{ _dirt_query->solution_plan.duration() };
      const double traj_duration{ _dirt_query->solution_traj.duration() };

      plan_to_file(_dirt_query->solution_plan);
      // DEBUG_VARS(plan_duration, traj_duration);
      if (plan_duration < request.solution_duration.toSec())
      {
        _dirt_query->solution_plan.copy_to(0, plan_duration, *_step_plan);
        // _dirt_query->solution_plan.copy_to(planning_duration, _dirt_query->solution_plan.duration(), *_rest_of_plan);
      }
      else
      {
        _dirt_query->solution_plan.copy_to(0, request.solution_duration.toSec(), *_step_plan);
      }

      // bool valid{ true };
      response.sln_tree.root = request.root.index;
      response.sln_tree.nodes.push_back(request.root);
      response.planner_output = prx_models::StelaKraft::Response::TYPE_SUCCESS;

      tree_from_plan_traj(response.sln_tree, *_step_plan, _dirt_query->solution_traj);
      _sln_tree_publisher.publish(response.sln_tree);
      LOG_MSG("Result ready");
    }

    prx_models::Tree ros_tree;
    motion_planning::copy<prx::dirt_replan_t::Node, prx::dirt_replan_t::Edge>(ros_tree, _dirt->tree());
    _tree_publisher.publish(ros_tree);
    _dirt->reset();

    LOG_MSG("IDLE");
    change_status(interface::ReplannerStatus::IDLE);
    // if (!_retain_previous)
    //   _dirt_query->clear_outputs();
    _tot_replans++;
    // return true;
    return true;
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

  return 0;
}