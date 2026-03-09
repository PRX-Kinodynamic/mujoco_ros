#include <ml4kp_bridge/defs.h>
#include <memory>
#include <prx/planning/planners/dirt_replanning.hpp>
#include <prx/planning/planners/planner.hpp>
#include <prx/utilities/general/condition_check.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx_models/defs.hpp>
#include "ml4kp_bridge/Trajectory.h"
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "control/MushrControlPropagation.h"
#include "motion_planning/replanner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "motion_planning/PlanningResult.h"
#include "mujoco_ros/Collision.h"
#include "std_msgs/Empty.h"
#include "utils/dbg_utils.hpp"
#include "utils/rosparams_utils.hpp"
#include <fstream>
#include <prx/utilities/general/type_conversions.hpp>
#include <utils/std_utils.hpp>
#include <motion_planning/tree_bridge.hpp>

#include <prx_models/mushr.hpp>
#include <prx_models/planner_utils.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <interface/ReplannerStatus.h>
#include <interface/PlannerClock.h>

#include <prx_models/StelaKraft.h>
#include <prx_models/tree_utils.hpp>
#include <interface/ExperimentParams.h>
#include <interface/NodeStatus.h>
#include <interface/node_status.hpp>

// Function to calculate safe distance based on speed
template <typename ParamsType>
double calculate_safe_distance(double speed, const ParamsType& params)
{
  // return 0.5;//
  return params["safe_min"].template as<double>() + params["safe_mul"].template as<double>() * std::fabs(speed) +
         params["safe_quad_mul"].template as<double>() * std::pow(std::fabs(speed), 2);
}

struct replanner_t
{
  // enum planning_mode_t
  // {
  //   REPLANNING = 0,
  //   SINGLE_SHOT,
  //   FINISHED
  // };
  //
  ml4kp_bridge::Trajectory _traj_msg;

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
  ros::Publisher _planner_stats_publisher;
  ros::Publisher _sln_traj_publisher;

  ros::Subscriber _z_tree_subscriber;
  ros::Subscriber _planner_clock_subscriber;

  ros::Timer _clock_timer, _replan_timer, _tree_timer;

  // int _max_cycles;
  double _postprocess_timeout;

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

  double _safe_distance;

  prx::param_loader _env_params;
  prx::param_loader _plant_params, _dirt_spec_params, _dirt_query_params;

  replanner_t(ros::NodeHandle& nh) : _z_received(false), _cycle_start(ros::Time::ZERO), _new_tree(false)
  {
    DEBUG_PRINT
    LOG_FILENAME("logs/replanner.txt");
    LOG_MSG("Replanner Initialized")

    std::string params_file;
    std::string sbmp_solution_tree_topic, sbmp_full_tree_topic;
    std::string planner_stats_topic_name;
    std::string& heuristic_map_filename{ _heuristic_map_filename };
    DEBUG_PRINT

    // int& max_cycles{ _max_cycles };

    // double& preprocess_timeout{ _preprocess_timeout };
    double& postprocess_timeout{ _postprocess_timeout };
    double& max_edge_duration{ _max_edge_duration };

    // std::string planning_mode;
    std::string& environment{ _environment };

    std::string plant_parameters, dirt_spec, dirt_query;

    DEBUG_PRINT
    using prx::simulation_step;
    int random_seed;
    // PARAM_SETUP(nh, plant_file);
    // PARAM_SETUP(nh, params_file);
    // PARAM_SETUP(nh, planning_mode);
    // PARAM_SETUP(nh, planner_sln_recovery_type);

    // PARAM_SETUP(nh, preprocess_timeout);
    // PARAM_SETUP(nh, max_cycles);

    // PARAM_SETUP(nh, estimation_tree_topic);

    DEBUG_PRINT
    PARAM_SETUP(nh, heuristic_map_filename);
    PARAM_SETUP(nh, postprocess_timeout);
    PARAM_SETUP(nh, sbmp_full_tree_topic);
    PARAM_SETUP(nh, sbmp_solution_tree_topic);
    PARAM_SETUP(nh, planner_stats_topic_name);

    DEBUG_VARS(heuristic_map_filename);
    DEBUG_VARS(postprocess_timeout);
    DEBUG_VARS(sbmp_full_tree_topic);
    DEBUG_VARS(sbmp_solution_tree_topic);
    DEBUG_VARS(planner_stats_topic_name);

    // PARAM_SETUP(nh, sbmp_solution_tree_topic);
    // PARAM_SETUP(nh, environment);

    DEBUG_PRINT
    GLOBAL_PARAM_SETUP(random_seed);
    GLOBAL_PARAM_SETUP(simulation_step);
    GLOBAL_PARAM_SETUP(plant_parameters);
    GLOBAL_PARAM_SETUP(dirt_spec);
    GLOBAL_PARAM_SETUP(dirt_query);
    GLOBAL_PARAM_SETUP(environment);

    PARAM_SETUP_WITH_DEFAULT(nh, max_edge_duration, max_edge_duration);

    prx::init_random(random_seed);

    // mode_check(planning_mode);

    DEBUG_PRINT
    // std::string plan_params_file;
    // PARAM_SETUP_WITH_DEFAULT(nh, plan_params_file, plan_params_file){ prx::param_loader(plan_params_file, "") };
    // params = prx::param_loader(params_file, "");
    // _plant_params = prx::param_loader(plant_file, "");
    _dirt_spec_params.from_string(dirt_spec);
    _dirt_query_params.from_string(dirt_query);
    _plant_params.from_string(plant_parameters);
    _env_params.from_string(_environment);

    // params["solution_type"].set(planner_sln_recovery_type);

    DEBUG_PRINT
    // Publisher
    _goal_pos_publisher = nh.advertise<geometry_msgs::Pose2D>("/kraft/goal_pose", 10, true);
    _goal_radius_publisher = nh.advertise<std_msgs::Float64>("/kraft/goal_radius", 10, true);
    _safety_radius_publisher = nh.advertise<std_msgs::Float64>("/kraft/safety_radius", 10, true);
    _planning_result_publisher = nh.advertise<motion_planning::PlanningResult>("/kraft/planning_result", 1, true);
    _reset_publisher = nh.advertise<std_msgs::Empty>("/kraft/reset", 1, true);
    _status_publisher = nh.advertise<interface::ReplannerStatus>("/kraft/status", 1, true);
    _sln_traj_publisher = nh.advertise<ml4kp_bridge::Trajectory>("/kraft/solution/trajectory", 1, true);
    _tree_publisher = nh.advertise<prx_models::Tree>(sbmp_full_tree_topic, 1, true);
    _sln_tree_publisher = nh.advertise<prx_models::Tree>(sbmp_solution_tree_topic, 1, true);
    _planner_stats_publisher = nh.advertise<prx_models::PlannerStats>(planner_stats_topic_name, 1, true);

    _status.state = interface::ReplannerStatus::INITIALIZING;
    _status_publisher.publish(_status);

    DEBUG_PRINT
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
    // _planner_clock_subscriber = nh.subscrib, 1, &replanner_t::clock_callback, this);

    // Timers
    const ros::Duration timer_duration(0.01);
    // _clock_timer = nh.createTimer(timer_duration, &replanner_t::timer_callback, this);
    _tree_timer = nh.createTimer(timer_duration, &replanner_t::tree_publish_callback, this);

    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);

    _replanning_service = nh.advertiseService("/kraft/replan", &replanner_t::replan, this);

    // LOG_FILENAME("logs/stela.txt");

    PRINT_MSG("Replanner Ready!");
  }

  ~replanner_t()
  {
    PRINT_MSG("Shutting down replanner...");
    _replanning_service.shutdown();
  }

  static void create_parameter_files(ros::NodeHandle& nh)
  {
    bool initialize_parameter_files{ false };
    PARAM_SETUP_WITH_DEFAULT(nh, initialize_parameter_files, initialize_parameter_files);
    DEBUG_VARS(initialize_parameter_files)
    if (initialize_parameter_files)
    {
      PRINT_MSG("Creating parameter files")
      std::string output_directory, plant_name;
      PARAM_SETUP(nh, output_directory);
      PARAM_SETUP(nh, plant_name);

      const std::string spec_file{ output_directory + "/dirt_replan_spec.yaml" };
      const std::string query_file{ output_directory + "/dirt_replan_query.yaml" };
      const std::string plant_file{ output_directory + "/" + plant_name + ".yaml" };

      prx::param_loader::create_file<prx::dirt_replan_specification_t>(spec_file);
      prx::param_loader::create_file<prx::dirt_replan_query_t>(query_file);
      prx::system_factory_t::initialization_parameters(plant_name).save(plant_file);

      DEBUG_VARS(spec_file)
      DEBUG_VARS(query_file)
      DEBUG_VARS(plant_file)
      ros::shutdown();
    }
  }

  void tree_publish_callback(const ros::TimerEvent& event)
  {
    if (_new_tree)
    {
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
    _plant = prx::system_factory_t::create_system(_plant_params);
    // const std::string plant_name{ plant_params["name"].as<std::string>() };
    // const std::string plant_path{ plant_params["path"].as<std::string>() };
    // _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    // _plant->init(plant_params);
    DEBUG_VARS(_plant);
    // prx_assert(_plant != nullptr, "Failed to create plant");

    std::tie(_planning_model, _system_group, _collision_group) = prx::world_model_t::create(_env_params, _plant);

    _state_space = _system_group->get_state_space();
    _control_space = _system_group->get_control_space();
    _param_space = _system_group->get_parameter_space();

    _step_plan = std::make_shared<prx::plan_t>(_control_space);
    _rest_of_plan = std::make_shared<prx::plan_t>(_control_space);
    _step_traj = std::make_shared<prx::trajectory_t>(_state_space);

    _future_state = _state_space->make_point();
    _dirt_spec = std::make_shared<prx::dirt_replan_specification_t>(_system_group, _collision_group);
    _dirt_spec->init(_dirt_spec_params);

    if (_dirt_spec_params["f_type"].as<>() == "f=g+h")
    {
      _dirt_spec->f_function = [&](const double& g, const double& h) { return g + h; };
    }
    else if (_dirt_spec_params["f_type"].as<>() == "f=h")
    {
      _dirt_spec->f_function = [&](const double& g, const double& h) { return h; };
    }
    else
    {
      prx_throw("Unknown PlannerSpec/f_type");
    }

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
        // TODO: Change this
        _safe_distance = calculate_safe_distance<decltype(params)>(speed, _dirt_spec_params["plan_safety_params"]);
        if (min_distance < _safe_distance)
        {
          return false;
        }
      }
      return true;
    };

    DEBUG_VARS(*_dirt_spec);
  }

  void init_heuristic_map()
  {
    // _collision_group
    auto heuristic_plant = prx::system_factory_t::create_system("2D_Point", "2D_Point");
    prx_assert(heuristic_plant != nullptr, "Failed to create plant");
    // prx::world_model_t heuristic_model({ heuristic_plant }, { _obstacle_list });
    // heuristic_model.create_context("heuristic_context", { "2D_Point" }, { _obstacle_names });
    // auto heuristic_context = heuristic_model.get_context("heuristic_context");
    auto [h_pm, h_sg, h_cg] = prx::world_model_t::create(_env_params, heuristic_plant);

    // std::vector<double> goal_config = params["goal_state"].as<std::vector<double>>();

    // std::cout << "Goal: " << ss->print_point(goal) << std::endl;

    // prx::heuristic_map_t heuristic_map(-0.5, 2.5, -1.0, 6.0, 0.01, 0.01, heuristic_context);
    // heuristic_map_t(double x_min, double x_max, double y_min, double y_max, double x_step, double y_step,
    prx::world_model_context context = { h_sg, h_cg };
    _heuristic_map = std::make_shared<prx::heuristic_map_t>(-0.5, 2.5, -1.0, 6.0, 0.01, 0.01, context);
    _heuristic_map->set_obstacle_grid();
    _heuristic_map->set_heuristic_grid(_dirt_query->goal_state);
    // _heuristic_map->set_u_rep_coeff(params["u_rep"].as<double>());
    _heuristic_map->set_u_rep_coeff(1.5);

    std::ofstream file(_heuristic_map_filename.c_str());
    file << *_heuristic_map;
    file.close();

    _dirt_spec->wavefront_h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
      return _heuristic_map->get_cost(s);
    };

    // int row = (0.0 - (-1.0)) / 0.01 = 100
    // int col = (1.0 - (-0.5)) / 0.01 = 150
    _dirt_spec->heuristic = [&](const prx::space_point_t& curr, const prx::space_point_t& goal) {
      return _heuristic_map->get_cost(curr) / 0.3;
      // return _dirt_spec->distance_function(s, s2) / 0.62;
      // return _dirt_spec->distance_function(s, s2) / 0.62;
      // return 0.0;
    };
  }

  void init_planner_query()
  {
    // _goal = _state_space->make_point(params["goal/state"]);
    // _goal_radius =

    // DEBUG_VARS(_goal, _goal_radius);

    _dirt_query = std::make_shared<prx::dirt_replan_query_t>(_state_space, _control_space);

    _dirt_query->init(_dirt_query_params);

    // std::shared_ptr<mushr_types::State::type> goal_state{ std::make_shared<mushr_types::State::type>() };

    _dirt_query->goal_check = [&](prx::space_point_t s) {  // return default_goal_check(s, goal_state,
                                                           // goal_region_radius);
      prx_models::mushr_types::State::type xi, xg;
      xi[0] = s->at(0);
      xi[1] = s->at(1);
      xi[2] = s->at(2);
      xg[0] = _dirt_query->goal_state->at(0);
      xg[1] = _dirt_query->goal_state->at(1);
      xg[2] = _dirt_query->goal_state->at(2);
      // const mushr_types::State::type between{ goal_state->between() };
      const prx_models::mushr_types::State::type between{ xi.between(xg) };
      const Eigen::Vector3d error{ prx_models::mushr_types::State::type::Logmap(between) };
      return error.norm() < _dirt_query->goal_region_radius;
    };
    // _dirt_query->start_state = _state_space->make_point(plant_params["/start_state"]);
    // _dirt_query->goal_state = _state_space->make_point(params["/goal/state"]);
    // _dirt_query->goal_region_radius = params["goal/radius"].as<double>();
    // _dirt_query->get_visualization = false;

    DEBUG_VARS(*_dirt_query)
    // DEBUG_VARS(_dirt_query->goal_state)
    // DEBUG_VARS(_dirt_query->goal_region_radius)
    // ROS_WARN("Using default goal check");
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

  prx::condition_check_t create_condition(prx_models::StelaKraft::Request& request)
  {
    if (request.condition == prx_models::StelaKraft::Request::CONDITION_ITERATIONS)
    {
      LOG_VARS(request.iterations);

      return prx::condition_check_t("iterations", request.iterations);
    }
    else if (request.condition == prx_models::StelaKraft::Request::CONDITION_TIME)
    {
      const ros::Time start_plan_stamp{ ros::Time::now() };
      const ros::Duration dt_available{ request.deadline - start_plan_stamp };
      const double time_limit{ std::max(dt_available.toSec() * _postprocess_timeout, 0.0) };

      LOG_VARS(request.deadline, dt_available, time_limit);

      return prx::condition_check_t("time", time_limit);
    }
    prx_throw("Unknown condition check")
  }

  // void replan()
  bool replan(prx_models::StelaKraft::Request& request, prx_models::StelaKraft::Response& response)
  {
    LOG_MSG("START REPLANNING");
    response.planner_output = prx_models::StelaKraft::Response::TYPE_FAILURE;

    LOG_MSG("PREPROCESSING");
    change_status(interface::ReplannerStatus::PREPROCESSING);

    // _dirt_spec->planning_cycle_duration = 1.0;  // TODO: Is this necessary?

    _dirt_query->clear_outputs();

    prx_assert(_dirt_query->start_state->size() == request.root.point.point.size(),
               "[mushr_replanning] Size "
               "mismatch ");

    ml4kp_bridge::copy(_dirt_query->start_state, request.root.point);

    _traj_msg.data.clear();
    _step_traj->clear();

    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());

    const ros::Time start_plan_stamp{ ros::Time::now() };
    const ros::Duration dt_available{ request.deadline - start_plan_stamp };

    double time_limit{ dt_available.toSec() - _postprocess_timeout };

    prx::condition_check_t checker{ create_condition(request) };

    change_status(interface::ReplannerStatus::PLANNING);
    LOG_MSG("PLANNING");

    _dirt->resolve_query(&checker);

    // const ros::Time end{ ros::Time::now() };
    // const double real_plan_dt{ (end - start_plan_stamp).toSec() };
    // const double dt_diff{ time_limit - real_plan_dt };
    change_status(interface::ReplannerStatus::POSTPROCESSING);

    LOG_MSG("POSTPROCESSING");

    _dirt->fulfill_query();

    std::shared_ptr<prx::planner_t::statistics_t> planner_stats{ _dirt->statistics() };
    // std::cout << planner_stats << std::endl;
    std::shared_ptr<prx::dirt_replan_t::statistics_t> stats{
      std::dynamic_pointer_cast<prx::dirt_replan_t::statistics_t>(planner_stats)
    };
    prx_assert(stats != nullptr, "Couldn't cast stats to dirt stats");
    prx_models::copy(response.stats, *stats);
    _planner_stats_publisher.publish(response.stats);
    LOG_VARS(_dirt_query->solution_traj.size());
    if (_dirt_query->solution_traj.size() > 0)
    {
      _step_plan->clear();
      _rest_of_plan->clear();
      const double plan_duration{ _dirt_query->solution_plan.duration() };
      const double traj_duration{ _dirt_query->solution_traj.duration() };
      ml4kp_bridge::copy(_traj_msg, _dirt_query->solution_traj);

      // if (_mode == planning_mode_t::REPLANNING)
      // {
      // plan_to_file(_dirt_query->solution_plan);
      // DEBUG_VARS(plan_duration, traj_duration);
      if (plan_duration < request.solution_duration.toSec())
      {
        _dirt_query->solution_plan.copy_to(0, plan_duration, *_step_plan);
        // _dirt_query->solution_plan.copy_to(planning_duration, _dirt_query->solution_plan.duration(),
        // *_rest_of_plan);
      }
      else
      {
        _dirt_query->solution_plan.copy_to(0, request.solution_duration.toSec(), *_step_plan);
      }
      // }
      // else if (_mode == planning_mode_t::SINGLE_SHOT)
      // {
      //   (*_step_plan) += _dirt_query->solution_plan;
      //   _mode = planning_mode_t::FINISHED;
      // }
      // else
      // {
      //   prx_throw("[replanner_t] Invalid mode (unreachable line?)");
      // }
      response.sln_tree.root = request.root.index;
      response.sln_tree.nodes.push_back(request.root);
      response.planner_output = prx_models::StelaKraft::Response::TYPE_SUCCESS;

      prx_models::tree_from_plan_traj(response.sln_tree, *_step_plan, _dirt_query->solution_traj, _max_edge_duration);
      _sln_tree_publisher.publish(response.sln_tree);
      // LOG_MSG("Result ready");
    }

    LOG_MSG("POSTPROCESSING");

    _sln_tree_publisher.publish(response.sln_tree);
    _sln_traj_publisher.publish(_traj_msg);

    prx_models::Tree ros_tree;
    motion_planning::copy<prx::dirt_replan_t::Node, prx::dirt_replan_t::Edge>(ros_tree, _dirt->tree());
    _tree_publisher.publish(ros_tree);
    _dirt->reset();

    change_status(interface::ReplannerStatus::IDLE);

    _tot_replans++;

    return true;
  }
};

struct replanner_experiment_t
{
  ros::NodeHandle& _nh;
  std::string planning_model;
  ros::ServiceServer _experiment_service;
  bool _new_experiment;

  std::shared_ptr<replanner_t> replanner;  //(nh);

  std::string _lib_path;

  replanner_experiment_t(ros::NodeHandle& nh) : _nh(nh)
  {
    _lib_path = prx::lib_path_safe("ML4KP_ROS");
    // _timer = nh.createTimer(ros::Duration(1.0), &replanner_experiment_t::timer_callback, this);
  }

  void run()
  {
    while (ros::ok())
    {
      if (_new_experiment)
      {
        replanner.reset();
        replanner = std::make_shared<replanner_t>(_nh);
        _new_experiment = false;
      }

      ros::Duration(1.0).sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrPlanner_example");
  ros::NodeHandle nh("~");

  DEBUG_PRINT
  replanner_t::create_parameter_files(nh);

  DEBUG_PRINT
  std::string experiments_node_id;
  PARAM_SETUP(nh, experiments_node_id)
  DEBUG_PRINT

  std::shared_ptr<interface::node_status_t> node_status;
  std::shared_ptr<interface::node_status_t> experiments_node_status;
  node_status = interface::node_status_t::create(nh);
  experiments_node_status = interface::node_status_t::create(nh, experiments_node_id, true);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (experiments_node_status->status() != interface::NodeStatus::FINISH)
  {
    node_status->status(interface::NodeStatus::INITIALIZING);

    replanner_t replanner(nh);
    node_status->status(interface::NodeStatus::RUNNING);

    while (node_status->status() != interface::NodeStatus::FINISH)
    {
      if (node_status->new_request())
      {
        node_status->status(node_status->requested_status());
      }
      ros::Duration(1.0).sleep();
    }
  }
  // ros::AsyncSpinner spinner(4);
  // spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}