#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
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
#include <motion_planning/clustering.hpp>

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
  visualization_msgs::Marker _traj_marker;

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

  // bool _use_contingency, _cycle_update;

  motion_planning::PlanningResult _planning_result_msg;

  // ros::Publisher _goal_pos_publisher;
  // ros::Publisher _goal_radius_publisher;
  // ros::Publisher _safety_radius_publisher;
  // ros::Publisher _planning_result_publisher;
  // ros::Publisher _reset_publisher;
  ros::Publisher _status_publisher;
  ros::Publisher _tree_publisher;
  ros::Publisher _sln_tree_publisher;
  ros::Publisher _planner_stats_publisher;
  ros::Publisher _sln_traj_publisher, _viz_traj_publisher;
  ros::Publisher _sln_plan_publisher;

  ros::Subscriber _z_tree_subscriber;
  ros::Subscriber _planner_clock_subscriber;

  ros::Timer _viz_timer;

  double _postprocess_rate;

  double _max_edge_duration;

  prx::space_point_t _future_state;  // start for the planner

  bool _z_received, _start_replanning;

  int _current_cycle;
  ros::Duration _cycle_duration;

  // bool _propagate_dynamics, retain_previous, use_contingency;
  interface::ReplannerStatus _status;

  std::shared_ptr<prx::trajectory_t> _step_traj;
  std::shared_ptr<prx::plan_t> _step_plan, _rest_of_plan, _retained_plan;

  ros::Time _preprocess_end_time, _query_fulfill_start_time;

  ros::ServiceServer _replanning_service;

  std::string _environment;

  prx::tree_t _full_tree;

  std::size_t _tot_replans;
  std::string _tree_file_prefix;

  double _safe_distance;

  prx::param_loader _env_params;
  prx::param_loader _plant_params, _dirt_spec_params, _dirt_query_params;

  std::mutex _service_mutex;

  using ClusterData = std::tuple<Eigen::Matrix<double, 1, 3>, Eigen::Vector3d>;
  motion_planning::cluster_in_out_t<gtsam::Pose2, ClusterData> _cluster_SE3;

  replanner_t(ros::NodeHandle& nh) : _z_received(false)
  {
    LOG_FILENAME("logs/replanner.txt");
    LOG_MSG("Replanner Initialized")

    // std::string params_file;
    std::string sbmp_solution_tree_topic, sbmp_full_tree_topic;
    std::string planner_stats_topic_name;
    std::string& heuristic_map_filename{ _heuristic_map_filename };

    double& max_edge_duration{ _max_edge_duration };

    // std::string planning_mode;
    std::string& environment{ _environment };

    std::string plant_parameters, dirt_spec, dirt_query;

    using prx::simulation_step;
    simulation_step = 0.0;  // Force to set simste
    int random_seed;
    // PARAM_SETUP(nh, plant_file);
    // PARAM_SETUP(nh, planner_sln_recovery_type);

    // PARAM_SETUP(nh, preprocess_timeout);
    // PARAM_SETUP(nh, max_cycles);

    std::string replanner_service;

    PARAM_SETUP(nh, heuristic_map_filename);
    // PARAM_SETUP(nh, postprocess_rate);
    PARAM_SETUP(nh, sbmp_full_tree_topic);
    PARAM_SETUP(nh, sbmp_solution_tree_topic);
    PARAM_SETUP(nh, planner_stats_topic_name);
    PARAM_SETUP(nh, replanner_service);

    DEBUG_VARS(heuristic_map_filename);
    // DEBUG_VARS(postprocess_rate);
    DEBUG_VARS(sbmp_full_tree_topic);
    DEBUG_VARS(sbmp_solution_tree_topic);
    DEBUG_VARS(planner_stats_topic_name);

    // PARAM_SETUP(nh, sbmp_solution_tree_topic);
    // PARAM_SETUP(nh, environment);

    PRINT_MSG("[Replanner] Retrieving parameters")

    GLOBAL_PARAM_BLOCKER(random_seed);
    GLOBAL_PARAM_BLOCKER(simulation_step);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(dirt_spec);
    GLOBAL_PARAM_BLOCKER(dirt_query);
    GLOBAL_PARAM_BLOCKER(environment);

    PRINT_MSG("Parameters set")

    PARAM_SETUP_WITH_DEFAULT(nh, max_edge_duration, max_edge_duration);

    prx::init_random(random_seed);

    _dirt_spec_params.from_string(dirt_spec);
    _dirt_query_params.from_string(dirt_query);
    _plant_params.from_string(plant_parameters);
    _env_params.from_string(_environment);

    // PUBLISHERS
    const std::string publishers_ns{ "/motion_planning/dirt" };
    const std::string trajectory_topic{ publishers_ns + "/solution/trajectory" };
    _status_publisher = nh.advertise<interface::ReplannerStatus>(publishers_ns + "/status", 1, true);
    _tree_publisher = nh.advertise<prx_models::Tree>(sbmp_full_tree_topic, 1, true);
    _sln_tree_publisher = nh.advertise<prx_models::Tree>(sbmp_solution_tree_topic, 1, true);
    _planner_stats_publisher = nh.advertise<prx_models::PlannerStats>(planner_stats_topic_name, 1, true);
    _sln_plan_publisher = nh.advertise<ml4kp_bridge::PlanStepStampedArray>(publishers_ns + "/solution/plan", 1, true);
    _sln_traj_publisher = nh.advertise<ml4kp_bridge::SpacePointStampedArray>(trajectory_topic, 1, true);
    _viz_traj_publisher = nh.advertise<visualization_msgs::Marker>(trajectory_topic + "/marker", 1, true);
    // _trajectory_marker_publisher =
    // nh.advertise<visualization_msgs::Marker>("/motion_planning/dirt/trajectory/marker", 1, true);

    _status.state = interface::ReplannerStatus::INITIALIZING;
    _status_publisher.publish(_status);

    init_planner_spec(nh);
    init_planner_query();

    bool h_initialized{ false };
    // h_initialized = init_cluster_heuristic();
    // h_initialized = h_initialized ? true : init_heuristic_map();
    h_initialized = init_heuristic_map();
    prx_assert(h_initialized, "Heuristic not initialized");
    // get an initial plan
    _dirt = std::make_shared<prx::dirt_replan_t>("dirt");
    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());

    // Subscribers

    // Timers
    const ros::Duration timer_duration(0.01);
    // _viz_timer = nh.createTimer(timer_duration, &replanner_t::viz_callback, this);

    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);

    _replanning_service = nh.advertiseService(replanner_service, &replanner_t::replan, this);

    // LOG_FILENAME("logs/stela.txt");
    init_marker();
    PRINT_MSG("Replanner Ready!");
  }

  ~replanner_t()
  {
  }

  void init_marker()
  {
    _traj_marker.header.frame_id = "world";
    _traj_marker.header.stamp = ros::Time();
    _traj_marker.ns = "trajectory";
    _traj_marker.id = 0;
    _traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    _traj_marker.action = visualization_msgs::Marker::ADD;

    _traj_marker.pose.position.x = 0;
    _traj_marker.pose.position.y = 0;
    _traj_marker.pose.position.z = 0;
    _traj_marker.pose.orientation.x = 0.0;
    _traj_marker.pose.orientation.y = 0.0;
    _traj_marker.pose.orientation.z = 0.0;
    _traj_marker.pose.orientation.w = 1.0;

    _traj_marker.scale.x = 0.01;
    _traj_marker.scale.y = 0.01;
    _traj_marker.scale.z = 0.01;

    _traj_marker.color.a = 1.0;  // Don't forget to set the alpha!
    _traj_marker.color.r = 0.0;
    _traj_marker.color.g = 1.0;
    _traj_marker.color.b = 0.0;
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

      // DEBUG_VARS(spec_file)
      // DEBUG_VARS(query_file)
      // DEBUG_VARS(plant_file)
      ros::shutdown();
    }
  }

  void publish_visualization(const std::vector<ml4kp_bridge::SpacePointStamped> trajectory)
  {
    _traj_marker.points.clear();
    for (auto pt : trajectory)
    {
      _traj_marker.points.emplace_back();
      _traj_marker.points.back().x = pt.space_point.point[0];
      _traj_marker.points.back().y = pt.space_point.point[1];
      _traj_marker.points.back().z = 0.0;
    }
    _viz_traj_publisher.publish(_traj_marker);
  }

  void init_planner_spec(ros::NodeHandle& nh)
  {
    _plant = prx::system_factory_t::create_system(_plant_params);
    // const std::string plant_name{ plant_params["name"].as<std::string>() };
    // const std::string plant_path{ plant_params["path"].as<std::string>() };
    // _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    // _plant->init(plant_params);
    // prx_assert(_plant != nullptr, "Failed to create plant");

    std::tie(_planning_model, _system_group, _collision_group) = prx::world_model_t::create(_env_params, _plant);
    DEBUG_VARS(_plant);

    _state_space = _system_group->get_state_space();
    _control_space = _system_group->get_control_space();
    _param_space = _system_group->get_parameter_space();

    _step_plan = std::make_shared<prx::plan_t>(_control_space);
    _rest_of_plan = std::make_shared<prx::plan_t>(_control_space);
    _retained_plan = std::make_shared<prx::plan_t>(_control_space);
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
    else if (_dirt_spec_params["f_type"].as<>() == "f=g")
    {
      _dirt_spec->f_function = [&](const double& g, const double& h) { return g; };
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

  bool init_heuristic_map()
  {
    prx_assert(_dirt_spec_params.exists("heuristic"), "Parameter 'heuristic' not found");
    auto heuristic_params = _dirt_spec_params["heuristic"];
    if (heuristic_params["type"].as<>() != "2DPoint")
      return false;

    PRINT_MSG("Using heuristic map");
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

    // _dirt_spec->wavefront_h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
    //   return _heuristic_map->get_cost(s);
    // };

    std::ofstream h_ofs(dbg::variables::lib_path + "/logs/h_" + _env_params["environment/name"].as<>() + ".txt");
    _heuristic_map->to_csv(h_ofs);
    h_ofs.close();
    // int row = (0.0 - (-1.0)) / 0.01 = 100

    double conversion_velocity{ 0.3 };

    if (_dirt_spec_params.exists("heuristic"))
    {
      auto heuristic_params = _dirt_spec_params["heuristic"];
      //     heuristic:
      // max_velocity: 0.3
      // multiplier: 1.0
      // h is in [m], need in duration (as g-value) <- h[m] * vel^-1 [s/m]
      const double max_vel{ heuristic_params["max_velocity"].as<double>() };
      const double multiplier{ heuristic_params["multiplier"].as<double>() };
      DEBUG_VARS(heuristic_params)
      DEBUG_VARS(max_vel, multiplier)
      conversion_velocity = max_vel * multiplier;
    }
    if (_dirt_spec_params["f_type"].as<>() == "f=g")
    {
      conversion_velocity = 0;
    }
    DEBUG_VARS(conversion_velocity)
    _dirt_spec->heuristic = [&, conversion_velocity](const prx::space_point_t& curr, const prx::space_point_t& goal) {
      const double distance{ _heuristic_map->get_cost(curr) };  // workspace - h
      const double h{ distance / conversion_velocity };
      // DEBUG_VARS(_dirt_spec_params)
      // DEBUG_VARS(conversion_velocity)
      // DEBUG_VARS(wh, conversion_velocity, h)
      return h;
      // return 0.0;
    };

    return true;
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
      const double goal_error{ error.head(2).norm() };
      // DEBUG_VARS(goal_error)
      return goal_error < _dirt_query->goal_region_radius;
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
      // const ros::Time start_plan_stamp{ ros::Time::now() };
      const double time_limit{ request.planning_time };
      LOG_VARS(time_limit);

      return prx::condition_check_t("time", time_limit);
    }
    prx_throw("Unknown condition check")
  }

  void shutdown()
  {
    PRINT_MSG("Shutting down replanner...");
    std::scoped_lock lock(_service_mutex);
    _replanning_service.shutdown();
  }

  void plan_retainment(prx_models::StelaKraft::Request& request)
  {
    if (request.retain_plan)
    {
      const double plan_duration{ _retained_plan->duration() };
      const double plan_offset{ request.retianment_offset.toSec() };
      const double offset_to_retain{ std::min(plan_duration, plan_offset) };
      DEBUG_VARS(plan_duration, plan_offset, offset_to_retain)
      _dirt_query->retained_plan.clear();
      _retained_plan->copy_to(offset_to_retain, plan_duration, _dirt_query->retained_plan);
      _dirt_query->retainment = true;
    }
  }

  void copy_solution(std::vector<ml4kp_bridge::SpacePointStamped>& trajectory,    // no-lint
                     std::vector<ml4kp_bridge::PlanStepStamped>& piecewise_plan,  // no-lint
                     const prx::trajectory_t& traj, const prx::plan_t plan,       // no-lint
                     const double& max_solution_duration, const ros::Time& root_stamp)
  {
    // DEBUG_VARS(traj)
    // DEBUG_VARS(plan)
    int idx{ 0 };
    double ti{ 0.0 };
    const double solution_duration{ std::min(max_solution_duration, plan.duration()) };
    // DEBUG_VARS(solution_duration, max_solution_duration, plan.duration(), traj.duration())
    for (; ti < solution_duration; ti += prx::simulation_step, ++idx)
    {
      trajectory.emplace_back();
      trajectory.back().header.stamp = root_stamp + ros::Duration(ti);
      ml4kp_bridge::copy(trajectory.back().space_point, traj[idx]);
    }
    // DEBUG_VARS(trajectory.size())
    idx = 0;
    ti = 0.0;
    double dt{ 0.0 };
    while (ti < solution_duration)
    {
      piecewise_plan.emplace_back();
      piecewise_plan.back().header.stamp = root_stamp + ros::Duration(ti);
      ml4kp_bridge::copy(piecewise_plan.back().plan_step.control, plan[idx].control);
      piecewise_plan.back().plan_step.duration.data = ros::Duration(prx::simulation_step);

      if (dt >= plan[idx].duration)
      {
        dt = 0;
        idx++;
      }
      dt += prx::simulation_step;
      ti += prx::simulation_step;
    }
    // DEBUG_VARS(piecewise_plan.size())
  }

  bool replan(prx_models::StelaKraft::Request& request, prx_models::StelaKraft::Response& response)
  {
    std::scoped_lock lock(_service_mutex);
    // PRINT_MSG("START REPLANNING");
    response.planner_output = prx_models::StelaKraft::Response::TYPE_FAILURE;

    // PRINT_MSG("PREPROCESSING");
    change_status(interface::ReplannerStatus::PREPROCESSING);

    plan_retainment(request);

    _dirt_query->clear_outputs();

    prx_assert(_dirt_query->start_state->size() == request.root_state.space_point.point.size(),
               "[mushr_replanning_dirt] Size mismatch: Expected: "
                   << _dirt_query->start_state->size() << " Got: " << request.root_state.space_point.point.size()
                   << ".\n Requested state: [ " << request.root_state.space_point.point << " ]\n");

    ml4kp_bridge::copy(_dirt_query->start_state, request.root_state.space_point);

    _traj_msg.data.clear();
    _step_traj->clear();

    _dirt->link_and_setup_spec(_dirt_spec.get());
    _dirt->preprocess();
    _dirt->link_and_setup_query(_dirt_query.get());

    prx::condition_check_t checker{ create_condition(request) };

    change_status(interface::ReplannerStatus::PLANNING);
    // PRINT_MSG("PLANNING");

    _dirt->resolve_query(&checker);

    // const ros::Time end{ ros::Time::now() };
    change_status(interface::ReplannerStatus::POSTPROCESSING);

    // PRINT_MSG("POSTPROCESSING");

    _dirt->fulfill_query();

    std::shared_ptr<prx::planner_t::statistics_t> planner_stats{ _dirt->statistics() };
    std::shared_ptr<prx::dirt_replan_t::statistics_t> stats{
      std::dynamic_pointer_cast<prx::dirt_replan_t::statistics_t>(planner_stats)
    };
    prx_assert(stats != nullptr, "Couldn't cast stats to dirt stats");
    prx_models::copy(response.stats, *stats);
    _planner_stats_publisher.publish(response.stats);
    prx_models::Tree sln_tree;

    if (_dirt_query->solution_traj.size() > 0)
    {
      copy_solution(response.trajectory.data, response.piecewise_plan.data,  // no-lint
                    _dirt_query->solution_traj, _dirt_query->solution_plan,  // no-lint
                    request.solution_duration.toSec(), request.root_state.header.stamp);

      response.planner_output = prx_models::StelaKraft::Response::TYPE_SUCCESS;
    }

    _sln_traj_publisher.publish(response.trajectory);
    _sln_plan_publisher.publish(response.piecewise_plan);

    prx_models::Tree ros_tree;
    motion_planning::copy<prx::dirt_replan_t::Node, prx::dirt_replan_t::Edge>(ros_tree, _dirt->tree());
    _tree_publisher.publish(ros_tree);
    _dirt->reset();

    publish_visualization(response.trajectory.data);
    change_status(interface::ReplannerStatus::IDLE);

    _tot_replans++;

    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrReplannerDirt");
  ros::NodeHandle nh("~");

  replanner_t::create_parameter_files(nh);

  std::string experiments_node_id;
  PARAM_SETUP(nh, experiments_node_id)

  std::shared_ptr<interface::node_status_t> node_status;
  std::shared_ptr<interface::node_status_t> experiments_node_status;
  node_status = interface::node_status_t::create(nh);
  experiments_node_status = interface::node_status_t::create(nh, experiments_node_id, true);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  node_status->status(interface::NodeStatus::INITIALIZING);
  std::shared_ptr<replanner_t> replanner;

  PRINT_MSG("[mushr_replanning_dirt] INIT")
  while (experiments_node_status->status() != interface::NodeStatus::FINISH)
  {
    // node_status->status(interface::NodeStatus::INITIALIZING, experiments_node_status->sequence_id());

    // DEBUG_VARS(node_status);
    if (node_status->new_request())
    {
      auto requested_status = interface::node_status_t::status_to_string(node_status->requested_status());
      DEBUG_VARS(requested_status);

      if (node_status->requested_status() == interface::NodeStatus::RESET or
          node_status->requested_status() == interface::NodeStatus::FINISH)
      {
        if (replanner)
        {
          PRINT_MSG("Resetting replanner")
          replanner->shutdown();
        }
        replanner = nullptr;
        node_status->status(node_status->requested_status());
        node_status->request_acknowledged();
      }
      else if (node_status->requested_status() == interface::NodeStatus::RUNNING)
      {
        if (not replanner)
        {
          PRINT_MSG("Initializing replanner")
          replanner = std::make_shared<replanner_t>(nh);
        }
        node_status->status(node_status->requested_status());
        node_status->request_acknowledged();
      }
      else
      {
        auto INVALID_STATUS_REQUESTED = *node_status;
        DEBUG_VARS(INVALID_STATUS_REQUESTED);
      }

      // node_status->status(node_status->requested_status());
    }
    if (node_status->sequence_id() != experiments_node_status->sequence_id())
    {
      node_status->status(interface::NodeStatus::RESET);
      if (replanner)
      {
        PRINT_MSG("Sequence id mismatch")
        replanner->shutdown();
      }
      replanner = nullptr;
    }
    ros::Duration(1.0).sleep();
    // }
    // replanner.shutdown();
  }
  PRINT_MSG("[mushr_replanning_dirt] FINISHED")
  // ros::AsyncSpinner spinner(4);
  // spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}