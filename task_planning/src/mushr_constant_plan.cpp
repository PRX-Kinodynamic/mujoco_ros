#include <ml4kp_bridge/defs.h>

#include <prx_models/MushrPlanner.h>
#include <prx/simulation/system.hpp>
#include <prx_models/mj_mushr.hpp>
#include <control/MushrControlPropagation.h>
#include <motion_planning/replanner_service.hpp>
#include <motion_planning/planner_client.hpp>
#include <motion_planning/PlanningResult.h>
#include <mujoco_ros/Collision.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <utils/dbg_utils.hpp>
#include <utils/std_utils.hpp>
#include <motion_planning/tree_bridge.hpp>

#include <prx_models/mushr.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <interface/ReplannerStatus.h>
#include <interface/PlannerClock.h>

#include <prx_models/StelaKraft.h>
#include <utils/time_profiler.hpp>

struct constant_plan_t
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

  std::shared_ptr<prx::plan_t> _plan;
  std::shared_ptr<prx::trajectory_t> _traj;

  prx::space_point_t _start_state;
  bool _new_traj;

  utils::time_profiler_t _profiler;

  constant_plan_t(ros::NodeHandle& nh) : _new_traj(false)
  {
    std::string params_file;
    std::string sbmp_solution_tree_topic, sbmp_full_tree_topic;

    double& max_edge_duration{ _max_edge_duration };

    PARAM_SETUP(nh, params_file);
    PARAM_SETUP(nh, sbmp_full_tree_topic);
    PARAM_SETUP(nh, sbmp_solution_tree_topic);

    PARAM_SETUP_WITH_DEFAULT(nh, max_edge_duration, max_edge_duration);

    const std::string path{ prx::lib_path_safe("ML4KP_ROS") };
    _profiler.set_filename(path + "/profile.txt");
    // Publisher
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
    DEBUG_VARS(prx::simulation_step)
    init_vars(nh);
    setup_plan(nh);

    // Timers
    const ros::Duration timer_duration(0.01);
    // _clock_timer = nh.createTimer(timer_duration, &constant_plan_t::timer_callback, this);
    _tree_timer = nh.createTimer(timer_duration, &constant_plan_t::tree_publish_callback, this);

    _status.state = interface::ReplannerStatus::IDLE;
    _status_publisher.publish(_status);

    _replanning_service = nh.advertiseService("/kraft/replan", &constant_plan_t::replan, this);

    PRINT_MSG("Replanner Ready!");
  }

  void init_vars(ros::NodeHandle& nh)
  {
    const std::string plant_name{ params["/plant/name"].as<std::string>() };
    const std::string plant_path{ params["/plant/path"].as<std::string>() };
    _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(_plant != nullptr, "Failed to create plant");

    // auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
    // auto obstacles = prx::load_obstacles(_environment);
    // _obstacle_list = obstacles.second;
    // _obstacle_names = obstacles.first;

    const std::vector<prx::system_ptr_t> all_systems{ { _plant } };
    const std::vector<std::shared_ptr<prx::movable_object_t>> all_obstacles{ { _obstacle_list } };

    _planning_model = std::make_shared<prx::world_model_t>(all_systems, all_obstacles);

    _planning_model->create_context("planner_context", { plant_name }, {});
    auto planning_context = _planning_model->get_context("planner_context");

    _system_group = prx::system_group(planning_context);
    _collision_group = prx::collision_group(planning_context);

    _state_space = _system_group->get_state_space();
    _control_space = _system_group->get_control_space();
    _param_space = _system_group->get_parameter_space();

    _state_space->init(params["/plant/state_space"]);
    _control_space->init(params["/plant/control_space"]);
    _param_space->init(params["/plant/parameter_space"]);

    _plan = std::make_shared<prx::plan_t>(_control_space);
    _step_plan = std::make_shared<prx::plan_t>(_control_space);
    _traj = std::make_shared<prx::trajectory_t>(_state_space);

    _start_state = _state_space->make_point();

    // _future_state = _state_space->make_point();
  }

  void setup_plan(ros::NodeHandle& nh)
  {
    XmlRpc::XmlRpcValue constant_plan;

    PARAM_SETUP(nh, constant_plan);
    for (int i = 0; i < constant_plan.size(); ++i)
    {
      const XmlRpc::XmlRpcValue ctrl_in{ constant_plan[i] };

      double duration;
      Eigen::Vector2d ctrl;
      utils::get_value(ctrl, ctrl_in["control"]);
      utils::get_value(duration, ctrl_in["duration"]);
      _plan->copy_onto_back(ctrl, duration);
    }
    DEBUG_VARS(*_plan)
  }

  void tree_publish_callback(const ros::TimerEvent& event)
  {
    // if (_new_traj)
    // {
    //   prx_models::Tree ros_tree;
    //   tree_from_plan_traj(ros_tree, *_plan, *_traj);
    //   _tree_publisher.publish(ros_tree);
    //   _new_traj = false;
    // }
  }

  void tree_from_plan_traj(prx_models::Tree& sln_tree, prx::plan_t& plan, prx::trajectory_t& traj)
  {
    if (traj.duration() < plan.duration())
    {
      PRINT_MSG("[Replanner::tree_from_plan_traj] Trajectory shorter than plan");
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
  }

  bool replan(prx_models::StelaKraft::Request& request, prx_models::StelaKraft::Response& response)
  {
    _profiler.start();
    // PRINT_MSG("Replanning...")
    response.planner_output = prx_models::StelaKraft::Response::TYPE_FAILURE;

    change_status(interface::ReplannerStatus::PREPROCESSING);

    _traj->clear();
    // _plan->clear();
    _step_plan->clear();

    ml4kp_bridge::copy(_start_state, request.root.point);

    _profiler.checkpoint("PREPROCESSING");
    change_status(interface::ReplannerStatus::PLANNING);
    _system_group->propagate(_start_state, *_plan, *_traj);

    // DEBUG_VARS(*_traj);
    _profiler.checkpoint("PLANNING");
    change_status(interface::ReplannerStatus::POSTPROCESSING);

    const double plan_duration{ _plan->duration() };
    const double traj_duration{ _traj->duration() };

    if (plan_duration < request.solution_duration.toSec())
    {
      _plan->copy_to(0, plan_duration, *_step_plan);
    }
    else
    {
      _plan->copy_to(0, request.solution_duration.toSec(), *_step_plan);
    }

    response.sln_tree.root = request.root.index;
    response.sln_tree.nodes.push_back(request.root);
    response.planner_output = prx_models::StelaKraft::Response::TYPE_SUCCESS;

    tree_from_plan_traj(response.sln_tree, *_step_plan, *_traj);
    _sln_tree_publisher.publish(response.sln_tree);

    prx_models::Tree ros_tree;
    ros_tree.root = request.root.index;
    ros_tree.nodes.push_back(request.root);
    tree_from_plan_traj(ros_tree, *_plan, *_traj);
    // DEBUG_VARS(*_traj)
    _tree_publisher.publish(ros_tree);

    _profiler.checkpoint("POSTPROCESSING");
    change_status(interface::ReplannerStatus::IDLE);

    _new_traj = true;

    _profiler.end("end");

    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrPlanner_example");
  ros::NodeHandle nh("~");
  // ros::NodeHandle private_nh("~");

  ros::AsyncSpinner spinner(4);

  constant_plan_t replanner(nh);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}