#include <ros/node_handle.h>
#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

#include <memory>
#include <utils/rosparams_utils.hpp>
#include <utils/std_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <interface/node_status.hpp>
#include <interface/SensorDataStamped.h>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/mushr.hpp>
#include <prx_models/PlannerStats.h>
#include "utils/dbg_utils.hpp"

struct runner_t
{
  using State = prx_models::mushr_types::State::type;

  ros::Timer _timer, _verbose_timer;
  ros::Subscriber _sensor_subscriber, _collision_subscriber, _planner_stats_subscriber;

  State _state, _goal;
  double _goal_radius, _timeout;

  bool _collision, _goal_reached, _initializing;
  std::shared_ptr<interface::node_status_t> _node_status;
  std::shared_ptr<interface::node_status_t> _mj_status, _stela_status, _rosbag_status;

  std::vector<std::shared_ptr<interface::node_status_t>> _all_ns;

  std::ofstream _ofs;
  std::ofstream _ofs_planner;
  ros::WallTime _start;
  std::string _file_prefix;

  int _curr_experiment, _total_experiments;

  Eigen::Vector3d _error;

  runner_t(ros::NodeHandle& nh)
    : _collision(false), _goal_reached(false), _initializing(true), _curr_experiment(0), _error(10, 10, 10)
  {
    std::string sensor_topic_name, collision_topic_name, planner_stats_topic_name;
    std::string stela_node_id, mj_node_id, rosbag_node_id;

    int& total_experiments{ _total_experiments };
    double& goal_radius{ _goal_radius };
    double& timeout{ _timeout };
    std::string& file_prefix{ _file_prefix };
    std::vector<double> goal;

    PARAM_SETUP(nh, goal);
    PARAM_SETUP(nh, timeout);
    PARAM_SETUP(nh, file_prefix);
    PARAM_SETUP(nh, goal_radius);
    PARAM_SETUP(nh, mj_node_id);
    PARAM_SETUP(nh, stela_node_id);
    PARAM_SETUP(nh, rosbag_node_id);
    PARAM_SETUP(nh, total_experiments);
    PARAM_SETUP(nh, sensor_topic_name);
    PARAM_SETUP(nh, collision_topic_name);
    PARAM_SETUP(nh, planner_stats_topic_name);

    prx_assert(goal.size() == 3, "goal needs to have size 3");
    _goal[0] = goal[0];
    _goal[1] = goal[1];
    _goal[2] = goal[2];

    _node_status = interface::node_status_t::create(nh);

    _mj_status = interface::node_status_t::create(nh, mj_node_id, true);
    _stela_status = interface::node_status_t::create(nh, stela_node_id, true);
    _rosbag_status = interface::node_status_t::create(nh, rosbag_node_id, true);

    _all_ns = { _mj_status, _stela_status, _rosbag_status };

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &runner_t::sensor_callback, this);
    _collision_subscriber = nh.subscribe(collision_topic_name, 1, &runner_t::collision_callback, this);
    _planner_stats_subscriber = nh.subscribe(planner_stats_topic_name, 1, &runner_t::planner_stats_callback, this);

    _timer = nh.createTimer(ros::Duration(1.0 / 10.0), &runner_t::timer_callback, this);
    _verbose_timer = nh.createTimer(ros::Duration(5.0), &runner_t::verbose_timer_callback, this);
    init();
    _mj_status->request_status(interface::NodeStatus::RESET);
  }

  ~runner_t()
  {
    _node_status->status(interface::NodeStatus::FINISH);
    ros::Duration(1.0).sleep();
  }

  void verbose_timer_callback(const ros::TimerEvent& event)
  {
    const double time_remaining{ (ros::WallTime::now() - _start).toSec() };
    const double& timeout{ _timeout };
    const auto goal_error = _error.transpose();
    DEBUG_VARS(time_remaining, timeout, goal_error);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    const ros::WallTime now(ros::WallTime::now());

    if (_initializing)  // Start
    {
      _node_status->status(interface::NodeStatus::INITIALIZING);

      int tot_running{ 0 };
      if (_mj_status->status() == interface::NodeStatus::RUNNING and
          _stela_status->status() == interface::NodeStatus::RUNNING)
      {
        // _rosbag_status->status() == interface::NodeStatus::RUNNING and
        PRINT_MSG("ALL RUNNING ");
        _rosbag_status->request_status(interface::NodeStatus::RUNNING);
        _initializing = false;
      }
      else if (_mj_status->status() == interface::NodeStatus::RUNNING and
               _rosbag_status->status() == interface::NodeStatus::READY)
      {
        // PRINT_MSG("MJ & Rosbag running, setting STELA to 'RUNNING' ");
        _stela_status->request_status(interface::NodeStatus::RUNNING);
        _start = ros::WallTime::now();
      }
      else
      {
        // DEBUG_VARS(_mj_status)
        // DEBUG_VARS(_rosbag_status)
        // DEBUG_VARS(_stela_status)
        _start = ros::WallTime::now();
        _mj_status->request_status(interface::NodeStatus::RESET);
        // _rosbag_status->request_status(interface::NodeStatus::RUNNING);
        ros::Duration(1.0).sleep();
        // _stela_status->request_status(interface::NodeStatus::RUNNING);
      }
    }
    else if (_collision)  // Collision detected
    {
      record("collision");
      _collision = false;
    }
    else if (_goal_reached)  // Goal Reached
    {
      record("goal_reached");
      _goal_reached = false;
    }
    else if ((now - _start).toSec() > _timeout)  // timeout check
    {
      record("timeout");
    }
    else  // keep running
    {
      if (_stela_status->status() != interface::NodeStatus::RUNNING)
      {
        _stela_status->request_status(interface::NodeStatus::RUNNING);
      }
      _node_status->status(interface::NodeStatus::RUNNING);
    }
  }

  void call_reset()
  {
    for (auto stat : _all_ns)
    {
      stat->request_status(interface::NodeStatus::RESET);
    }
  }

  void init()
  {
    const std::string file_path{ _file_prefix + "_" + utils::timestamp() + ".txt" };
    const std::string file_planner_path{ _file_prefix + "_planner_" + utils::timestamp() + ".txt" };
    _ofs.open(file_path);
    _ofs_planner.open(file_planner_path);
    DEBUG_VARS(file_path);
    DEBUG_VARS(file_planner_path);
    ros::Duration(5.0).sleep();
    _mj_status->request_status(interface::NodeStatus::RESET);
  }

  void record(const std::string reason)
  {
    const double dt{ (ros::WallTime::now() - _start).toSec() };
    _ofs << dt << " ";
    _ofs << reason << " ";
    _ofs << _state[0] << " ";
    _ofs << _state[1] << " ";
    _ofs << _state[2] << " ";
    _ofs << std::endl;  // Force a write

    const std::string msg{ "[Mushr Experiment]" };
    DEBUG_VARS(msg, reason, _curr_experiment, _total_experiments);
    _curr_experiment++;

    if (_curr_experiment == _total_experiments)
    {
      _node_status->status(interface::NodeStatus::FINISH);
      _ofs.close();
      for (auto stat : _all_ns)
      {
        stat->request_status(interface::NodeStatus::FINISH);
      }
      ros::Duration(5.).sleep();
      ros::shutdown();
    }
    _node_status->status(interface::NodeStatus::RESET);
    call_reset();
    ros::Duration(5.).sleep();  // sleep for resets to happen
    _initializing = true;
  }

  void planner_stats_callback(const prx_models::PlannerStats msg)
  {
    _ofs_planner << msg.planned_duration << " ";
    _ofs_planner << msg.iteration_count << " ";
    _ofs_planner << msg.total_nodes << " ";
    _ofs_planner << msg.cost_current_solution << " ";
    _ofs_planner << msg.time_current_solution << " ";
    _ofs_planner << msg.iters_current_solution << " ";
    _ofs_planner << "\n";
  }

  void collision_callback(const std_msgs::BoolConstPtr msg)
  {
    _collision = msg->data;
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<std_msgs::Float64>& zi{ msg->raw_sensor_data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3].data, zi[4].data, zi[5].data, zi[6].data) };
    _state[0] = zi[0].data;
    _state[1] = zi[1].data;
    _state[2] = prx::quaternion_to_euler(q)[2];

    if (not _goal_reached)
    {
      const State between{ _state.between(_goal) };
      _error = State::Logmap(between);
      _goal_reached = _error.norm() < _goal_radius;
    }
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrExperiments" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // std::string stela_node_id, mj_node_id, rosbag_node_id;
  runner_t runner(nh);

  // PARAM_SETUP(nh, mj_node_id);
  // PARAM_SETUP(nh, stela_node_id);
  // PARAM_SETUP(nh, rosbag_node_id);

  // const std::string root{ ros::this_node::getName() };

  ros::spin();
  return 0;
}