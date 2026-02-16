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

struct runner_t
{
  using State = prx_models::mushr_types::State::type;

  ros::Timer _timer;
  ros::Subscriber _sensor_subscriber, _collision_subscriber;

  State _state, _goal;
  double _goal_radius, _timeout;

  bool _collision, _goal_reached, _initializing;
  std::shared_ptr<interface::node_status_t> _node_status;
  std::shared_ptr<interface::node_status_t> _mj_status, _stela_status, _rosbag_status;

  std::vector<std::shared_ptr<interface::node_status_t>> _all_ns;

  std::ofstream _ofs;
  ros::WallTime _start;
  std::string _file_prefix;

  int _curr_experiment, _total_experiments;

  runner_t(ros::NodeHandle& nh) : _collision(false), _goal_reached(false), _initializing(true), _curr_experiment(0)
  {
    std::string sensor_topic_name, collision_topic_name;
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

    _mj_status->request_status(interface::NodeStatus::RESET);

    _timer = nh.createTimer(ros::Duration(1.0 / 10.0), &runner_t::timer_callback, this);
    init();
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    const ros::WallTime now(ros::WallTime::now());
    if (_initializing)  // Start
    {
      _node_status->status(interface::NodeStatus::INITIALIZING);
      // bool start_experiment{ true };
      // const interface::NodeStatus mj_ns{ mj_status->status() };
      // const interface::NodeStatus stela_ns{ stela_status->status() };    // == interface::NodeStatus::READY;
      // const interface::NodeStatus rosbag_ns{ rosbag_status->status() };  // == interface::NodeStatus::READY;

      int tot_running{ 0 };
      if (_stela_status->status() == interface::NodeStatus::RUNNING)
      {
        // _mj_status->request_status(interface::NodeStatus::RESET);
        _rosbag_status->request_status(interface::NodeStatus::RUNNING);
      }
      else
      {
        _stela_status->request_status(interface::NodeStatus::RUNNING);
        _rosbag_status->request_status(interface::NodeStatus::PAUSED);
      }
      for (auto node_stat : _all_ns)
      {
        if (node_stat->status() == interface::NodeStatus::RUNNING)
        {
          tot_running++;
        }
      }
      if (_all_ns.size() == tot_running)
      {
        _initializing = false;
        _start = ros::WallTime::now();
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
    std::string file_path{ _file_prefix + "_" + utils::timestamp() + ".txt" };
    _ofs.open(file_path);
  }

  void record(const std::string reason)
  {
    const double dt{ (ros::WallTime::now() - _start).toSec() };
    _ofs << dt << " ";
    _ofs << reason << " ";
    _ofs << _state[0] << " ";
    _ofs << _state[1] << " ";
    _ofs << _state[2] << " ";
    _ofs << "\n";

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
      const Eigen::VectorXd error{ State::Logmap(between) };
      _goal_reached = error.norm() < _goal_radius;
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