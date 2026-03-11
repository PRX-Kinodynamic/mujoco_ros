#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

#include <iterator>
#include <memory>
#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/general/param_loader.hpp>
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
#include <utils/dbg_utils.hpp>
#include <prx_models/planner_utils.hpp>

struct runner_t
{
  using State = prx_models::mushr_types::State::type;

  ros::Timer _timer, _verbose_timer;
  ros::Subscriber _sensor_subscriber, _collision_subscriber, _planner_stats_subscriber;

  State _state, _goal;
  double _goal_radius, _timeout;

  bool _collision, _goal_reached, _initializing;
  std::shared_ptr<interface::node_status_t> _mj_status, _stela_status, _rosbag_status, _replanner_status;

  std::vector<std::shared_ptr<interface::node_status_t>> _all_ns;

  std::ofstream _ofs;
  std::ofstream _ofs_planner;
  ros::WallTime _start;
  std::string _file_prefix;

  int _curr_experiment, _total_experiments, _experiment_num;

  Eigen::Vector3d _error;
  ros::NodeHandle _nh;

  prx::param_loader _experiment_params, _env_params;

  runner_t(ros::NodeHandle& nh, prx::param_loader& experiment_params, prx::param_loader& env_params)
    : _collision(false)
    , _goal_reached(false)
    , _initializing(true)
    , _curr_experiment(0)
    , _error(10, 10, 10)
    , _nh(nh)
    , _experiment_params(experiment_params)
    , _env_params(env_params)
  // , _experiment_num(experiment_num)
  {
    // DEBUG_VARS(param_file)
    // DEBUG_VARS(_experiment_params);
    prx_assert(_experiment_params["experiment_set"].as<bool>(), "Experiment not set!");
    const std::string sensor_topic_name{ _experiment_params["sensor_topic_name"].as<>() };
    const std::string collision_topic_name{ _experiment_params["collision_topic_name"].as<>() };
    const std::string planner_stats_topic_name{ _experiment_params["planner_stats_topic_name"].as<>() };
    const std::string stela_node_id{ _experiment_params["stela_node_id"].as<>() };
    const std::string mj_node_id{ _experiment_params["mj_node_id"].as<>() };
    const std::string rosbag_node_id{ _experiment_params["rosbag_node_id"].as<>() };
    const std::string replanner_node_id{ _experiment_params["replanner_node_id"].as<>() };
    _timeout = _experiment_params["timeout"].as<double>();
    _total_experiments = _experiment_params["total_experiments"].as<int>();
    _file_prefix = _experiment_params["file_prefix"].as<>();

    // _experiments_reader = std::make_shared<prx::utilities::csv_reader_t>(experiments_file);

    DEBUG_VARS(stela_node_id, mj_node_id, rosbag_node_id)

    _mj_status = interface::node_status_t::create(nh, mj_node_id, true);
    _stela_status = interface::node_status_t::create(nh, stela_node_id, true);
    _rosbag_status = interface::node_status_t::create(nh, rosbag_node_id, true);
    _replanner_status = interface::node_status_t::create(nh, replanner_node_id, true);

    _all_ns = { _mj_status, _stela_status, _rosbag_status, _replanner_status };

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &runner_t::sensor_callback, this);
    _collision_subscriber = nh.subscribe(collision_topic_name, 1, &runner_t::collision_callback, this);
    _planner_stats_subscriber = nh.subscribe(planner_stats_topic_name, 1, &runner_t::planner_stats_callback, this);

    _timer = nh.createTimer(ros::Duration(1.0 / 10.0), &runner_t::timer_callback, this);
    _verbose_timer = nh.createTimer(ros::Duration(5.0), &runner_t::verbose_timer_callback, this);
    init_experiment();

    // _stela_status->request_status(interface::NodeStatus::FINISH);
    // _replanner_status->request_status(interface::NodeStatus::FINISH);

    _mj_status->request_status(interface::NodeStatus::RESET);
    ros::Duration(1.0).sleep();
  }

  ~runner_t()
  {
    PRINT_MSG("finishing and waiting...");
    ros::param::del("/environment");
    DEBUG_PRINT
    _stela_status->request_and_wait(interface::NodeStatus::FINISH);
    DEBUG_PRINT
    _replanner_status->request_and_wait(interface::NodeStatus::FINISH);
    DEBUG_PRINT
    _rosbag_status->request_and_wait(interface::NodeStatus::FINISH);
    DEBUG_PRINT
  }

  void init_experiment()
  {
    // auto exp_iter = experiment_params["/experiments"].begin();  ///.begin() + _exp_num;
    // // exp_i = *(exp_i.begin() + _curr_experiment);
    // std::advance(exp_iter, experiment_num);
    // if (exp_iter != experiment_params["/experiments"].end())
    // {
    //   env_params = *exp_iter;
    //   return true;
    // prx::param_loader param = *exp_iter;
    std::vector<double> next_goal{ _env_params["goal/state"].as<std::vector<double>>() };
    _goal[0] = next_goal[0];
    _goal[1] = next_goal[1];
    _goal[2] = next_goal[2];

    _goal_radius = _env_params["goal/radius"].as<double>();
    const std::string environment{ _env_params["environment"].as<std::string>() };

    std::ifstream infile_env{ environment };

    const std::string env_file{ std::istreambuf_iterator<char>(infile_env), std::istreambuf_iterator<char>() };

    // DEBUG_VARS(env_file)
    ros::param::set("/environment", env_file);

    const std::string rosbag_directory{ _env_params["rosbag/directory"].as<std::string>() };
    const std::string rosbag_prefix{ _env_params["rosbag/prefix"].as<std::string>() };
    ros::param::set("/rosbag/directory", rosbag_directory);
    ros::param::set("/rosbag/prefix", rosbag_prefix);

    const std::string timestamp{ utils::timestamp() };
    _ofs.open(rosbag_directory + "/data_" + timestamp + ".txt");
    _ofs_planner.open(rosbag_directory + "/planner_data_" + timestamp + ".txt");
    // return true;
  }

  void verbose_timer_callback(const ros::TimerEvent& event)
  {
    const double time_spent{ (ros::WallTime::now() - _start).toSec() };
    const double& timeout{ _timeout };
    const auto goal_error = _error.transpose();
    DEBUG_VARS(time_spent, timeout, goal_error);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    const ros::WallTime now(ros::WallTime::now());

    if (_initializing)  // Start
    {
      // _node_status->status(interface::NodeStatus::INITIALIZING);
      DEBUG_VARS(*_mj_status, *_stela_status, *_rosbag_status, *_replanner_status)
      int tot_running{ 0 };
      if (_mj_status->status() == interface::NodeStatus::RUNNING and
          _stela_status->status() == interface::NodeStatus::RUNNING and
          _replanner_status->status() == interface::NodeStatus::RUNNING)
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
        _replanner_status->request_status(interface::NodeStatus::RUNNING);
        _start = ros::WallTime::now();
      }
      else
      {
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
      // _node_status->status(interface::NodeStatus::RUNNING);
    }
  }

  void call_reset()
  {
    for (auto stat : _all_ns)
    {
      stat->request_status(interface::NodeStatus::RESET);
    }
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

    // if (_curr_experiment == _total_experiments)
    // {
    // _node_status->status(interface::NodeStatus::FINISH);
    // _ofs.close();
    // for (auto stat : _all_ns)
    // {
    //   stat->request_status(interface::NodeStatus::FINISH);
    // }
    // ros::Duration(5.).sleep();
    // ~runner_t();
    // ros::shutdown();
    // }
    // _node_status->status(interface::NodeStatus::RESET);
    call_reset();
    ros::Duration(5.).sleep();  // sleep for resets to happen
    _curr_experiment++;
    _initializing = true;
  }

  void run()
  {
    while (_curr_experiment < _total_experiments)
    {
      DEBUG_VARS(_curr_experiment, _total_experiments)
      ros::Duration(1.).sleep();
    }
    PRINT_MSG("Experiments done!");
    _timer.stop();
    _verbose_timer.stop();
    // _node_status->status(interface::NodeStatus::FINISH);
    _ofs.close();
  }

  void planner_stats_callback(const prx_models::PlannerStats msg)
  {
    prx_models::to_stream(_ofs_planner, msg);
    _ofs_planner << std::endl;
  }

  void collision_callback(const std_msgs::BoolConstPtr msg)
  {
    _collision = msg->data;
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<double>& zi{ msg->raw_sensor_data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3], zi[4], zi[5], zi[6]) };
    _state[0] = zi[0];
    _state[1] = zi[1];
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

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string experiments_file;
  PARAM_SETUP(nh, experiments_file)
  prx::utilities::csv_reader_t reader(experiments_file);
  std::shared_ptr<interface::node_status_t> _node_status{ interface::node_status_t::create(nh) };

  int experiment_number{ 0 };
  while (reader.has_next_line())
  {
    DEBUG_VARS(experiment_number);
    _node_status->status(interface::NodeStatus::INITIALIZING);
    auto line = reader.next_line();
    std::string dir{ line[0] };
    const std::string replan_spec_filename{ dir + "/dirt_replan_spec.yaml" };
    const std::string replan_query_filename{ dir + "/dirt_replan_query.yaml" };
    const std::string experiment_filename{ dir + "/dirt_experiment.yaml" };
    const std::string stela_kraft_filename{ dir + "/stela_kraft_request.yaml" };

    std::ifstream infile_spec{ replan_spec_filename };
    std::ifstream infile_query{ replan_query_filename };
    std::ifstream infile_stela_kraft{ stela_kraft_filename };
    // std::ifstream infile_experiment{ experiment_filename };

    const std::string spec_file{ std::istreambuf_iterator<char>(infile_spec), std::istreambuf_iterator<char>() };
    const std::string query_file{ std::istreambuf_iterator<char>(infile_query), std::istreambuf_iterator<char>() };
    const std::string stela_kraft_file{ std::istreambuf_iterator<char>(infile_stela_kraft),
                                        std::istreambuf_iterator<char>() };
    // const std::string experiment_file{ std::istreambuf_iterator<char>(infile_experiment),
    //                                    std::istreambuf_iterator<char>() };

    DEBUG_VARS(spec_file)
    DEBUG_VARS(query_file)
    nh.setParam("/dirt_spec", spec_file);
    nh.setParam("/dirt_query", query_file);
    nh.setParam("/stela_kraft_request_params", stela_kraft_file);

    // prx::param_loader env_params;
    prx::param_loader experiment_params(experiment_filename);
    // bool valid_experiment{ get_experiment_params(env_params, experiment_params, experiment_num) };
    auto exp_params = experiment_params["/experiments"];  ///.begin() + _exp_num;

    for (auto exp_iter : exp_params)
    {
      prx::param_loader exp_param(exp_iter);  // = exp_iter;
      // exp_param = exp_iter;
      runner_t runner(nh, experiment_params, exp_param);
      _node_status->status(interface::NodeStatus::RUNNING);
      runner.run();
      PRINT_MSG("Done?")
    }

    experiment_number++;
    // _experiment_params.reset(new prx::param_loader());
    // _experiment_params->from_string(experiment_filename);
  }
  _node_status->status(interface::NodeStatus::FINISH);
  ros::waitForShutdown();
  spinner.stop();

  // ros::spin();
  return 0;
}