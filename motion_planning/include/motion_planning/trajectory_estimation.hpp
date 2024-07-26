#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <utils/std_utils.cpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_planning/StelaGraphTraversalAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class trajectory_estimation_t : public Base
{
  using Derived = trajectory_estimation_t<SystemInterface, Base>;

  using SF = prx::fg::symbol_factory_t;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;
  using Observation = typename SystemInterface::Observation;

  using StateKeys = typename SystemInterface::StateKeys;
  using ControlKeys = typename SystemInterface::ControlKeys;

  using StateEstimates = typename SystemInterface::StateEstimates;

  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename SystemInterface::ConfigFromState>;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  trajectory_estimation_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, true, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _tf_listener(_tf_buffer)
    , _isam(_isam_params)
    , _tree_recevied(false)
    , _experiment_id("test"){};

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string control_topic;
    std::string collision_topic;
    std::string environment;

    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& output_dir{ _output_dir };
    std::string& obstacle_mode{ _obstacle_mode };
    std::string& experiment_id{ _experiment_id };

    int z_frequency;
    // ROS_PARAM_SETUP(private_nh, random_seed);
    // ROS_PARAM_SETUP(private_nh, plant_config_file);
    // ROS_PARAM_SETUP(private_nh, planner_config_file);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, output_dir)
    PARAM_SETUP(private_nh, collision_topic)
    PARAM_SETUP(private_nh, environment)
    PARAM_SETUP(private_nh, z_frequency)
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id)

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string pose_with_cov_topic{ ros::this_node::getNamespace() + "/pose_with_cov" };
    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };
    const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    const ros::Duration z_timer_duration(1.0 / z_frequency);
    _z_timer = private_nh.createTimer(z_timer_duration, &Derived::estimate, this);

    _collision_subscriber = private_nh.subscribe(collision_topic, 1, &Derived::collision_callback, this);
    _control_subscriber = private_nh.subscribe(control_topic, 1, &Derived::control_callback, this);
    _stamped_control_subscriber =
        private_nh.subscribe(stamped_control_topic, 1, &Derived::stamped_control_callback, this);

    _finish_publisher = private_nh.advertise<std_msgs::Bool>(finish_topic, 1, true);

    _viz_obstacles_publisher = private_nh.advertise<visualization_msgs::Marker>(obstacle_viz_topic, 1, false);

    _prev_header.stamp = ros::Time::now();

    auto obstacles = prx::load_obstacles(environment);
    _obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

    _obstacles_marker.header.frame_id = "world";
    _obstacles_marker.header.stamp = ros::Time();
    _obstacles_marker.ns = "nodes";
    _obstacles_marker.id = 0;
    _obstacles_marker.type = visualization_msgs::Marker::LINE_LIST;
    _obstacles_marker.action = visualization_msgs::Marker::ADD;
    _obstacles_marker.pose.position.x = 0;
    _obstacles_marker.pose.position.y = 0;
    _obstacles_marker.pose.position.z = 0;
    _obstacles_marker.pose.orientation.x = 0.0;
    _obstacles_marker.pose.orientation.y = 0.0;
    _obstacles_marker.pose.orientation.z = 0.0;
    _obstacles_marker.pose.orientation.w = 1.0;
    _obstacles_marker.scale.x = 0.1;
    _obstacles_marker.scale.y = 0.1;
    _obstacles_marker.scale.z = 0.1;

    _obstacles_marker.color.a = 1.0;  // Don't forget to set the alpha!
    _obstacles_marker.color.r = 0.98;
    _obstacles_marker.color.g = 0.55;
    _obstacles_marker.color.b = 0.02;
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    if (msg->data)
    {
      to_file(true);
    }
  }

  void to_file(const bool collision = false)
  {
    gtsam::Values estimate{ _isam.calculateEstimate() };
    const std::string filename{ _output_dir + "/TE_" + _experiment_id + "_" + utils::timestamp() + ".txt" };
    const std::string filename_branch_gt{ _output_dir + "/TE_branch_gt_" + _experiment_id + "_" + utils::timestamp() +
                                          ".txt" };
    const std::string filename_data{ _output_dir + "/TE_data_" + _experiment_id + "_" + utils::timestamp() + ".txt" };

    DEBUG_VARS(filename);
    DEBUG_VARS(filename_branch_gt);
    DEBUG_VARS(filename_data);
    std::ofstream ofs(filename);
    std::ofstream ofs_data(filename_data);
    const double elapsed_time{ (ros::Time::now() - _start_time).toSec() };
    ofs_data << "ElapsedTime: " << elapsed_time << "\n";
    ofs_data << "Collision: " << (collision ? "true" : "false") << "\n";
    ofs_data << "ObstacleDistanceTolerance: " << _obstacle_distance_tolerance << "\n";
    ofs_data << "ObstacleMode: " << _mode << "\n";

    ofs << "# id key_x x[...] xCov[...] key_xdot xdot[...] xdotCov[...]\n";
    for (int i = 0; i < _id_x_hat; ++i)
    {
      ofs << i << " ";
      const StateKeys keys{ SystemInterface::keyState(-1, i) };
      estimates_to_file<0>(ofs, estimate, keys);
    }

    ofs.close();
    ofs_data.close();

    std_msgs::Bool msg;
    msg.data = true;
    _finish_publisher.publish(msg);
    _tree_recevied = false;

    ros::Rate rate(1);
    rate.sleep();
    ros::shutdown();
  }

  void estimate(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      add_observations();
    }
  }

  inline void control_callback(const ml4kp_bridge::SpacePointConstPtr msg)
  {
    ml4kp_bridge::copy(_u01, msg);
  }

  inline void stamped_control_callback(const ml4kp_bridge::SpacePointStampedConstPtr msg)
  {
    ml4kp_bridge::copy(_u01, msg);
  }

  bool query_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  void add_observations()
  {
    const bool new_observation{ query_tf() };
    if (new_observation and _tf.header.stamp > _prev_header.stamp and not _last_local_goal)
    {
      const double dt{ (_tf.header.stamp - _prev_header.stamp).toSec() };
      SystemInterface::copy(_z_new, _tf);
      _time_remaining = (_next_node_time - ros::Time::now()).toSec();

      const GraphValues graph_values_z{ SystemInterface::add_observation_factor(_id_x_hat,
                                                                                _id_x_hat + 1,           // no-lint
                                                                                _state_estimates, _u01,  // no-lint
                                                                                _z_new, dt, 0.01) };

      _isam2_result = _isam.update(graph_values_z.first, graph_values_z.second);

      _id_x_hat++;

      _key_u01 = SystemInterface::keyU(-1 * _id_x_hat, _x_next);

      // const StateKeys state_keys{ SystemInterface::keyState(-1, _id_x_hat) };
      // update_estimates<0>(_state_estimates, state_keys);

      // SystemInterface::copy(_feedback.xhat, _state_estimates);

      _prev_header = _tf.header;
    }
  }

private:
  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  inline void update_estimates(StateEstimates& state_estimates, const StateKeys& keys)
  {
    using EstimateType = typename std::tuple_element<I, StateEstimates>::type;
    std::get<I>(state_estimates) = _isam.calculateEstimate<EstimateType>(keys[I]);
    update_estimates<I + 1>(state_estimates, keys);
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>
  inline void update_estimates(StateEstimates& state_estimates, const StateKeys& keys)
  {
  }

  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  double compute_error(const StateKeys& keys)
  {
    using StateType = typename std::tuple_element<I, StateEstimates>::type;
    const gtsam::Key& key{ keys[I] };
    const StateType& x{ std::get<I>(_node_estimates) };
    const StateType x_sbmp{ _values.at<StateType>(key) };

    const Eigen::MatrixXd cov{ _isam.marginalCovariance(key) };
    const Eigen::MatrixXd S{ cov.inverse() };
    const StateType diff{ x - x_sbmp };
    return diff.transpose() * S * diff + compute_error<I + 1>(keys);
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  double compute_error(const StateKeys& keys)
  {
    return 0;
  }

  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    using StateType = typename std::tuple_element<I, StateEstimates>::type;
    const gtsam::Key key{ keys[I] };
    const StateType state{ estimate.at<StateType>(key) };
    const Eigen::MatrixXd cov{ _isam.marginalCovariance(key) };
    const Eigen::VectorXd diagonal{ cov.diagonal() };

    // ofs << i << " ";
    ofs << SF::formatter(key) << " ";
    for (int i = 0; i < diagonal.size(); ++i)
    {
      ofs << state[i] << " ";
    }
    for (int i = 0; i < diagonal.size(); ++i)
    {
      ofs << diagonal[i] << " ";
    }
    estimates_to_file<I + 1>(ofs, estimate, keys);
    // ofs << "\n";
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    ofs << "\n";
  }

  Values _values;
  // FactorGraph _factor_graph;

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;

  StateKeys _state_keys;
  StateEstimates _state_estimates;
  StateEstimates _node_estimates;
  StateEstimates _sbmp_nodes;

  gtsam::Key _key_last_esimated_state;
  gtsam::Key _key_u01;

  ml4kp_bridge::SpacePointStamped _control_stamped;
  prx_models::Tree _tree;

  ros::Subscriber _collision_subscriber;
  ros::Subscriber _control_subscriber;
  ros::Subscriber _stamped_control_subscriber;

  ros::Publisher _finish_publisher;
  ros::Publisher _viz_obstacles_publisher;

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  std::size_t _id_x_hat;
  ros::Timer _z_timer;

  State _state;
  Control _u01;
  Observation _z_new;

  double _time_remaining;

  bool _goal_received;

  std::uint64_t _x_next;

  bool _tree_recevied;
  motion_planning::tree_manager_t _tree_manager;
  ros::Time _next_node_time;
  ros::Time _start_time;
  bool _last_local_goal;

  // File/output
  std::string _output_dir;
  std::string _experiment_id;

  // Obstacle-relates stuff
  std::string _obstacle_mode, _mode;
  gtsam::NonlinearFactorGraph _obstacle_graph;
  double _obstacle_distance_tolerance;
  typename SystemInterface::ConfigFromState _config_from_state;
  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  typename ObstacleFactor::ToleranceResult _obstacle_tolerance_result;
  typename ObstacleFactor::DistanceResult _obstacle_distance_result;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;
  visualization_msgs::Marker _obstacles_marker;
};
}  // namespace motion_planning
