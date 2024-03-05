#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

// #include <interface/utils.hpp>
// #include <interface/utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <prx_models/mj_mushr.hpp>
#include <utils/dbg_utils.hpp>

namespace control
{
class determine_carrot_t : public nodelet::Nodelet
{
  using Control = Eigen::Vector<double, 2>;
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };
  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };

public:
  determine_carrot_t() : _tf_listener(_tf_buffer), _closest_state_idx(0)
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    double simulation_step;
    int window_size;
    double time_ahead;
    std::string trajectory_topic;
    std::string world_frame;
    std::string robot_frame;
    std::string carrot_topic;

    NODELET_PARAM_SETUP(private_nh, trajectory_topic);
    NODELET_PARAM_SETUP(private_nh, world_frame);
    NODELET_PARAM_SETUP(private_nh, robot_frame);
    NODELET_PARAM_SETUP(private_nh, carrot_topic);
    NODELET_PARAM_SETUP(private_nh, simulation_step);
    NODELET_PARAM_SETUP(private_nh, window_size);
    NODELET_PARAM_SETUP(private_nh, time_ahead);

    prx_assert(window_size > 0, "Window size must be positive!");
    prx_assert(time_ahead > 0, "Time ahead must be positive!");
    prx::simulation_step = simulation_step;
    _window = window_size;
    _robot_frame = robot_frame;
    _world_frame = world_frame;
    _carrot_time_ahead = time_ahead;

    const std::string plant_name{ "mushr" };
    const std::string plant_path{ "mushr" };

    _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(_plant != nullptr, "Couldn't create plant");
    _state_space = _plant->get_state_space();
    _traj = std::make_shared<prx::trajectory_t>(_state_space);

    _trajectory_subscriber = private_nh.subscribe(trajectory_topic, 1, &determine_carrot_t::get_trajectory, this);
    _timer = private_nh.createTimer(ros::Duration(0.01), &determine_carrot_t::compute_carrot, this);
    _carrot_publisher = private_nh.advertise<geometry_msgs::Pose2D>(carrot_topic, 1, true);
  }

  void compute_carrot(const ros::TimerEvent& event)
  {
    if (get_current_pose() and _traj->size() > 0)
    {
      get_closest_state();
      _carrot_idx = _closest_state_idx + std::floor(_carrot_time_ahead / prx::simulation_step);
      _carrot_idx = std::min(_carrot_idx, _traj->size() - 1);
      _carrot.x = _traj->at(_carrot_idx)->at(0);
      _carrot.y = _traj->at(_carrot_idx)->at(1);
      _carrot.theta = _traj->at(_carrot_idx)->at(2);
      _carrot_publisher.publish(_carrot);
    }
  }

  inline double angle_diff(const double& a, const double& b)
  {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
  {
    Eigen::Vector3d diff{ a - b };
    diff[2] = angle_diff(a[2], b[2]);
    return diff.norm();
  }

  void get_closest_state()
  {
    // DEBUG_PRINT;
    const unsigned window_size{ std::min(_window + _closest_state_idx, _traj->size()) };

    // DEBUG_VARS(_traj->size(), _closest_state_idx, states_left, window_size);

    double dist{ distance(Vec(_traj->at(_closest_state_idx)).head(3), _current_pose) };
    // NODELET_INFO_STREAM("---------------------------------:");
    // NODELET_INFO_STREAM("Traj size: " << _traj->size());
    // NODELET_INFO_STREAM("Current pose: " << _current_pose.transpose());
    // NODELET_INFO_STREAM("Prev pt: " << Vec(_traj->at(_closest_state_idx)).head(3).transpose()
    //                                 << " prev: " << _closest_state_idx);
    // NODELET_INFO_STREAM("initial_dist: " << dist);
    for (unsigned i = _closest_state_idx + 1; i < window_size; ++i)
    {
      const double new_dist{ distance(Vec(_traj->at(i)).head(3), _current_pose) };
      // NODELET_INFO_STREAM("i: " << i << " Pt:" << _traj->at(i) << " dist: " << new_dist);
      if (new_dist < dist)
      {
        dist = new_dist;
        _current_state_in_traj = _traj->at(i);
        _closest_state_idx = i;
      }
    }

    // NODELET_INFO_STREAM("Closest pt: " << Vec(_traj->at(_closest_state_idx)).head(3).transpose()
    //                                    << " closest_idx: " << _closest_state_idx);
    // NODELET_INFO_STREAM("curr dist: " << dist);
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStampedConstPtr message)
  {
    ml4kp_bridge::copy(_traj, message);
    _closest_state_idx = 0;
  }

  bool get_current_pose()
  {
    try
    {
      _tf_in = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      prx_models::copy(_current_pose, _tf_in);
      // tf2::fromMsg(_tf_in.transform.rotation, _quat);
      // const double theta{ prx::quaternion_to_euler(_quat)[2] };

      // _current_pose[0] = _tf_in.transform.translation.x;
      // _current_pose[1] = _tf_in.transform.translation.y;
      // _current_pose[2] = theta;
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  ros::Timer _timer;
  ros::Subscriber _trajectory_subscriber;
  ros::Publisher _carrot_publisher;

  std::string _robot_frame, _world_frame;

  double _carrot_time_ahead;
  unsigned _closest_state_idx, _window, _carrot_idx;
  Eigen::Vector3d _current_pose;
  Eigen::Quaterniond _quat;

  geometry_msgs::Pose2D _carrot;

  prx::system_ptr_t _plant;
  prx::space_t* _state_space;
  prx::space_point_t _current_state_in_traj;
  std::shared_ptr<prx::trajectory_t> _traj;

  geometry_msgs::TransformStamped _tf_in;
  tf2_ros::TransformListener _tf_listener;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformBroadcaster _tf_broadcaster;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::determine_carrot_t, nodelet::Nodelet);