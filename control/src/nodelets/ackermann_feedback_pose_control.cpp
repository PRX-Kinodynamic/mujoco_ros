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
class ackermann_feedback_pose_control_t : public nodelet::Nodelet
{
  static constexpr std::size_t steering_idx{ prx_models::mushr_t::control::steering_idx };
  static constexpr std::size_t velocity_idx{ prx_models::mushr_t::control::velocity_idx };

public:
  ackermann_feedback_pose_control_t()
    : _tf_listener(_tf_buffer), _reverse(0), _goal_radius(0), _desired_pose_received(false)
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    std::string pose_topic;
    std::string control_topic;
    std::string world_frame;
    std::string robot_frame;
    std::string goal_radius_topic;
    double kp;
    double ka;
    double kb;

    NODELET_PARAM_SETUP(private_nh, pose_topic);
    NODELET_PARAM_SETUP(private_nh, control_topic);
    NODELET_PARAM_SETUP(private_nh, world_frame);
    NODELET_PARAM_SETUP(private_nh, robot_frame);
    NODELET_PARAM_SETUP(private_nh, goal_radius_topic);
    NODELET_PARAM_SETUP(private_nh, kp);
    NODELET_PARAM_SETUP(private_nh, ka);
    NODELET_PARAM_SETUP(private_nh, kb);

    _kp = kp;
    _ka = ka;
    _kb = kb;

    _robot_frame = robot_frame;
    _world_frame = world_frame;

    _pose_subscriber = private_nh.subscribe(pose_topic, 1, &ackermann_feedback_pose_control_t::get_desired_pose, this);
    _goal_rad_subscriber =
        private_nh.subscribe(goal_radius_topic, 1, &ackermann_feedback_pose_control_t::get_goal_rad, this);
    _timer = private_nh.createTimer(ros::Duration(0.01), &ackermann_feedback_pose_control_t::control, this);
    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(2);
  }

  inline double angle_diff(const double& a, const double& b)
  {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  void get_goal_rad(const std_msgs::Float64ConstPtr message)
  {
    _goal_radius = message->data;
  }

  void control(const ros::TimerEvent& event)
  {
    if (get_current_pose() and _desired_pose_received and not goal_reached())
    {
      regulator(_current_pose, _desired_pose, _reverse);
    }
    else
    {
      _reverse = 0;
    }
    _control_publisher.publish(_ctrl);
  }

  void regulator(Eigen::Vector3d current_state, Eigen::Vector3d desired_state, int& reverse)
  {
    const Eigen::Vector3d delta{ desired_state - current_state };
    const double theta{ current_state[2] };

    const double p{ std::sqrt(std::pow(delta[0], 2) + std::pow(delta[1], 2)) };
    const double a{ angle_diff(std::atan2(delta[1], delta[0]), theta) };
    static const double pi2{ prx::constants::pi / 2.0 };
    const int curr_reverse{ (-pi2 < a and a <= pi2) ? 1 : -1 };
    if (reverse == 0)
      reverse = curr_reverse;
    // const double beta{ reverse > 0 ? +theta + a - desired_state[2] : theta - a + desired_state[2] };
    // const double beta{  -theta - a + desired_state[2] };
    const double beta{ prx::norm_angle_pi(angle_diff(-theta, -a) + desired_state[2]) };
    const double v{ _kp * p };
    const double omega{ (_ka * a + _kb * beta) };
    DEBUG_VARS(a, curr_reverse, reverse, theta, beta, omega);
    // return Eigen::Vector2d(curr_reverse * omega, curr_reverse * v);
    _ctrl.point[steering_idx].data = omega;
    _ctrl.point[velocity_idx].data = reverse * v;
  }

  bool get_current_pose()
  {
    try
    {
      _tf_in = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      tf2::fromMsg(_tf_in.transform.rotation, _quat);
      const double theta{ prx::quaternion_to_euler(_quat)[2] };

      _current_pose[0] = _tf_in.transform.translation.x;
      _current_pose[1] = _tf_in.transform.translation.y;
      _current_pose[2] = theta;
      // DEBUG_VARS(_current_pose.transpose());
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }
  void get_desired_pose(const geometry_msgs::Pose2D& goal_configuration)
  {
    _desired_pose[0] = goal_configuration.x;
    _desired_pose[1] = goal_configuration.y;
    _desired_pose[2] = goal_configuration.theta;
    _desired_pose_received = true;
    _reverse = 0;
  }

  bool goal_reached()
  {
    if ((_desired_pose.head(2) - _current_pose.head(2)).norm() < _goal_radius)
    {
      _ctrl.point[0].data = 0;
      _ctrl.point[1].data = 0;
      _desired_pose_received = false;
      return true;
    }
    return false;
  }

  ros::Timer _timer;
  ros::Subscriber _pose_subscriber, _goal_rad_subscriber;
  ros::Publisher _control_publisher;

  ml4kp_bridge::SpacePoint _ctrl;

  Eigen::Vector3d _desired_pose;
  Eigen::Vector3d _current_pose;
  Eigen::Quaterniond _quat;
  double _kp;
  double _ka;
  double _kb;
  double _goal_radius;

  int _reverse;
  bool _desired_pose_received;

  std::string _robot_frame, _world_frame;

  geometry_msgs::TransformStamped _tf_in;
  tf2_ros::TransformListener _tf_listener;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformBroadcaster _tf_broadcaster;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::ackermann_feedback_pose_control_t, nodelet::Nodelet);