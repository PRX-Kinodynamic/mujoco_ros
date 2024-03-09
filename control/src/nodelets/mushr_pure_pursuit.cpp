#include <ml4kp_bridge/defs.h>
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ml4kp_bridge/TrajectoryStamped.h>
#include <prx_models/MushrObservation.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

namespace control
{
class mushr_pure_pursuit_t : public nodelet::Nodelet
{
public:
  mushr_pure_pursuit_t()
    : _trajectory_received(false), _pose_received(false), wheelbase(0.2965),  _nearest_index(0)
  {
  }

  void onInit()
  {
    ros::NodeHandle& nh{ getPrivateNodeHandle() };
    std::string trajectory_topic, pose_topic, control_topic, reset_topic, goal_pose_topic, goal_radius_topic;
    double frequency;

    nh.getParam("trajectory_topic", trajectory_topic);
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("control_topic", control_topic);
    nh.getParam("reset_topic", reset_topic);
    nh.getParam("goal_pose_topic", goal_pose_topic);
    nh.getParam("goal_radius_topic", goal_radius_topic);
    nh.getParam("frequency", frequency);
    nh.getParam("k_p", kp);
    nh.getParam("k_a", ka);
    nh.getParam("k_b", kb);
    nh.getParam("time_window", time_window);
    nh.getParam("time_ahead", time_ahead);

    _timer_duration = ros::Duration(1.0 / frequency);
    _window = time_window / 0.01;
    _lookahead = time_ahead / 0.01;
    _timer = nh.createTimer(_timer_duration, &mushr_pure_pursuit_t::control, this);
    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(2);

    _trajectory_subscriber = nh.subscribe(trajectory_topic, 1, &mushr_pure_pursuit_t::get_trajectory, this);
    _pose_subscriber = nh.subscribe(pose_topic, 1, &mushr_pure_pursuit_t::get_pose, this);
    _reset_subscriber = nh.subscribe(reset_topic, 1, &mushr_pure_pursuit_t::reset, this);
    _goal_pose_subscriber = nh.subscribe(goal_pose_topic, 1, &mushr_pure_pursuit_t::get_goal_pose, this);
    _goal_rad_subscriber = nh.subscribe(goal_radius_topic, 1, &mushr_pure_pursuit_t::get_goal_rad, this);
  }

  void get_goal_pose(const geometry_msgs::Pose2DConstPtr message)
  {
    goal_configuration = *message;
  }

  void get_goal_rad(const std_msgs::Float64ConstPtr message)
  {
    goal_radius = *message;
  }

  bool goal_reached()
  {
    if (!_pose_received)
    {
      return false;
    }
    return (std::hypot(goal_configuration.x - current_pose[0], goal_configuration.y - current_pose[1]) <
            goal_radius.data);
  }

  inline double angle_diff(const double a, const double b)
  {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  void control(const ros::TimerEvent& event)
  {
    if (goal_reached())
    {
      _trajectory_received = false;
      _pose_received = false;
      trajectory.clear();
      _reverse = 0;
    }
    if (_trajectory_received && _pose_received)
    {
      unsigned window_size = _window + _nearest_index;
      if (window_size >= trajectory.size())
        window_size = trajectory.size();

      double dist = std::numeric_limits<double>::max();
      for (unsigned i = _nearest_index + 1; i < window_size; ++i)
      {
        Eigen::Vector3d diff = trajectory[i] - current_pose;
        diff[2] = prx::norm_angle_pi(diff[2]);
        const double d = diff.norm();
        if (d < dist)
        {
          dist = d;
          _nearest_index = i;
        }
      }

      unsigned _lookahead_idx = _nearest_index + _lookahead;
      if (_lookahead_idx >= trajectory.size())
        _lookahead_idx = trajectory.size() - 1;

      const Eigen::Vector2d delta = trajectory[_lookahead_idx].head(2) - current_pose.head(2);
      const double theta = current_pose[2];

      const double p = std::sqrt(std::pow(delta[0], 2) + std::pow(delta[1], 2));
      const double a = angle_diff(std::atan2(delta[1], delta[0]), theta);
      const int curr_reverse = (-prx::constants::pi / 2.0 < a && a <= prx::constants::pi / 2.0) ? 1 : -1;
      const double beta = prx::norm_angle_pi(angle_diff(-theta, a) + trajectory[_lookahead_idx][2]);
      if (_reverse == 0)
        _reverse = curr_reverse;

      const double v = kp * p;
      const double omega = (ka * a + kb * beta);

      _ctrl.point[0].data = std::max(-1.0, std::min(1.0, omega));
      _ctrl.point[1].data = std::max(-1.0, std::min(1.0, _reverse * v));
    }
    else
    {
      _ctrl.point[0].data = 0;
      _ctrl.point[1].data = 0;
    }
    _control_publisher.publish(_ctrl);
  }

  void reset(const std_msgs::EmptyConstPtr message)
  {
    _trajectory_received = false;
    _pose_received = false;
    _reverse = 0;
    trajectory.clear();
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStampedConstPtr message)
  {
    _trajectory = *message;
    _trajectory_received = true;
    _nearest_index = 0;

    trajectory.clear();
    for (int i = 0; i < _trajectory.trajectory.data.size(); i++)
    {
      Eigen::VectorXd point = Eigen::VectorXd::Zero(3);
      point << _trajectory.trajectory.data[i].point[0].data, _trajectory.trajectory.data[i].point[1].data,
          _trajectory.trajectory.data[i].point[2].data;
      trajectory.push_back(point);
    }
  }

  void get_pose(const prx_models::MushrObservationConstPtr message)
  {
    _pose = *message;
    _pose_received = true;

    Eigen::Quaterniond q(_pose.pose.orientation.w, _pose.pose.orientation.x, _pose.pose.orientation.y,
                         _pose.pose.orientation.z);
    Eigen::Vector3d euler = prx::quaternion_to_euler(q);

    current_pose.resize(3);
    current_pose << _pose.pose.position.x, _pose.pose.position.y, euler[2];
  }

private:
  bool _trajectory_received, _pose_received;
  double wheelbase, kp, ka, kb, time_window, time_ahead;
  unsigned _nearest_index, _window, _lookahead;
  int _reverse;

  ros::Subscriber _trajectory_subscriber, _pose_subscriber, _reset_subscriber, _goal_pose_subscriber,
      _goal_rad_subscriber;
  ros::Publisher _control_publisher;
  ros::Timer _timer;
  ros::Duration _timer_duration;

  ml4kp_bridge::TrajectoryStamped _trajectory;
  prx_models::MushrObservation _pose;
  geometry_msgs::Pose2D goal_configuration;
  std_msgs::Float64 goal_radius;

  Eigen::VectorXd nearest_point, current_pose;
  std::vector<Eigen::VectorXd> trajectory;

  ml4kp_bridge::SpacePoint _ctrl;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::mushr_pure_pursuit_t, nodelet::Nodelet);
