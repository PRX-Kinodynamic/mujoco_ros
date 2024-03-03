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
class mushr_stanley_t : public nodelet::Nodelet
{
public:
  mushr_stanley_t()
    : _trajectory_received(false), _pose_received(false), wheelbase(0.2965), k_path(0.5), current_speed(0.0)
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
    nh.getParam("frequency", frequency);
    nh.getParam("reset_topic", reset_topic);
    nh.getParam("goal_pose_topic", goal_pose_topic);
    nh.getParam("goal_radius_topic", goal_radius_topic);

    _timer_duration = ros::Duration(1.0 / frequency);
    discretization = 1.0 / (0.01 * frequency);
    _timer = nh.createTimer(_timer_duration, &mushr_stanley_t::control, this);
    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(2);

    _trajectory_subscriber = nh.subscribe(trajectory_topic, 1, &mushr_stanley_t::get_trajectory, this);
    _pose_subscriber = nh.subscribe(pose_topic, 1, &mushr_stanley_t::get_pose, this);
    _reset_subscriber = nh.subscribe(reset_topic, 1, &mushr_stanley_t::reset, this);
    _goal_pose_subscriber = nh.subscribe(goal_pose_topic, 1, &mushr_stanley_t::get_goal_pose, this);
    _goal_rad_subscriber = nh.subscribe(goal_radius_topic, 1, &mushr_stanley_t::get_goal_rad, this);
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
    return (std::hypot(goal_configuration.x - current_state.point[0].data,
                       goal_configuration.y - current_state.point[1].data) < goal_radius.data);
  }

  void control(const ros::TimerEvent& event)
  {
    if (goal_reached())
    {
      _trajectory_received = false;
      _pose_received = false;
      discretized_trajectory.clear();
    }
    if (_trajectory_received && _pose_received)
    {
      std::vector<Eigen::Vector2d> diffs, projections;
      std::vector<double> l2s, dots, ts, dists;
      for (size_t i = 1; i < discretized_trajectory.size(); ++i)
      {
        diffs.push_back(discretized_trajectory[i] - discretized_trajectory[i - 1]);
        l2s.push_back(diffs.back().norm());
      }

      for (size_t i = 0; i < discretized_trajectory.size() - 1; ++i)
      {
        dots.push_back((front_axle_position - discretized_trajectory[i]).dot(diffs[i]) / l2s[i]);
        if (dots[i] / l2s[i] < 0.0)
          ts.push_back(0.0);
        else if (dots[i] / l2s[i] > 1.0)
          ts.push_back(1.0);
        else
          ts.push_back(dots[i] / l2s[i]);
        projections.push_back(discretized_trajectory[i] + (ts[i] * diffs[i]));
      }
      double min_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < projections.size(); ++i)
      {
        dists.push_back((front_axle_position - projections[i]).norm());
        if (dists[i] < min_dist)
        {
          min_dist = dists[i];
          _nearest_index = i;
        }
      }
      nearest_point = projections[_nearest_index];

      Eigen::Vector2d vec_dist_nearest_point = front_axle_position - nearest_point;
      Eigen::Vector2d front_axle_vec_rotation;
      front_axle_vec_rotation << std::cos(current_state.point[2].data - prx::constants::pi / 2.0),
          std::sin(current_state.point[2].data - prx::constants::pi / 2.0);
      double cross_track_error = vec_dist_nearest_point.dot(front_axle_vec_rotation);

      double theta_line = discretized_orientations[_nearest_index];
      double theta_e = prx::norm_angle_pi(theta_line - current_state.point[2].data);
      double theta_d = std::atan2(k_path * cross_track_error, current_state.point[3].data);

      _ctrl.point[0].data = std::max(-1.0, std::min(1.0, theta_e + theta_d));
      _ctrl.point[1].data = discretized_velocities[_nearest_index];
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
    discretized_trajectory.clear();
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStampedConstPtr message)
  {
    _trajectory = *message;
    _trajectory_received = true;

    discretized_trajectory.clear();
    for (int i = 0; i < _trajectory.trajectory.data.size(); i += discretization)
    {
      Eigen::VectorXd point = Eigen::VectorXd::Zero(2);
      point << _trajectory.trajectory.data[i].point[0].data, _trajectory.trajectory.data[i].point[1].data;
      // for (int j = 0; j < _trajectory.trajectory.data[i].point.size(); ++j)
      discretized_trajectory.push_back(point);
      discretized_orientations.push_back(_trajectory.trajectory.data[i].point[2].data);
      discretized_velocities.push_back(_trajectory.trajectory.data[i].point[3].data);
    }

    if (_trajectory.trajectory.data.size() % discretization != 0)
    {
      Eigen::VectorXd point = Eigen::VectorXd::Zero(2);
      point << _trajectory.trajectory.data.back().point[0].data, _trajectory.trajectory.data.back().point[1].data;
      // for (int j = 0; j < _trajectory.trajectory.data.back().point.size(); ++j)
      discretized_trajectory.push_back(point);
      discretized_orientations.push_back(_trajectory.trajectory.data.back().point[2].data);
      discretized_velocities.push_back(_trajectory.trajectory.data.back().point[3].data);
    }

    ROS_INFO("Discretized trajectory size: %d", discretized_trajectory.size());
  }

  void get_pose(const prx_models::MushrObservationConstPtr message)
  {
    if (_pose_received)
    {
      double dt = (message->header.stamp - _pose.header.stamp).toSec();
      current_speed = std::sqrt(std::pow(message->pose.position.x - _pose.pose.position.x, 2) +
                                std::pow(message->pose.position.y - _pose.pose.position.y, 2)) /
                      dt;
    }

    _pose = *message;
    _pose_received = true;

    Eigen::Quaterniond q(_pose.pose.orientation.w, _pose.pose.orientation.x, _pose.pose.orientation.y,
                         _pose.pose.orientation.z);
    Eigen::Vector3d euler = prx::quaternion_to_euler(q);

    current_state.point.resize(4);
    current_state.point[0].data = _pose.pose.position.x;
    current_state.point[1].data = _pose.pose.position.y;
    current_state.point[2].data = euler[2];
    current_state.point[3].data = current_speed;

    front_axle_position.resize(2);
    front_axle_position << _pose.pose.position.x + wheelbase * std::cos(euler[2]),
        _pose.pose.position.y + wheelbase * std::sin(euler[2]);
  }

private:
  bool _trajectory_received, _pose_received;
  int discretization;
  double wheelbase, k_path, current_speed;
  size_t _nearest_index;

  ros::Subscriber _trajectory_subscriber, _pose_subscriber, _reset_subscriber, _goal_pose_subscriber,
      _goal_rad_subscriber;
  ros::Publisher _control_publisher;
  ros::Timer _timer;
  ros::Duration _timer_duration;

  ml4kp_bridge::TrajectoryStamped _trajectory;
  prx_models::MushrObservation _pose;
  ml4kp_bridge::SpacePoint current_state;
  geometry_msgs::Pose2D goal_configuration;
  std_msgs::Float64 goal_radius;

  Eigen::VectorXd front_axle_position, nearest_point;
  // TODO: Figure out nice Eigen way to do this
  std::vector<Eigen::Vector2d> discretized_trajectory;
  std::vector<double> discretized_orientations, discretized_velocities;

  ml4kp_bridge::SpacePoint _ctrl;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::mushr_stanley_t, nodelet::Nodelet);
