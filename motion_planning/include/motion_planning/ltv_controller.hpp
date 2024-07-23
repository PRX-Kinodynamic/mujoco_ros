#pragma once
#include <algorithm>
#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
#include <motion_planning/motion_planning_types.hpp>
namespace motion_planning
{

template <class Base>
class ltv_controller_t : public Base
{
  using Derived = ltv_controller_t<Base>;

public:
  ltv_controller_t() : _tf_listener(_tf_buffer), _current_idx(-1)
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string traj_topic_name{};
    std::string control_topic{};
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    int& lookahead{ _lookahead };
    int frequency{};

    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, traj_topic_name);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, lookahead);
    PARAM_SETUP(private_nh, frequency);

    // subscribers
    _traj_subscriber = private_nh.subscribe(traj_topic_name, 1, &Derived::trajectory_callback, this);

    // publishers
    _stamped_control_publisher =
        private_nh.advertise<ml4kp_bridge::SpacePointStamped>(control_topic + "_stamped", 1, true);

    const ros::Duration timer_duration{ ros::Duration(1.0 / frequency) };

    _timer = private_nh.createTimer(timer_duration, &Derived::timer_callback, this);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "LtvPosController";
  }

protected:
  void timer_callback(const ros::TimerEvent& event)
  {
    if (_current_idx >= 0)
    {
      update_tf();
      find_closest();
      const Eigen::Vector2d u{ ctrl() };
      ml4kp_bridge::copy(_control_stamped.space_point, u);
      _control_stamped.header.seq++;
      _control_stamped.header.stamp = ros::Time::now();
      _stamped_control_publisher.publish(_control_stamped);
    }
  }

  inline Eigen::Vector2d ctrl()
  {
    const int idx{ std::min(_current_idx + _lookahead, static_cast<int>(_traj.data.size() - 1)) };
    const Eigen::Vector2d xi{ _traj.data[idx].point[0], _traj.data[idx].point[1] };
    // DEBUG_VARS(_z.transpose(), xi.transpose());
    return xi - _z;
  }

  void trajectory_callback(const ml4kp_bridge::TrajectoryConstPtr msg)
  {
    _traj = *msg;
    _current_idx = 0;
  }

  void find_closest()
  {
    double min{ std::numeric_limits<double>::max() };
    int min_idx{ -1 };
    Eigen::Matrix2d Q{ Eigen::Matrix2d::Identity() };
    for (int i = _current_idx; i < _traj.data.size(); ++i)
    {
      const Eigen::Vector2d xi{ _traj.data[i].point[0], _traj.data[i].point[1] };
      const Eigen::Vector2d xdiff{ _z - xi };
      const double val{ xdiff.transpose() * Q * xdiff };
      if (val < min)
      {
        min = val;
        min_idx = i;
      }
    }
    _current_idx = min_idx;
  }

  bool update_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      _z[0] = _tf.transform.translation.x;
      _z[1] = _tf.transform.translation.y;
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  // Topic names
  std::string _tree_topic_name;
  std::string _viz_control_name;

  // Subscribers
  ros::Subscriber _traj_subscriber;

  // Publishers
  ros::Publisher _stamped_control_publisher;

  // Timers
  ros::Timer _timer;

  // TF
  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;
  Eigen::Vector2d _z;

  // Traj
  ml4kp_bridge::Trajectory _traj;
  ml4kp_bridge::SpacePointStamped _control_stamped;

  int _lookahead;
  int _current_idx;
};
}  // namespace motion_planning