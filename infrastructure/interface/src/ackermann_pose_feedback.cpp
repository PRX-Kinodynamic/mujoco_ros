#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ml4kp_bridge/TrajectoryStamped.h>
#include <ml4kp_bridge/PlanStamped.h>
#include <ml4kp_bridge/defs.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <XmlRpcValue.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "interface/defs.hpp"
#include "interface/execution_status.hpp"
#include "interface/rosbag_record.hpp"
#include "interface/StampedMarkers.h"

class feedback_ctrl_t
{
public:
  feedback_ctrl_t(ros::NodeHandle& nh, const std::string pose_topic, const std::string control_topic, double kp,
                  double ka, double kb, const std::string world_frame, const std::string robot_frame,
                  const std::string goal_rad_topic)
    : _reverse(0)
    , _kp(kp)
    , _ka(ka)
    , _kb(kb)
    , _tf_listener(_tf_buffer)
    , _world_frame(world_frame)
    , _robot_frame(robot_frame)
    , _desired_pose_received(false)
    , _goal_radius(0)
  {
    _pose_subscriber = nh.subscribe(pose_topic, 1, &feedback_ctrl_t::get_desired_pose, this);
    _goal_rad_subscriber = nh.subscribe(goal_rad_topic, 1, &feedback_ctrl_t::get_goal_rad, this);
    _timer = nh.createTimer(ros::Duration(0.01), &feedback_ctrl_t::control, this);
    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(2);
  }

private:
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
    const double a{ std::atan2(delta[1], delta[0]) - theta };
    static const double pi2{ prx::constants::pi / 2.0 };
    const int curr_reverse{ (-pi2 < a and a <= pi2) ? 1 : -1 };
    if (reverse == 0)
      reverse = curr_reverse;
    // const double beta{ reverse > 0 ? +theta + a - desired_state[2] : theta - a + desired_state[2] };
    const double beta{ -theta - a + desired_state[2] };

    const double v{ _kp * p };
    const double omega{ (_ka * a + _kb * beta) };
    // return Eigen::Vector2d(curr_reverse * omega, curr_reverse * v);
    _ctrl.point[0].data = curr_reverse * omega;
    _ctrl.point[1].data = reverse * v;
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
  }

  bool goal_reached()
  {
    if ((_desired_pose.head(2) - _current_pose.head(2)).norm() < _goal_radius)
    {
      _ctrl.point[0].data = 0;
      _ctrl.point[1].data = 0;
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ackermann_pose_feedback");

  ros::NodeHandle nh("~");

  std::string topic_name;
  std::string video_filename;

  double kp;
  double ka;
  double kb;
  std::string goal_radius_topic;

  std::string pose_topic, control_topic;
  std::string robot_frame, world_frame;

  ROS_PARAM_SETUP(nh, pose_topic);
  ROS_PARAM_SETUP(nh, control_topic);
  ROS_PARAM_SETUP(nh, kp);
  ROS_PARAM_SETUP(nh, ka);
  ROS_PARAM_SETUP(nh, kb);
  ROS_PARAM_SETUP(nh, robot_frame);
  ROS_PARAM_SETUP(nh, world_frame);
  ROS_PARAM_SETUP(nh, goal_radius_topic);

  feedback_ctrl_t ctrl(nh, pose_topic, control_topic, kp, ka, kb, world_frame, robot_frame, goal_radius_topic);

  ros::spin();

  return 0;
}