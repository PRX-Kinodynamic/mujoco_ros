#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>

namespace interface
{
template <typename Observation, typename Control>
class plant_estimator_t : public nodelet::Nodelet
{
  using Transform = Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;

public:
  plant_estimator_t() : _robot_transform(Transform::Identity())
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    std::string world_frame{};
    std::string robot_frame{};
    std::string observation_topic{};
    std::string control_topic{};

    NODELET_PARAM_SETUP(private_nh, world_frame);
    NODELET_PARAM_SETUP(private_nh, robot_frame);
    NODELET_PARAM_SETUP(private_nh, observation_topic);
    NODELET_PARAM_SETUP(private_nh, control_topic);

    _world_frame = world_frame;
    _robot_frame = robot_frame;

    _header.seq = 0;
    _header.stamp = ros::Time::now();
    _header.frame_id = "world";

    _pose_timer = private_nh.createTimer(ros::Duration(0.0333), &plant_estimator_t::estimate_pose, this);
    _observation_publisher = private_nh.advertise<Observation>(observation_topic, 1, true);
    _control_subscriber = private_nh.subscribe(control_topic, 1, &plant_estimator_t::control_callback, this);
  }

  void control_callback(const Control& control)
  {
    _most_recent_control = control;
  }

  void estimate_pose(const ros::TimerEvent& event)
  {
    if (_tf_listener.canTransform(_robot_frame, _world_frame, ros::Time(0)))
    {
      _tf_listener.lookupTransform(_world_frame, _robot_frame, ros::Time(0), _tf_robot);
      tf::transformTFToEigen(_tf_robot, _robot_transform);
      linear_speed.data = _most_recent_control.velocity.data * 0.6228;
      // linear_speed.data = _most_recent_control.velocity.data * 0.6343;
      _previous_robot_transform = _robot_transform;
    }
    copy(_observation, _robot_transform);
    _header.seq++;
    _header.stamp = ros::Time::now();
    _observation.header = _header;
    _observation.float_extra.clear();
    _observation.float_extra.push_back(linear_speed);
    _observation_publisher.publish(_observation);
  }

  std_msgs::Header _header;
  ros::Timer _pose_timer;

  ros::Subscriber _control_subscriber;
  ros::Publisher _observation_publisher;
  Observation _observation;
  Control _most_recent_control;

  Transform _robot_transform, _previous_robot_transform;
  std_msgs::Float64 linear_speed;
  tf::StampedTransform _tf_robot;
  tf::TransformListener _tf_listener;

  std::string _world_frame;
  std::string _robot_frame;
};
}  // namespace interface