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
template<typename Observation>
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

    NODELET_PARAM_SETUP(private_nh, world_frame);
    NODELET_PARAM_SETUP(private_nh, robot_frame);
    NODELET_PARAM_SETUP(private_nh, observation_topic);

    _world_frame = world_frame;
    _robot_frame = robot_frame;

    _header.seq = 0;
    _header.stamp = ros::Time::now();
    _header.frame_id = "world";

    _pose_timer = private_nh.createTimer(ros::Duration(0.0333), &plant_estimator_t::estimate_pose, this);
    _observation_publisher = private_nh.advertise<Observation>(observation_topic, 1, true);
  }

  void estimate_pose(const ros::TimerEvent& event)
  {
    if (_tf_listener.canTransform(_robot_frame, _world_frame, ros::Time(0)))
    {
      _tf_listener.lookupTransform(_world_frame, _robot_frame, ros::Time(0), _tf_robot);
      tf::transformTFToEigen(_tf_robot, _robot_transform);
    }
    copy(_observation, _robot_transform);
    _header.seq++;
    _header.stamp = ros::Time::now();
    _observation.header = _header;
    _observation_publisher.publish(_observation);
  }

  std_msgs::Header _header;
  ros::Timer _pose_timer;

  ros::Publisher _observation_publisher;
  Observation _observation;

  Transform _robot_transform;
  tf::StampedTransform _tf_robot;
  tf::TransformListener _tf_listener;

  std::string _world_frame;
  std::string _robot_frame;

};
}  // namespace interface