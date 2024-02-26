#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.h>
#include <utils/rosparams_utils.hpp>
#include <interface/StampedMarkers.h>
#include <aruco/aruco_nano.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <utils/dbg_utils.hpp>

namespace estimation
{
class robot_estimation_simple_nodelet_t : public nodelet::Nodelet
{
public:
  robot_estimation_simple_nodelet_t() : _tf_listener(_tf_buffer)
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    std::string robot_frame_0;
    std::string robot_frame_1;

    NODELET_PARAM_SETUP(private_nh, robot_frame_0);
    NODELET_PARAM_SETUP(private_nh, robot_frame_1);

    _robot_frames.push_back(robot_frame_0);
    _robot_frames.push_back(robot_frame_1);

    _world_frame = "world";
    _tf_out.header.frame_id = _world_frame;

    _tf_out.child_frame_id = "robot_0";
    _timer = private_nh.createTimer(ros::Duration(0.05), &robot_estimation_simple_nodelet_t::estimate, this);
  }

  Eigen::Quaterniond to_quat(geometry_msgs::TransformStamped& tf)
  {
    const geometry_msgs::Quaternion& q{ tf.transform.rotation };
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  Eigen::Vector4d quat_to_vec(const geometry_msgs::TransformStamped& tf)
  {
    const geometry_msgs::Quaternion& q{ tf.transform.rotation };
    return Eigen::Vector4d(q.w, q.x, q.y, q.z);
  }

  Eigen::Quaterniond average_quat(const geometry_msgs::TransformStamped& tf1,
                                  const geometry_msgs::TransformStamped& tf2)
  {
    const Eigen::Vector4d q1{ quat_to_vec(tf1) };
    const Eigen::Vector4d q2{ quat_to_vec(tf2) };

    // quat;
    const double w{ (q1.w() + q2.w()) / 2.0 };
    const double x{ (q1.x() + q2.x()) / 2.0 };
    const double y{ (q1.y() + q2.y()) / 2.0 };
    const double z{ (q1.z() + q2.z()) / 2.0 };
    DEBUG_VARS(q1)
    DEBUG_VARS(q2)
    DEBUG_VARS(w, x, y, z)
    return Eigen::Quaterniond(w, x, y, z);
    // const double w1{ q1[0] };
    // const double w2{ q2[0] };
    // const double q1Tq2{ q1.transpose() * q2 };
    // const double z{ std::sqrt(std::pow(w1 - w2, 2) + 4 * w1 * w2 * std::pow(q1Tq2, 2)) };
    // const double m_q1{ std::sqrt((w1 * (w1 - w2 + z)) / (z * (w1 + w2 + z))) };
    // const double m_q2{ std::sqrt((w2 * (w2 - w1 + z)) / (z * (w1 + w2 + z))) };
    // const int sign_q1Tq2{ q1Tq2 >= 0 ? 1 : -1 };
    // const Eigen::Vector4d res{ m_q1 * q1 + sign_q1Tq2 * m_q2 * q2 };
    // return Eigen::Quaterniond(res[0], res[1], res[2], res[3]);
  }

  void estimate(const ros::TimerEvent& event)
  {
    int obsevations{ 0 };
    try
    {
      _tf_in_1 = _tf_buffer.lookupTransform(_world_frame, _robot_frames[0], ros::Time(0));
      // _tf_out.transform.translation = _tf_in_1.transform.translation;
      // _quat = to_quat(_tf_in_1);
      // w_quats.block<4, 1>(0, 0) = quat_to_vec(_tf_in_1);
      if ((ros::Time::now() - _tf_in_1.header.stamp) < ros::Duration(0.1))
      {
        _tf_out.transform.translation = _tf_in_1.transform.translation;
        _quat = to_quat(_tf_in_1);
        obsevations++;
      }
    }
    catch (tf2::TransformException& ex)
    {
      // ROS_WARN("%s", ex.what());
    }

    try
    {
      // ROS_INFO_STREAM("C0: " << _tf_in_1.transform.rotation);
      _tf_in_2 = _tf_buffer.lookupTransform(_world_frame, _robot_frames[1], ros::Time(0));
      // _tf_out.transform.translation = _tf_in_1.transform.translation;
      // w_quats.block<4, 1>(0, 1) = quat_to_vec(_tf_in);
      // ROS_INFO_STREAM("C1: " << _tf_in_2.transform.rotation);
      // (4 x 2)(2*4)
      // Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> eg(w_quats * w_quats.transpose());
      // auto eigenvalues = eg.eigenvalues();
      // std::cout << "The eigenvalues of A are:" << std::endl << eigenvalues << std::endl;
      // _tf_out.transform.translation.x = (_tf_out.transform.translation.x + _tf_in_2.transform.translation.x) / 2.0;
      // _tf_out.transform.translation.y = (_tf_out.transform.translation.y + _tf_in_2.transform.translation.y) / 2.0;
      // _tf_out.transform.translation.z = (_tf_out.transform.translation.z + _tf_in_2.transform.translation.z) / 2.0;

      // _quat = average_quat(_tf_in_1, _tf_in_2);
      if ((ros::Time::now() - _tf_in_2.header.stamp) < ros::Duration(0.1))
      {
        _tf_out.transform.translation = _tf_in_2.transform.translation;
        _quat = to_quat(_tf_in_2);
        obsevations++;
      }
      // ROS_INFO_STREAM("C_out: " << quat);
    }
    catch (tf2::TransformException& ex)
    {
      // ROS_WARN("%s", ex.what());
      // continue;
    }

    if (obsevations == 2)
    {
      _quat = to_quat(_tf_in_2);
      // _quat = average_quat(_tf_in_1, _tf_in_2);
      _tf_out.transform.translation.x = (_tf_in_1.transform.translation.x + _tf_in_2.transform.translation.x) / 2.0;
      _tf_out.transform.translation.y = (_tf_in_1.transform.translation.y + _tf_in_2.transform.translation.y) / 2.0;
      _tf_out.transform.translation.z = (_tf_in_1.transform.translation.z + _tf_in_2.transform.translation.z) / 2.0;
    }

    if (obsevations > 0)
    {
      _tf_out.transform.rotation.w = _quat.w();
      _tf_out.transform.rotation.x = _quat.x();
      _tf_out.transform.rotation.y = _quat.y();
      _tf_out.transform.rotation.z = _quat.z();

      _tf_out.header.stamp = ros::Time::now();
      _tf_broadcaster.sendTransform(_tf_out);
    }
  }

  Eigen::Quaterniond _quat;
  std::string _world_frame;
  std::vector<std::string> _robot_frames;
  ros::Timer _timer;
  geometry_msgs::TransformStamped _tf_in_1, _tf_in_2, _tf_out;

  tf2_ros::TransformListener _tf_listener;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformBroadcaster _tf_broadcaster;

  Eigen::Matrix<double, 4, 2> w_quats;
};  // namespace estimation
}  // namespace estimation
PLUGINLIB_EXPORT_CLASS(estimation::robot_estimation_simple_nodelet_t, nodelet::Nodelet);