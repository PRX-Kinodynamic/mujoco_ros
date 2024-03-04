#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <interface/StampedMarkers.h>
#include <aruco/aruco_nano.h>
#include <ml4kp_bridge/TrajectoryStamped.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace estimation
{
class aruco_wTc_nodelet_t : public nodelet::Nodelet
{
  using Transform = Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;

public:
  aruco_wTc_nodelet_t()
    : _camera_matrix(3, 3, CV_64FC1)
    , _Tvec(3, 1, CV_64FC1)
    , _Rvec(1, 3, CV_64FC1)
    , _img_topic_name("/wTc/image")
    , _cv_world_rot(3, 3, CV_64FC1)
    , _goal_color(0, 255, 0, 128)
    , _color_rgb({ cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0) })
    , _robot_pose_offset(0.2, 0.0, 0.0)
  {
  }

private:
  enum PointIdx
  {
    zero = 0,  // zero coordinate in world frame
    x_normal,  // Origin: x normal
    y_normal,  // Origin: y normal
    z_normal,  // Origin: z normal
    robot_center,
    robot_front,
    goal_rad,
    goal_pose,
    tf,
    trajectory,
    TOTAL  // Keep this one at the end to get the number of enums
  };

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    _img_topic_name = ros::this_node::getNamespace() + _img_topic_name;

    std::vector<double> camera_matrix;
    std::vector<double> dist_coeffs;
    std::vector<std::string> trajectory_topics;

    std::string image_topic;
    std::string goal_pose_topic;
    std::string goal_rad_topic;
    // std::string trajectory_topic;
    std::string camera_frame;
    std::string world_frame;
    std::string robot_frame;
    std::string reset_topic;
    double robot_pose_offset;

    NODELET_PARAM_SETUP(private_nh, dist_coeffs);
    NODELET_PARAM_SETUP(private_nh, camera_matrix);
    NODELET_PARAM_SETUP(private_nh, image_topic);
    NODELET_PARAM_SETUP(private_nh, goal_pose_topic);
    NODELET_PARAM_SETUP(private_nh, goal_rad_topic);
    NODELET_PARAM_SETUP(private_nh, trajectory_topics);
    NODELET_PARAM_SETUP(private_nh, camera_frame);
    NODELET_PARAM_SETUP(private_nh, world_frame);
    NODELET_PARAM_SETUP(private_nh, robot_frame);
    NODELET_PARAM_SETUP(private_nh, reset_topic);
    NODELET_PARAM_SETUP_WITH_DEFAULT(private_nh, robot_pose_offset, _robot_pose_offset[0])

    _robot_pose_offset[0] = robot_pose_offset;
    _camera_frame = camera_frame;
    _world_frame = world_frame;
    _robot_frame = robot_frame;
    _dist_coeffs.push_back(dist_coeffs);

    _camera_matrix.at<double>(0, 0) = camera_matrix[0];
    _camera_matrix.at<double>(0, 1) = camera_matrix[1];
    _camera_matrix.at<double>(0, 2) = camera_matrix[2];
    _camera_matrix.at<double>(1, 0) = camera_matrix[3];
    _camera_matrix.at<double>(1, 1) = camera_matrix[4];
    _camera_matrix.at<double>(1, 2) = camera_matrix[5];
    _camera_matrix.at<double>(2, 0) = 0;
    _camera_matrix.at<double>(2, 1) = 0;
    _camera_matrix.at<double>(2, 2) = 1;

    _cv_points.resize(PointIdx::TOTAL);
    _image_points.resize(PointIdx::TOTAL);
    _msgs_received.resize(PointIdx::TOTAL);
    _cv_points[PointIdx::x_normal] = cv::Point3d(1, 0, 0);
    _cv_points[PointIdx::y_normal] = cv::Point3d(0, 1, 0);
    _cv_points[PointIdx::z_normal] = cv::Point3d(0, 0, 1);
    _rgb_subscriber = private_nh.subscribe(image_topic, 1, &aruco_wTc_nodelet_t::get_image, this);
    _goal_pose_subscriber = private_nh.subscribe(goal_pose_topic, 1, &aruco_wTc_nodelet_t::get_goal_pose, this);
    _goal_rad_subscriber = private_nh.subscribe(goal_rad_topic, 1, &aruco_wTc_nodelet_t::get_goal_rad, this);
    _reset_subscriber = private_nh.subscribe(reset_topic, 1, &aruco_wTc_nodelet_t::reset, this);
    _frame_publisher = private_nh.advertise<sensor_msgs::Image>(_img_topic_name, 1);
    for (int i = 0; i < trajectory_topics.size(); ++i)
    {
      _cv_trajs.emplace_back();
      _image_trajs.emplace_back();
      _traj_colors.emplace_back(_color_rgb[i] + _color_rgb[2]);
      _trajectory_subscribers.push_back(private_nh.subscribe<ml4kp_bridge::TrajectoryStamped>(
          trajectory_topics[i], 1, boost::bind(&aruco_wTc_nodelet_t::get_trajectory, this, _1, i)));
    }
  }

  void reset(const std_msgs::Empty empty)
  {
    _image_traj_exec.clear();
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStampedConstPtr message, const std::size_t& traj_idx)
  {
    _cv_trajs[traj_idx].clear();
    for (auto& state : message->trajectory.data)
    {
      // x,y,z: setting z=cte for now
      _cv_trajs[traj_idx].emplace_back(state.point[0].data, state.point[1].data, 0.2295);
      _msgs_received[PointIdx::trajectory] = true;
    }
    if (_cv_trajs[traj_idx].size() > 0)
    {
      cv::projectPoints(_cv_trajs[traj_idx], _Rvec, _Tvec, _camera_matrix, _dist_coeffs, _image_trajs[traj_idx]);
    }
  }

  void get_goal_rad(const std_msgs::Float64ConstPtr message)
  {
    _goal_rad = message->data;
    _msgs_received[PointIdx::goal_rad] = true;
  }

  void get_goal_pose(const geometry_msgs::Pose2DConstPtr message)
  {
    _cv_points[PointIdx::goal_pose].x = message->x;
    _cv_points[PointIdx::goal_pose].y = message->y;
    _cv_points[PointIdx::goal_pose].z = 0.0;
    _msgs_received[PointIdx::goal_pose] = true;
  }

  void get_image(const sensor_msgs::ImageConstPtr message)
  {
    cv_bridge::CvImagePtr frame{ cv_bridge::toCvCopy(message) };

    if (_tf_listener.canTransform(_world_frame, _camera_frame, ros::Time(0)))
    {
      _tf_listener.lookupTransform(_camera_frame, _world_frame, ros::Time(0), _tf);

      _Tvec.at<double>(0) = _tf.getOrigin().x();
      _Tvec.at<double>(1) = _tf.getOrigin().y();
      _Tvec.at<double>(2) = _tf.getOrigin().z();

      _cv_world_rot.at<double>(0, 0) = _tf.getBasis()[0][0];
      _cv_world_rot.at<double>(0, 1) = _tf.getBasis()[0][1];
      _cv_world_rot.at<double>(0, 2) = _tf.getBasis()[0][2];
      _cv_world_rot.at<double>(1, 0) = _tf.getBasis()[1][0];
      _cv_world_rot.at<double>(1, 1) = _tf.getBasis()[1][1];
      _cv_world_rot.at<double>(1, 2) = _tf.getBasis()[1][2];
      _cv_world_rot.at<double>(2, 0) = _tf.getBasis()[2][0];
      _cv_world_rot.at<double>(2, 1) = _tf.getBasis()[2][1];
      _cv_world_rot.at<double>(2, 2) = _tf.getBasis()[2][2];
      cv::Rodrigues(_cv_world_rot, _Rvec);

      if (_tf_listener.canTransform(_robot_frame, _world_frame, ros::Time(0)))
      {
        _tf_listener.lookupTransform(_world_frame, _robot_frame, ros::Time(0), _tf_robot);
        _cv_points[PointIdx::robot_center].x = _tf_robot.getOrigin().x();
        _cv_points[PointIdx::robot_center].y = _tf_robot.getOrigin().y();
        _cv_points[PointIdx::robot_center].z = _tf_robot.getOrigin().z();
        _msgs_received[PointIdx::robot_center] = true;

        const tf::Quaternion& tf_q{ _tf_robot.getRotation() };
        const Eigen::Quaterniond quat{ tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ() };
        const Eigen::Vector3d offset_rot{ quat * _robot_pose_offset };
        _cv_points[PointIdx::robot_front].x = offset_rot[0] + _tf_robot.getOrigin().x();
        _cv_points[PointIdx::robot_front].y = offset_rot[1] + _tf_robot.getOrigin().y();
        _cv_points[PointIdx::robot_front].z = offset_rot[2] + _tf_robot.getOrigin().z();
        _msgs_received[PointIdx::robot_front] = true;
      }

      if (_msgs_received[PointIdx::goal_pose] and _msgs_received[PointIdx::goal_rad])
      {
        _cv_points[PointIdx::goal_rad] = _cv_points[PointIdx::goal_pose] + cv::Point3d(_goal_rad, 0, 0);
      }

      cv::projectPoints(_cv_points, _Rvec, _Tvec, _camera_matrix, _dist_coeffs, _image_points);

      if (_msgs_received[PointIdx::goal_pose] and _msgs_received[PointIdx::goal_rad])
      {
        const cv::Point2d pt_aux{ _image_points[PointIdx::goal_pose] - _image_points[PointIdx::goal_rad] };
        const double rad{ std::sqrt(pt_aux.ddot(pt_aux)) };

        cv::circle(frame->image, _image_points[PointIdx::goal_pose], rad, _goal_color, 3, cv::LineTypes::LINE_AA);
      }

      cv::line(frame->image, _image_points[PointIdx::zero], _image_points[PointIdx::x_normal], _color_rgb[0], 2);
      cv::line(frame->image, _image_points[PointIdx::zero], _image_points[PointIdx::y_normal], _color_rgb[1], 2);
      cv::line(frame->image, _image_points[PointIdx::zero], _image_points[PointIdx::z_normal], _color_rgb[2], 2);

      if (_msgs_received[PointIdx::robot_front])
      {
        cv::arrowedLine(frame->image, _image_points[PointIdx::robot_center], _image_points[PointIdx::robot_front],
                        _color_rgb[0], 3);
        _image_traj_exec.push_back(_image_points[PointIdx::robot_center]);
        _msgs_received[PointIdx::robot_front] = false;

        for (int i = 0; i < _image_traj_exec.size(); ++i)
        {
          cv::circle(frame->image, _image_traj_exec[i], 1, _color_rgb[0] + _color_rgb[1] / 2, -1,
                     cv::LineTypes::FILLED);
        }
      }

      if (_msgs_received[PointIdx::trajectory])
      {
        for (int i = 0; i < _image_trajs.size(); ++i)
        {
          const std::vector<cv::Point2d>& img_traj{ _image_trajs[i] };
          const cv::Scalar& color{ _traj_colors[i] };
          for (int j = 0; j < img_traj.size() - 1; ++j)
          {
            cv::line(frame->image, img_traj[j], img_traj[j + 1], color, 2);
          }
        }
      }

      _frame_publisher.publish(frame);
    }
  }

  tf::TransformListener _tf_listener;
  tf::StampedTransform _tf;
  tf::StampedTransform _tf_robot;
  std::string _camera_frame, _world_frame, _robot_frame;

  ros::Subscriber _rgb_subscriber;
  ros::Subscriber _goal_pose_subscriber;
  ros::Subscriber _goal_rad_subscriber;
  ros::Subscriber _reset_subscriber;
  std::vector<ros::Subscriber> _trajectory_subscribers;

  ros::Publisher _frame_publisher;

  std::string _img_topic_name;

  cv::Mat _camera_matrix;
  cv::Mat _dist_coeffs;

  cv::Mat _Rvec;
  cv::Mat _Tvec;

  const std::vector<cv::Scalar> _color_rgb;
  std::vector<cv::Scalar> _traj_colors;

  double _goal_rad;
  const cv::Scalar _goal_color;

  std::vector<cv::Point3d> _cv_points;
  std::vector<cv::Point2d> _image_points;
  std::vector<cv::Point2d> _image_traj_exec;
  std::vector<bool> _msgs_received;
  std::vector<std::vector<cv::Point2d>> _image_trajs;
  std::vector<std::vector<cv::Point3d>> _cv_trajs;

  cv::Mat _cv_world_rot;
  Eigen::Vector3d _robot_pose_offset;
};
}  // namespace estimation

PLUGINLIB_EXPORT_CLASS(estimation::aruco_wTc_nodelet_t, nodelet::Nodelet);
