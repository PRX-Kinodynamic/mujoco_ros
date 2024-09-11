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
// #include <interface/utils.hpp>
#include <interface/StampedMarkers.h>
#include <aruco/aruco_nano.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace estimation
{
class aruco_cTw_nodelet_t : public nodelet::Nodelet
{
public:
  aruco_cTw_nodelet_t()
    : _robot_pose_topic_name("/aruco_cTw/robot_pose")
    , _markers_viz_topic_name("/aruco_cTw/markers")
    , _Tvec(3, 1, CV_64FC1)
    , _Rvec(1, 3, CV_64FC1)
    , _cv_rot(3, 3, CV_64FC1)
    , _camera_matrix(3, 3, CV_64FC1)
    , _eg_rot(_cv_rot.ptr<double>())
    , _eg_tvec(_Tvec.ptr<double>())
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    _robot_pose_topic_name = ros::this_node::getNamespace() + _robot_pose_topic_name;
    _markers_viz_topic_name = ros::this_node::getNamespace() + _markers_viz_topic_name;

    int robot_id;
    int main_marker;
    int front_corner;
    double marker_size;
    std::string markers_topic_name;
    int camera_id;
    std::vector<double> rotation_offset;
    std::vector<double> camera_matrix;
    std::vector<double> dist_coeffs;
    bool vizualize_markers;

    NODELET_PARAM_SETUP(private_nh, camera_id);
    NODELET_PARAM_SETUP(private_nh, camera_matrix);
    NODELET_PARAM_SETUP(private_nh, dist_coeffs);
    NODELET_PARAM_SETUP(private_nh, front_corner);
    NODELET_PARAM_SETUP(private_nh, main_marker);
    NODELET_PARAM_SETUP(private_nh, marker_size);
    NODELET_PARAM_SETUP(private_nh, markers_topic_name);
    NODELET_PARAM_SETUP(private_nh, robot_id);

    NODELET_PARAM_SETUP_WITH_DEFAULT(private_nh, vizualize_markers, false);
    NODELET_PARAM_SETUP_WITH_DEFAULT(private_nh, rotation_offset, std::vector<double>({ 0.0, 0.0, 0.0 }))

    _camera_matrix.at<double>(0, 0) = camera_matrix[0];
    _camera_matrix.at<double>(0, 1) = camera_matrix[1];
    _camera_matrix.at<double>(0, 2) = camera_matrix[2];
    _camera_matrix.at<double>(1, 0) = camera_matrix[3];
    _camera_matrix.at<double>(1, 1) = camera_matrix[4];
    _camera_matrix.at<double>(1, 2) = camera_matrix[5];
    _camera_matrix.at<double>(2, 0) = 0;
    _camera_matrix.at<double>(2, 1) = 0;
    _camera_matrix.at<double>(2, 2) = 1;

    _dist_coeffs.push_back(dist_coeffs);

    _vizualize_markers = vizualize_markers;
    _camera_name = "C" + std::to_string(camera_id);
    _robot_id = robot_id;
    DEBUG_VARS(_camera_name);
    _rot_offset = Eigen::AngleAxisd(rotation_offset[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(rotation_offset[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(rotation_offset[2], Eigen::Vector3d::UnitZ());

    _robot_pose_publisher = private_nh.advertise<geometry_msgs::PoseStamped>(_robot_pose_topic_name, 1);
    _markers_viz_publisher = private_nh.advertise<visualization_msgs::Marker>(_markers_viz_topic_name, 1);

    _markers_subscriber = private_nh.subscribe(markers_topic_name, 1, &aruco_cTw_nodelet_t::detect, this);

    DEBUG_PRINT
    _marker_corners = { { -marker_size / 2.0, marker_size / 2.0, 0.0 },
                        { marker_size / 2.0, marker_size / 2.0, 0.0 },
                        { marker_size / 2.0, -marker_size / 2.0, 0.0 },
                        { -marker_size / 2.0, -marker_size / 2.0, 0.0 } };

    _eg_corners = { { -marker_size / 2.0, marker_size / 2.0, 0.0 },
                    { marker_size / 2.0, marker_size / 2.0, 0.0 },
                    { marker_size / 2.0, -marker_size / 2.0, 0.0 },
                    { -marker_size / 2.0, -marker_size / 2.0, 0.0 } };

    _front = Eigen::Vector3d(0, _eg_corners[front_corner][1], 0);

    _marker.resize(4);
    _viz_markers.header.frame_id = _camera_name;
    _viz_markers.type = visualization_msgs::Marker::LINE_LIST;
    _viz_markers.color.r = 1;
    _viz_markers.color.g = 0;
    _viz_markers.color.b = 0;
    _viz_markers.color.a = 1;
    _viz_markers.scale.x = 0.01;
  }

  static void copy(geometry_msgs::Quaternion& quat_msg, const Eigen::Quaterniond& quat)
  {
    quat_msg.w = quat.w();
    quat_msg.x = quat.x();
    quat_msg.y = quat.y();
    quat_msg.z = quat.z();
  }
  static void copy(geometry_msgs::Point& point, const Eigen::Vector3d& vec)
  {
    point.x = vec[0];
    point.y = vec[1];
    point.z = vec[2];
  }

  void add_marker_viz()
  {
    if (_vizualize_markers)
    {
      const Eigen::Vector3d v0{ _eg_rot * _eg_corners[0] + _eg_tvec };
      const Eigen::Vector3d v1{ _eg_rot * _eg_corners[1] + _eg_tvec };
      const Eigen::Vector3d v2{ _eg_rot * _eg_corners[2] + _eg_tvec };
      const Eigen::Vector3d v3{ _eg_rot * _eg_corners[3] + _eg_tvec };

      const std::size_t idx{ _viz_markers.points.size() };
      _viz_markers.points.insert(_viz_markers.points.end(), 8, geometry_msgs::Point());
      // 0-1
      copy(_viz_markers.points[idx + 0], v0);
      copy(_viz_markers.points[idx + 1], v1);
      // 1-2
      copy(_viz_markers.points[idx + 2], v1);
      copy(_viz_markers.points[idx + 3], v2);
      // 2-3
      copy(_viz_markers.points[idx + 4], v2);
      copy(_viz_markers.points[idx + 5], v3);
      // 3-0
      copy(_viz_markers.points[idx + 6], v3);
      copy(_viz_markers.points[idx + 7], v0);
      // Colors: needed per point
      _viz_markers.colors.insert(_viz_markers.colors.end(), 8, _viz_markers.color);
      copy(_viz_markers.pose.position, Eigen::Vector3d::Zero());
      copy(_viz_markers.pose.orientation, Eigen::Quaterniond::Identity());
    }
  }

  void detect(const interface::StampedMarkersConstPtr message)
  {
    geometry_msgs::PoseStamped pose_msg;
    _viz_markers.points.clear();
    _viz_markers.colors.clear();
    pose_msg.header.frame_id = "camera";

    // for (int i = 0; i < message->markers.size(); ++i)
    for (auto marker : message->markers)
    {
      // const interface::marker& marker{ message->markers[i] };
      _marker[0].x = marker.x1;
      _marker[0].y = marker.y1;
      _marker[1].x = marker.x2;
      _marker[1].y = marker.y2;
      _marker[2].x = marker.x3;
      _marker[2].y = marker.y3;
      _marker[3].x = marker.x4;
      _marker[3].y = marker.y4;

      cv::solvePnP(_marker_corners, _marker, _camera_matrix, _dist_coeffs, _Rvec, _Tvec, false, cv::SOLVEPNP_IPPE);

      cv::Rodrigues(_Rvec, _cv_rot);

      add_marker_viz();
      // Pose reference for all markers.

      const Eigen::Quaterniond quat{ _eg_rot * _rot_offset };
      tf::vectorEigenToTF(_eg_tvec, _tf_vec);
      tf::quaternionEigenToTF(quat, _tf_quat);
      _transform.setOrigin(_tf_vec);
      _transform.setRotation(_tf_quat);
      std::string frame_name{ _camera_name + "_marker_" + std::to_string(marker.id) };

      if (marker.id == _robot_id)
      {
        const Eigen::Quaterniond quat_robot{ _eg_rot * _rot_offset };
        copy(pose_msg.pose.position, _eg_tvec);
        copy(pose_msg.pose.orientation, quat_robot);
        tf::quaternionEigenToTF(quat_robot, _tf_quat);
        _transform.setRotation(_tf_quat);
        _robot_pose_publisher.publish(pose_msg);
        frame_name = _camera_name + "_robot_" + std::to_string(_robot_id);
      }

      _transform_broadcaster.sendTransform(
          tf::StampedTransform(_transform, ros::Time::now(), _camera_name, frame_name));
    }

    _markers_viz_publisher.publish(_viz_markers);
  }

  tf::Vector3 _tf_vec;
  tf::Quaternion _tf_quat;
  tf::Transform _transform;
  tf::TransformBroadcaster _transform_broadcaster;

  cv::Mat _Rvec, _Tvec;
  cv::Mat _cv_rot;
  Eigen::Map<Eigen::Matrix3d> _eg_rot;
  Eigen::Map<Eigen::Vector3d> _eg_tvec;
  Eigen::Vector3d _vec;
  std::vector<cv::Point2d> _marker;
  Eigen::Vector3d _front;

  std::vector<cv::Point3d> _marker_corners;
  std::vector<Eigen::Vector3d> _eg_corners;

  cv::Mat _camera_matrix;
  cv::Mat _dist_coeffs;
  ros::Publisher _robot_pose_publisher;
  ros::Publisher _markers_viz_publisher;
  ros::Subscriber _markers_subscriber;
  std::string _robot_pose_topic_name;
  std::string _markers_viz_topic_name;
  std::string _camera_name;

  Eigen::Matrix3d _rot_offset;
  int _robot_id;

  visualization_msgs::Marker _viz_markers;
  bool _vizualize_markers;
};
}  // namespace estimation
PLUGINLIB_EXPORT_CLASS(estimation::aruco_cTw_nodelet_t, nodelet::Nodelet);
