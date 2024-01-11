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
#include <interface/utils.hpp>
#include <interface/stamped_markers.h>
#include <interface/defs.hpp>
#include <aruco/aruco_nano.h>

namespace estimation
{
class aruco_cTw_nodelet_t : public nodelet::Nodelet
{
public:
  aruco_cTw_nodelet_t()
    : _publish_markers_img(false)
    , _robot_pose_topic_name("/aruco_cTw/robot_pose")
    , _markers_viz_topic_name("/aruco_cTw/markers")
    , _Tvec(3, 1, CV_64FC1)
    , _Rvec(1, 3, CV_64FC1)
    , _cv_rot(3, 3, CV_64FC1)
    , _camera_matrix(3, 3, CV_64FC1)
    , _eg_rot(_cv_rot.ptr<double>())
    , _eg_tvec(_Tvec.ptr<double>())
    , _header()
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
    double marker_length;
    std::string markers_topic_name;
    std::vector<double> rotation_offset;
    std::vector<double> camera_matrix;
    std::vector<double> dist_coeffs;

    NODELET_PARAM_SETUP(private_nh, camera_matrix);
    NODELET_PARAM_SETUP(private_nh, dist_coeffs);
    NODELET_PARAM_SETUP(private_nh, front_corner);
    NODELET_PARAM_SETUP(private_nh, main_marker);
    NODELET_PARAM_SETUP(private_nh, marker_length);
    NODELET_PARAM_SETUP(private_nh, markers_topic_name);
    NODELET_PARAM_SETUP(private_nh, robot_id);

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

    _robot_id = robot_id;
    _main_marker = main_marker;
    _rot_offset = Eigen::AngleAxisd(rotation_offset[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(rotation_offset[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(rotation_offset[2], Eigen::Vector3d::UnitZ());

    _robot_pose_publisher = private_nh.advertise<geometry_msgs::PoseStamped>(_robot_pose_topic_name, 1);
    _markers_viz_publisher = private_nh.advertise<visualization_msgs::Marker>(_markers_viz_topic_name, 1);

    _markers_subscriber = private_nh.subscribe(markers_topic_name, 1, &aruco_cTw_nodelet_t::detect, this);

    _marker_corners = { { -marker_length / 2.0, marker_length / 2.0, 0.0 },
                        { marker_length / 2.0, marker_length / 2.0, 0.0 },
                        { marker_length / 2.0, -marker_length / 2.0, 0.0 },
                        { -marker_length / 2.0, -marker_length / 2.0, 0.0 } };

    _eg_corners = { { -marker_length / 2.0, marker_length / 2.0, 0.0 },
                    { marker_length / 2.0, marker_length / 2.0, 0.0 },
                    { marker_length / 2.0, -marker_length / 2.0, 0.0 },
                    { -marker_length / 2.0, -marker_length / 2.0, 0.0 } };

    _front = Eigen::Vector3d(0, _eg_corners[front_corner][1], 0);

    _marker.resize(4);
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

  void detect(const interface::stamped_markersConstPtr message)
  {
    geometry_msgs::PoseStamped pose_msg;
    visualization_msgs::Marker markers_polygon;
    markers_polygon.header.frame_id = "camera";
    pose_msg.header.frame_id = "camera";
    markers_polygon.type = visualization_msgs::Marker::LINE_LIST;
    markers_polygon.color.r = 1;
    markers_polygon.color.g = 0;
    markers_polygon.color.b = 0;
    markers_polygon.color.a = 1;
    markers_polygon.scale.x = 0.01;

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

      // 0-1
      _vec = _eg_rot * _eg_corners[0] + _eg_tvec;
      markers_polygon.points.emplace_back();
      copy(markers_polygon.points.back(), _vec);
      _vec = _eg_rot * _eg_corners[1] + _eg_tvec;
      markers_polygon.points.emplace_back();
      copy(markers_polygon.points.back(), _vec);

      // 1-2
      _vec = _eg_rot * _eg_corners[2] + _eg_tvec;
      markers_polygon.points.push_back(markers_polygon.points.back());
      markers_polygon.points.emplace_back();
      copy(markers_polygon.points.back(), _vec);

      // 2-3
      _vec = _eg_rot * _eg_corners[3] + _eg_tvec;
      markers_polygon.points.push_back(markers_polygon.points.back());
      markers_polygon.points.emplace_back();
      copy(markers_polygon.points.back(), _vec);

      // 3-0
      markers_polygon.points.push_back(markers_polygon.points.back());
      markers_polygon.points.push_back(markers_polygon.points[markers_polygon.points.size() - 7]);

      // Colors: needed per point
      markers_polygon.colors.insert(markers_polygon.colors.end(), 8, markers_polygon.color);

      // Pose reference for all markers.
      copy(markers_polygon.pose.position, Eigen::Vector3d::Zero());
      copy(markers_polygon.pose.orientation, Eigen::Quaterniond::Identity());
      const Eigen::Quaterniond quat{ _eg_rot * _rot_offset };
      tf::vectorEigenToTF(_eg_tvec, _tf_vec);
      tf::quaternionEigenToTF(quat, _tf_quat);
      _transform.setOrigin(_tf_vec);
      _transform.setRotation(_tf_quat);
      std::string frame_name{ "marker_" + std::to_string(marker.id) };

      if (marker.id == _robot_id)
      {
        // _vec = _eg_rot * _front + _eg_tvec;
        const Eigen::Quaterniond quat{ _eg_rot * _rot_offset };
        copy(pose_msg.pose.position, _eg_tvec);
        copy(pose_msg.pose.orientation, quat);
        _robot_pose_publisher.publish(pose_msg);
        frame_name = "robot_" + std::to_string(marker.id);
      }

      _transform_broadcaster.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "camera", frame_name));
    }
    _markers_viz_publisher.publish(markers_polygon);
  }

  tf::Vector3 _tf_vec;
  tf::Quaternion _tf_quat;
  tf::Transform _transform;
  tf::TransformBroadcaster _transform_broadcaster;

  bool _publish_markers_img;

  cv::VideoCapture _cap;

  std_msgs::Header _header;
  interface::stamped_markers _markers_msg;
  std::vector<aruconano::Marker> _markers;

  sensor_msgs::Image _dbg_msg;

  cv::Mat _Rvec, _Tvec;
  cv::Mat _cv_rot;
  Eigen::Map<Eigen::Matrix3d> _eg_rot;
  Eigen::Map<Eigen::Vector3d> _eg_tvec;
  Eigen::Vector3d _vec;
  std::vector<cv::Point2d> _marker;
  Eigen::Vector3d _front;

  // std::vector<cv::Point2f> _marker;
  std::vector<cv::Point3d> _marker_corners;
  std::vector<Eigen::Vector3d> _eg_corners;

  cv::Mat _camera_matrix;
  cv::Mat _dist_coeffs;
  double _marker_size;
  ros::Publisher _robot_pose_publisher;
  ros::Publisher _markers_viz_publisher;
  ros::Subscriber _markers_subscriber;
  std::string _robot_pose_topic_name;
  std::string _markers_viz_topic_name;

  Eigen::Matrix3d _rot_offset;
  int _robot_id;
  int _main_marker;
};
}  // namespace estimation
PLUGINLIB_EXPORT_CLASS(estimation::aruco_cTw_nodelet_t, nodelet::Nodelet);