#include <stdio.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.h>
#include <interface/StampedMarkers.h>
#include <aruco/aruco_nano.h>
#include <utils/rosparams_utils.hpp>

namespace perception
{
class aruco_detection_nodelet_t : public nodelet::Nodelet
{
public:
  aruco_detection_nodelet_t()
    : _publish_markers_img(false)
    , _rgb_topic_name("/camera/rgb")
    , _img_topic_name("/camera/markers")
    , _markers_topic_name("/markers")
    , _header(){};

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    _rgb_topic_name = ros::this_node::getNamespace() + _rgb_topic_name;
    _img_topic_name = ros::this_node::getNamespace() + _img_topic_name;
    _markers_topic_name = ros::this_node::getNamespace() + _markers_topic_name;

    bool publish_markers_img;
    NODELET_PARAM_SETUP(private_nh, publish_markers_img);
    _publish_markers_img = publish_markers_img;

    _header.seq = 0;
    _header.stamp = ros::Time::now();
    _header.frame_id = _img_topic_name;

    _markers_msg.header.seq = 0;
    _markers_msg.header.stamp = ros::Time::now();
    _markers_msg.header.frame_id = _markers_topic_name;

    _frame_marker_publisher = private_nh.advertise<sensor_msgs::Image>(_img_topic_name, 1);
    _markers_publisher = private_nh.advertise<interface::StampedMarkers>(_markers_topic_name, 1);
    _rgb_subscriber = private_nh.subscribe(_rgb_topic_name, 1, &aruco_detection_nodelet_t::detect, this);
  }

  void detect(const sensor_msgs::ImageConstPtr message)
  {
    cv_bridge::CvImageConstPtr frame{ cv_bridge::toCvShare(message) };
    _markers = aruconano::MarkerDetector::detect(frame->image);
    if (_markers.size() > 0)
    {
      _markers_msg.header.seq++;
      _markers_msg.header.stamp = ros::Time::now();
      _markers_msg.markers.clear();

      for (auto e : _markers)
      {
        _markers_msg.markers.emplace_back();
        _markers_msg.markers.back().id = e.id;
        _markers_msg.markers.back().x1 = e[0].x;
        _markers_msg.markers.back().y1 = e[0].y;
        _markers_msg.markers.back().x2 = e[1].x;
        _markers_msg.markers.back().y2 = e[1].y;
        _markers_msg.markers.back().x3 = e[2].x;
        _markers_msg.markers.back().y3 = e[2].y;
        _markers_msg.markers.back().x4 = e[3].x;
        _markers_msg.markers.back().y4 = e[3].y;
      }

      _markers_publisher.publish(_markers_msg);
    }
    if (_publish_markers_img)
    {
      cv_bridge::CvImagePtr frame_dbg{ cv_bridge::toCvCopy(message) };

      if (_markers.size() > 0)
      {
        for (auto e : _markers)
        {
          e.draw(frame_dbg->image);
        }
      }
      else
      {
        const cv::Point center(frame_dbg->image.cols / 2, frame_dbg->image.rows / 2);
        const int font_size{ 1 };
        const cv::Scalar font_Color(255, 0, 0);
        const int font_weight{ 2 };
        cv::putText(frame_dbg->image, "NO MARKER DETECTED", center, cv::FONT_HERSHEY_COMPLEX, font_size, font_Color,
                    font_weight);  // Putting the text in the matrix//
      }

      _header.seq++;
      _header.stamp = ros::Time::now();
      // const sensor_msgs::ImagePtr msg_dbg{ frame_dbg };
      frame_dbg->toImageMsg(_dbg_msg);
      _frame_marker_publisher.publish(_dbg_msg);
    }
  }

  bool _publish_markers_img;

  ros::Publisher _frame_marker_publisher;
  ros::Publisher _markers_publisher;
  ros::Subscriber _rgb_subscriber;

  std::string _img_topic_name;
  std::string _markers_topic_name;
  std::string _rgb_topic_name;

  std_msgs::Header _header;
  interface::StampedMarkers _markers_msg;
  std::vector<aruconano::Marker> _markers;

  sensor_msgs::Image _dbg_msg;
};
}  // namespace perception
PLUGINLIB_EXPORT_CLASS(perception::aruco_detection_nodelet_t, nodelet::Nodelet);