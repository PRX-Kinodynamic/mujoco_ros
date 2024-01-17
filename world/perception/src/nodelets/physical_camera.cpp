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
#include <utils/rosparams_utils.hpp>
// Nodelet to read from a physical camera and publish to '*Namespace*/camera/rgb' at a given framerate
// As a nodelet, publishing to another nodelet in the same manager is equivalent to a pointer copy
namespace perception
{
class physical_camera_nodelet_t : public nodelet::Nodelet
{
public:
  physical_camera_nodelet_t() : _frame(), _topic_name("/camera/rgb"), _header(){};

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    _topic_name = ros::this_node::getNamespace() + _topic_name;
    int height{ 0 };
    int width{ 0 };
    double frequency{ 30 };
    std::string camera{ 0 };
    NODELET_PARAM_SETUP(private_nh, camera);
    NODELET_PARAM_SETUP(private_nh, height);
    NODELET_PARAM_SETUP(private_nh, width);
    NODELET_PARAM_SETUP(private_nh, frequency);
#if __linux__
    // _cap.open(_camera_id, cv::CAP_V4L2);
    _cap.open(camera, cv::CAP_ANY);  // This is supposedly more general
#else
    _cap.open(std::stoi(camera), cv::CAP_ANY);
#endif

    _cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    _cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    _cap.set(cv::CAP_PROP_FPS, frequency);
    _cap.set(cv::CAP_PROP_BUFFERSIZE, 10);

    _cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
    _cap.set(cv::CAP_PROP_AUTO_WB, 1);
    _cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);

    NODELET_INFO_STREAM("Camera: " << camera);
    NODELET_INFO_STREAM("cv::CAP_PROP_FRAME_WIDTH: " << _cap.get(cv::CAP_PROP_FRAME_WIDTH));
    NODELET_INFO_STREAM("cv::CAP_PROP_FRAME_HEIGHT: " << _cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    NODELET_INFO_STREAM("cv::CAP_PROP_FPS: " << _cap.get(cv::CAP_PROP_FPS));

    if (!_cap.isOpened())
    {
      ROS_ERROR_STREAM("Error opening the video source '" << camera << "'.");
      ROS_ERROR_STREAM("On linux, consider using command 'v4l2-ctl --list-devices'");
      exit(-1);
    }
    _header.seq = 0;
    _header.stamp = ros::Time::now();
    _header.frame_id = _topic_name;

    _frame_publisher = private_nh.advertise<sensor_msgs::Image>(_topic_name, 1, true);
    _timer = private_nh.createTimer(ros::Duration(1.0 / frequency), &physical_camera_nodelet_t::capture_img, this);
  }

  void capture_img(const ros::TimerEvent& event)
  {
    if (_cap.read(_frame))
    {
      _header.seq++;
      _header.stamp = ros::Time::now();
      const sensor_msgs::ImagePtr msg{ cv_bridge::CvImage(std_msgs::Header(), "bgr8", _frame).toImageMsg() };
      _frame_publisher.publish(msg);
    }
  }

  ros::Timer _timer;
  ros::Publisher _frame_publisher;

  cv::VideoCapture _cap;

  cv::Mat _frame;
  std::string _topic_name;

  std_msgs::Header _header;
};
}  // namespace perception
PLUGINLIB_EXPORT_CLASS(perception::physical_camera_nodelet_t, nodelet::Nodelet);