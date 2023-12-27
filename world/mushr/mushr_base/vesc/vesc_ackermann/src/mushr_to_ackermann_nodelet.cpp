
#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

namespace vesc_ackermann
{

class MushrToAckermannNodelet : public nodelet::Nodelet
{
public:
  MushrToAckermannNodelet()
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    _rgb_topic_name = ros::this_node::getNamespace() + _rgb_topic_name;
    _img_topic_name = ros::this_node::getNamespace() + _img_topic_name;
    _markers_topic_name = ros::this_node::getNamespace() + _markers_topic_name;

    _markers_msg.header.seq = 0;
    _markers_msg.header.stamp = ros::Time::now();
    _markers_msg.header.frame_id = _markers_topic_name;

    _ackermann_publisher = private_nh.advertise<ackermann_msgs::AckermannDriveStamped>(_markers_topic_name, 1);
    _mushr_subscriber = private_nh.subscribe(_rgb_topic_name, 1, &aruco_detection_nodelet_t::detect, this);
  }

  ros::Publisher _ackermann_publisher;
  ros::Subscriber _mushr_subscriber;
};  // class AckermannToVescNodelet

}  // namespace vesc_ackermann

PLUGINLIB_EXPORT_CLASS(vesc_ackermann::AckermannToVescNodelet, nodelet::Nodelet);
