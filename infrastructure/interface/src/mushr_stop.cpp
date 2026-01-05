#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>
// #include <prx_models/mj_mushr.hpp>

// #include <interface/mushr_translation.hpp>
// #include <interface/msg_translator.hpp>
#include <utils/rosparams_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

struct stop_callback
{
  stop_callback(ros::NodeHandle& nh, std::string control_topic, std::string stop_topic)
    : _publisher(nh.advertise<ackermann_msgs::AckermannDriveStamped>(control_topic, 1, true))
    , _subscriber(nh.subscribe(stop_topic, 1, &stop_callback::callback, this))
  {
    _ctrl_msg.drive.steering_angle = 0.0;
    _ctrl_msg.drive.speed = 0.0;
    _ctrl_msg.drive.acceleration = 0.0;
  }

  void callback(const std_msgs::Bool& msg)
  {
    _ctrl_msg.header.stamp = ros::Time::now();
    _publisher.publish(_ctrl_msg);
    PRINT_MSG("MUSHR STOP CALLED")
  }
  ackermann_msgs::AckermannDriveStamped _ctrl_msg;

  ros::Publisher _publisher;
  ros::Subscriber _subscriber;
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrStop" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getName() };

  std::string stop_topic{};
  std::string control_topic{};

  PARAM_SETUP(nh, stop_topic);
  PARAM_SETUP(nh, control_topic);

  stop_callback stop(nh, control_topic, stop_topic);

  ros::spin();
  return 0;
}