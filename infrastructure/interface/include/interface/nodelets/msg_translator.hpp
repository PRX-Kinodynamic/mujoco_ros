#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <interface/msg_translation.hpp>

namespace interface
{
template <typename To, typename From>
class msg_translator_t : public nodelet::Nodelet
{
  using MsgCopy = msg_translator_t<To, From>;
  using FromConstPtr = boost::shared_ptr<From const>;

public:
  msg_translator_t()
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    std::string publisher_topic{};
    std::string subscriber_topic{};
    private_nh.getParam("publisher_topic", publisher_topic);
    private_nh.getParam("subscriber_topic", subscriber_topic);
    _publisher = private_nh.advertise<To>(publisher_topic, 1, true);
    _subscriber = private_nh.subscribe(subscriber_topic, 1, &MsgCopy::copy_callback, this);
  }
  void copy_callback(const FromConstPtr msg_in)
  {
    translate_msg(_to, msg_in);
    _publisher.publish(_to);
  }
  To _to;
  ros::Publisher _publisher;
  ros::Subscriber _subscriber;
};
}  // namespace interface