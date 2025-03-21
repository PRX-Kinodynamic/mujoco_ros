#pragma once
#include <std_msgs/Empty.h>
#include <utils/std_utils.cpp>
namespace utils
{

// Publish to shutdown topic when another topic
template <typename Topic, class Base>
class shutdown_from_topic_t : public Base
{
  using Derived = shutdown_from_topic_t<Topic, Base>;
  using TopicConstPtr = boost::shared_ptr<Topic const>;

public:
  shutdown_from_topic_t() : Base()
  {
  }

protected:
  virtual ~shutdown_from_topic_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string topic_name;
    std::string reset_topic;
    std::string shutdown_topic{ "" };

    std::string& on_reset_value{ _on_reset_value };  // To avoid the _ in parameter

    PARAM_SETUP(private_nh, directory);
    PARAM_SETUP(private_nh, fileprefix);
    PARAM_SETUP(private_nh, topic_name);
    PARAM_SETUP(private_nh, reset_topic);
    PARAM_SETUP_WITH_DEFAULT(private_nh, on_reset_value, on_reset_value);
    PARAM_SETUP_WITH_DEFAULT(private_nh, shutdown_topic, shutdown_topic);

    _topic_subscriber = private_nh.subscribe(topic_name, 10, &Derived::topic_to_file_callback, this);
    _reset_subscriber = private_nh.subscribe(reset_topic, 1, &Derived::reset_callback, this);

    if (shutdown_topic != "")
      _shutdown_subscriber = private_nh.subscribe(shutdown_topic, 1, &utils::shutdown_callback<std_msgs::Bool>);
  }

  void topic_to_file_callback(const TopicConstPtr& msg)
  {
  }

  // Subscribers
  ros::Subscriber _topic_subscriber;
  ros::Subscriber _reset_subscriber;
  ros::Subscriber _shutdown_subscriber;
};
}  // namespace utils