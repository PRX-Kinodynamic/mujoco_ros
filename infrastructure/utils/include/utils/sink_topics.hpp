#pragma once
#include <utils/rosparams_utils.hpp>

namespace utils
{

template <typename Msg, class Base>
class sink_topics_t : public Base
{
  using Derived = sink_topics_t<Msg, Base>;
  using MsgConstPtr = boost::shared_ptr<Msg const>;

public:
  sink_topics_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string out_topic;
    std::vector<std::string> in_topics;

    ROS_PARAM_SETUP(private_nh, out_topic);
    ROS_PARAM_SETUP(private_nh, in_topics);

    // publishers
    _publisher = private_nh.advertise<Msg>(out_topic, 1, true);

    DEBUG_VARS(out_topic);
    DEBUG_VARS(in_topics);
    for (auto topic : in_topics)
    {
      _subscribers.push_back(private_nh.subscribe(topic, 1, &Derived::callback, this));
    }
    // _subscribers.push_back(private_nh.subscribe(in_topics[0], 1, &Derived::callback, this));
    // _reset_subscriber = private_nh.subscribe(reset_topic, 1, &Derived::reset_callback, this);
  }

  void callback(const MsgConstPtr msg)
  {
    _publisher.publish(msg);
  }

protected:
  // Subscribers
  std::vector<ros::Subscriber> _subscribers;

  // Publishers
  ros::Publisher _publisher;
};
}  // namespace utils