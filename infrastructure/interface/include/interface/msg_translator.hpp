#pragma once

#include <interface/msg_translation.hpp>

namespace interface
{
template <class To, class From>
class msg_translator_t
{
  using FromConstPtr = typename From::ConstPtr;
  using MsgTranslator = msg_translator_t<To, From>;

public:
  msg_translator_t(ros::NodeHandle& nh, const std::string subscriber_topic, const std::string publisher_topic)
    : _subscriber_topic(subscriber_topic), _publisher_topic(publisher_topic), _to_msg()
  {
    _subscriber = nh.subscribe(_subscriber_topic, 1, &MsgTranslator::subscriber_callback, this);
    _publisher = nh.advertise<To>(_publisher_topic, 1, true);
  }

  void subscriber_callback(const FromConstPtr& from_msg)
  {
    translate_msg(_to_msg, *from_msg);
    _publisher.publish(_to_msg);
  }

private:
  ros::Publisher _publisher;
  ros::Subscriber _subscriber;

  const std::string _publisher_topic;
  const std::string _subscriber_topic;

  To _to_msg;
};
}  // namespace interface