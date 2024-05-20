#pragma once
#include <std_msgs/Empty.h>
#include <utils/std_utils.cpp>
namespace utils
{

template <typename Topic, void (*ToFile)(const Topic&, std::ofstream&), class Base>
class topic_to_file_t : public Base
{
  using Derived = topic_to_file_t<Topic, ToFile, Base>;
  using TopicConstPtr = boost::shared_ptr<Topic const>;

public:
  topic_to_file_t() : Base(), _on_reset_value("\n")
  {
  }

protected:
  virtual ~topic_to_file_t()
  {
    _ofs_topic.close();
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string directory;
    std::string fileprefix;
    std::string topic_name;
    std::string reset_topic;

    std::string& on_reset_value{ _on_reset_value };  // To avoid the _ in parameter

    PARAM_SETUP(private_nh, directory);
    PARAM_SETUP(private_nh, fileprefix);
    PARAM_SETUP(private_nh, topic_name);
    PARAM_SETUP(private_nh, reset_topic);
    PARAM_SETUP_WITH_DEFAULT(private_nh, on_reset_value, on_reset_value);

    std::filesystem::path path{ directory };
    path /= fileprefix;
    path += "_" + utils::timestamp() + ".txt";
    // + file_prefix + "_" + utils::timestamp() + ".txt" };

    _ofs_topic.open(path, std::ofstream::trunc);  // TODO: support other modes?

    DEBUG_VARS(topic_name, path);
    // subscriber
    _topic_subscriber = private_nh.subscribe(topic_name, 10, &Derived::topic_to_file_callback, this);
    _reset_subscriber = private_nh.subscribe(reset_topic, 1, &Derived::reset_callback, this);
  }

  void reset_callback(const std_msgs::Empty& msg)
  {
    _ofs_topic << _on_reset_value;
  }

  void topic_to_file_callback(const TopicConstPtr& msg)
  {
    ToFile(*msg, _ofs_topic);
  }

  std::string _on_reset_value;
  // Subscribers
  ros::Subscriber _topic_subscriber;
  ros::Subscriber _reset_subscriber;

  std::ofstream _ofs_topic;
};
}  // namespace utils