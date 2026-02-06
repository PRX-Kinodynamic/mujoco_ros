#pragma once
#include <rosbag/bag.h>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/execution_status.hpp>

namespace interface
{
inline void init_bag(rosbag::Bag* bag, const std::string rosbag_directory, const std::string prefix = "")
{
  std::ostringstream bag_name;
  bag_name << rosbag_directory << "/b_";
  if (prefix != "")
  {
    bag_name << prefix << "_";
  }
  bag_name << utils::timestamp() << ".bag";
  bag->open(bag_name.str(), rosbag::bagmode::Write);
  ROS_INFO_STREAM("Bag name: " << bag_name.str());
}

template <typename Msg>
class queued_callback_t
{
public:
  using TupleQueue = std::queue<std::tuple<ros::Time, typename Msg::ConstPtr>>;
  queued_callback_t() : _t0(ros::Time::now()){};
  queued_callback_t(const std::string topic_name) : _topic_name(topic_name), _t0(ros::Time::now())
  {
  }

  static inline std::mutex _queue_mutex;
  static inline TupleQueue _queue;

  void callback(const ros::MessageEvent<Msg const>& event)
  {
    const ros::Time t_now{ ros::Time::now() };
    if (t_now > _t0)
    {
      // const auto& map_str = event.getConnectionHeader();
      // for (auto& pair : map_str)
      // {
      // DEBUG_VARS(_topic_name);
      // DEBUG_VARS(event.getConnectionHeader());
      // }

      // DEBUG_VARS(event.getMessage())
      // DEBUG_VARS(event.getConnectionHeader())
      // DEBUG_VARS(event.getConnectionHeader().at("topic"))
      // const std::string topic = event.getConnectionHeader().at("topic");
      // DEBUG_VARS(_topic_name, topic, topic == _topic_name);
      // prx_assert(topic == _topic_name, "Topics don't match. Expected: " << _topic_name << " Got: " << topic);
      try
      {
        _queue.push(std::make_tuple(t_now, event.getMessage()));
      }
      catch (...)
      {
        std::cout << "Error at topic: " << _topic_name << std::endl;
      }
    }
  }

  std::string topic_name() const
  {
    return _topic_name;
  }

private:
  std::string _topic_name;
  const ros::Time _t0;
};

template <typename Msg>
class queues_t
{
public:
  using QCallback = queued_callback_t<Msg>;
  using Subscribers = std::vector<ros::Subscriber>;

  bool register_topic(const std::string& topic_name, const std::string topic_type, const std::string expected_type,
                      ros::NodeHandle& nh)
  {
    bool status{ false };
    if (topic_type == expected_type)  // Must be a nicer way of checking MsgType/topic_type == expected
    {
      _queues.emplace_back(topic_name);
      // subscribers.push_back(nh.subscribe(topic_name, 100, &QCallback::callback, &_queues.back()));
      _subscribers.push_back(nh.subscribe(topic_name, 100, &QCallback::callback, &_queues.back()));
      status = true;
    }
    return status;
  }

  std::size_t size() const
  {
    return _queues.size();
  }

  QCallback operator[](const std::size_t& idx) const
  {
    return _queues[idx];
  }

  QCallback& operator[](const std::size_t& idx)
  {
    return _queues[idx];
  }

private:
  std::vector<QCallback> _queues;
  Subscribers _subscribers;
};

}  // namespace interface
