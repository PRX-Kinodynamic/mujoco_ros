#pragma once
#include <any>
#include <utils/std_utils.cpp>
#include <utils/rosparams_utils.hpp>
#include <utils/execution_status.hpp>

namespace interface
{
void init_bag(rosbag::Bag* bag, const std::string rosbag_directory)
{
  std::ostringstream bag_name;
  bag_name << rosbag_directory << "/b_" << utils::timestamp() << ".bag";
  bag->open(bag_name.str(), rosbag::bagmode::Write);
  ROS_INFO_STREAM("Bag name: " << bag_name.str());
}

template <typename Msg>
class queued_callback_t
{
public:
  using TupleQueue = std::queue<std::tuple<std::string, ros::Time, typename Msg::ConstPtr>>;
  queued_callback_t() : _t0(ros::Time::now()){};
  queued_callback_t(const std::string topic_name) : _topic_name(topic_name)
  {
    std::cout << "topic_name: " << _topic_name << std::endl;
  };
  static inline std::mutex _queue_mutex;
  static inline TupleQueue _queue;

  void callback(const ros::MessageEvent<Msg const>& event)
  {
    const ros::Time t_now{ ros::Time::now() };
    if (t_now > _t0)
    {
      const std::string topic{ event.getConnectionHeader().at("topic") };
      _queue.push(std::make_tuple(topic, t_now, event.getMessage()));
    }
  }

private:
  std::string _topic_name;
  const ros::Time _t0;
};

class queues_base_t
{
  using Subscribers = std::vector<ros::Subscriber>;

public:
  queues_base_t(){};
  virtual bool register_topic(const std::string& topic_name, const std::string topic_type,
                              const std::string expected_type, Subscribers& subscribers, ros::NodeHandle& nh) = 0;
  virtual std::size_t size() const = 0;
  virtual std::size_t process_queue(rosbag::Bag& bag) = 0;
};

template <typename Msg>
class queues_t : public queues_base_t
{
public:
  using QCallback = queued_callback_t<Msg>;
  using Subscribers = std::vector<ros::Subscriber>;

  virtual bool register_topic(const std::string& topic_name, const std::string topic_type,
                              const std::string expected_type, Subscribers& subscribers, ros::NodeHandle& nh) override
  {
    bool status{ false };
    if (topic_type == expected_type)  // Must be a nicer way of checking MsgType/topic_type == expected
    {
      _queues.emplace_back(topic_name);
      subscribers.push_back(nh.subscribe(topic_name, 100, &QCallback::callback, &_queues.back()));
      status = true;
    }
    return status;
  }

  virtual std::size_t size() const override
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

  virtual std::size_t process_queue(rosbag::Bag& bag) override
  {
    std::size_t msgs_left{ 0 };
    for (std::size_t idx = 0; idx < _queues.size(); ++idx)
    {
      if (!_queues[idx]._queue.empty())
      {
        msgs_left += _queues[idx]._queue.size();
        auto msg = _queues[idx]._queue.front();
        bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
        _queues[idx]._queue.pop();
      }
    }
    return msgs_left;
  }

private:
  std::vector<QCallback> _queues;
};

}  // namespace interface