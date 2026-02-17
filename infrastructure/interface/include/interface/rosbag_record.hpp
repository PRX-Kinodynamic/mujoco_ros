#pragma once
#include <rosbag/bag.h>
#include <atomic>
#include <cstddef>
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
  bag_name << utils::timestamp();
  if (prefix != "")
  {
    bag_name << "_" << prefix;
  }
  bag_name << ".bag";
  bag->open(bag_name.str(), rosbag::bagmode::Write);
  ROS_INFO_STREAM("Bag name: " << bag_name.str());
}

template <typename Msg>
class rosbag_queue_t
{
  using This = rosbag_queue_t<Msg>;
  using Subscribers = std::vector<ros::Subscriber>;

public:
  Subscribers _subscribers;
  using Element = ros::MessageEvent<Msg>;
  // using Element = ros::MessageEvent<std::pair<std::string, Msg>>;
  // using Element = std::tuple<ros::Time, typename Msg::ConstPtr>;
  using TupleQueue = std::queue<Element>;

  rosbag_queue_t(const std::string type) : _expected_type(type), _t0(ros::Time::now()){};

  // queued_callback_t() : _t0(ros::Time::now()){};
  // queued_callback_t(const std::string topic_name) : _topic_name(topic_name), _t0(ros::Time::now())
  // {
  // }
  bool register_topic(const std::string& topic_name, const std::string topic_type, ros::NodeHandle& nh)
  {
    bool status{ false };
    if (topic_type == _expected_type)  // Must be a nicer way of checking MsgType/topic_type == expected
    {
      // _queues.emplace_back(topic_name);
      // subscribers.push_back(nh.subscribe(topic_name, 100, &QCallback::callback, &_queues.back()));
      _subscribers.push_back(nh.subscribe(topic_name, 10000, &This::callback, this));
      status = true;
    }
    return status;
  }

  void pause(const bool p)
  {
    _pause = p;
  }
  void reset()
  {
    PRINT_MSG("[rosbag_queue_t] RESETTING");
    _queue_mutex.lock();
    _total_msgs = 0;
    _t0 = ros::Time::now();
    _queue_mutex.unlock();
  }

  void callback(const ros::MessageEvent<Msg const>& event)
  {
    if (_pause)
      return;

    const ros::Time t_now{ event.getReceiptTime() };
    if (not t_now.isZero() and t_now > _t0)
    {
      try
      {
        _queue_mutex.lock();

        // auto topic_name = event.getConnectionHeaderPtr()->at("topic");

        _queue.push(event);
        // _queue.push(std::make_tuple(t_now, event.getMessage()));
        _total_msgs++;
        _queue_mutex.unlock();
      }
      catch (...)
      {
        std::cout << "Error at [queued_callback_t]" << std::endl;
      }
    }
  }

  std::size_t total_msgs() const
  {
    // _queue_mutex.lock();
    // const std::size_t msgs{ _total_msgs };
    // _queue_mutex.unlock();
    return _total_msgs;
    // return msgs;
    // return 1;
  }
  // std::string topic_name() const
  // {
  //   return _topic_name;
  // }

  std::size_t size() const
  {
    return _queue.size();
  }
  bool empty() const
  {
    return _queue.empty();
  }

  Element get_next()
  {
    _queue_mutex.lock();
    const Element msg{ _queue.front() };
    _queue.pop();
    _queue_mutex.unlock();
    return msg;
  }

protected:
  static inline std::mutex _queue_mutex;
  static inline TupleQueue _queue;

  static inline std::atomic<std::size_t> _total_msgs;
  // std::size_t _total_msgs;
  // std::string _topic_name;
  std::string _expected_type;
  ros::Time _t0;

  bool _pause;
};

// template <typename Msg>
// class queues_t
// {
// public:
//   using QCallback = queued_callback_t<Msg>;
//   using Subscribers = std::vector<ros::Subscriber>;

//   queues_t(const std::string type) : _expected_type(type)
//   {
//   }

//   bool register_topic(const std::string& topic_name, const std::string topic_type, ros::NodeHandle& nh)
//   {
//     bool status{ false };
//     if (topic_type == _expected_type)  // Must be a nicer way of checking MsgType/topic_type == expected
//     {
//       // _queues.emplace_back(topic_name);
//       // subscribers.push_back(nh.subscribe(topic_name, 100, &QCallback::callback, &_queues.back()));
//       _subscribers.push_back(nh.subscribe(topic_name, 10000, &QCallback::callback, &_queues.back()));
//       status = true;
//     }
//     return status;
//   }

//   void pause(const bool p)
//   {
//     for (int i = 0; i < size(); ++i)
//     {
//       _queues[i].pause(p);
//     }
//   }

//   void reset()
//   {
//     for (int i = 0; i < size(); ++i)
//     {
//       _queues[i].reset();
//     }
//   }

//   std::size_t size() const
//   {
//     return _queues.size();
//   }

//   QCallback operator[](const std::size_t& idx) const
//   {
//     return _queue;
//   }

//   QCallback& operator[](const std::size_t& idx)
//   {
//     return _queue;
//   }
//   std::size_t total_msgs() const
//   {
//     std::size_t tot{ 0 };
//     for (int i = 0; i < size(); ++i)
//     {
//       // tot += 1;
//       tot += _queue.total_msgs();
//     }
//     return tot;
//   }

// private:
//   // std::vector<QCallback> _queues;
//   QCallback _queue;
//   Subscribers _subscribers;
//   std::string _expected_type;
// };

}  // namespace interface
