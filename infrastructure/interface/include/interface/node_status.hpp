#pragma once

// Ros
#include <ros/ros.h>

#include <interface/NodeStatus.h>
#include <utils/rosparams_utils.hpp>

namespace interface
{

class node_status_t
{
  using This = node_status_t;
  using StatusType = uint8_t;

  void init(ros::NodeHandle& nh, const std::string node_id)
  {
    _msg.status = NodeStatus::INITIALIZING;

    const std::string current_topic{ "/nodes/status/" + node_id + "/current" };
    const std::string change_topic{ "/nodes/status/" + node_id + "/change" };
    if (_observer)
    {
      _status_subscriber = nh.subscribe(current_topic, 1, &This::callback, this);
    }
    else
    {
      _status_publisher = nh.advertise<interface::NodeStatus>(current_topic, 1, true);
      _timer = nh.createTimer(ros::Rate(1.0), &This::update, this);
      _status_publisher.publish(_msg);
      _status_subscriber = nh.subscribe(change_topic, 1, &This::callback, this);
    }
    // DEBUG_VARS(change_topic)
  }

public:
  // using NodeSatus = interface::NodeStatus;

  // TODO: Should move it to private
  node_status_t(ros::NodeHandle& nh) : _observer(false)
  {
    std::string node_id;
    PARAM_SETUP(nh, node_id);
    init(nh, node_id);
  }

  node_status_t(ros::NodeHandle& nh, const std::string node_id, bool observer = false) : _observer(observer)
  {
    init(nh, node_id);
  }

  ~node_status_t()
  {
    // TODO: Publish stopped, but constructor needs to be private
  }

  template <typename... Ts>
  static std::shared_ptr<node_status_t> create(ros::NodeHandle& nh, Ts... args)
  {
    return std::make_shared<node_status_t>(nh, args...);
  }

  void update(const ros::TimerEvent& t)
  {
    _status_publisher.publish(_msg);
  }

  // TODO: add option to call custom function to check status
  virtual void callback(const interface::NodeStatusConstPtr msg)
  {
    try
    {
      status_change(msg->status);
    }
    catch (std::exception)
    {
      PRINT_MSG("[node_status_t] invalid message received.");
    }
  }

  friend std::ostream& operator<<(std::ostream& ost, const node_status_t& obj)
  {
    std::string str;
    switch (obj._msg.status)
    {
      case NodeStatus::INITIALIZING:
        str = "INITIALIZING";
        break;
      case NodeStatus::READY:
        str = "READY";
        break;
      case NodeStatus::RUNNING:
        str = "RUNNING";
        break;
      case NodeStatus::WAITING:
        str = "WAITING";
        break;
      case NodeStatus::PAUSED:
        str = "PAUSED";
        break;
      case NodeStatus::FINISH:
        str = "FINISH";
        break;
      case NodeStatus::RESET:
        str = "RESET";
        break;
      case NodeStatus::ERROR:
        str = "RESET";
        break;
      default:
        prx_throw("[node_status_t] Status unknown");
    }
    ost << str;
    return ost;
  }

  friend std::ostream& operator<<(std::ostream& ost, const std::shared_ptr<node_status_t>& obj)
  {
    ost << *obj;
    return ost;
  }

  inline StatusType status() const
  {
    return _msg.status;
  }

  inline void status(const StatusType new_status)
  {
    status_change(new_status);
  }

private:
  void status_change(const StatusType new_status)
  {
    const StatusType current{ _msg.status };

    _msg.status = new_status;
    if (not _observer)
      _status_publisher.publish(_msg);
  }

  interface::NodeStatus _msg;

  bool _observer;  // Monitor another node

  ros::Timer _timer;
  ros::Publisher _status_publisher;
  ros::Subscriber _status_subscriber;
};
}  // namespace interface