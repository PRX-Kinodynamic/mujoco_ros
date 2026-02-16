
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

  void init(ros::NodeHandle& nh, const std::string node_id)
  {
    _node_id = node_id;
    _msg.status = NodeStatus::INITIALIZING;
    _new_request = false;

    const std::string current_topic{ "/nodes/status/" + node_id + "/current" };
    const std::string change_topic{ "/nodes/status/" + node_id + "/change" };
    if (_observer)
    {
      _status_subscriber = nh.subscribe(current_topic, 1, &This::callback, this);
      _change_publisher = nh.advertise<interface::NodeStatus>(change_topic, 1, false);
    }
    else
    {
      _status_publisher = nh.advertise<interface::NodeStatus>(current_topic, 1, true);
      _timer = nh.createTimer(ros::Rate(2.0), &This::update, this);
      _status_subscriber = nh.subscribe(change_topic, 1, &This::callback, this);

      _status_publisher.publish(_msg);
    }
    // DEBUG_VARS(change_topic)
  }

public:
  using StatusType = uint8_t;
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

  std::string id() const
  {
    return _node_id;
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
      _requested_status = msg->status;
      _new_request = true;
      // status_change(msg->status);
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
    ost << "[" << obj._node_id << "]: " << str;
    return ost;
  }

  friend std::ostream& operator<<(std::ostream& ost, const std::shared_ptr<node_status_t>& obj)
  {
    ost << *obj;
    return ost;
  }

  inline void request_status(const StatusType status)
  {
    if (_observer)
    {
      _msg.status = status;
      _change_publisher.publish(_msg);
    }
    else
    {
      ROS_WARN("[node_status_t] Request change status on remote node while not being observer");
    }
  }

  inline bool new_request() const
  {
    return _new_request;
  }

  inline void request_acknowledged()
  {
    _new_request = false;
  }

  inline StatusType requested_status() const
  {
    return _requested_status;
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

  std::string _node_id;

  bool _new_request;
  StatusType _requested_status;

  bool _observer;  // Monitor another node

  ros::Timer _timer;
  ros::Publisher _change_publisher;
  ros::Publisher _status_publisher;
  ros::Subscriber _status_subscriber;
};
}  // namespace interface