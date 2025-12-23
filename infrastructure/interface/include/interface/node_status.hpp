#pragma once

// Ros
#include <ros/ros.h>

// #include <ml4kp_bridge/defs.hpp>
#include <ml4kp_bridge/defs.h>
#include <interface/NodeStatus.h>
// #include <tensegrity_utils/assert.hpp>
#include <utils/dbg_utils.hpp>
namespace interface
{

class node_status_t
{
  using This = node_status_t;
  using StatusType = uint8_t;
  // using interface::NodeStatus

public:
  // TODO: Should move it to private
  node_status_t(ros::NodeHandle& nh, const std::string node_id)
  {
    _msg.status = NodeStatus::PREPARING;
    const std::string change_topic{ node_id + "/status/change" };
    const std::string current_topic{ node_id + "/status/current" };
    _status_publisher = nh.advertise<interface::NodeStatus>(current_topic, 1, true);
    _status_subscriber = nh.subscribe(change_topic, 1, &This::callback, this);

    _timer = nh.createTimer(ros::Rate(1.0), &This::update, this);
    _status_publisher.publish(_msg);
  }

  ~node_status_t()
  {
    // TODO: Publish stopped, but constructor needs to be private
  }

  static std::shared_ptr<node_status_t> create(ros::NodeHandle& nh, const std::string node_id)
  {
    return std::make_shared<node_status_t>(nh, node_id);
  }

  void update(const ros::TimerEvent& t)
  {
    if (_msg.status == NodeStatus::FINISH)
    {
      ros::shutdown();
    }
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
      case NodeStatus::PREPARING:
        str = "PREPARING";
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
      case NodeStatus::STOPPED:
        str = "STOPPED";
        break;
      case NodeStatus::FINISH:
        str = "FINISH";
        break;
      case NodeStatus::ERROR:
        str = "ERROR";
        break;
      case NodeStatus::RESTART:
        str = "RESTART";
        break;
      case NodeStatus::PAUSED:
        str = "PAUSED";
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
    _msg.status = new_status;
    _status_publisher.publish(_msg);
  }

  interface::NodeStatus _msg;

  ros::Timer _timer;
  ros::Publisher _status_publisher;
  ros::Subscriber _status_subscriber;
};
}  // namespace interface