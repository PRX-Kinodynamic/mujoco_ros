#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

#include <mujoco_ros/simulator.hpp>
#include <prx_models/mj_mushr.hpp>

template <typename CtrlMsg>
class controller_listener_t
{
public:
  controller_listener_t(ros::NodeHandle& nh, mjData* mj_data, const std::string control_topic_name = "")
    : _mj_data(mj_data)
  {
    std::string ctrl_topic{ control_topic_name };
    if (topic_name == "")
    {
      const std::string root{ ros::this_node::getNamespace() };
      ctrl_topic = root + "/control";
    }
    _control_subscriber = nh.subscribe(ctrl_topic, 1000, &controller_listener_t::control_callback, this);
  }

  void control_callback(const boost::shared_ptr<CtrlMsg const>& msg)
  {
    prx_models::copy(_mj_data->ctrl, msg);
  }

private:
  mjData* const _mj_data;
  ros::Subscriber _control_subscriber;
  ros::Subscriber _plan_subscriber;
};