#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

#include <mujoco_ros/simulator.hpp>
#include <mj_models/mj_mushr.hpp>

template <typename CtrlMsg, typename PlanMsg>
class controller_listener_t
{
public:
  controller_listener_t(const std::string root, ros::NodeHandle& nh, mjData* mj_data) : _mj_data(mj_data)
  {
    const std::string ctrl_topic{ root + "/control" };
    const std::string plan_topic{ root + "/plan" };
    _control_subscriber = nh.subscribe(ctrl_topic, 1000, &controller_listener_t::control_callback, this);
    _plan_subscriber = nh.subscribe(plan_topic, 1000, &controller_listener_t::plan_callback, this);
  }

  void control_callback(const boost::shared_ptr<CtrlMsg const>& msg)
  {
    mj_models::copy(_mj_data->ctrl, msg);
  }

  void plan_callback(const boost::shared_ptr<PlanMsg const>& msg)
  {
    const std::size_t dur_size{ msg->durations.size() };
    const std::size_t ctrl_size{ msg->control.size() };
    ROS_ASSERT(dur_size == ctrl_size);  // Check assertions work
    for (int i = 0; i < dur_size; ++i)
    {
      const double ctrl_dur{ _mj_data->time + msg->durations[i].data };
      mj_models::copy(_mj_data->ctrl, msg->control[i]);
      while (_mj_data->time < ctrl_dur)
      {
        // do nothing...
        // Could add a sleep here, but is risky since this is mujoco's sim time, not wall time
      }
    }
  }

private:
  mjData* const _mj_data;
  ros::Subscriber _control_subscriber;
  ros::Subscriber _plan_subscriber;
};