#pragma once
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ml4kp_bridge/plan_bridge.hpp>
namespace ml4kp_bridge
{
template <class PublisherType, typename CopyMsg>
class plan_stepper_t : public nodelet::Nodelet
{
  using PlanStepper = plan_stepper_t<PublisherType, CopyMsg>;

public:
  plan_stepper_t() : _plan_received(false)
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    std::string publisher_topic{};
    std::string subscriber_topic{};
    double frequency;
    private_nh.getParam("publisher_topic", publisher_topic);
    private_nh.getParam("subscriber_topic", subscriber_topic);
    private_nh.getParam("frequency", frequency);

    _timer_duration = ros::Duration(1.0 / frequency);
    _timer = private_nh.createTimer(_timer_duration, &PlanStepper::timer_callback, this);
    _publisher = private_nh.advertise<PublisherType>(publisher_topic, 1, true);
    _subscriber = private_nh.subscribe(subscriber_topic, 1, &PlanStepper::get_plan, this);
    // _rgb_subscriber = private_nh.subscribe(_rgb_topic_name, 1, &aruco_detection_nodelet_t::detect, this);
  }

  void get_plan(const ml4kp_bridge::PlanStampedConstPtr plan_in)
  {
    _plan_stamped = plan_in;
    _current_plan_step = 0;
    _next_time = ros::Time::now();
    _plan_received = true;
  }
  void timer_callback(const ros::TimerEvent& event)
  {
    if (_plan_received && _current_plan_step < _plan_stamped->plan.steps.size())
    {
      if (ros::Time::now() > _next_time)
      {
        _copy(_msg, _plan_stamped->plan.steps[_current_plan_step].control);
        _next_time = ros::Time::now() + _plan_stamped->plan.steps[_current_plan_step].duration.data;
        _current_plan_step++;
      }
      _publisher.publish(_msg);
    }
  }

  bool _plan_received;
  CopyMsg _copy;
  ros::Duration _timer_duration;
  ros::Timer _timer;
  ros::Publisher _publisher;
  ros::Subscriber _subscriber;

  ml4kp_bridge::PlanStampedConstPtr _plan_stamped;

  std::size_t _current_plan_step;
  ros::Time _next_time;

  PublisherType _msg;
};
}  // namespace ml4kp_bridge