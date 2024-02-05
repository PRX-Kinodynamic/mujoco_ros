#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/plan_bridge.hpp>
namespace ml4kp_bridge
{

// Class to execute a plan: given a plan, step over it at `frequency` rate publishing
// each control as ml4kp_bridge::SpacePoint on the topic defined by parameter `publisher_topic`.
// To be used with a msg_translator nodelet to avoid copies/networking of msgs.
class plan_stepper_t : public nodelet::Nodelet
{
public:
  plan_stepper_t() : _plan_received(false), _plan(), _current_plan_step(0)
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
    _timer = private_nh.createTimer(_timer_duration, &plan_stepper_t::timer_callback, this);
    _publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(publisher_topic, 1, true);
    _subscriber = private_nh.subscribe(subscriber_topic, 1, &plan_stepper_t::get_plan, this);
    _next_time = ros::Time::now();
  }

  void get_plan(const ml4kp_bridge::PlanStampedConstPtr plan_in)
  {
    // This copy is relatively expensive, but should be fine as long as plan_in is *small*
    // The copy is needed to be able to append plans
    std::copy(plan_in->plan.steps.begin(), plan_in->plan.steps.end(), std::back_inserter(_plan.steps));
    _plan_received = true;
  }
  void timer_callback(const ros::TimerEvent& event)
  {
    if (_plan_received)
    {
      if (_current_plan_step < _plan.steps.size())
      {
        if (ros::Time::now() > _next_time)
        {
          // We still have some plan steps to execute
          _next_time = ros::Time::now() + _plan.steps[_current_plan_step].duration.data;
          _ctrl = _plan.steps[_current_plan_step].control;
          _current_plan_step++;
          ROS_DEBUG("Next time: %f", _next_time.toSec());
        }
      }
      else if (_current_plan_step == _plan.steps.size())
      {
        // Executing the last step of the current plan
        if (ros::Time::now() > _next_time)
        {
          // We have exhausted the current plan
          _plan.steps.clear();
          _current_plan_step = 0;
          _plan_received = false;
        }
      }
      else
      {
        // We have exhausted the current plan
        _plan.steps.clear();
        _current_plan_step = 0;
        _plan_received = false;
      }
    }
    if (_plan_received)
    {
      _publisher.publish(_ctrl);
    }
  }

  ml4kp_bridge::SpacePoint _ctrl;

  bool _plan_received;
  ros::Duration _timer_duration;
  ros::Timer _timer;
  ros::Publisher _publisher;
  ros::Subscriber _subscriber;

  ml4kp_bridge::Plan _plan;

  std::size_t _current_plan_step;
  ros::Time _next_time;
};
}  // namespace ml4kp_bridge
PLUGINLIB_EXPORT_CLASS(ml4kp_bridge::plan_stepper_t, nodelet::Nodelet);
