#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <ml4kp_bridge/plan_bridge.hpp>

namespace ml4kp_bridge
{

// Class to execute a plan: given a plan, step over it at `frequency` rate publishing
// each control as ml4kp_bridge::SpacePoint on the topic defined by parameter `publisher_topic`.
// To be used with a msg_translator nodelet to avoid copies/networking of msgs.
class plan_stepper_t : public nodelet::Nodelet
{
public:
  plan_stepper_t() : _plan_received(false), _plan(), _current_plan_step(0), _use_stamped_control(false)
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    std::string publisher_topic{};
    std::string subscriber_topic{};
    std::string reset_topic{};
    std::string finished_topic{ "" };

    double frequency;

    private_nh.getParam("publisher_topic", publisher_topic);
    private_nh.getParam("subscriber_topic", subscriber_topic);
    private_nh.getParam("frequency", frequency);
    private_nh.getParam("reset_topic", reset_topic);
    private_nh.getParam("reset_topic", reset_topic);
    private_nh.getParam("use_stamped_control", _use_stamped_control);
    // PARAM_SETUP_WITH_DEFAULT(private_nh, finished_topic, finished_topic);
    if (!private_nh.getParam("finished_topic", finished_topic))
    {
      finished_topic = ros::this_node::getNamespace() + "/PlanStepper/finished";
    }

    const std::string stamped_topic_name{ publisher_topic + "_stamped" };
    _timer_duration = ros::Duration(1.0 / frequency);
    _timer = private_nh.createTimer(_timer_duration, &plan_stepper_t::timer_callback, this);

    _publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(publisher_topic, 1, true);
    _stamped_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_topic_name, 1, true);
    _finished_publisher = private_nh.advertise<std_msgs::Bool>(finished_topic, 1, true);

    _subscriber = private_nh.subscribe(subscriber_topic, 1, &plan_stepper_t::get_plan, this);

    _reset_subscriber = private_nh.subscribe(reset_topic, 1, &plan_stepper_t::reset_callback, this);

    _next_time = ros::Time::now();
    // string frame_id
    _ctrl_stamped.header.frame_id = "PlanStepper";
  }

  void reset_callback(const std_msgs::Empty& msg)
  {
    _plan.steps.clear();
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
    const ros::Time& now{ event.current_real };
    if (_plan_received)
    {
      if (_current_plan_step < _plan.steps.size())
      {
        if (now >= _next_time)
        {
          // We still have some plan steps to execute
          _next_time = now + _plan.steps[_current_plan_step].duration.data - _timer_duration;
          _ctrl_stamped.space_point = _plan.steps[_current_plan_step].control;
          _current_plan_step++;
        }
      }
      else if (_current_plan_step == _plan.steps.size())
      {
        // Executing the last step of the current plan
        if (now > _next_time)
        {
          // We have exhausted the current plan
          _plan.steps.clear();
          _current_plan_step = 0;
          _plan_received = false;

          std_msgs::Bool msg;
          msg.data = true;
          _finished_publisher.publish(msg);
        }
      }
      else
      {
        // We have exhausted the current plan
        _plan.steps.clear();
        _current_plan_step = 0;
        _plan_received = false;
        std_msgs::Bool msg;
        msg.data = true;
        _finished_publisher.publish(msg);
      }
    }
    if (_plan_received)
    {
      if (_use_stamped_control)
      {
        // string frame_id
        _ctrl_stamped.header.seq++;
        _ctrl_stamped.header.stamp = ros::Time::now();
        _stamped_publisher.publish(_ctrl_stamped);
      }
      else
      {
        _publisher.publish(_ctrl_stamped.space_point);
      }
    }
  }

  // ml4kp_bridge::SpacePoint _ctrl;
  ml4kp_bridge::SpacePointStamped _ctrl_stamped;

  bool _plan_received;
  bool _use_stamped_control;
  ros::Duration _timer_duration;
  ros::Timer _timer;
  ros::Publisher _publisher;
  ros::Publisher _stamped_publisher;
  ros::Publisher _finished_publisher;
  ros::Subscriber _subscriber;
  ros::Subscriber _reset_subscriber;

  ml4kp_bridge::Plan _plan;

  std::size_t _current_plan_step;
  ros::Time _next_time;
};
}  // namespace ml4kp_bridge
PLUGINLIB_EXPORT_CLASS(ml4kp_bridge::plan_stepper_t, nodelet::Nodelet);