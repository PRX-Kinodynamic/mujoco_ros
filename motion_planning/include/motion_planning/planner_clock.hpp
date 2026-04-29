#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>

#include <iterator>
#include <memory>
#include <string>
#include <std_msgs/Bool.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <interface/PlannerClock.h>

#include <motion_planning/utils.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <utils/dbg_utils.hpp>

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
class planner_clock_t
{
public:
  planner_clock_t(ros::NodeHandle nh)
  {
    double cycle_duration{ 1.0 };
    std::string planner_clock_topic{ "/motion_planning/clock" };
    // Clock
    PARAM_SETUP(nh, cycle_duration);
    PARAM_SETUP_WITH_DEFAULT(nh, planner_clock_topic, planner_clock_topic);

    DEBUG_VARS(cycle_duration)
    DEBUG_VARS(planner_clock_topic)
    // Clock
    _planner_clock_publisher = nh.advertise<interface::PlannerClock>(planner_clock_topic, 1);

    _planner_clock_msg.cycle_duration = ros::Duration(cycle_duration);
    _planner_clock_msg.header.stamp = ros::Time::now();
    _planner_clock_msg.cycle_start = ros::Time::now();
    _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_start + _planner_clock_msg.cycle_duration;

    const ros::Duration timer_duration(0.001);

    _clock_timer = nh.createTimer(timer_duration, &planner_clock_t::clock_timer_callback, this);
  }

  ~planner_clock_t()
  {
  }

  void clock_timer_callback(const ros::TimerEvent& event)
  {
    _planner_clock_msg.state = interface::PlannerClock::LOW;
    if (_planner_clock_msg.cycle_start > ros::Time::now())
    {
      DEBUG_VARS(_planner_clock_msg.cycle_start, ros::Time::now())
      return;
    }

    _planner_clock_msg.header.stamp = ros::Time::now();
    if (_planner_clock_msg.header.stamp > _planner_clock_msg.cycle_end)
    {
      const std::scoped_lock lock{ _clock_mutex };
      _planner_clock_msg.state = interface::PlannerClock::HIGH;
      _planner_clock_msg.cycle++;
      _planner_clock_msg.cycle_start = _planner_clock_msg.cycle_end;
      _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_end + _planner_clock_msg.cycle_duration;
    }
    _planner_clock_publisher.publish(_planner_clock_msg);
  }

  ros::Time cycle_start() const
  {
    const std::scoped_lock lock{ _clock_mutex };
    return _planner_clock_msg.cycle_start;
  }
  ros::Time cycle_end() const
  {
    const std::scoped_lock lock{ _clock_mutex };
    return _planner_clock_msg.cycle_end;
  }

  int cycle() const
  {
    const std::scoped_lock lock{ _clock_mutex };
    return _planner_clock_msg.cycle;
  }
  ros::Duration cycle_duration() const
  {
    return _planner_clock_msg.cycle_duration;
  }

private:
  mutable std::mutex _clock_mutex;

  ros::Publisher _planner_clock_publisher;

  ros::Timer _clock_timer;

  interface::PlannerClock _planner_clock_msg;
};
}  // namespace motion_planning
