#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>
#include <visualization_msgs/Marker.h>

#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <interface/PlannerClock.h>
#include <motion_planning/motion_planning_types.hpp>

namespace motion_planning
{

template <class Base>
class clock_sync_t : public Base
{
  using Derived = clock_sync_t<Base>;

public:
  clock_sync_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    int start_delay{ 1 };
    double cycle_duration{ 1.0 };

    std::string planner_clock_topic;

    PARAM_SETUP(private_nh, planner_clock_topic);
    PARAM_SETUP(private_nh, start_delay);
    PARAM_SETUP_WITH_DEFAULT(private_nh, cycle_duration, cycle_duration);

    _cycle_duration = ros::Duration(cycle_duration);

    // publishers
    _planner_clock_publisher = private_nh.advertise<interface::PlannerClock>(planner_clock_topic, 1);

    const ros::Duration timer_duration(0.1);

    _planner_clock_msg.header.stamp = ros::Time::now();
    _planner_clock_msg.cycle_start = ros::Time::now() + ros::Duration(start_delay);
    _planner_clock_msg.cycle_end = ros::Time::now() + ros::Duration(start_delay) + _cycle_duration;
    _planner_clock_msg.cycle_duration = _cycle_duration;
    DEBUG_VARS(_planner_clock_msg)

    _timer = private_nh.createTimer(timer_duration, &Derived::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    _planner_clock_msg.state = interface::PlannerClock::LOW;
    if (_planner_clock_msg.cycle_start > ros::Time::now())
    {
      // DEBUG_VARS(_planner_clock_msg.cycle_start, ros::Time::now())
      return;
    }

    _planner_clock_msg.header.stamp = ros::Time::now();
    if (_planner_clock_msg.header.stamp > _planner_clock_msg.cycle_end)
    {
      _planner_clock_msg.state = interface::PlannerClock::HIGH;
      _planner_clock_msg.cycle++;
      _planner_clock_msg.cycle_start = _planner_clock_msg.cycle_end;
      _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_end + _cycle_duration;
      _planner_clock_msg.cycle_duration = _cycle_duration;
      // DEBUG_VARS(_planner_clock_msg)
    }
    _planner_clock_publisher.publish(_planner_clock_msg);
  }

  ros::Duration _cycle_duration;
  interface::PlannerClock _planner_clock_msg;

  // Publishers
  ros::Publisher _planner_clock_publisher;

  ros::Timer _timer;
};
}  // namespace motion_planning