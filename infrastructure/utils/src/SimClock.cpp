#include <thread>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/SetDuration.h>

struct sim_clock_t
{
  sim_clock_t(ros::NodeHandle& nh, double sim_step, double sleep_dur)
    : now(0.0), keep_going(true), steps(0), simulation_step(sim_step), d(sim_step)
  {
    const std::string duration_service_name{ "/sim_clock/set_duration" };

    duration_service = nh.advertiseService(duration_service_name, &sim_clock_t::service_callback, this);
    clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    nanosecs = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>{ sleep_dur });
    msg.clock.sec = 0.0;
    msg.clock.nsec = 0.0;
  }

  bool service_callback(interface::SetDuration::Request& req, interface::SetDuration::Response& res)
  {
    const ros::Duration dur{ req.data };
    steps = std::floor(dur.toSec() / simulation_step);
    res.success = true;
    step_and_publish();
    return true;
  }

  void keyboard_input()
  {
    while (keep_going)
    {
      int c_in = getchar();
      steps++;
      if (c_in == 'q')
        keep_going = false;
      if ('1' <= c_in and c_in <= '9')
      {
        steps = c_in - '0';
        steps = std::floor(steps / simulation_step) - 1;  // -1 to remove the extra return
      }
      step_and_publish();
    }
  }

  void step_and_publish()
  {
    for (; steps > 0; steps--)
    {
      now = now + d;
      msg.clock.sec = now.sec;
      msg.clock.nsec = now.nsec;
      clock_publisher.publish(msg);
      std::this_thread::sleep_for(nanosecs);
    }
  }

  std::atomic<int> steps;
  const double simulation_step;
  const ros::Duration d;
  ros::Time now;

  std::atomic<bool> keep_going;

  ros::ServiceServer duration_service;
  ros::Publisher clock_publisher;

  std::chrono::nanoseconds nanosecs;
  rosgraph_msgs::Clock msg;
};

int main(int argc, char** argv)
{
  const std::string node_name{ "SimClock" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  double SimStep{ 0.01 };
  double realtime_factor{ 1.0 };
  // double StepDuration{ -1.0 };
  PARAM_SETUP_WITH_DEFAULT(nh, SimStep, SimStep);
  PARAM_SETUP_WITH_DEFAULT(nh, realtime_factor, realtime_factor);

  prx_assert(realtime_factor > 0, "Real time factor must be greater than 0");
  // PARAM_SETUP_WITH_DEFAULT(nh, StepDuration, StepDuration);

  int c_in;

  const std::string message{ "Press 'return' to step, 'q'+'return' to quit." };
  DEBUG_VARS(message);
  DEBUG_VARS(SimStep);
  // DEBUG_VARS(StepDuration);
  const double sleep_dur{ SimStep / realtime_factor };
  // auto nanosecs = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>{ sleep_dur });
  // millisecs = millisecs / realtime_factor;
  // int steps{ 1 };
  // DEBUG_VARS(nanosecs.count());
  sim_clock_t sim_clock(nh, SimStep, sleep_dur);

  std::thread keyboard_thread(&sim_clock_t::keyboard_input, &sim_clock);
  while (sim_clock.keep_going)
  {
    // while (sim_clock.steps > 1)

    ros::spinOnce();
    // sim_clock.steps = 0;

    // if (StepDuration > 0.0)
    //   break;
    // delete line
    // std::cout << "\n";       // Move cursor up one
    // std::cout << "\x1b[1A";  // Move cursor up one
    // std::cout << "\x1b[2K";  // Delete the entire line
  }
  keyboard_thread.join();
  return 0;
}