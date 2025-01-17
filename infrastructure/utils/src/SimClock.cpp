#include <thread>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>

#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/SetDuration.h>

struct sim_clock_t
{
  sim_clock_t(ros::NodeHandle& nh, double sim_step, double sleep_dur, int control_frequency, std::string replan_topic)
    : now(0.0), keep_going(true), steps(0), simulation_step(sim_step), d(sim_step), control_frequency(control_frequency), replan_topic(replan_topic)
  {
    const std::string duration_service_name{ "/sim_clock/set_duration" };

    nanosecs = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>{ sleep_dur });
    msg.clock.sec = 0.0;
    msg.clock.nsec = 0.0;

    control_steps = std::floor(1.0 / (sim_step * control_frequency));

    duration_service = nh.advertiseService(duration_service_name, &sim_clock_t::service_callback, this);
    clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
    control_subscriber = nh.subscribe("/stela/sbmp/control_stamped", 1, &sim_clock_t::control_callback, this);
    replan_publisher = nh.advertise<std_msgs::Empty>(replan_topic, 1);
  }

  void control_callback(const ml4kp_bridge::SpacePointStampedConstPtr message)
  {
    steps = control_steps;
    step_and_publish();
    std_msgs::Empty msg;
    replan_publisher.publish(msg);
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

  int control_frequency;
  std::string replan_topic;
  std::atomic<int> steps;
  int control_steps;
  const double simulation_step;
  const ros::Duration d;
  ros::Time now;

  std::atomic<bool> keep_going;

  ros::ServiceServer duration_service;
  ros::Publisher clock_publisher;
  ros::Subscriber control_subscriber;
  ros::Publisher replan_publisher;

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
  int warmup_steps{ 0 };
  int control_frequency{ 30 };
  std::string replan_scate_topic;
  // double StepDuration{ -1.0 };
  PARAM_SETUP(nh, replan_scate_topic);
  PARAM_SETUP_WITH_DEFAULT(nh, SimStep, SimStep);
  PARAM_SETUP_WITH_DEFAULT(nh, realtime_factor, realtime_factor);
  PARAM_SETUP_WITH_DEFAULT(nh, warmup_steps, warmup_steps);

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
  sim_clock_t sim_clock(nh, SimStep, sleep_dur, control_frequency, replan_scate_topic);

  // std::thread keyboard_thread(&sim_clock_t::keyboard_input, &sim_clock);

  sim_clock.steps = warmup_steps;
  sim_clock.step_and_publish();
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
  // keyboard_thread.join();
  return 0;
}