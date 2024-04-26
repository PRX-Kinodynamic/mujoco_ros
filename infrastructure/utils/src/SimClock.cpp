#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

int main(int argc, char** argv)
{
  const std::string node_name{ "SimClock" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  double sim_step{ 0.01 };
  PARAM_SETUP_WITH_DEFAULT(nh, sim_step, sim_step);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  int c_in;
  rosgraph_msgs::Clock msg;
  msg.clock.sec = 0.0;
  msg.clock.nsec = 0.0;
  ros::Time t{ 0.0 };
  const ros::Duration d{ sim_step };
  const std::string message{ "Press 'return' to step, 'q'+'return' to quit." };
  DEBUG_VARS(message);
  while (ros::ok())
  {
    c_in = getchar();
    if (c_in == 'q')
      break;
    t = t + d;
    msg.clock.sec = t.sec;
    msg.clock.nsec = t.nsec;
    clock_publisher.publish(msg);
  }

  return 0;
}