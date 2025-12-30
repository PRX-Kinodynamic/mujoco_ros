#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <motion_planning/stela_sliding_window.hpp>

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MushrStela" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  motion_planning::stela_windowed_t<prx_models::mushr_stela_t> stela;
  stela.onInit(nh);

  ros::AsyncSpinner spinner(4);
  prx_assert(spinner.canStart(), "AsyncSpinner cannot start");
  spinner.start();
  stela.replanner_service_main();

  ros::waitForShutdown();
  // template <typename RobotInterface>

  return 0;
}