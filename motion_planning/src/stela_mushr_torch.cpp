#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <prx_models/mushr_mujoco.hpp>
#include <prx_models/mushr_torch.hpp>

#include <motion_planning/stela_sliding_window.hpp>

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MushrStela" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(4);

  DEBUG_PRINT;
  // motion_planning::stela_windowed_t<prx_models::mushr_torch_stela_t> stela;
  // DEBUG_PRINT;
  // stela.onInit(nh);
  // DEBUG_PRINT;

  // prx_assert(spinner.canStart(), "AsyncSpinner cannot start");
  // spinner.start();
  // DEBUG_PRINT;
  // stela.replanner_service_main();
  // DEBUG_PRINT;
  // spinner.stop();
  DEBUG_PRINT;

  ros::waitForShutdown();

  return 0;
}