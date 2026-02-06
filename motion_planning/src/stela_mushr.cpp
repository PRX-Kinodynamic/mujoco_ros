#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <motion_planning/stela_sliding_window.hpp>
#include "utils/dbg_utils.hpp"

template <typename MushrModel>
void run(ros::NodeHandle& nh)
{
  ros::AsyncSpinner spinner(4);
  motion_planning::stela_windowed_t<MushrModel> stela;
  stela.onInit(nh);

  prx_assert(spinner.canStart(), "AsyncSpinner cannot start");
  spinner.start();
  stela.replanner_service_main();
  spinner.stop();
}

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MushrStela" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string mushr_model;

  PARAM_SETUP(nh, mushr_model);

  if (mushr_model == "analytical")
  {
    run<prx_models::mushr_stela_t>(nh);
  }
  else if (mushr_model == "mujoco")
  {
    run<prx_models::mushr_mujoco_stela_t>(nh);
  }
#ifndef TORCH_NOT_BUILT
  else if (mushr_model == "torch")
  {
    run<prx_models::mushr_torch_stela_t>(nh);
  }
#endif
  else
  {
    PRINT_MSG("Invalid mushr type: " + mushr_model)
  }
  ros::waitForShutdown();
  // template <typename RobotInterface>

  return 0;
}