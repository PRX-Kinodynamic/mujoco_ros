#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <motion_planning/stela_sliding_window.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>

template <typename MushrModel>
void run(ros::NodeHandle nh)
{
  ros::MultiThreadedSpinner spinner(8);
  try
  {
    motion_planning::stela_windowed_t<MushrModel> stela;
    stela.onInit(nh);
    spinner.spin();

    // stela.replanner_service_main();
  }
  catch (...)
  {
    PRINT_MSG("Stela exception...\n");
  }
}

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrStela" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string mushr_model;

  PARAM_SETUP(nh, mushr_model);
  // experiment_t experiment(nh);

  DEBUG_VARS(mushr_model)
  if (mushr_model == "mushr")
  {
    run<prx_models::mushr_stela_t>(nh);
  }
#ifndef TORCH_NOT_BUILT
  else if (mushr_model == "mushr_torch")
  {
    run<prx_models::mushr_torch_stela_t>(nh);
  }
#endif
  else
  {
    PRINT_MSG("Invalid mushr type: " + mushr_model)
  }

  // ros::waitForShutdown();
  // spinner.stop();
  // template <typename RobotInterface>

  return 0;
}