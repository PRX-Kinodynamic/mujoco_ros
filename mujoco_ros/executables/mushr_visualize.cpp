#include <thread>
#include "mujoco_ros/control_listener.hpp"
#include "mujoco_ros/control_listener.hpp"
#include "mj_models/mj_mushr.hpp"

int main(int argc, char** argv)
{
  using CtrlMsg = mj_models::MushrControl;
  using PlanMsg = mj_models::MushrPlan;
  const std::string node_name{ "Mujoco_Ros_viz_example" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  const std::string package_path{ ros::package::getPath("mujoco_ros") };

  std::string model_path;
  bool visualize;
  if (!n.getParam("/model_path", model_path))
  {
    ROS_ERROR("Failed to get param 'model_path'.");
  }

  const std::string param_name_visualize{ ros::this_node::getName() + "/visualize" };
  if (!n.getParam(param_name_visualize, visualize))
  {
    ROS_ERROR("Failed to get param 'visualize'.");
  }

  const std::string root{ "/mushr" };
  std::shared_ptr<mujoco_simulator_t> sim{ std::make_shared<mujoco_simulator_t>(model_path, false, visualize) };
  controller_listener_t<CtrlMsg, PlanMsg> controller_listener(root, n, sim->d);

  // Set the threads
  std::thread step_thread(&mujoco_simulator_t::run, &(*sim));  // Mj sim
  mujoco_simulator_visualizer_t visualizer{ sim->m, sim->d };  // Mj Viz
  ros::AsyncSpinner spinner(1);                                // 1 thread for the controller

  // Run threads: Mj sim is already running at this point
  spinner.start();
  visualizer();  // Blocking

  // Join the non-visual threads
  step_thread.join();
  spinner.stop();

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}
