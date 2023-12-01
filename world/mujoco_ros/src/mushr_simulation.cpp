#include <thread>
#include "mujoco_ros/control_listener.hpp"
#include "mujoco_ros/sensordata_publisher.hpp"
#include "prx_models/mj_mushr.hpp"

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHRSimulation" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;
  const std::string root{ ros::this_node::getNamespace() };

  const std::string package_path{ ros::package::getPath("mujoco_ros") };

  std::string model_path;
  bool visualize, save_trajectory;
  if (!n.getParam("/model_path", model_path))
  {
    ROS_ERROR("Failed to get param 'model_path'.");
  }

  const std::string param_name_visualize{ ros::this_node::getName() + "/visualize" };
  if (!n.getParam(param_name_visualize, visualize))
  {
    ROS_ERROR("Failed to get param 'visualize'.");
  }

  const std::string param_name_save_trajectory{ ros::this_node::getName() + "/save_trajectory" };
  if (!n.getParam(param_name_save_trajectory, save_trajectory))
  {
    ROS_ERROR("Failed to get param 'save_trajectory'.");
  }

  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(model_path, save_trajectory) };
  controller_listener_t<CtrlMsg, PlanMsg> controller_listener(n, sim->d);
  mj_ros::sensordata_publisher_t sensordata_publisher(n, sim, 15);

  ros::Subscriber reset_subscriber;
  reset_subscriber = n.subscribe(root + "/reset", 1000, &mj_ros::simulator_t::reset_simulation, sim.get());

  // Set the threads
  std::thread step_thread(&mj_ros::simulator_t::run, &(*sim));  // Mj sim
  std::thread publisher_thread(&mj_ros::sensordata_publisher_t::run, &sensordata_publisher);
  std::shared_ptr<mj_ros::simulator_visualizer_t> visualizer;
  if (visualize)
    visualizer = std::make_shared<mj_ros::simulator_visualizer_t>(sim);
  ros::AsyncSpinner spinner(1);  // 1 thread for the controller

  ros::Subscriber goal_pos_subscriber, goal_radius_subscriber;
  if (visualize)
  {
    goal_pos_subscriber =
        n.subscribe(root + "/goal_pos", 1000, &mj_ros::simulator_visualizer_t::set_goal_pos, visualizer.get());
    goal_radius_subscriber =
        n.subscribe(root + "/goal_radius", 1000, &mj_ros::simulator_visualizer_t::set_goal_radius, visualizer.get());
  }

  // Run threads: Mj sim is already running at this point
  spinner.start();
  if (visualize)
    visualizer->run();  // Blocking

  // Join the non-visual threads
  step_thread.join();
  publisher_thread.join();
  spinner.stop();

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}