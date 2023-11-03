#include <thread>
#include <mujoco_ros/simulator.hpp>
#include <mj_models/mj_mushr.hpp>
#include "mujoco_ros/feedback_service.hpp"
#include <std_msgs/Empty.h>

int main(int argc, char** argv)
{
  using FeedbackClient = mj_ros::feedback_client_t<mj_models::MushrFeedback>;
  const std::string node_name{ "Mujoco_Ros_feedback_example" };
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
  FeedbackClient feedback_client(root, n, sim->d, 10);

  ros::Subscriber reset_subscriber;
  reset_subscriber = n.subscribe(root + "/reset", 1000, &mujoco_simulator_t::reset_simulation, sim.get());

  // Set the threads
  std::thread step_thread(&mujoco_simulator_t::run, &(*sim));          // Mj sim
  mujoco_simulator_visualizer_t visualizer{ sim->m, sim->d };          // Mj Viz
  std::thread feedback_thread(&FeedbackClient::run, feedback_client);  // Mj sim

  // Run threads: Mj sim is already running at this point
  visualizer();  // Blocking

  // Join the non-visual threads
  step_thread.join();

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}