#include <thread>
#include <mujoco_ros/simulator.hpp>
#include <prx_models/mj_mushr.hpp>
#include <std_msgs/Empty.h>

#include "mujoco_ros/feedback_service.hpp"

int main(int argc, char** argv)
{
  using FeedbackClient = mj_ros::feedback_client_t<prx_models::MushrFeedback>;
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

  const std::string root{ ros::this_node::getNamespace() };
  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(model_path, false) };
  FeedbackClient feedback_client(n, sim, 10);

  ros::Subscriber reset_subscriber;
  reset_subscriber = n.subscribe(root + "/reset", 1000, &mj_ros::simulator_t::reset_simulation, sim.get());

  // Set the threads
  std::thread feedback_thread(&FeedbackClient::run, feedback_client);  // Mj sim

  const std::size_t callback_threads{ 1 };
  run_simulation(sim, callback_threads, visualize);  // Blocking

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}
