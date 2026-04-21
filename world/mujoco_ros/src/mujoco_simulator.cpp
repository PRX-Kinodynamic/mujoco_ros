#include <memory>
#include <thread>

#include <ros/ros.h>
#include <mujoco_ros/control_listener.hpp>
#include <mujoco_ros/sensordata_publisher.hpp>
#include <mujoco_ros/Collision.h>
#include <prx_models/mj_mushr.hpp>
#include <utils/rosparams_utils.hpp>

#include "mujoco_ros/camera_publisher.hpp"
// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MjSim" };
  ros::init(argc, argv, node_name);
  // ros::NodeHandle n;
  ros::NodeHandle nh_priv("~");
  const std::string root{ ros::this_node::getNamespace() };
  const std::string node_name_prefix{ ros::this_node::getName() };

  std::shared_ptr<interface::node_status_t> node_status{ interface::node_status_t::create(nh_priv) };
  bool visualize_sim;
  bool publish_camera{ true };

  PARAM_SETUP(nh_priv, visualize_sim)
  PARAM_SETUP_WITH_DEFAULT(nh_priv, publish_camera, publish_camera);

  while (node_status->status() != interface::NodeStatus::EXIT)
  {
    if (node_status->new_request())
    {
      node_status->status(node_status->requested_status());
      node_status->request_acknowledged();
    }
    if (node_status->status() == interface::NodeStatus::INITIALIZING)
    {
      continue;
    }
    mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(nh_priv, node_status) };
    mj_ros::VisualizerPtr visualizer{ mj_ros::simulator_visualizer_t::initialize(sim, node_status, visualize_sim) };

    std::shared_ptr<mj_ros::camera_rgb_publisher_t> camera_publisher;  //(nh_priv, sim, "observer_camera");
    if (publish_camera)
    {
      camera_publisher = std::make_shared<mj_ros::camera_rgb_publisher_t>(nh_priv, sim, "observer_camera");
    }
    mj_ros::run_simulation(sim, visualizer, 3, camera_publisher);
  }

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}