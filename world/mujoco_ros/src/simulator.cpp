#include <thread>
#include "mujoco_ros/control_listener.hpp"
#include "mujoco_ros/sensordata_publisher.hpp"
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>

#include "mujoco_ros/camera_publisher.hpp"

int main(int argc, char** argv)
{
  using Control = ml4kp_bridge::SpacePoint;
  using ControlStamped = ml4kp_bridge::SpacePointStamped;
  ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
  const std::string root{ ros::this_node::getNamespace() };

  std::string model_path;
  std::string control_topic_name;
  std::string reset_topic_name{ root + "/reset" };
  bool visualize{ true };
  std::vector<std::string> cameras{};
  int sensor_frequency{ 10 };

  PARAM_SETUP(private_nh, model_path);
  PARAM_SETUP(private_nh, control_topic_name);
  PARAM_SETUP_WITH_DEFAULT(private_nh, cameras, cameras);
  PARAM_SETUP_WITH_DEFAULT(private_nh, visualize, visualize);
  PARAM_SETUP_WITH_DEFAULT(private_nh, sensor_frequency, sensor_frequency);
  PARAM_SETUP_WITH_DEFAULT(private_nh, reset_topic_name, reset_topic_name);

  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(model_path, save_trajectory) };

  const std::string control_stamped_topic_name{ control_topic_name + "_stamped" };

  controller_listener_t<Control> control_listener(n, sim->d, control_topic_name);
  controller_listener_t<ControlStamped> control_stamped_listener(n, sim->d, control_stamped_topic_name);

  ros::Subscriber reset_subscriber_for_sim;
  reset_subscriber_for_sim = n.subscribe(reset_topic_name, 10, &mj_ros::simulator_t::reset_simulation, sim.get());

  std::vector<ros::Subscriber> sim_subscribers;

  const std::string goal_pose_topic_name{ root + "/visualization/goal/pos" };
  const std::string goal_rad_topic_name{ root + "/visualization/goal/radius" };
  const std::string trajectory_viz_topic_name{ root + "/visualization/traj" };

  mj_ros::VisualizerPtr visualizer{ mj_ros::simulator_visualizer_t::initialize(sim, visualize) };
  if (visualize)
  {
    sim_subscribers.push_back(
        n.subscribe(goal_pose_topic_name, 10, &mj_ros::simulator_visualizer_t::set_goal_pos, visualizer.get()));
    sim_subscribers.push_back(
        n.subscribe(goal_rad_topic_name, 10, &mj_ros::simulator_visualizer_t::set_goal_radius, visualizer.get()));
    sim_subscribers.push_back(n.subscribe(
        trajectory_viz_topic_name, 10, &mj_ros::simulator_visualizer_t::set_trajectory_to_visualize, visualizer.get()));
    sim_subscribers.push_back(
        n.subscribe(reset_topic_name, 10, &mj_ros::simulator_visualizer_t::reset, visualizer.get()));
  }

  std::vector<mj_ros::camera_rgb_publisher_t> camera_publishers;  //(n, sim, "camera_0");
  for (auto cam : cameras)
  {
    camera_publishers.emplace_back(n, sim, cam);
  }
  mj_ros::run_simulation(sim, visualizer, 3, camera_publishers);

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}