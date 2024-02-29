#include <thread>
#include "mujoco_ros/control_listener.hpp"
#include "mujoco_ros/sensordata_publisher.hpp"
#include "mujoco_ros/Collision.h"
#include "prx_models/mj_mushr.hpp"
#include <utils/rosparams_utils.hpp>

#include "mujoco_ros/camera_publisher.hpp"
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
  const std::string node_name_prefix { ros::this_node::getName() };

  bool visualize, publish_ground_truth_pose;

  utils::get_param_and_check(n, node_name_prefix + "/visualize", visualize);
  utils::get_param_and_check(n, node_name_prefix + "/publish_ground_truth_pose", publish_ground_truth_pose);

  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(node_name_prefix, n) };
  controller_listener_t<CtrlMsg, PlanMsg> controller_listener(n, sim->d);
  mj_ros::sensordata_publisher_t sensordata_publisher(n, sim, 15);

  ros::Subscriber reset_subscriber_for_sim, reset_subscriber_for_viz;
  reset_subscriber_for_sim = n.subscribe(root + "/reset", 1000, &mj_ros::simulator_t::reset_simulation, sim.get());

  ros::ServiceServer collision_service = n.advertiseService(root + "/collision", &mj_ros::simulator_t::in_collision, sim.get());

  std::vector<ros::Subscriber> sim_subscribers;
  mj_ros::VisualizerPtr visualizer{ mj_ros::simulator_visualizer_t::initialize(sim, visualize) };
  if (visualize)
  {
    sim_subscribers.push_back(
        n.subscribe(root + "/goal_pos", 1000, &mj_ros::simulator_visualizer_t::set_goal_pos, visualizer.get()));
    sim_subscribers.push_back(
        n.subscribe(root + "/goal_radius", 1000, &mj_ros::simulator_visualizer_t::set_goal_radius, visualizer.get()));
    sim_subscribers.push_back(n.subscribe(
        root + "/ml4kp_traj", 1000, &mj_ros::simulator_visualizer_t::set_trajectory_to_visualize, visualizer.get()));
    sim_subscribers.push_back(
        n.subscribe(root + "/reset", 1000, &mj_ros::simulator_visualizer_t::reset, visualizer.get()));
  }

  mj_ros::camera_rgb_publisher_t camera_publisher(n, sim, "observer_camera");
  if (publish_ground_truth_pose)
  { 
    mj_ros::run_simulation(sim, visualizer, 2, sensordata_publisher);
  }
  else
  {
    mj_ros::run_simulation(sim, visualizer, 2, camera_publisher);
  }

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}