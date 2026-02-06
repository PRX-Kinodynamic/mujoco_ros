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
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MuSHRSimulation" };
  ros::init(argc, argv, node_name);
  // ros::NodeHandle n;
  ros::NodeHandle nh_priv("~");
  const std::string root{ ros::this_node::getNamespace() };
  const std::string node_name_prefix{ ros::this_node::getName() };

  bool visualize_sim, visualize_output, publish_ground_truth_pose;

  PARAM_SETUP(nh_priv, visualize_sim)
  PARAM_SETUP(nh_priv, visualize_output)
  PARAM_SETUP(nh_priv, publish_ground_truth_pose)
  // utils::get_param_and_check(n, node_name_prefix + "/visualize_sim", visualize_sim);
  // utils::get_param_and_check(n, node_name_prefix + "/visualize_output", visualize_output);
  // utils::get_param_and_check(n, node_name_prefix + "/publish_ground_truth_pose", publish_ground_truth_pose);

  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(node_name_prefix, nh_priv) };
  controller_listener_t<CtrlMsg, PlanMsg> controller_listener(nh_priv, sim->d);
  mj_ros::sensordata_publisher_t sensordata_publisher(nh_priv, sim, 15);

  ros::Subscriber reset_subscriber_for_sim, reset_subscriber_for_viz;
  reset_subscriber_for_sim =
      nh_priv.subscribe(root + "/reset", 1000, &mj_ros::simulator_t::reset_simulation, sim.get());

  ros::ServiceServer collision_service =
      nh_priv.advertiseService(root + "/collision", &mj_ros::simulator_t::in_collision, sim.get());
  ros::Timer timer = nh_priv.createTimer(ros::Duration(0.1), &mj_ros::simulator_t::collision_updater, sim.get());

  std::vector<ros::Subscriber> sim_subscribers;
  mj_ros::VisualizerPtr visualizer{ mj_ros::simulator_visualizer_t::initialize(sim, visualize_sim) };
  sim_subscribers.push_back(
      nh_priv.subscribe(root + "/reset", 1000, &mj_ros::simulator_visualizer_t::reset, visualizer.get()));
  if (visualize_output)
  {
    sim_subscribers.push_back(
        nh_priv.subscribe(root + "/goal_pose", 1000, &mj_ros::simulator_visualizer_t::set_goal_pos, visualizer.get()));
    sim_subscribers.push_back(nh_priv.subscribe(root + "/goal_radius", 1000,
                                                &mj_ros::simulator_visualizer_t::set_goal_radius, visualizer.get()));
    sim_subscribers.push_back(nh_priv.subscribe(
        root + "/ml4kp_traj", 1000, &mj_ros::simulator_visualizer_t::set_trajectory_to_visualize, visualizer.get()));
  }

  bool publish_camera{ true };
  PARAM_SETUP_WITH_DEFAULT(nh_priv, publish_camera, publish_camera);

  if (publish_camera)
  {
    mj_ros::camera_rgb_publisher_t camera_publisher(nh_priv, sim, "observer_camera");
    mj_ros::run_simulation(sim, visualizer, 3, sensordata_publisher, camera_publisher);
  }
  else
  {
    mj_ros::run_simulation(sim, visualizer, 3, sensordata_publisher);
  }
  // if (publish_ground_truth_pose)
  // {
  // }
  // else
  // {
  // }

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}