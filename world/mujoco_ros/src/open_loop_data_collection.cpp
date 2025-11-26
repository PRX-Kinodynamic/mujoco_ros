#include <thread>
#include "mujoco_ros/control_listener.hpp"
#include "mujoco_ros/sensordata_publisher.hpp"
#include "mujoco_ros/Collision.h"
#include "prx_models/mj_mushr.hpp"
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

#include "mujoco_ros/camera_publisher.hpp"

using CtrlMsg = prx_models::MushrControl;
using PlanMsg = prx_models::MushrPlan;

struct data_collector_t
{
  // int step, curr_step;
  ros::Timer timer;
  CtrlMsg control;
  std::vector<std::vector<double>> data;
  mj_ros::SimulatorPtr _sim;
  std::ofstream ofs;
  double duration, elapsed;

  controller_listener_t<CtrlMsg, PlanMsg> controller_listener;
  data_collector_t(ros::NodeHandle& nh, mj_ros::SimulatorPtr sim)
    : controller_listener(nh, sim->d), _sim(sim), elapsed(0.0)
  {
    double& steering_angle{ control.steering_angle.data };
    double& velocity{ control.velocity.data };
    double frequency;
    std::string filename;

    PARAM_SETUP(nh, steering_angle);
    PARAM_SETUP(nh, velocity);
    PARAM_SETUP(nh, frequency);
    PARAM_SETUP(nh, filename);
    PARAM_SETUP(nh, duration);

    DEBUG_VARS(filename);
    ofs.open(filename.c_str(), std::ofstream::trunc);

    const ros::Duration control_timer(1.0 / frequency);
    timer = nh.createTimer(control_timer, &data_collector_t::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    controller_listener.apply_control(control);
    // _message.raw_sensor_data[i].data = _sim->d->sensordata[i];
    // data.emplace_back();
    const double dt{ (event.current_real - event.last_real).toSec() };
    if (event.last_real.isZero())
    {
      return;
    }
    else
    {
      // printf("current_real: %.10f\n", event.current_real.toSec());
      // printf("last_real: %.10f\n", event.last_real.toSec());
      elapsed += dt;
      DEBUG_VARS(elapsed, duration);
    }
    ofs << dt << " ";
    for (int i = 0; i < _sim->m->nsensordata; ++i)
    {
      ofs << _sim->d->sensordata[i] << " ";
      // data.back().push_back(_sim->d->sensordata[i]);
    }
    ofs << "\n";

    if (elapsed > duration)
    {
      PRINT_MSG("Finished!");
      ofs.close();
      ros::shutdown();
      // exit(0);
    }
  }

  void run()
  {
    // ofs <<
    //   if (curr_step % step == 0)
    //   {
    //     controller_listener.apply_control(control);
    //     _message.raw_sensor_data[i].data = _sim->d->sensordata[i];
    //   }
    //   curr_step++;
  }
};
// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MuSHRDataCollection" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };
  const std::string node_name_prefix{ ros::this_node::getName() };

  bool visualize_sim, visualize_output, publish_ground_truth_pose;
  visualize_sim = true;
  visualize_output = true;
  // utils::get_param_and_check(n, node_name_prefix + "/visualize_sim", visualize_sim);
  // utils::get_param_and_check(n, node_name_prefix + "/visualize_output", visualize_output);
  // utils::get_param_and_check(n, node_name_prefix + "/publish_ground_truth_pose", publish_ground_truth_pose);

  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(node_name_prefix, nh) };
  mj_ros::sensordata_publisher_t sensordata_publisher(nh, sim, 15);

  data_collector_t data_collector(nh, sim);

  ros::Subscriber reset_subscriber_for_sim, reset_subscriber_for_viz;
  reset_subscriber_for_sim = nh.subscribe(root + "/reset", 1000, &mj_ros::simulator_t::reset_simulation, sim.get());

  ros::ServiceServer collision_service =
      nh.advertiseService(root + "/collision", &mj_ros::simulator_t::in_collision, sim.get());
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), &mj_ros::simulator_t::collision_updater, sim.get());

  std::vector<ros::Subscriber> sim_subscribers;
  mj_ros::VisualizerPtr visualizer{ mj_ros::simulator_visualizer_t::initialize(sim, visualize_sim) };
  sim_subscribers.push_back(
      nh.subscribe(root + "/reset", 1000, &mj_ros::simulator_visualizer_t::reset, visualizer.get()));
  // if (visualize_output)
  // {
  //   sim_subscribers.push_back(
  //       n.subscribe(root + "/goal_pose", 1000, &mj_ros::simulator_visualizer_t::set_goal_pos, visualizer.get()));
  //   sim_subscribers.push_back(
  //       n.subscribe(root + "/goal_radius", 1000, &mj_ros::simulator_visualizer_t::set_goal_radius,
  //       visualizer.get()));
  //   sim_subscribers.push_back(n.subscribe(
  //       root + "/ml4kp_traj", 1000, &mj_ros::simulator_visualizer_t::set_trajectory_to_visualize, visualizer.get()));
  // }

  mj_ros::camera_rgb_publisher_t camera_publisher(nh, sim, "observer_camera");
  // mj_ros::run_simulation(sim, visualizer, 2, sensordata_publisher);
  mj_ros::run_simulation(sim, visualizer, 3, sensordata_publisher);
  // if (publish_ground_truth_pose)
  // {
  // }
  // else
  // {
  // }

  ROS_INFO_STREAM(node_name << " finished.");
  return 0;
}