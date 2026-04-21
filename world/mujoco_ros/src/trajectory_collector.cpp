#include <memory>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <string>
#include <thread>

#include <ros/duration.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <mujoco_ros/control_listener.hpp>
#include <mujoco_ros/sensordata_publisher.hpp>
#include <mujoco_ros/Collision.h>
#include <prx_models/mj_mushr.hpp>
#include <utils/rosparams_utils.hpp>

#include <prx/utilities/spaces/sampler.hpp>
#include <vector>

#include <ml4kp_bridge/PlanTrajectory.h>
#include <ros/time.h>
#include "mujoco_ros/camera_publisher.hpp"
#include "utils/dbg_utils.hpp"

struct collector_t
{
  double step_size;
  bool visualize_sim;
  double sensor_frequency;
  double min_trajectory_duration;
  int total_trajectories, collected_trajs;

  std::string model_path;
  std::vector<int> steps_bounds;
  std::vector<double> control_bounds_min;
  std::vector<double> control_bounds_max;

  mj_ros::SimulatorPtr sim;

  ros::Publisher publisher;

  prx::sampler_t<int> steps_sampler;
  prx::sampler_t<std::vector<double>> controls_sampler;
  double sensor_dt;

  collector_t(ros::NodeHandle nh) : collected_trajs(0)
  {
    std::string data_topic;

    PARAM_SETUP(nh, step_size)
    PARAM_SETUP(nh, data_topic)
    PARAM_SETUP(nh, model_path)
    PARAM_SETUP(nh, steps_bounds)
    PARAM_SETUP(nh, control_bounds_min)
    PARAM_SETUP(nh, control_bounds_max)
    PARAM_SETUP(nh, sensor_frequency)
    PARAM_SETUP(nh, total_trajectories)
    PARAM_SETUP(nh, min_trajectory_duration)

    steps_sampler.bounds(steps_bounds[0], steps_bounds[1]);
    controls_sampler.bounds(control_bounds_min, control_bounds_max);

    sensor_dt = 1.0 / sensor_frequency;

    sim = mj_ros::simulator_t::initialize(model_path);

    publisher = nh.advertise<ml4kp_bridge::PlanTrajectory>(data_topic, 1000, true);
  }

  void run()
  {
    PRINT_MSG("Running");
    while (collected_trajs < total_trajectories)
    {
      sim->reset_simulation();
      ml4kp_bridge::PlanTrajectory plan_traj{ collect_trajectory() };
      publisher.publish(plan_traj);
      collected_trajs++;
    }
    PRINT_MSG("Done!");
  }

  ml4kp_bridge::PlanTrajectory collect_trajectory()
  {
    ml4kp_bridge::PlanTrajectory result;

    double tz{ 0.0 };
    double t_accum{ 0.0 };
    const double sim_step{ sim->m->opt.timestep };
    std::vector<ml4kp_bridge::PlanStepStamped>& plan{ result.plan };
    std::vector<ml4kp_bridge::SpacePointStamped>& trajectory{ result.trajectory };

    const ros::Time start{ ros::Time::now() };
    trajectory.emplace_back();
    trajectory.back().header.stamp = start + ros::Duration(t_accum);
    // trajectory.back().space_point;  //.push_back();
    sim->sense(trajectory.back().space_point.point);

    // DEBUG_VARS(trajectory)
    while (t_accum < min_trajectory_duration)
    {
      const int random_steps{ steps_sampler() };
      const double edge_length{ static_cast<double>(step_size * random_steps) };

      // DEBUG_VARS(step_size, random_steps, edge_length)
      plan.emplace_back();

      plan.back().header.stamp = start + ros::Duration(t_accum);
      plan.back().plan_step.control.point = controls_sampler();
      plan.back().plan_step.duration.data = ros::Duration(edge_length);
      sim->set_control(plan.back().plan_step.control.point);

      // DEBUG_VARS(plan.steps.back())

      for (double ti = 0; ti < edge_length; ti += sim_step, t_accum += sim_step, tz += ti)
      {
        // DEBUG_VARS(ti)
        sim->step_simulation();

        if (tz > sensor_dt)
        {
          trajectory.emplace_back();
          trajectory.back().header.stamp = start + ros::Duration(t_accum);
          // trajectory.back().space_point;  //.push_back();
          sim->sense(trajectory.back().space_point.point);
          tz = 0.0;
        }
      }
    }
    // DEBUG_VARS(trajectory.back())

    return result;
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MjSimTrajCollector" };
  ros::init(argc, argv, node_name);
  // ros::NodeHandle n;
  ros::NodeHandle nh("~");

  bool visualize_sim;
  int total_simulations;
  PARAM_SETUP(nh, visualize_sim)
  PARAM_SETUP(nh, total_simulations)

  prx_assert(total_simulations > 0, "total_simulations must be positive");
  std::vector<collector_t> collectors;

  for (int i = 0; i < total_simulations; ++i)
  {
    collectors.emplace_back(nh);
  }

  std::vector<std::thread> threads{};

  ros::AsyncSpinner spinner(2);  // 1 thread for the controller
  spinner.start();

  mj_ros::VisualizerPtr visualizer{ mj_ros::simulator_visualizer_t::initialize(collectors[0].sim, nullptr,
                                                                               visualize_sim) };

  for (auto&& collector : collectors)
  {
    threads.emplace_back(&collector_t::run, &collector);
  }
  if (visualizer)
  {
    visualizer->run();  // Blocking
  }

  for (auto& thread : threads)
  {
    thread.join();
  }

  spinner.stop();

  return 0;
}