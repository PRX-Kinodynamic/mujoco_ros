#include <thread>
#include <fstream>
#include <ml4kp_bridge/defs.h>
#include <prx_models/MushrPlanner.h>
#include <prx_models/mj_mushr.hpp>
#include <motion_planning/single_shot_planner_service.hpp>
#include <motion_planning/planner_client.hpp>

#include <utils/std_utils.cpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

#include <interface/SetInt.h>
#include <interface/SetDuration.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "StelaOpenLoop");
  ros::NodeHandle nh("~");

  int random_seed{ 921123 };
  int total_trajs{ 0 };
  int max_trajectories{ 10'000 };
  std::string state_topic{ "" };
  std::string trajs_service_name{ "" };
  std::string start_state{};
  std::string reset_topic{};

  DEBUG_VARS(max_trajectories);

  PARAM_SETUP(nh, total_trajs);
  PARAM_SETUP(nh, state_topic);
  PARAM_SETUP(nh, trajs_service_name);
  PARAM_SETUP(nh, start_state);
  PARAM_SETUP(nh, reset_topic);
  PARAM_SETUP_WITH_DEFAULT(nh, random_seed, random_seed);
  PARAM_SETUP_WITH_DEFAULT(nh, max_trajectories, max_trajectories);

  prx::init_random(random_seed);
  ros::ServiceClient client{ nh.serviceClient<interface::SetInt>(trajs_service_name) };
  ros::ServiceClient clock_client{ nh.serviceClient<interface::SetDuration>("/sim_clock/set_duration") };
  ros::Publisher state_publisher{ nh.advertise<ml4kp_bridge::SpacePoint>(state_topic, 10, true) };
  ros::Publisher reset_publisher{ nh.advertise<std_msgs::Empty>(reset_topic, 1, true) };

  interface::SetInt srv{};
  interface::SetDuration clock_srv{};
  ml4kp_bridge::SpacePoint start_state_msg{};

  std_msgs::Empty empty_msg{};
  prx::param_loader str_to_vec;  // start_state);
  str_to_vec["start_state"] = YAML::Load(start_state);
  std::vector<double> vec_ss{ str_to_vec["start_state"].as<std::vector<double>>() };
  start_state_msg.point.resize(vec_ss.size());
  for (std::size_t i = 0; i < vec_ss.size(); ++i)
  {
    start_state_msg.point[i] = vec_ss[i];
  }

  bool prev_fail{ false };
  double duration{ 0.0 };

  ros::Duration dur_to_sleep{};
  auto nanosecs = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>{ 1.0 });
  for (int i = 0; i < total_trajs;)
  {
    DEBUG_VARS(i);
    srv.request.value = prx::uniform_int_random(0, max_trajectories);
    DEBUG_VARS(srv.request.value);
    if (client.call(srv))
    {
      DEBUG_VARS(srv.response.answer);
      duration = prx::utilities::convert_to<double>(srv.response.answer);
      nanosecs = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>{ 1.5 * duration * 1e9 });
      clock_srv.request.data = ros::Duration(duration);
      // dur_to_sleep = ros::Duration(duration);
      max_trajectories = prev_fail ? max_trajectories * 1.5 : max_trajectories;
      ++i;
    }
    else
    {
      clock_srv.request.data = ros::Duration(1.0);
      nanosecs = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>{ 1.0 });

      max_trajectories = max_trajectories / 2;
      if (max_trajectories == 0)
        break;
    }
    // DEBUG_VARS("Calling clock");
    clock_client.call(clock_srv);
    state_publisher.publish(start_state_msg);
    reset_publisher.publish(empty_msg);
    // DEBUG_VARS("Going to sleep", nanosecs.count());
    // dur_to_sleep.sleep();
    // std::this_thread::sleep_for(nanosecs);
    // DEBUG_VARS("Waking up");
  }

  return 0;
}