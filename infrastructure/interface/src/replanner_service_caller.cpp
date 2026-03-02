#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>
// #include <prx_models/mj_mushr.hpp>

// #include <interface/mushr_translation.hpp>
// #include <interface/msg_translator.hpp>
#include <utils/rosparams_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <interface/ExperimentParams.h>

int main(int argc, char** argv)
{
  const std::string node_name{ "ReplannerServiceCall" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getName() };

  ros::ServiceClient stela_service_client{ nh.serviceClient<interface::ExperimentParams>("/experiment/stela") };
  ros::ServiceClient replanner_service_client{ nh.serviceClient<interface::ExperimentParams>("/experiment/replanner") };

  interface::ExperimentParams experiment_params;

  std::string& stela_model{ experiment_params.request.stela_model };
  std::string& planning_model{ experiment_params.request.planning_model };

  bool validation_plan_feasibility;
  bool validation_collision_only;
  std::string& replanning_condition{ experiment_params.request.replanning_condition };
  float& cycle_duration{ experiment_params.request.cycle_duration };
  int& replanning_iterations{ experiment_params.request.replanning_iterations };
  int& total_replanning_calls{ experiment_params.request.total_replanning_calls };

  PARAM_SETUP(nh, stela_model);
  PARAM_SETUP(nh, planning_model);
  PARAM_SETUP(nh, validation_plan_feasibility);
  PARAM_SETUP(nh, validation_collision_only);
  PARAM_SETUP(nh, replanning_condition);
  PARAM_SETUP(nh, cycle_duration);
  PARAM_SETUP(nh, replanning_iterations);
  PARAM_SETUP(nh, total_replanning_calls);

  experiment_params.request.validation_plan_feasibility = validation_plan_feasibility;
  experiment_params.request.validation_collision_only = validation_collision_only;

  stela_service_client.call(experiment_params);
  replanner_service_client.call(experiment_params);

  // stop_callback stop(nh, control_topic, stop_topic);

  // ros::spin();
  return 0;
}