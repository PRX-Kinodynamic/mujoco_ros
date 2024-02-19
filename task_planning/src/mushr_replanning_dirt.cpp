#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "motion_planning/replanner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "motion_planning/PlanningResult.h"
#include <utils/std_utils.cpp>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrPlanner_example");
  ros::NodeHandle n;

  const std::string root{ ros::this_node::getNamespace() };
  std::string params_fname;
  n.getParam(ros::this_node::getName() + "/params_file", params_fname);
  auto params = prx::param_loader(params_fname);

  // TODO: Does this need to be set inside the client/service for better determinism?
  prx::init_random(params["random_seed"].as<int>());

  prx::simulation_step = params["simulation_step"].as<double>();
  std::string plant_name = params["/plant/name"].as<std::string>();
  std::string plant_path = params["/plant/path"].as<std::string>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");

  auto obstacles = prx::load_obstacles("environments/obstacle_0.yaml");
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  prx::world_model_t planning_model({ plant }, {obstacle_list});
  planning_model.create_context("planner_context", { plant_name }, {obstacle_names});
  auto planning_context = planning_model.get_context("planner_context");
  auto ss = planning_context.first->get_state_space();
  auto cs = planning_context.first->get_control_space();
  auto ps = planning_context.first->get_parameter_space();
  std::vector<double> min_control_limits = params["/plant/control_space/lower_bound"].as<std::vector<double> >();
  std::vector<double> max_control_limits = params["/plant/control_space/upper_bound"].as<std::vector<double> >();
  std::vector<double> min_state_limits = params["/plant/state_space/lower_bound"].as<std::vector<double> >();
  std::vector<double> max_state_limits = params["/plant/state_space/upper_bound"].as<std::vector<double> >();
  ss->set_bounds(min_state_limits, max_state_limits);
  cs->set_bounds(min_control_limits, max_control_limits);
  std::vector<double> param_values = params["/plant/parameter_space/values"].as<std::vector<double> >();
  ps->copy_from(param_values);

  std::shared_ptr<prx::dirt_t> dirt = std::make_shared<prx::dirt_t>("dirt");

  prx::dirt_specification_t* dirt_spec = new prx::dirt_specification_t(planning_context.first, planning_context.second);
  ROS_WARN("Using defaults for distance function and heuristic");
  dirt_spec->min_control_steps = 0.1 * 1.0 / prx::simulation_step;
  dirt_spec->max_control_steps = 2.0 * 1.0 / prx::simulation_step;
  dirt_spec->blossom_number = 25;
  dirt_spec->use_pruning = false;

  std::string goal_config_str;
  n.getParam(ros::this_node::getName() + "/goal_config", goal_config_str);
  std::vector<double> goal_config = utils::split<double>(goal_config_str, ',');

  geometry_msgs::Pose2D goal_configuration;
  goal_configuration.x = goal_config[0];
  goal_configuration.y = goal_config[1];
  goal_configuration.theta = goal_config[2];

  std_msgs::Float64 goal_radius;
  goal_radius.data = 0.25;

  prx::dirt_query_t* dirt_query = new prx::dirt_query_t(ss, cs);
  dirt_query->start_state = ss->make_point();
  dirt_query->goal_state = ss->make_point();
  dirt_query->goal_region_radius = goal_radius.data;
  dirt_query->get_visualization = false;
  ROS_WARN("Using default goal check");

  dirt->link_and_setup_spec(dirt_spec);
  dirt->preprocess();
  dirt->link_and_setup_query(dirt_query);

  ros::AsyncSpinner spinner(2);

  using PlannerService = mj_ros::planner_service_t<std::shared_ptr<prx::dirt_t>, prx::dirt_specification_t*,
                                                   prx::dirt_query_t*, prx_models::MushrPlanner>;
  PlannerService planner_service(n, dirt, dirt_spec, dirt_query);

  bool visualize_trajectory;
  n.getParam(ros::this_node::getName() + "/visualize", visualize_trajectory);
  using PlannerClient = mj_ros::planner_client_t<prx_models::MushrPlanner, prx_models::MushrObservation>;
  PlannerClient planner_client(n, visualize_trajectory, cs->get_dimension());

  ros::Publisher goal_pos_publisher = n.advertise<geometry_msgs::Pose2D>(root + "/goal_pos", 10, true);
  ros::Publisher goal_radius_publisher = n.advertise<std_msgs::Float64>(root + "/goal_radius", 10, true);
  ros::Publisher planning_result_publisher = n.advertise<motion_planning::PlanningResult>(root + "/planning_result", 1, true);
  motion_planning::PlanningResult planning_result_msg;

  int max_cycles;
  double planning_cycle_duration, preprocess_timeout, postprocess_timeout;
  n.getParam(ros::this_node::getName() + "/planning_cycle_duration", planning_cycle_duration);
  n.getParam(ros::this_node::getName() + "/preprocess_timeout", preprocess_timeout);
  n.getParam(ros::this_node::getName() + "/postprocess_timeout", postprocess_timeout);
  n.getParam(ros::this_node::getName() + "/max_cycles", max_cycles);
  planner_service.set_preprocess_timeout(preprocess_timeout);
  planner_service.set_postprocess_timeout(postprocess_timeout);

  spinner.start();
  goal_pos_publisher.publish(goal_configuration);
  goal_radius_publisher.publish(goal_radius);
  double prev_time = ros::Time::now().toSec();
  double start_time = ros::Time::now().toSec();
  double current_time = ros::Time::now().toSec();
  int current_cycle = 0;
  while (ros::ok() && current_cycle <= max_cycles)
  {
    if (!planner_client.is_goal_reached(goal_configuration, goal_radius))
    {
      current_time = ros::Time::now().toSec();
      if (current_time - prev_time >= planning_cycle_duration)
      {
        ROS_INFO("Cycle %d Elapsed Time: %f", current_cycle, current_time - start_time);
        planner_client.call_service(goal_configuration, goal_radius, planning_cycle_duration);
        prev_time = current_time;
        current_cycle++;
        ROS_INFO("Preprocess time: %f", planner_service.get_preprocess_time() - planner_client.get_preprocess_time());
        ROS_INFO("Query fulfill time: %f",
                 planner_client.get_query_fulfill_time() - planner_service.get_query_fulfill_time());
      }
    }
    else
    {
      ROS_WARN("Goal reached, not replanning");
      planning_result_msg.goal_reached.data = true;
      planning_result_msg.total_time.data = current_cycle * planning_cycle_duration;
      planning_result_publisher.publish(planning_result_msg);
      break;
    }
    ros::spinOnce();
  }

  if (!planner_client.is_goal_reached(goal_configuration, goal_radius))
  {
    ROS_WARN("Goal not reached");
    planning_result_msg.goal_reached.data = false;
    planning_result_msg.total_time.data = current_cycle * planning_cycle_duration;
    planning_result_publisher.publish(planning_result_msg);
  }

  spinner.stop();

  return 0;
}