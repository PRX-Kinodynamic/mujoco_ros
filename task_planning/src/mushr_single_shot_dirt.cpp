#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "motion_planning/single_shot_planner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "motion_planning/PlanningResult.h"
#include "mujoco_ros/Collision.h"
#include "std_msgs/Empty.h"

#include <utils/std_utils.cpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

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

  auto obstacles = prx::load_obstacles(params["environment"].as<std::string>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  prx::world_model_t planning_model({ plant }, { obstacle_list });
  planning_model.create_context("planner_context", { plant_name }, { obstacle_names });
  auto planning_context = planning_model.get_context("planner_context");
  auto ss = planning_context.first->get_state_space();
  auto cs = planning_context.first->get_control_space();
  auto ps = planning_context.first->get_parameter_space();
  std::vector<double> min_control_limits = params["/plant/control_space/lower_bound"].as<std::vector<double>>();
  std::vector<double> max_control_limits = params["/plant/control_space/upper_bound"].as<std::vector<double>>();
  std::vector<double> min_state_limits = params["/plant/state_space/lower_bound"].as<std::vector<double>>();
  std::vector<double> max_state_limits = params["/plant/state_space/upper_bound"].as<std::vector<double>>();
  ss->set_bounds(min_state_limits, max_state_limits);
  cs->set_bounds(min_control_limits, max_control_limits);
  std::vector<double> param_values = params["/plant/parameter_space/values"].as<std::vector<double>>();
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
  dirt_query->get_visualization = true;
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
  ros::Publisher planning_result_publisher =
      n.advertise<motion_planning::PlanningResult>(root + "/planning_result", 1, true);
  ros::Publisher reset_publisher = n.advertise<std_msgs::Empty>(root + "/reset", 1, true);
  motion_planning::PlanningResult planning_result_msg;

  ros::ServiceClient collision_client = n.serviceClient<mujoco_ros::Collision>(root + "/collision");
  mujoco_ros::Collision collision_srv;

  double planning_cycle_duration;
  n.getParam(ros::this_node::getName() + "/planning_cycle_duration", planning_cycle_duration);
  ROS_INFO("Planning duration: %f", planning_cycle_duration);

  spinner.start();
  if (visualize_trajectory)
  {
    goal_pos_publisher.publish(goal_configuration);
    goal_radius_publisher.publish(goal_radius);
  }

  planner_client.call_service(goal_configuration, goal_radius, planning_cycle_duration);

  double start_time = ros::Time::now().toSec();
  ROS_INFO("Preprocess time: %f", planner_service.get_preprocess_time() - planner_client.get_preprocess_time());
  ROS_INFO("Query fulfill time: %f",
           planner_client.get_query_fulfill_time() - planner_service.get_query_fulfill_time());

  while (ros::ok())
  {
    if (planner_client.is_goal_reached(goal_configuration, goal_radius))
    {
      ROS_INFO("Goal reached");
      planning_result_msg.goal_reached.data = true;
      planning_result_msg.in_collision.data = false;
      planning_result_msg.total_time.data = ros::Time::now().toSec() - start_time;
      planning_result_publisher.publish(planning_result_msg);
      break;
    }
    if (collision_client.call(collision_srv))
    {
      if (collision_srv.response.collision_result.data)
      {
        ROS_WARN("Collision detected");
        planning_result_msg.goal_reached.data = false;
        planning_result_msg.in_collision.data = true;
        planning_result_msg.total_time.data = ros::Time::now().toSec() - start_time;
        planning_result_publisher.publish(planning_result_msg);
        std_msgs::Empty reset_msg;
        reset_publisher.publish(reset_msg);
        break;
      }
    }
    ros::Duration(0.1).sleep();
  }

  std::string plan_file_name;
  n.getParam(ros::this_node::getName() + "/plan_file", plan_file_name);
  std::string plan_file_path = ros::package::getPath("task_planning") + "/data/" + plan_file_name;
  ROS_INFO("Saving plan to %s", plan_file_path.c_str());

  std::string traj_file_name;
  n.getParam(ros::this_node::getName() + "/traj_file", traj_file_name);
  std::string traj_file_path = ros::package::getPath("task_planning") + "/data/" + traj_file_name;
  ROS_INFO("Saving trajectory to %s", traj_file_path.c_str());

  std::ofstream plan_file, traj_file;
  plan_file.open(plan_file_path);
  traj_file.open(traj_file_path);
  plan_file << dirt_query->solution_plan.print() << std::endl;
  traj_file << dirt_query->solution_traj.print() << std::endl;
  plan_file.close();
  traj_file.close();

  std::string out_html;
  n.getParam(ros::this_node::getName() + "/out_html", out_html);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });
  std::string body_name = plant_name + "/body";
  vis_group->add_vis_infos(prx::info_geometry_t::FULL_LINE, dirt_query->tree_visualization, body_name, ss);
  vis_group->add_animation(dirt_query->solution_traj, ss, dirt_query->start_state);
  vis_group->output_html(out_html);
  delete vis_group;

  return 0;
}