#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "motion_planning/planner_service.hpp"
#include "motion_planning/planner_client.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrPlanner_example");
  ros::NodeHandle n;
  prx::simulation_step = 0.01;

  const std::string root{ ros::this_node::getNamespace() };
  int random_seed;
  n.getParam(ros::this_node::getName() + "/random_seed", random_seed);
  prx::init_random(random_seed);

  // std::string plant_name = "MushrAnalytical";
  // std::string plant_path = "MushrAnalytical";
  std::string plant_name = "mushr";
  std::string plant_path = "mushr";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");

  prx::world_model_t planning_model({ plant }, {});
  planning_model.create_context("planner_context", { plant_name }, {});
  auto planning_context = planning_model.get_context("planner_context");
  auto ss = planning_context.first->get_state_space();
  auto cs = planning_context.first->get_control_space();
  // std::vector<double> min_control_limits = { -1., -0.5 };
  // std::vector<double> max_control_limits = { 1., 0.5 };
  std::vector<double> min_control_limits = { -.5, -1. };
  std::vector<double> max_control_limits = { .5, 1. };
  cs->set_bounds(min_control_limits, max_control_limits);

  std::shared_ptr<prx::dirt_t> dirt = std::make_shared<prx::dirt_t>("dirt");

  prx::dirt_specification_t* dirt_spec = new prx::dirt_specification_t(planning_context.first, planning_context.second);
  ROS_WARN("Using defaults for distance function and heuristic");
  dirt_spec->min_control_steps = 0.1 * 1.0 / prx::simulation_step;
  dirt_spec->max_control_steps = 2.0 * 1.0 / prx::simulation_step;
  dirt_spec->blossom_number = 25;
  dirt_spec->use_pruning = false;

  double goal_x, goal_y;
  n.getParam(ros::this_node::getName() + "/goal_x", goal_x);
  n.getParam(ros::this_node::getName() + "/goal_y", goal_y);

  geometry_msgs::Pose2D goal_configuration;
  goal_configuration.x = goal_x;
  goal_configuration.y = goal_y;
  goal_configuration.theta = 0.0;

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

  using PlannerClient =
      mj_ros::planner_client_t<prx_models::MushrPlanner, prx_models::MushrObservation, ml4kp_bridge::Plan>;
  PlannerClient planner_client(n);

  ros::Publisher goal_pos_publisher = n.advertise<geometry_msgs::Pose2D>(root + "/goal_pos", 10, true);
  ros::Publisher goal_radius_publisher = n.advertise<std_msgs::Float64>(root + "/goal_radius", 10, true);

  double planning_cycle_duration;
  n.getParam(ros::this_node::getName() + "/planning_duration", planning_cycle_duration);

  spinner.start();
  goal_pos_publisher.publish(goal_configuration);
  goal_radius_publisher.publish(goal_radius);
  planner_client.call_service(goal_configuration, goal_radius, planning_cycle_duration);

  ROS_INFO("Preprocess time: %f", planner_service.get_preprocess_time() - planner_client.get_preprocess_time());
  ROS_INFO("Query fulfill time: %f", planner_client.get_query_fulfill_time() - planner_service.get_query_fulfill_time());

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

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});
  std::string body_name = plant_name + "/body";
  vis_group->add_vis_infos(prx::info_geometry_t::FULL_LINE, dirt_query->tree_visualization, body_name, ss);
  vis_group->add_animation(dirt_query->solution_traj, ss, dirt_query->start_state);
  vis_group->output_html(out_html);
  delete vis_group;

  return 0;
}