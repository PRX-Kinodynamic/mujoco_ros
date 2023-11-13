#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "motion_planning/planner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "prx_models/mj_mushr.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrPlanner_example");
  ros::NodeHandle n;
  prx::simulation_step = 0.1;
  ROS_WARN("Using simulation step %f", prx::simulation_step);

  std::string plant_name = "MushrAnalytical";
  std::string plant_path = "MushrAnalytical";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");

  prx::world_model_t planning_model({ plant }, {});
  planning_model.create_context("planner_context", { plant_name }, {});
  auto planning_context = planning_model.get_context("planner_context");
  auto ss = planning_context.first->get_state_space();
  auto cs = planning_context.first->get_control_space();
  std::vector<double> min_control_limits = { -1., -1. };
  std::vector<double> max_control_limits = { 1., 1. };
  cs->set_bounds(min_control_limits, max_control_limits);

  std::shared_ptr<prx::dirt_t> dirt = std::make_shared<prx::dirt_t>("dirt");

  prx::dirt_specification_t* dirt_spec = new prx::dirt_specification_t(planning_context.first, planning_context.second);
  ROS_WARN("Using defaults for distance function and heuristic");
  dirt_spec->min_control_steps = 1;
  dirt_spec->max_control_steps = 25;
  dirt_spec->blossom_number = 25;
  dirt_spec->use_pruning = false;

  prx::dirt_query_t* dirt_query = new prx::dirt_query_t(ss, cs);
  dirt_query->start_state = ss->make_point();
  dirt_query->goal_state = ss->make_point();
  dirt_query->goal_region_radius = 0.5;
  dirt_query->get_visualization = true;
  ROS_WARN("Using default goal check");

  dirt->link_and_setup_spec(dirt_spec);
  dirt->preprocess();
  dirt->link_and_setup_query(dirt_query);

  ros::AsyncSpinner spinner(1);

  using PlannerService = mj_ros::planner_service_t<std::shared_ptr<prx::dirt_t>, prx::dirt_specification_t*,
                                                   prx::dirt_query_t*, prx_models::MushrPlanner>;
  PlannerService planner_service(n, dirt, dirt_spec, dirt_query);

  using PlannerClient =
      mj_ros::planner_client_t<prx_models::MushrPlanner, prx_models::MushrObservation, prx_models::MushrPlan>;
  PlannerClient planner_client(n);

  const std::string root{ ros::this_node::getNamespace() };
  ros::Publisher goal_pos_publisher = n.advertise<geometry_msgs::Pose2D>(root + "/goal_pos", 1000);
  ros::Publisher goal_radius_publisher = n.advertise<std_msgs::Float64>(root + "/goal_radius", 1000);

  geometry_msgs::Pose2D goal_configuration;
  goal_configuration.x = 5.0;
  goal_configuration.y = 5.0;
  goal_configuration.theta = 0.0;

  std_msgs::Float64 goal_radius;
  goal_radius.data = 0.5;

  spinner.start();
  planner_client.call_service(goal_configuration);
  goal_pos_publisher.publish(goal_configuration);
  goal_radius_publisher.publish(goal_radius);
  spinner.stop();

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});
  std::string body_name = plant_name + "/body";
  vis_group->add_vis_infos(prx::info_geometry_t::FULL_LINE, dirt_query->tree_visualization, body_name, ss);
  vis_group->add_animation(dirt_query->solution_traj, ss, dirt_query->start_state);
  vis_group->output_html("ROS_output.html");
  delete vis_group;

  return 0;
}