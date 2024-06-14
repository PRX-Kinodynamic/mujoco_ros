#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "control/MushrControlPropagation.h"
#include "motion_planning/replanner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include "motion_planning/PlanningResult.h"
#include "mujoco_ros/Collision.h"
#include "std_msgs/Empty.h"
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

  auto sg = planning_context.first;
  auto cg = planning_context.second;

  std::shared_ptr<prx::dirt_replan_t> dirt = std::make_shared<prx::dirt_replan_t>("dirt");
  prx::dirt_replan_specification_t* dirt_spec =
      new prx::dirt_replan_specification_t(planning_context.first, planning_context.second);
  dirt_spec->h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
    return dirt_spec->distance_function(s, s2) / 0.62;
  };

  /*
  prx::plan_t plan(cs);
  plan.append_onto_back(1.0);
  prx::trajectory_t traj(ss);

  std::vector<std::vector<double>> control_list = { { -1.0, 1.0 },  { 0.0, 1.0 },  { 1.0, 1.0 },
                                                    { -1.0, -1.0 }, { 0.0, -1.0 }, { 1.0, -1.0 } };
  dirt_spec->expand = [&](prx::space_point_t& s, std::vector<prx::plan_t*>& plans,
                          std::vector<prx::trajectory_t*>& trajs, int bn, bool blossom_expand) {
    if (blossom_expand)
    {
      for (unsigned i = 0; i < control_list.size(); i++)
      {
        traj.clear();
        cs->copy(plan.back().control, control_list[i]);
        sg->propagate(s, plan, traj);
        plans.push_back(new prx::plan_t(plan));
        trajs.push_back(new prx::trajectory_t(traj));
      }
    }
    else
    {
      prx::default_expand(s, plans, trajs, 1, sg, dirt_spec->sample_plan, dirt_spec->propagate);
    }
  };
  */

  bool propagate_dynamics, retain_previous, use_contingency;
  n.getParam(ros::this_node::getName() + "/propagate_dynamics", propagate_dynamics);
  n.getParam(ros::this_node::getName() + "/retain_previous", retain_previous);
  n.getParam(ros::this_node::getName() + "/use_contingency", use_contingency);

  dirt_spec->min_control_steps = params["min_time"].as<double>() * 1.0 / prx::simulation_step;
  dirt_spec->max_control_steps = params["max_time"].as<double>() * 1.0 / prx::simulation_step;
  dirt_spec->blossom_number = params["blossom_number"].as<int>();
  dirt_spec->use_pruning = false;
  dirt_spec->use_contingency = use_contingency;

  dirt_spec->contingency_check = [&](prx::trajectory_t& traj) {
    for (auto&& s : traj)
    {
      auto pqp_distance = prx::default_obstacle_distance_function(s, ss, cg);
      double min_distance = std::numeric_limits<double>::max();
      for (auto&& d : pqp_distance.distances)
      {
        if (d < min_distance)
        {
          min_distance = d;
        }
      }
      if (min_distance < params["safe_distance"].as<double>())
      {
        return false;
      }
    }
    return true;
  };

  std::vector<double> goal_config = params["goal_state"].as<std::vector<double>>();
  geometry_msgs::Pose2D goal_configuration;
  goal_configuration.x = goal_config[0];
  goal_configuration.y = goal_config[1];
  goal_configuration.theta = goal_config[2];

  std_msgs::Float64 goal_radius;
  goal_radius.data = params["goal_region_radius"].as<double>();

  prx::dirt_replan_query_t* dirt_query = new prx::dirt_replan_query_t(ss, cs);
  dirt_query->start_state = ss->make_point();
  dirt_query->goal_state = ss->make_point();
  dirt_query->goal_region_radius = goal_radius.data;
  dirt_query->get_visualization = false;
  ROS_WARN("Using default goal check");

  dirt->link_and_setup_spec(dirt_spec);
  dirt->preprocess();
  dirt->link_and_setup_query(dirt_query);

  ros::AsyncSpinner spinner(2);

  using PlannerService =
      mj_ros::planner_service_t<std::shared_ptr<prx::dirt_replan_t>, prx::dirt_replan_specification_t*,
                                prx::dirt_replan_query_t*, prx_models::MushrPlanner, prx_models::MushrObservation>;

  PlannerService planner_service(n, dirt, dirt_spec, dirt_query, propagate_dynamics, retain_previous);

  using PlannerClient = mj_ros::planner_client_t<prx_models::MushrPlanner, prx_models::MushrObservation>;
  PlannerClient planner_client(n, cs->get_dimension());

  ros::Publisher goal_pos_publisher = n.advertise<geometry_msgs::Pose2D>(root + "/goal_pose", 10, true);
  ros::Publisher goal_radius_publisher = n.advertise<std_msgs::Float64>(root + "/goal_radius", 10, true);
  ros::Publisher planning_result_publisher =
      n.advertise<motion_planning::PlanningResult>(root + "/planning_result", 1, true);
  ros::Publisher reset_publisher = n.advertise<std_msgs::Empty>(root + "/reset", 1, true);
  motion_planning::PlanningResult planning_result_msg;

  ros::ServiceClient collision_client = n.serviceClient<mujoco_ros::Collision>(root + "/collision");
  mujoco_ros::Collision collision_srv;

  int max_cycles;
  double planning_cycle_duration, preprocess_timeout, postprocess_timeout;
  n.getParam(ros::this_node::getName() + "/planning_cycle_duration", planning_cycle_duration);
  n.getParam(ros::this_node::getName() + "/preprocess_timeout", preprocess_timeout);
  n.getParam(ros::this_node::getName() + "/postprocess_timeout", postprocess_timeout);
  n.getParam(ros::this_node::getName() + "/max_cycles", max_cycles);
  planner_service.set_preprocess_timeout(preprocess_timeout);
  planner_service.set_postprocess_timeout(postprocess_timeout);
  dirt_spec->planning_cycle_duration = planning_cycle_duration;

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
        ROS_DEBUG("Preprocess time: %f", planner_service.get_preprocess_time() - planner_client.get_preprocess_time());
        ROS_DEBUG("Query fulfill time: %f",
                 planner_client.get_query_fulfill_time() - planner_service.get_query_fulfill_time());
      }
    }
    else
    {
      ROS_WARN("Goal reached, not replanning");
      planning_result_msg.goal_reached.data = true;
      planning_result_msg.in_collision.data = false;
      planning_result_msg.total_time.data = current_cycle * planning_cycle_duration;
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
        planning_result_msg.total_time.data = current_cycle * planning_cycle_duration;
        planning_result_publisher.publish(planning_result_msg);
        std_msgs::Empty reset_msg;
        reset_publisher.publish(reset_msg);
        break;
      }
    }
    ros::spinOnce();
  }

  if (current_cycle > max_cycles && !planner_client.is_goal_reached(goal_configuration, goal_radius))
  {
    ROS_WARN("Goal not reached");
    planning_result_msg.goal_reached.data = false;
    planning_result_msg.total_time.data = current_cycle * planning_cycle_duration;
    planning_result_publisher.publish(planning_result_msg);
  }

  int id;
  n.getParam(ros::this_node::getName() + "/id", id);

  auto error_data = planner_client.get_error_data();
  prx::space_point_t print_state = ss->make_point();

  std::ofstream error_file;
  error_file.open("/home/aravind/error_data_" + std::to_string(id) + ".txt");
  for (unsigned i = 0; i < std::get<0>(error_data).size(); i++)
  {
    ml4kp_bridge::copy(print_state, std::get<0>(error_data)[i]);
    error_file << ss->print_point(print_state) << ", ";
    ml4kp_bridge::copy(print_state, std::get<1>(error_data)[i]);
    error_file << ss->print_point(print_state) << ", ";
    prx_models::copy(print_state, std::get<2>(error_data)[i]);
    error_file << ss->print_point(print_state) << std::endl;
  }

  spinner.stop();

  return 0;
}