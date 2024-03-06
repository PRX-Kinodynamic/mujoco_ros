#include <unordered_set>

#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "motion_planning/replanner_service.hpp"
#include "motion_planning/planner_client.hpp"
#include <utils/std_utils.cpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <prx/utilities/geometry/basic_geoms/box.hpp>
#include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer tf_buffer;
bool get_obstacle_pose(const std::string& world_frame, const std::string& obstacle_frame,
                       geometry_msgs::TransformStamped& tf)
{
  try
  {
    tf = tf_buffer.lookupTransform(world_frame, obstacle_frame, ros::Time(0));

    return true;
  }
  catch (tf2::TransformException& ex)
  {
  }
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacles_to_ml4kp_world");
  ros::NodeHandle nh("~");
  prx::simulation_step = 0.01;

  const std::string root{ ros::this_node::getNamespace() };
  prx::init_random(112392);

  tf2_ros::TransformListener tf_listener(tf_buffer);

  std::vector<std::string> obstacle_frames;
  std::string world_frame, robot_frame;
  ROS_PARAM_SETUP(nh, obstacle_frames);
  ROS_PARAM_SETUP(nh, world_frame);
  ROS_PARAM_SETUP(nh, robot_frame);

  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Failed to create plant");

  prx::world_model_t planning_model({ plant }, {});
  const std::string context_name{ "planner_context" };
  planning_model.create_context(context_name, { plant_name }, {});
  auto planning_context = planning_model.get_context(context_name);
  auto ss = planning_context.first->get_state_space();
  auto cs = planning_context.first->get_control_space();
  std::vector<double> min_control_limits = { -1., -0.5 };
  std::vector<double> max_control_limits = { 1., 0.5 };
  cs->set_bounds(min_control_limits, max_control_limits);

  ros::Rate rate(30);
  geometry_msgs::TransformStamped tf;

  bool pose_received{ false };
  // Eigen::Vector3d box_dims{ 0.32, 1.0, 0.28 };
  Eigen::Vector3d box_dims{ 0.41, 0.32, 0.28 };

  std::unordered_set<std::string> obstacle_set{};

  std::ofstream ofs_obstacles;  // ("/tmp/ml4kp_world_obstacles.txt", std::ofstream::trunc);
  std::size_t iters{ 0 };
  prx::trajectory_t traj{ ss };

  while (ros::ok())
  {
    ofs_obstacles = std::ofstream("/tmp/ml4kp_world_obstacles.txt", std::ofstream::trunc);
    for (auto obstacle : obstacle_frames)
    {
      if (get_obstacle_pose(world_frame, obstacle, tf))
      {
        prx::transform_t pose{ tf2::transformToEigen(tf) };
        // pose.translation()[2] *= 2;  // Make Z double, as ml4kp considers the center and this returns the top
        pose.translation()[2] = box_dims[2] / 2;  // Make Z half the height of box to "Zero" it

        if (obstacle_set.count(obstacle) == 0)
        {
          planning_model.emplace_obstacle<prx::box_t>(context_name, obstacle, box_dims[0], box_dims[1], box_dims[2],
                                                      pose);
          obstacle_set.insert(obstacle);
        }
        else
        {
          const std::shared_ptr<prx::movable_object_t> box_movable{ planning_model.obstacle(obstacle) };
          std::shared_ptr<prx::transform_t> box_tf{ box_movable->transform_ptr("body") };
          *box_tf = pose;  // Move it out so there is no collision
          // DEBUG_VARS(obstacle, pose.translation().transpose());
        }
      }
      // ofs_obstacles << pose.translation()[0];
    }
    if (get_obstacle_pose(world_frame, robot_frame, tf))
    {
      prx_models::copy(*(ss), tf);
      traj.copy_onto_back(ss);
      // DEBUG_VARS(robot_frame, *ss);
    }
    rate.sleep();
  }

  auto obstacles = planning_model.get_obstacles();
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacles });
  prx::space_point_t default_state{ ss->make_point() };
  ss->copy(default_state, Eigen::Vector4d::Zero());

  vis_group->add_animation(traj, ss, default_state);

  // std::string body_name = plant_name + "/" + "body";

  vis_group->output_html("ros_ml4kp.html");

  return 0;
}