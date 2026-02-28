#include <cstddef>
#include <fstream>
#include <iterator>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/random.hpp>
#include <thread>
// #include "mujoco_ros/control_listener.hpp"
// #include "mujoco_ros/sensordata_publisher.hpp"
// #include "mujoco_ros/Collision.h"
#include <Eigen/src/Core/Matrix.h>
#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/StelaKraft.h"
#include "prx_models/mj_mushr.hpp"
#include "utils/dbg_utils.hpp"
// #include "control/MushrControlPropagation.h"
// #include "motion_planning/replanner_service.hpp"
// #include "motion_planning/planner_client.hpp"
// #include "motion_planning/PlanningResult.h"
// #include "mujoco_ros/Collision.h"
// #include "std_msgs/Empty.h"
#include <utils/std_utils.hpp>

#include <prx_models/defs.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>

#include <utils/rosparams_utils.hpp>

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "TreeStatsCollector" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string params_file;
  std::string file_out;
  std::string plan_file;
  std::string output_file;
  std::string errors_filename;
  prx::simulation_step = 0.1;

  int total_trees, total_iterations;
  // double planning_time;

  // PARAM_SETUP(nh, params_file);
  // PARAM_SETUP(nh, plan_file);
  PARAM_SETUP(nh, output_file);
  PARAM_SETUP(nh, total_iterations);
  PARAM_SETUP(nh, total_trees);

  DEBUG_PRINT;
  ros::ServiceClient _planner_service_client{ nh.serviceClient<prx_models::StelaKraft>("/kraft/replan") };

  prx_models::StelaKraft planner_service_call;
  DEBUG_PRINT;

  planner_service_call.request.use_contingency = true;
  planner_service_call.request.solution_duration = ros::Duration(100);
  planner_service_call.request.condition = prx_models::StelaKraft::Request::CONDITION_ITERATIONS;
  planner_service_call.request.iterations = total_iterations;
  planner_service_call.request.deadline = ros::Time::ZERO;
  planner_service_call.request.root.index = 0;
  planner_service_call.request.root.parent = 0;
  planner_service_call.request.root.parent_edge = 0;
  planner_service_call.request.root.children.clear();
  planner_service_call.request.root.point.point = { 1.0, 0.0, 1.57, 0.0, 0.0, 0.0 };
  planner_service_call.request.root.cost = 0.0;
  DEBUG_PRINT;

  std::ofstream ofs(output_file.c_str());

  ofs << "# planned_duration iteration_count total_nodes ";
  ofs << "cost_current_solution time_current_solution iters_current_solution\n";
  DEBUG_PRINT;

  while (ros::ok() and total_trees > 0)
  {
    DEBUG_PRINT;
    const bool replanner_available{ _planner_service_client.exists() };

    // planner_service_call.request.deadline = ros::Time::now() + ros::Duration(planning_time);
    planner_service_call.request.root.stamp = ros::Time::now();

    // const bool valid_root{ get_node_at(_planner_service_call.request.root, _planner_clock_msg.cycle_end) };

    // const std::size_t root_idx{ _planner_service_call.request.root.index };
    // LOG_MSG("CALLING REPLANNER")
    // LOG_VARS(cycle_start, _planner_service_call.request.deadline, root_idx, _x_curr);

    // const ros::Time start_plan_stamp{ ros::Time::now() };

    // _current_replanning_root = _planner_service_call.request.root.index;

    if (_planner_service_client.call(planner_service_call))
    {
      const bool planner_status{ planner_service_call.response.planner_output ==
                                 prx_models::StelaKraft::Response::TYPE_SUCCESS };
      const std::string response_flag{ planner_status ? "SUCESS" : "FAILURE" };

      const prx_models::PlannerStats& stats{ planner_service_call.response.stats };
      const double planned_duration{ stats.planned_duration };
      const int iteration_count{ stats.iteration_count };
      const int total_nodes{ stats.total_nodes };
      const double cost_current_solution{ stats.cost_current_solution };
      const double time_current_solution{ stats.time_current_solution };
      const int iters_current_solution{ stats.iters_current_solution };

      ofs << planned_duration << " ";
      ofs << iteration_count << " ";
      ofs << total_nodes << " ";
      ofs << cost_current_solution << " ";
      ofs << time_current_solution << " ";
      ofs << iters_current_solution << " ";
      ofs << "\n";
      total_trees--;
    }

    ros::Duration(10.0).sleep();
  }
  ofs.close();

  return 0;
}