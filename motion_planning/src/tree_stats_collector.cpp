#include <cstddef>
#include <fstream>
#include <iterator>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
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
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>

#include <utils/rosparams_utils.hpp>
#include <prx_models/stela_kraft_utils.hpp>

template <typename State>
double distance(const State& x0, const State x1)
{
  const State btw{ gtsam::traits<State>::Between(x0, x1) };
  const auto diff = gtsam::traits<State>::Logmap(btw);

  return diff.norm();
}

struct collector_t
{
  ros::Publisher _state_publisher;
  ros::ServiceClient _planner_service_client;

  prx_models::StelaKraft _planner_service_call;
  prx::param_loader _request_params;
  std::ofstream _ofs;

  double _goal_radius;
  prx::fg::SE2_t _x_curr, _x_goal;

  int _total_repetitions;
  collector_t(ros::NodeHandle& nh)
  {
    std::string stela_kraft_request_params;

    std::string output_file, state_topic;

    int& total_repetitions{ _total_repetitions };
    // PARAM_SETUP(nh, total_iterations);
    PARAM_SETUP(nh, state_topic);
    PARAM_SETUP(nh, total_repetitions);
    PARAM_SETUP(nh, output_file);
    PARAM_SETUP(nh, stela_kraft_request_params);
    _ofs.open(output_file.c_str());

    _state_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(state_topic, 1, true);
    _planner_service_client = nh.serviceClient<prx_models::StelaKraft>("/kraft/replan");
    _request_params.add_file(stela_kraft_request_params);

    _ofs << "# " << prx_models::header(prx_models::PlannerStats()) << "\n";

    // _x_goal[0] =
  }

  void get_next_state(prx_models::tree_msg_wrapper_t& new_tree)
  {
    prx_models::tree_msg_wrapper_t::NodeIdx curr_node_idx{ new_tree.root };
    while (new_tree.nodes[curr_node_idx].children.size() > 0)
    {
      const prx_models::tree_msg_wrapper_t::NodeIdx child_idx{ new_tree.nodes[curr_node_idx].children[0] };
      const prx_models::Node& node{ new_tree.nodes[child_idx] };
      const prx_models::Edge& edge{ new_tree.edges[node.parent_edge] };

      curr_node_idx = child_idx;
    }
    _x_curr[0] = new_tree.nodes[curr_node_idx].point.point[0];
    _x_curr[1] = new_tree.nodes[curr_node_idx].point.point[1];
    _x_curr[2] = new_tree.nodes[curr_node_idx].point.point[2];
  }

  void run_experiment()
  {
    prx_models::copy(_planner_service_call.request, _request_params);

    bool goal_reached{ false };
    while (not goal_reached)
    {
      const bool replanner_available{ _planner_service_client.exists() };
      _planner_service_call.request.root.stamp = ros::Time::now();

      if (_planner_service_client.call(_planner_service_call))
      {
        const bool planner_status{ _planner_service_call.response.planner_output ==
                                   prx_models::StelaKraft::Response::TYPE_SUCCESS };
        prx_assert(planner_status, "Planner error!");

        prx_models::tree_msg_wrapper_t wrapped_tree(_planner_service_call.response.sln_tree);

        get_next_state(wrapped_tree);
        goal_reached = distance(_x_curr, _x_goal) < _goal_radius;
        prx_models::to_stream(_ofs, _planner_service_call.response.stats);
        _ofs << "\n";

        ml4kp_bridge::SpacePointStamped state_msg;
        state_msg.space_point.point.push_back(_x_curr[0]);
        state_msg.space_point.point.push_back(_x_curr[1]);
        state_msg.space_point.point.push_back(_x_curr[2]);
        state_msg.space_point.point.push_back(0.0);
        state_msg.space_point.point.push_back(0.0);
        state_msg.space_point.point.push_back(0.0);
        _state_publisher.publish(state_msg);
      }
    }
  }

  void run()
  {
    while (ros::ok() and _total_repetitions > 0)
    {
    }
    _ofs.close();
  }
};

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

  // planner_service_call.request.use_contingency = true;
  // planner_service_call.request.solution_duration = ros::Duration(100);
  // planner_service_call.request.condition = prx_models::StelaKraft::Request::CONDITION_ITERATIONS;
  // planner_service_call.request.iterations = total_iterations;
  // planner_service_call.request.deadline = ros::Time::ZERO;
  // planner_service_call.request.root.index = 0;
  // planner_service_call.request.root.parent = 0;
  // planner_service_call.request.root.parent_edge = 0;
  // planner_service_call.request.root.children.clear();
  // planner_service_call.request.root.point.point = { 1.0, 0.0, 1.57, 0.0, 0.0, 0.0 };
  // planner_service_call.request.root.cost = 0.0;

  // while (ros::ok() and total_trees > 0)
  // {
  // const bool replanner_available{ _planner_service_client.exists() };

  // planner_service_call.request.root.stamp = ros::Time::now();

  // if (_planner_service_client.call(planner_service_call))
  // {
  // const bool planner_status{ planner_service_call.response.planner_output ==
  //                            prx_models::StelaKraft::Response::TYPE_SUCCESS };
  // const std::string response_flag{ planner_status ? "SUCESS" : "FAILURE" };

  // const prx_models::PlannerStats& stats{ planner_service_call.response.stats };
  // const double planned_duration{ stats.planned_duration };
  // const int iteration_count{ stats.iteration_count };
  // const int total_nodes{ stats.total_nodes };
  // const double cost_current_solution{ stats.cost_current_solution };
  // const double time_current_solution{ stats.time_current_solution };
  // const int iters_current_solution{ stats.iters_current_solution };

  // ofs << planned_duration << " ";
  // ofs << iteration_count << " ";
  // ofs << total_nodes << " ";
  // ofs << cost_current_solution << " ";
  // ofs << time_current_solution << " ";
  // ofs << iters_current_solution << " ";
  // ofs << "\n";
  // total_trees--;
  //   }

  //   ros::Duration(10.0).sleep();
  // }
  // ofs.close();

  return 0;
}