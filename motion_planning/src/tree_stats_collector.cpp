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
#include <prx_models/planner_utils.hpp>
#include <prx_models/stela_kraft_utils.hpp>
#include <interface/node_status.hpp>

// template <typename State>
// double distance(const State& x0, const State x1)
// {
//   const State btw{ gtsam::traits<State>::Between(x0, x1) };
//   const auto diff = gtsam::traits<State>::Logmap(btw);

//   return diff.norm();
// }

struct collector_t
{
  ros::Publisher _state_publisher;
  ros::ServiceClient _planner_service_client;

  prx_models::StelaKraft _planner_service_call;
  prx::param_loader _request_params;
  // std::ofstream _ofs;

  double _goal_radius;
  prx::fg::SE2_t _x_curr, _x_goal;

  // int _total_repetitions;

  std::shared_ptr<interface::node_status_t> _node_status;
  std::string _prev_stela_kraft_request_params;

  collector_t(ros::NodeHandle& nh) : _prev_stela_kraft_request_params("")
  {
    prx::simulation_step = 0.1;

    std::string state_topic;

    PARAM_SETUP(nh, state_topic);

    // _ofs.open(output_file.c_str());

    _node_status = interface::node_status_t::create(nh);

    _state_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(state_topic, 1, true);
    _planner_service_client = nh.serviceClient<prx_models::StelaKraft>("/kraft/replan");

    // _ofs << "# " << prx_models::header(prx_models::PlannerStats()) << "\n";

    _node_status->status(interface::NodeStatus::WAITING);

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

    _planner_service_call.request.root = new_tree.nodes[curr_node_idx];
  }

  void run_experiment()
  {
    // bool goal_reached{ false };
    while (true)
    {
      manage_node_status();
      if (_node_status->status() == interface::NodeStatus::RUNNING)
      {
      }
      if (_node_status->status() == interface::NodeStatus::READY)
      {
        continue;
      }
      else if (_node_status->status() == interface::NodeStatus::RESET)
      {
        return;
      }
      else if (_node_status->status() == interface::NodeStatus::WAITING)
      {
        std::string stela_kraft_request_params;

        GLOBAL_PARAM_SETUP_DEFAULT(stela_kraft_request_params, _prev_stela_kraft_request_params);

        if (stela_kraft_request_params != _prev_stela_kraft_request_params)
        {
          DEBUG_VARS(stela_kraft_request_params)
          _request_params.from_string(stela_kraft_request_params);

          DEBUG_VARS(_request_params)
          prx_models::copy(_planner_service_call.request, _request_params);
          _prev_stela_kraft_request_params = stela_kraft_request_params;
        }
        if (_prev_stela_kraft_request_params.size() > 0)
        {
          _node_status->status(interface::NodeStatus::READY);
        }
        continue;
      }
      else if (_node_status->status() == interface::NodeStatus::PAUSED)
      {
        ros::Duration(1.0).sleep();
        continue;
      }
      else if (_node_status->status() == interface::NodeStatus::FINISH)
      {
        return;
      }

      // else if (_node_status->status() == interface::NodeStatus::FINISH)
      // {
      //   ros::shutdown();
      // }

      const bool replanner_available{ _planner_service_client.exists() };
      _planner_service_call.request.root.stamp = ros::Time::now();
      _planner_service_call.request.deadline =
          ros::Time::now() + ros::Duration(_planner_service_call.request.solution_duration);
      // _planner_service_call.request.use_contingency = _use_contingency;

      if (_planner_service_client.call(_planner_service_call))
      {
        const bool planner_status{ _planner_service_call.response.planner_output ==
                                   prx_models::StelaKraft::Response::TYPE_SUCCESS };
        // prx_assert(planner_status, "Planner error!");
        if (planner_status)
        {
          prx_models::tree_msg_wrapper_t wrapped_tree(_planner_service_call.response.sln_tree);

          get_next_state(wrapped_tree);
          // goal_reached = distance(_x_curr, _x_goal) < _goal_radius;
          // prx_models::to_stream(_ofs, _planner_service_call.response.stats);
          // _ofs << "\n";

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
  }

  void manage_node_status()
  {
    if (_node_status->new_request())
    {
      const interface::node_status_t::StatusType current_status{ _node_status->status() };
      const interface::node_status_t::StatusType req_status{ _node_status->requested_status() };
      _node_status->status(req_status);
      _node_status->request_acknowledged();
    }
  }

  // void run()
  // {
  //   while (ros::ok())
  //   {
  //     run_experiment();
  //   }
  //   // _ofs.close();
  // }
};

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "TreeStatsCollector" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string experiments_node_id;
  PARAM_SETUP(nh, experiments_node_id)

  std::shared_ptr<interface::node_status_t> experiments_node_status;
  experiments_node_status = interface::node_status_t::create(nh, experiments_node_id, true);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (experiments_node_status->status() != interface::NodeStatus::FINISH)
  {
    collector_t collector(nh);
    collector.run_experiment();
    ros::Duration(2.0).sleep();
  }

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}