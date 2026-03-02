#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <motion_planning/stela_sliding_window.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>

struct experiment_t
{
  std::string stela_model;
  ros::ServiceServer _experiment_service;
  ros::Timer _timer;

  ros::NodeHandle& _nh;
  std::shared_ptr<interface::node_status_t> _stela_status;

  bool _new_experiment, _stela_is_running, _experiment_set;
  std::string _lib_path;

  experiment_t(ros::NodeHandle& nh) : _nh(nh), _new_experiment(false), _stela_is_running(false), _experiment_set(false)
  {
    _lib_path = prx::lib_path_safe("ML4KP_ROS");

    // DEBUG_VARS(_lib_path);

    // std::string stela_node_id;
    // PARAM_SETUP(nh, stela_node_id);

    _experiment_service = nh.advertiseService("/experiment/stela", &experiment_t::new_experiment, this);
    _stela_status = interface::node_status_t::create(nh, "stela", true);

    _timer = nh.createTimer(ros::Duration(1.0), &experiment_t::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    // DEBUG_VARS(_new_experiment, _stela_is_running)
    if (_new_experiment)
    {
      if (_stela_is_running)
      {
        _stela_status->request_status(interface::NodeStatus::FINISH);
      }
    }
  }

  template <typename MushrModel>
  void run()
  {
    try
    {
      motion_planning::stela_windowed_t<MushrModel> stela;
      stela.onInit(_nh);
      _stela_is_running = true;

      stela.replanner_service_main();
    }
    catch (...)
    {
      PRINT_MSG("Stela exception...\n");
    }
    _stela_is_running = false;
  }

  void run_stela()
  {
    while (ros::ok())
    {
      if (_new_experiment)
      {
        PRINT_MSG("[StelaMushr] Running new experiment");
      }
      _new_experiment = false;

      if (not _experiment_set)
      {
        PRINT_MSG("Experiment not set yet...");
        ros::Duration(10.0).sleep();
        continue;
      }
      // if (_new_experiment)
      // {
      //   ros::Duration(10.).sleep();  // sleep for 10 secs
      // }

      if (stela_model == "mushr")
      {
        run<prx_models::mushr_stela_t>();
      }
      else if (stela_model == "mujoco")
      {
        const std::string param_file{ _lib_path + "src/mujoco_ros/motion_planning/config/mushr.yaml" };
        _nh.setParam("params_file", param_file);

        run<prx_models::mushr_mujoco_stela_t>();
      }
#ifndef TORCH_NOT_BUILT
      else if (stela_model == "mushr_torch")
      {
        const std::string param_file{ _lib_path + "src/mujoco_ros/motion_planning/config/mushr_torch.yaml" };
        _nh.setParam("params_file", param_file);
        run<prx_models::mushr_torch_stela_t>();
      }
#endif
      else
      {
        PRINT_MSG("Invalid mushr type: " + stela_model)
      }
    }
  }

  bool new_experiment(interface::ExperimentParams::Request& request, interface::ExperimentParams::Response& response)
  {
    stela_model = request.stela_model;

    _nh.setParam("validation_plan_feasibility", request.validation_plan_feasibility);
    _nh.setParam("validation_collision_only", request.validation_collision_only);
    _nh.setParam("replanning_condition", request.replanning_condition);
    _nh.setParam("cycle_duration", request.cycle_duration);
    _nh.setParam("replanning_iterations", request.replanning_iterations);
    _nh.setParam("total_replanning_calls", request.total_replanning_calls);
    // _nh.setParam("params_file", request.replanning_iterations);

    _new_experiment = true;
    _experiment_set = true;
    PRINT_MSG("[StelaMushr] Setting new experiment...");
    return true;
  }
};
// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MushrStela" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // std::string mushr_model;

  // PARAM_SETUP(nh, mushr_model);
  experiment_t experiment(nh);

  ros::AsyncSpinner spinner(4);
  prx_assert(spinner.canStart(), "AsyncSpinner cannot start");
  spinner.start();

  experiment.run_stela();

  spinner.stop();
  ros::waitForShutdown();
  // template <typename RobotInterface>

  return 0;
}