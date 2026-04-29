#include <chrono>
#include <future>
#include <iterator>
#include <memory>

#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <utils/dbg_utils.hpp>

#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/playback/trajectory.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <string>
#include <utils/std_utils.hpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <interface/PlannerClock.h>

#include <motion_planning/utils.hpp>
#include <motion_planning/sdf_factor.hpp>
#include <motion_planning/tree_bridge.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <ml4kp_bridge/StelaTrajectory.h>

#include <prx_models/tree_msg_wrapper.hpp>
#include <prx_models/StelaKraft.h>
#include <interface/StelaStatus.h>

#include <utils/time_profiler.hpp>
#include <vector>
#include <prx_models/Node.h>
#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <interface/NodeStatus.h>
#include <interface/node_status.hpp>

#include <motion_planning/planner_clock.hpp>
#include <motion_planning/sbmp_caller.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename RobotInterface>
class stela_windowed_t
{
  using Derived = stela_windowed_t<RobotInterface>;
  using SF = prx::fg::symbol_factory_t;

  using Control = typename RobotInterface::Control;
  using State = typename RobotInterface::State;

  using Observation = typename RobotInterface::Observation;

  using StateKeys = typename RobotInterface::StateKeys;
  using ControlKeys = typename RobotInterface::ControlKeys;
  using TimeKeys = typename RobotInterface::TimeKeys;

  using StateEstimates = typename RobotInterface::StateEstimates;
  using ControlEstimates = typename RobotInterface::ControlEstimates;

  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename RobotInterface::ConfigFromState,
                                                    prx::fg::collision_info_t::CollisionErrorType::STEP>;

  using Sdf = utils::signed_distance_field_t;
  using SdfPtr = std::shared_ptr<Sdf>;
  using SdfFactor = motion_planning::sdf_factor_t<State, typename RobotInterface::ConfigFromState>;

  using RePlanner = motion_planning::sbmp_caller_t;
  using RePlannerPlan = RePlanner::Plan;
  using RePlannerResult = RePlanner::Result;

  static constexpr Eigen::Index XDim{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index UDim{ gtsam::traits<Control>::dimension };

  // using ControlTranspose = Eigen::RowVector<double, UDim>;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  stela_windowed_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, false, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _isam(_isam_params)
    , _isam2_update_params(gtsam::ISAM2UpdateParams())
  // , _total_future_nodes(10)
  // , _total_past_nodes(10)
  // , _lm_params(prx::fg::default_levenberg_marquardt_parameters())
#ifdef GTSAM_USE_TBB
    , _tbb_control(tbb::global_control::max_allowed_parallelism, 8)
#endif
  {
  }

  virtual void onInit(ros::NodeHandle& nh)
  {
    _node_status = interface::node_status_t::create(nh);
    _clock = std::make_unique<motion_planning::planner_clock_t>(ros::NodeHandle(nh, "clock"));
    _replanner = std::make_shared<motion_planning::sbmp_caller_t>(ros::NodeHandle(nh, "replanner"));

    // PARAM_SETUP(private_nh, estimated_tree_topic);

    // Control parameters
    // PARAM_SETUP(nh, control_topic);
    // PARAM_SETUP(nh, control_frequency);

    // Replanning
    // PARAM_SETUP(private_nh, validation_plan_feasibility)
    // PARAM_SETUP(private_nh, validation_collision_only)

    // Utils
    // PARAM_SETUP_WITH_DEFAULT(private_nh, visualize, visualize)
    // PARAM_SETUP_WITH_DEFAULT(private_nh, report_control_frequency, report_control_frequency)

    // Window
    // PARAM_SETUP(nh, total_future_nodes)
    // PARAM_SETUP(nh, total_past_nodes)

    // _planner_service_call.request.solution_duration = ros::Duration(replanner_solution_duration);

    // _robot = std::make_shared<RobotInterface>(private_nh);

    // LOG_VARS(params_file)
    // if (params_file != "")
    // {
    //   prx::param_loader params{ prx::param_loader(params_file, "") };
    //   // const std::vector<double> param_values{ params["/parameter_space/values"].as<std::vector<double>>() };
    //   // _robot->set_params(param_values);
    //   _robot->init(params);
    //   // _ctrl_lower_bound = params["/control_space/lower_bound"].as<std::vector<double>>();
    //   // _ctrl_upper_bound = params["/control_space/upper_bound"].as<std::vector<double>>();
    // }
    // LOG_MSG("Robot initialized")
    // _robot->print_params();
    // _robot->log_params();
    // if (obstacle_mode == "sdf")
    // {
    // _sdf = Sdf::create(ros::NodeHandle(private_nh, "sdf"));
    // LOG_MSG("SDF initialized")
    // }

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    // const std::string stamped_control_topic{ control_topic + "_stamped" };
    // const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    // const ros::Duration control_timer(1.0 / control_frequency);
    // const ros::Duration estimation_timer(1.0 / estimation_pub_freq);
    // const ros::Duration observation_timer(1.0 / observation_frquency);

    // _control_timer = private_nh.createTimer(control_timer, &Derived::main_timer_callback, this);
    // _estimation_timer = private_nh.createTimer(estimation_timer, &Derived::estimation_timer_callback, this);

    // if (report_control_frequency)
    // {
    //   const ros::Duration control_freq_timer(1.0);
    // _control_frequency_timer = private_nh.createTimer(control_freq_timer, &Derived::check_frequency, this);
    // }

    // _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    // _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1,
    // true);

    // _isam_status_publisher = private_nh.advertise<interface::StelaStatus>("/stela/isam/status", 1, true);
    // _replanning_status_publisher = private_nh.advertise<interface::StelaStatus>("/stela/replanning/status", 1, true);

    LOG_MSG("Publisher / Subscribers initialized")

    // change_status(stela_thread_t::ISAM, interface::StelaStatus::INITIALIZING);

    // _robot_collision_ptr = _robot->collision_geometry();
    // _obstacle_noise = gtsam::noiseModel::Isotropic::Sigma(1, obstacle_sigma);

    // initialize_graph();
    LOG_MSG("Graph initialized")

    _node_status->status(interface::NodeStatus::RUNNING);
  }

  ~stela_windowed_t()
  {
  }

  void handle_node_state()
  {
    if (_node_status->new_request())
    {
      _node_status->status(_node_status->requested_status());
    }
  }

  ml4kp_bridge::SpacePointStamped get_replanner_x0(const ros::Time& deadline)
  {
    ml4kp_bridge::SpacePointStamped root;
    root.header.stamp = _clock->cycle_end();
    root.space_point.point = { 1.0, 0.0, 1.57, 0.0, 0.0, 0.0 };
    return root;
  }

  RePlannerPlan get_retainment_plan()
  {
    return RePlannerPlan();
  }

  void replanning_loop()
  {
    int current_cycle{ -1 };
    // std::variant<ros::Time, int> deadline_or_iterations;

    while (ros::ok())
    {
      handle_node_state();
      if (_node_status->status() != interface::NodeStatus::RUNNING)
        continue;
      if (not _replanner->valid())
      {
        PRINT_MSG("Replanning not available...")
        ros::Duration(1.0).sleep();
        continue;
      }

      if (current_cycle < _clock->cycle())
      {
        current_cycle = _clock->cycle();

        const ros::Time deadline{ _clock->cycle_end() };

        const ml4kp_bridge::SpacePointStamped root_state{ get_replanner_x0(deadline) };
        const RePlannerPlan plan{ get_retainment_plan() };

        const double planning_time{ (deadline - ros::Time::now()).toSec() };
        const auto future_limit = std::chrono::steady_clock::now() + std::chrono::duration<double>(planning_time);
        std::future<RePlannerResult> future_result{ std::async(&RePlanner::call, _replanner,  // no-lint
                                                               planning_time, root_state, plan) };

        std::future_status status{ future_result.wait_until(future_limit) };

        if (status == std::future_status::ready)
        {
          RePlannerResult result{ future_result.get() };
        }
        else
        {
          const ros::Time replanner_failed_time{ ros::Time::now() };
          DEBUG_VARS(replanner_failed_time)
          // PRINT_MSG("Replanner failed!")
        }
      }
    }
  }

private:
  Values _values;

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;
  gtsam::ISAM2UpdateParams _isam2_update_params;

  std::unique_ptr<motion_planning::planner_clock_t> _clock;
  std::shared_ptr<motion_planning::sbmp_caller_t> _replanner;

  std::shared_ptr<interface::node_status_t> _node_status;

  ros::Duration _postprocessing_duration;
#ifdef GTSAM_USE_TBB
  tbb::global_control _tbb_control;
#endif
};
}  // namespace motion_planning
