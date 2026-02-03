#include <chrono>
#include <Eigen/src/Core/Matrix.h>
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

#include <iterator>
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
#include "prx_models/Node.h"
#include "utils/dbg_utils.hpp"
#include "utils/rosparams_utils.hpp"

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename RobotInterface>
class stela_windowed_t
{
  enum stela_state_t
  {
    INITIALIZING = 0,  // Starting and setting up
    IDLE,              // Adding idle state (i.e. stay in place)
    TREE_RECEIVED,     // Got a tree but the start is in the future
    TREE_EXECUTING,    // Following the tree
    FINISHING          // Closing files and the like
  };

  enum stela_thread_t
  {
    REPLANNING = 0,
    ISAM
  };

  using Derived = stela_windowed_t<RobotInterface>;

  using SF = prx::fg::symbol_factory_t;

  using Control = typename RobotInterface::Control;
  using State = typename RobotInterface::State;

  // using StateDot = typename RobotInterface::StateDot;
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

  static constexpr Eigen::Index XDim{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index UDim{ gtsam::traits<Control>::dimension };

  using ControlTranspose = Eigen::RowVector<double, UDim>;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  stela_windowed_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, true, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _tf_listener(_tf_buffer)
    , _isam(_isam_params)
    , _isam2_update_params(gtsam::ISAM2UpdateParams())
    , _tree_valid(false)
    , _last_local_goal(true)
    , _experiment_id("test")
    , _files_created(false)
    , _name("STELA_SW")
    , _total_future_nodes(10)
    , _total_past_nodes(10)
    , _goal_id(std::numeric_limits<std::size_t>::max())
    , _goal_reached(false)
    , _visualize(false)
    , _using_stepper(false)
    // , _trees_received(0)
    , _max_observation_delay(1.0)
    , _control_frequency(30)
    , _total_z_calls(0)
    , _profiler()
    , _time_as_variable(true)
    // , _next_node_index(0)
    , _state(stela_state_t::INITIALIZING)
    , _call_replanner(false)
    , _fg_initialized(false)
    , _new_tree_available(false)
    , _lm_params(prx::fg::default_levenberg_marquardt_parameters())
#ifdef GTSAM_USE_TBB
    , _tbb_control(tbb::global_control::max_allowed_parallelism, 8)
#endif
  {
  }

  virtual void onInit(ros::NodeHandle& private_nh)
  {
    _nh = private_nh;
    // ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    PRINT_MSG("Starting Stela Windowed");

    std::string input_tree_topic_name;
    std::string control_topic;
    std::string collision_topic;
    std::string environment;
    std::string estimated_tree_topic;
    double obstacle_sigma{ 1.0 };
    double observation_frquency{ 30 };
    double& control_frequency{ _control_frequency };
    double& obstacle_distance_tolerance{ _obstacle_distance_tolerance };
    double& obstacle_factor_include_distance{ _obstacle_factor_include_distance };

    std::string sdf_params;
    std::string estimated_trajectory_topic;
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& output_dir{ _output_dir };
    std::string& obstacle_mode{ _obstacle_mode };
    std::string& experiment_id{ _experiment_id };

    std::string params_file{ "" };
    // std::vector<double> plant_parameters{};

    bool report_control_frequency{ true };
    bool& visualize{ _visualize };
    bool& using_stepper{ _using_stepper };
    bool& time_as_variable{ _time_as_variable };
    int& total_future_nodes{ _total_future_nodes };
    int& total_past_nodes{ _total_past_nodes };
    int estimation_pub_freq{ 30 };
    // ROS_PARAM_SETUP(private_nh, random_seed);
    // ROS_PARAM_SETUP(private_nh, plant_config_file);
    // ROS_PARAM_SETUP(private_nh, planner_config_file);
    double cycle_duration{ 1.0 };
    double replanner_solution_duration{ 1.0 };
    std::string planner_clock_topic;

    bool& use_contingency{ _use_contingency };
    int& total_replanning_calls{ _replanning_calls };

    double start_delay;
    _lm_params.setVerbosityLM("SILENT");
    _lm_params.setMaxIterations(1);

    PARAM_SETUP(private_nh, start_delay);
    PARAM_SETUP(private_nh, total_replanning_calls);
    PARAM_SETUP(private_nh, replanner_solution_duration);
    PARAM_SETUP(private_nh, cycle_duration);
    PARAM_SETUP(private_nh, use_contingency);
    PARAM_SETUP(private_nh, estimated_tree_topic);
    PARAM_SETUP(private_nh, input_tree_topic_name);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, control_frequency);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, output_dir)
    PARAM_SETUP(private_nh, collision_topic)
    PARAM_SETUP(private_nh, environment)
    PARAM_SETUP(private_nh, obstacle_mode)
    PARAM_SETUP(private_nh, obstacle_distance_tolerance)
    PARAM_SETUP(private_nh, obstacle_factor_include_distance)
    PARAM_SETUP(private_nh, estimated_trajectory_topic)
    PARAM_SETUP(private_nh, planner_clock_topic);
    PARAM_SETUP_WITH_DEFAULT(private_nh, visualize, visualize)
    PARAM_SETUP_WITH_DEFAULT(private_nh, time_as_variable, time_as_variable)
    PARAM_SETUP_WITH_DEFAULT(private_nh, obstacle_sigma, obstacle_sigma)
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id)
    PARAM_SETUP_WITH_DEFAULT(private_nh, params_file, params_file)
    PARAM_SETUP_WITH_DEFAULT(private_nh, report_control_frequency, report_control_frequency)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_future_nodes, total_future_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_past_nodes, total_past_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)
    PARAM_SETUP_WITH_DEFAULT(private_nh, using_stepper, using_stepper)
    PARAM_SETUP_WITH_DEFAULT(private_nh, estimation_pub_freq, estimation_pub_freq);
    PARAM_SETUP_WITH_DEFAULT(private_nh, observation_frquency, observation_frquency);

    _planner_service_call.request.solution_duration = ros::Duration(replanner_solution_duration);

    _robot = std::make_shared<RobotInterface>(private_nh);

    if (params_file != "")
    {
      prx::param_loader params{ prx::param_loader(params_file, "") };
      // const std::vector<double> param_values{ params["/parameter_space/values"].as<std::vector<double>>() };
      // _robot->set_params(param_values);
      _robot->init(params);
      // _ctrl_lower_bound = params["/control_space/lower_bound"].as<std::vector<double>>();
      // _ctrl_upper_bound = params["/control_space/upper_bound"].as<std::vector<double>>();
    }
    _robot->print_params();
    // _robot->log_params();
    if (obstacle_mode == "sdf")
    {
      if (sdf_params == "")
        prx_throw("No SDF params!");
      prx::param_loader sdf_param_loader{};
      sdf_param_loader = Sdf::default_parameters();
      sdf_param_loader.add_file(sdf_params);
      sdf_param_loader["environment"].set(environment);
      // ml4kp_bridge::check_for_ros_params(sdf_param_loader, private_nh);
      _sdf = Sdf::create(sdf_param_loader);
    }

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };
    const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    const ros::Duration control_timer(1.0 / control_frequency);
    const ros::Duration estimation_timer(1.0 / estimation_pub_freq);
    const ros::Duration observation_timer(1.0 / observation_frquency);

    _control_timer = private_nh.createTimer(control_timer, &Derived::main_timer_callback, this);
    // _estimation_timer = private_nh.createTimer(estimation_timer, &Derived::estimation_timer_callback, this);

    // How much time can it pass between observations before declaring failure
    // _observations_freq_timer = private_nh.createTimer(observation_timer, &Derived::observation_timer_callback, this);

    if (report_control_frequency)
    {
      const ros::Duration control_freq_timer(1.0);
      _control_frequency_timer = private_nh.createTimer(control_freq_timer, &Derived::check_frequency, this);
    }

    _tree_subscriber = private_nh.subscribe(input_tree_topic_name, 1, &Derived::tree_callback, this);
    _collision_subscriber = private_nh.subscribe(collision_topic, 1, &Derived::collision_callback, this);
    // _planner_clock_subscriber = nh.subscribe(planner_clock_topic, 1, &replanner_t::clock_callback, this);

    _finish_publisher = private_nh.advertise<std_msgs::Bool>(finish_topic, 1, true);
    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);

    _viz_obstacles_publisher = private_nh.advertise<visualization_msgs::Marker>(obstacle_viz_topic, 1, false);

    _planner_clock_publisher = private_nh.advertise<interface::PlannerClock>(planner_clock_topic, 1);
    _estimated_tree_publisher = private_nh.advertise<prx_models::Tree>(estimated_tree_topic, 1, true);
    _estimated_traj_publisher =
        private_nh.advertise<ml4kp_bridge::StelaTrajectory>(estimated_trajectory_topic, 1, true);

    _isam_status_publisher = private_nh.advertise<interface::StelaStatus>("/stela/isam/status", 1, true);
    _replanning_status_publisher = private_nh.advertise<interface::StelaStatus>("/stela/replanning/status", 1, true);

    change_status(stela_thread_t::ISAM, interface::StelaStatus::INITIALIZING);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "StelaControl";

    _prev_header.stamp = ros::Time::now();
    _next_node_time = ros::Time::now();

    auto obstacles = prx::load_obstacles(environment);
    // _obstacle_list = obstacles.second;
    _obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

    _robot_collision_ptr = _robot->collision_geometry();
    _obstacle_noise = gtsam::noiseModel::Isotropic::Sigma(1, obstacle_sigma);

    _obstacles_marker.header.frame_id = "world";
    _obstacles_marker.header.stamp = ros::Time();
    _obstacles_marker.ns = "nodes";
    _obstacles_marker.id = 0;
    _obstacles_marker.type = visualization_msgs::Marker::LINE_LIST;
    _obstacles_marker.action = visualization_msgs::Marker::ADD;
    _obstacles_marker.pose.position.x = 0;
    _obstacles_marker.pose.position.y = 0;
    _obstacles_marker.pose.position.z = 0;
    _obstacles_marker.pose.orientation.x = 0.0;
    _obstacles_marker.pose.orientation.y = 0.0;
    _obstacles_marker.pose.orientation.z = 0.0;
    _obstacles_marker.pose.orientation.w = 1.0;
    _obstacles_marker.scale.x = 0.05;
    _obstacles_marker.scale.y = 0.05;
    _obstacles_marker.scale.z = 0.05;

    _obstacles_marker.color.a = 0.8;  // Don't forget to set the alpha!
    _obstacles_marker.color.r = 0.98;
    _obstacles_marker.color.g = 0.55;
    _obstacles_marker.color.b = 0.02;

    _x0_start_time = ros::Time::ZERO;
    _tf.header.stamp = ros::Time::ZERO;
    _dt01 = 0.0;

    _timestamp = utils::timestamp();
    const std::string path{ _output_dir + "/" + _name };
    const std::string filename{ path + "_" + _experiment_id + "_" + _timestamp + ".txt" };
    _ofs.open(filename);
    _ofs << "# id key_x x[...] xCov[...] key_xdot xdot[...] xdotCov[...]\n";

    _profiler.set_filename(path + "_freq_" + _experiment_id + "_" + _timestamp + ".txt");

    initialize_graph();

    _planner_clock_msg.cycle_duration = ros::Duration(cycle_duration);
    _planner_clock_msg.header.stamp = ros::Time::now();
    _planner_clock_msg.cycle_start = ros::Time::now() + ros::Duration(start_delay);
    _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_start + _planner_clock_msg.cycle_duration;
    // DEBUG_VARS(_planner_clock_msg)
    const ros::Duration timer_duration(0.01);
    // const ros::Duration timer_duration(0.01);
    _clock_timer = private_nh.createTimer(timer_duration, &Derived::clock_timer_callback, this);
    // _replan_timer = private_nh.createTimer(timer_duration, &Derived::replan_timer_callback, this);

    // update_estimated_tree();
    change_status(stela_thread_t::ISAM, interface::StelaStatus::IDLE);

    PRINT_MSG("Stela Ready")
  }

  ~stela_windowed_t()
  {
    to_file();
  }

  bool check_new_tree(prx_models::tree_msg_wrapper_t& new_tree) const
  {
    // LOG_MSG("CHECKING NEW TREE")
    const ros::WallTime start{ ros::WallTime::now() };

    gtsam::Values proposed_values;
    gtsam::NonlinearFactorGraph proposed_graph;

    prx_models::tree_msg_wrapper_t::NodeIdx curr_node_idx{ new_tree.root };

    // StateEstimates estimates;
    // std::vector<Eigen::MatrixXd> covariances;
    // const StateKeys state_keys{ _robot->keyState(1, curr_node_idx) };

    const ros::WallTime estimation_start_stamp{ ros::WallTime::now() };
    // _fg_mutex.lock();

    // update_estimates<0>(estimates, _isam, state_keys);
    // compute_covariances<0>(covariances, _isam, state_keys);

    // _fg_mutex.unlock();
    const ros::WallTime estimation_stamp{ ros::WallTime::now() };

    GraphValues graph_values_0{ _robot->estimate_to_prior(curr_node_idx, _replanning_root_estimates,
                                                          _replanning_root_covariances) };

    proposed_graph.push_back(graph_values_0.first);
    proposed_values.insert(graph_values_0.second);

    while (new_tree.nodes[curr_node_idx].children.size() > 0)
    {
      const prx_models::tree_msg_wrapper_t::NodeIdx child_idx{ new_tree.nodes[curr_node_idx].children[0] };
      const prx_models::Node& node{ new_tree.nodes[child_idx] };
      const prx_models::Edge& edge{ new_tree.edges[node.parent_edge] };

      GraphValues graph_values{ _robot->node_edge_to_fg(node, edge) };
      obstacle_factors(graph_values.first, node.point, edge.target);

      proposed_graph.push_back(graph_values.first);
      proposed_values.insert(graph_values.second);

      curr_node_idx = child_idx;
    }
    const ros::WallTime fg_built_stamp{ ros::WallTime::now() };

    try
    {
      gtsam::LevenbergMarquardtOptimizer optimizer(proposed_graph, proposed_values, _lm_params);
      const gtsam::Values result{ optimizer.optimize() };
      const double initial_error{ proposed_graph.error(proposed_values) };
      const double validation_error{ optimizer.error() };

      const bool accept_tree{ validation_error < 1.0 };
      // log_graph("Incoming graph", proposed_graph, result);
      // LOG_VARS(proposed_error, new_graph_error, accept_tree)
      // DEBUG_VARS(proposed_error, new_graph_error, accept_tree)

      const ros::WallTime optimization_stamp{ ros::WallTime::now() };

      const std::size_t total_iterations{ optimizer.iterations() };
      const std::size_t total_factors{ proposed_graph.size() };
      const std::size_t total_variables{ proposed_values.size() };
      const double estimation_dt{ (estimation_stamp - estimation_start_stamp).toSec() };
      const double fg_built_dt{ (fg_built_stamp - estimation_stamp).toSec() };
      const double optim_dt{ (optimization_stamp - fg_built_stamp).toSec() };
      LOG_VARS(estimation_dt, fg_built_dt, optim_dt, total_iterations, total_factors, total_variables, validation_error,
               initial_error);
      // _new_tree_available = new_graph_error < 1.0;
      // _tree_valid = new_graph_error < 1.0;
      return accept_tree;
    }
    catch (gtsam::ValuesKeyDoesNotExist e)
    {
      // DEBUG_PRINT
      PRINT_MSG("check_new_tree");
      PRINT_MSG("[check_new_tree] Problem with test factor graph");
      PRINT_KEYS(e.key())
      DEBUG_VARS(e.what())

      // failure_to_file(e.what());
      throw e;
    }
    // DEBUG_PRINT
    // LOG_MSG("--- check_new_tree FINISHED ---")
    return false;
  }

  void replanner_service_main()
  {
    change_status(stela_thread_t::REPLANNING, interface::StelaStatus::IDLE);
    while (ros::ok() and _replanning_calls > 0)
    {
      // _planner_clock_msg.header.stamp = ;
      // const bool call_replanner{ ros::Time::now() > _planner_clock_msg.cycle_end };
      // const bool call_replanner{ ros::Time::now() > _end_of_next_cycle };
      // DEBUG_VARS(replanner_available)
      // if (not replanner_available)
      if (_call_replanner)
      {
        _call_replanner = false;
        const bool replanner_available{ _planner_service_client.exists() };
        if (not replanner_available)
        {
          change_status(stela_thread_t::REPLANNING, interface::StelaStatus::INITIALIZING);
          _planner_service_client = _nh.serviceClient<prx_models::StelaKraft>("/kraft/replan");
          const bool replanner_init{ _planner_service_client.exists() };
          LOG_VARS(replanner_available, replanner_init)
          if (not replanner_init)
          {
            change_status(stela_thread_t::REPLANNING, interface::StelaStatus::ERROR);
            continue;
          }
        }

        // DEBUG_VARS(ros::Time::now())
        // LOG_VARS("Replanning!");
        change_status(stela_thread_t::REPLANNING, interface::StelaStatus::REPLANNING);
        // DEBUG_VARS(_x_curr, _x_next);
        _planner_service_call.request.deadline = _planner_clock_msg.cycle_end;
        _planner_service_call.request.use_contingency = _use_contingency;
        _planner_service_call.request.root.stamp = _planner_clock_msg.cycle_end;
        const bool valid_root{ get_node_at(_planner_service_call.request.root, _planner_clock_msg.cycle_end) };
        // DEBUG_VARS(valid_root)
        LOG_MSG("CALLING REPLANNER")
        const ros::Time start_plan_stamp{ ros::Time::now() };

        if (not valid_root)
        {
          prx_warn("Invalid root when calling the planner");
          change_status(stela_thread_t::REPLANNING, interface::StelaStatus::ERROR);
          continue;
        }
        _current_replanning_root = _planner_service_call.request.root.index;

        if (_planner_service_client.call(_planner_service_call))
        {
          if (_planner_service_call.response.planner_output == prx_models::StelaKraft::Response::TYPE_SUCCESS)
          {
            change_status(stela_thread_t::REPLANNING, interface::StelaStatus::VALIDATING);
            LOG_MSG("REPLANNER ANSWERED")
            // const ros::Time plan_received_stamp{ ros::Time::now() };

            // DEBUG_VARS(_planner_service_call.response.planner_output);
            // const std::size_t root_idx{ _planner_service_call.response.sln_tree.root };
            // const prx_models::Node sln_root{ motion_planning::get_root(_planner_service_call.response.sln_tree) };
            // _new_tree_available = false;
            const std::size_t root_idx{ _planner_service_call.response.sln_tree.root };
            // const prx_models::Node& new_root{ _planner_service_call.response.sln_tree.nodes[root_idx] };
            // DEBUG_VARS(_x_curr, _x_next, new_root.index)
            // DEBUG_VARS(root_idx, _x_next, root_idx >= _x_next)
            // if (root_idx >= _x_next)
            // {
            prx_models::tree_msg_wrapper_t wrapped_tree(_planner_service_call.response.sln_tree);
            _new_tree_available = check_new_tree(wrapped_tree);
            // LOG_VARS(_new_tree_available)
            const ros::Time plan_validated_stamp{ ros::Time::now() };
            // const double plan_received_dt{ (plan_received_stamp - start_plan_stamp).toSec() };
            // const double plan_validated_dt{ (plan_validated_stamp - start_plan_stamp).toSec() };
            // const double validation_dt{ plan_validated_dt - plan_received_dt };
            // LOG_VARS(plan_received_dt, plan_validated_dt, validation_dt);
            if (_new_tree_available)
            {
              _new_tree = wrapped_tree;

              // LOG_MSG("Tree checked and accepted")
              // LOG_VARS(_new_tree_available)
              // LOG_VARS(_new_tree)
              // _new_tree = prx_models::tree_msg_wrapper_t(_planner_service_call.response.sln_tree);
            }
            change_status(stela_thread_t::REPLANNING, interface::StelaStatus::IDLE);

            // }
            // if (_new_tree_available)
            // {
            //   _new_tree = _planner_service_call.response.sln_tree;
            // }
            // DEBUG_VARS(_planner_service_call.request.deadline, ros::Time::now());
            // DEBUG_VARS(_x_curr, _x_next);
            // DEBUG_VARS(sln_root);
          }
        }
        else
        {
          change_status(stela_thread_t::REPLANNING, interface::StelaStatus::ERROR);
        }
        _replanning_calls--;
      }
      // else
      // {
      //   change_status(stela_thread_t::REPLANNING, interface::StelaStatus::IDLE);
      //   //   if (not replanner_available)
      //   //   {
      //   //     DEBUG_VARS(replanner_available)
      //   //   }
      // }
    }
    PRINT_MSG("Maxed out on replanning cycles!");
    // return;
  }

  void clock_timer_callback(const ros::TimerEvent& event)
  {
    // PRINT_MSG("clock_timer_callback")
    _planner_clock_msg.state = interface::PlannerClock::LOW;
    if (_planner_clock_msg.cycle_start > ros::Time::now())
    {
      return;
    }

    _planner_clock_msg.header.stamp = ros::Time::now();
    if (_planner_clock_msg.header.stamp > _planner_clock_msg.cycle_end)
    {
      _planner_clock_msg.state = interface::PlannerClock::HIGH;
      _planner_clock_msg.cycle++;
      _planner_clock_msg.cycle_start = _planner_clock_msg.cycle_end;
      _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_end + _planner_clock_msg.cycle_duration;
      // _planner_clock_msg.cycle_duration = _cycle_duration;
      _call_replanner = true;
    }
    _planner_clock_publisher.publish(_planner_clock_msg);
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    if (msg->data)
    {
      to_file(true);
    }
  }

  // void clock_callback(const interface::PlannerClockConstPtr msg)
  // {
  // if (_current_cycle != msg->cycle)
  // {
  //   _current_cycle = msg->cycle;
  //   _cycle_end = msg->cycle_end;
  //   _cycle_start = msg->cycle_start;
  //   _cycle_duration = msg->cycle_duration;
  // }
  // }

  void to_file(const bool collision = false, const bool rasied_exception = false, const bool network_problem = false)
  {
    if (_files_created)
      return;
    _collision_subscriber.shutdown();
    std_msgs::Bool msg;
    msg.data = true;
    _finish_publisher.publish(msg);

    const std::string filename_branch_gt{ _output_dir + "/" + _name + "_" + "branch_gt_" + _experiment_id + "_" +
                                          _timestamp + ".txt" };
    const std::string filename_data{ _output_dir + "/" + _name + "_" + "data_" + _experiment_id + "_" + _timestamp +
                                     ".txt" };

    std::ofstream ofs_branch(filename_branch_gt);
    std::ofstream ofs_data(filename_data);

    const double elapsed_time{ (ros::Time::now() - _start_time).toSec() };
    const double avg_freq{ _total_calls / elapsed_time };
    const double avg_obervation_freq{ static_cast<double>(_total_z_calls) / elapsed_time };
    const std::string network_res{ network_problem ? "true" : "false" };
    // const auto dt_real = _dt_real.toSec();
    // const auto dt_expected = _dt_expected.toSec();

    // ofs_data << "Initialized: " << (_tree_received ? "true" : "false") << "\n";
    ofs_data << "ElapsedTime: " << elapsed_time << "\n";
    ofs_data << "Collision: " << (collision ? "true" : "false") << "\n";
    ofs_data << "ObstacleDistanceTolerance: " << _obstacle_distance_tolerance << "\n";
    ofs_data << "ObstacleMode: " << _obstacle_mode << "\n";
    ofs_data << "ExceptionRaised: " << (rasied_exception ? "true" : "false") << "\n";
    ofs_data << "NetworkProblem: " << network_res << "\n";
    ofs_data << "AverageFrequency: " << avg_freq << "\n";
    ofs_data << "ObservationFrequency: " << avg_obervation_freq << "\n";
    ofs_data << "TotalFutureNodes: " << _total_future_nodes << "\n";
    ofs_data << "TotalFastNodes: " << _total_past_nodes << "\n";

    // DEBUG_VARS(avg_freq);
    ofs_data.close();

    // PRINT_MSG("[TODO] Data files for STELA_SW not implemented.");
    // gtsam::Values estimate{ _isam.calculateEstimate() };
    ofs_branch << "# id point[...]\n";
    for (auto node_id : _selected_nodes)
    {
      // ofs << node_id << " ";
      // const StateKeys keys{ RobotInterface::keyState(1, node_id) };
      // estimates_to_file<StateEstimates, 0>(ofs, _values, keys, _isam, false);
      // ofs_covariance
      ofs_branch << node_id << " ";
      ml4kp_bridge::to_file(_tree.nodes[node_id].point, ofs_branch);
      ofs_branch << "\n";
    }

    // DEBUG_PRINT
    while (not _past_factors_queue.empty())
    {
      const std::size_t id{ _past_factors_queue.front() };

      _past_factors_queue.pop_front();
      _active_nodes.erase(id);
      node_info_to_file(id);
    }

    _ofs.close();
    ofs_branch.close();

    _files_created = true;

    // _tree_received = false;

    PRX_DBG_VARS(collision);

    ros::Rate rate(1);
    rate.sleep();
    ros::shutdown();
  }

  void failure_to_file(const std::string msg)
  {
    const std::string filename{ _output_dir + "/" + _name + "_fail_" + _experiment_id + "_" + _timestamp + ".txt" };

    std::ofstream ofs(filename);
    ofs << "STELA failure: " << msg << "\n";
    ofs.close();
    to_file(false, true);
  }

  void print_error(const std::string msg)
  {
    const gtsam::Values values{ _isam.calculateBestEstimate() };
    const double current_error{ _isam.getFactorsUnsafe().error(values) };
    DEBUG_VARS(msg, current_error);
  }

  void check_frequency(const ros::TimerEvent& event)
  {
    // if (_tree_received)
    // {
    // DEBUG_PRINT
    // _isam.calculateBestEstimate()
    // const double current_error{ _isam.error(_isam.getDelta()) };
    // const gtsam::Values values{ _isam.calculateBestEstimate() };
    // const double current_error{ _isam.getFactorsUnsafe().error(values) };

    const double dt{ (event.current_real - event.last_real).toSec() };
    const double stela_frequency{ _freq_counter / dt };
    const double& target_frequency{ _control_frequency };
    // DEBUG_VARS(stela_frequency, target_frequency);
    // const std::string frq{ "stela_frequency" };
    // DEBUG_VARS(stela_frequency, current_error);
    _freq_counter = 0;

    // const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
    //     printCondition = [&](const gtsam::Factor* f, double err, size_t) { return f != nullptr and err > 0.1; };
    // _isam.getFactorsUnsafe().printErrors(values, "Problem graph", SF::formatter, printCondition);

    // if (current_error > 1.0)
    // {
    //   to_file(false, true);
    // }
    // }
  }

  // void observation_timer_callback(const ros::TimerEvent& event)
  // {
  // if (_tree_received)
  // DEBUG_PRINT
  // add_observations();
  // _total_z_calls++;
  // DEBUG_PRINT
  // const ros::Duration dt{ ros::Time::now() - _tf.header.stamp };
  // if (dt > _max_observation_delay)
  // {
  //   to_file(false, false, true);
  // }
  // }
  // }

  // void estimation_timer_callback(const ros::TimerEvent& event)
  // {
  // DEBUG_VARS(_tree_received, _observations_added, _idle_initialized)
  // if (_tree_received)
  // {
  // DEBUG_PRINT
  // DEBUG_PRINT

  // update_estimated_tree();

  // // if (_visualize)
  // // {
  // // _viz_obstacles_publisher.publish(_obstacles_marker);
  // // }
  // _estimated_tree_publisher.publish(_estimated_tree.to_msg());
  // _estimated_traj_publisher.publish(_estimated_trajectory);
  // }
  // else if (_observations_added and _idle_initialized)
  // {
  // DEBUG_PRINT
  // update_estimated_tree();
  // _estimated_tree_publisher.publish(_estimated_tree);
  // _estimated_traj_publisher.publish(_estimated_trajectory);
  // PRINT_MSG("[Stela] estimated tree sent ");
  // }
  // }

  void remove_future_factors(const std::size_t idx)
  {
    std::size_t future_id{ idx };
    std::vector<gtsam::Key> all_keys;
    // for (auto future_id : _x_queue)
    // _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter);

    gtsam::FactorIndices all_indices;
    while (future_id <= _x_queue.back())
    {
      // LOG_VARS(future_id)
      const gtsam::FactorIndices& indices{ _inserted_factors[future_id] };
      all_indices.insert(all_indices.end(), indices.begin(), indices.end());

      const TimeKeys time01_keys{ _robot->keyTime(future_id, future_id + 1) };
      const ControlKeys ctrl01_keys{ _robot->keyControl(future_id, future_id + 1) };
      const StateKeys state1_keys{ _robot->keyState(1, future_id + 1) };
      all_keys.insert(all_keys.end(), time01_keys.begin(), time01_keys.end());
      all_keys.insert(all_keys.end(), ctrl01_keys.begin(), ctrl01_keys.end());
      all_keys.insert(all_keys.end(), state1_keys.begin(), state1_keys.end());

      future_id++;
    }

    // DEBUG_VARS(_x_curr, _x_next, idx, future_id)
    // PRINT_KEYS_CONTAINER(all_keys);
    for (auto& fidx : all_indices)
    {
      if (_isam.getFactorsUnsafe()[fidx] == nullptr)
        continue;
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[fidx]->keys() };
      // PRINT_KEYS_CONTAINER(factor_keys);
      for (const gtsam::Key& key : factor_keys)
      {
        auto k0_res = std::find(all_keys.begin(), all_keys.end(), key);
        if (k0_res != all_keys.end())
        {
          // PRINT_KEY(key);
          _isam2_update_params.removeFactorIndices.push_back(fidx);

          break;
        }
      }
    }

    while (idx != _x_queue.back())
    {
      // LOG_VARS(idx, _x_queue.back())
      _estimated_tree.erase_node(_x_queue.back());
      _x_queue.pop_back();
      // _current_future_nodes--;
    }
    _current_future_nodes = _x_queue.size();
    // _estimated_tree.nodes[_x_queue.back()].
    // _estimated_tree.erase_edge();
    // LOG_VARS(_estimated_tree)
    // _x_queue.push_back(_x_queue.back() + 1);
    // LOG_VARS(_x_curr, _x_next, idx, _x_queue.back())
    // LOG_VARS(_x_queue)
    // _x_queue.pop_back();

    remove_factors();

    // LOG_VARS(_x_curr, _x_next);
    // const std::function<bool(const gtsam::Factor*, double, size_t)> printCondition =
    // [](const gtsam::Factor* factor, double, size_t) { return factor != nullptr; };
    // _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter, printCondition);
    // _isam.getFactorsUnsafe().printErrors(_values, "Graph after removal", SF::formatter, printCondition);
  }

  void add_new_tree()
  {
    // LOG_VARS(_current_future_nodes, _total_future_nodes);
    _tree.copy(_new_tree);

    // const prx_models::Node& new_node{ _new_tree.nodes[_new_tree.root] };
    const prx_models::Node& node{ _tree.nodes[_new_tree.root] };

    // LOG_VARS(new_node);
    // LOG_VARS(node);
    // LOG_VARS(node)
    // LOG_MSG("add_new_tree")
    prx_assert(node.children.size() > 0, "[Stela::add_new_tree] Node has no children!");
    const std::size_t old_next_tree_edge{ _next_tree_edge };

    // DEBUG_VARS(_new_tree.root, _next_tree_edge)
    // DEBUG_VARS(_tree)
    // _next_tree_edge = _tree.nodes[_new_tree.root].parent_edge;
    _next_tree_edge = _tree.nodes[node.children[0]].parent_edge;
    // DEBUG_PRINT
    // _next_tree_edge--;
    // _next_tree_edge = _new_tree.root;
    // LOG_VARS(_new_tree.root, old_next_tree_edge, _next_tree_edge)

    for (; _current_future_nodes < _total_future_nodes; ++_current_future_nodes)
    {
      // const prx_models::Edge& edge{ _tree.edges[_next_tree_edge] };
      // if (_tree.nodes[edge.target].children.size() > 0)
      // if (_tree_valid)
      // {
      // LOG_VARS(edge.target)
      add_tree_node();
      // LOG_VARS(_estimated_tree.nodes.size())
      // }
      // else
      // {
      //   LOG_MSG("tree node has no children?");
      //   LOG_VARS(edge.target, _next_tree_edge)
      //   LOG_VARS(_new_tree);
      //   LOG_VARS(_tree);
      // }
    }

    _state = stela_state_t::TREE_EXECUTING;

    // const std::function<bool(const gtsam::Factor*, double, size_t)> printCondition =
    //     [](const gtsam::Factor* factor, double, size_t) { return factor != nullptr; };
    // _isam.getFactorsUnsafe().printErrors(_values, "Graph after removal", SF::formatter, printCondition);

    // const prx_models::Edge& edge{ _tree.edges[_next_tree_edge] };
  }

  void update_from_replan_tree()
  {
    // DEBUG_VARS(_new_tree_available)
    if (_new_tree_available)
    {
      _new_tree_available = false;
      // LOG_MSG("Accepting tree and updating")
      // DEBUG_VARS(_current_future_nodes, _total_future_nodes);
      const prx_models::Node& new_root{ _new_tree.nodes[_new_tree.root] };
      LOG_VARS(_x_curr, new_root.index, _x_next)
      if (new_root.index >= _x_next)
      {
        change_status(stela_thread_t::ISAM, interface::StelaStatus::ADDING_TREE);
        LOG_MSG("Adding new tree!")
        // LOG_VARS(_x_next, new_root.index)
        remove_future_factors(new_root.index);

        add_new_tree();
        _tree_valid = true;
        // LOG_MSG("New tree set as valid")
        // DEBUG_VARS(_current_future_nodes, _total_future_nodes);
        // DEBUG_VARS(_x_queue);
        // DEBUG_VARS(_estimated_tree)
        change_status(stela_thread_t::ISAM, interface::StelaStatus::IDLE);
      }
      else
      {
        PRINT_MSG("Can't attach new tree");
        DEBUG_VARS(_x_curr, _x_next, new_root.index)
      }
      // const ros::Duration eps_duration(0.1);
      // if (_new_root.stamp > ros::Time::now() + eps_duration)
      // {
      // }
    }
  }

  void print_stela_state()
  {
    if (_state == stela_state_t::IDLE)
    {
      PRINT_MSG("[Stela::State] IDLE");
    }
    else if (_state == stela_state_t::TREE_EXECUTING)
    {
      PRINT_MSG("[Stela::State] TREE_EXECUTING");
    }
  }

  void change_status(const stela_thread_t& thread_id, const int16_t new_status)
  {
    _status.header.stamp = ros::Time::now();
    _status.state = new_status;
    if (thread_id == stela_thread_t::REPLANNING)
    {
      _replanning_status_publisher.publish(_status);
    }
    else if (thread_id == stela_thread_t::ISAM)
    {
      _isam_status_publisher.publish(_status);
    }
  }

  void main_timer_callback(const ros::TimerEvent& event)
  {
    // if (_tree_received)
    // {
    _profiler.start();

    // print_stela_state();

    _fg_mutex.lock();

    update_from_replan_tree();

    add_observations();

    update_next_goal();

    update_estimated_tree();

    // if (_visualize)
    // {
    // _viz_obstacles_publisher.publish(_obstacles_marker);
    // }
    // _profiler.checkpoint();
    // print_error("After updating goal");
    // const bool valid_observations{ add_observations() };
    // _profiler.checkpoint();
    // print_error("After adding observations");
    // if (not _goal_reached and valid_observations)
    // {

    publish_control();

    _fg_mutex.unlock();

    _estimated_tree_publisher.publish(_estimated_tree.to_msg());
    _estimated_traj_publisher.publish(_estimated_trajectory);

    // }
    _profiler.end();
    _freq_counter++;
    _total_calls++;
    // }
  }

  bool get_node_at(prx_models::Node& node, ros::Time& timestamp)
  {
    ros::Time curr_time{ ros::Time::now() };

    prx_models::Node curr_node{ _estimated_tree.nodes[_estimated_tree.root] };
    prx_models::Node last_valid_node;
    bool valid{ false };
    while (curr_time < timestamp and curr_node.children.size() > 0)
    {
      // LOG_VARS(curr_time)

      // DEBUG_VARS(timestamp, curr_time);
      // DEBUG_VARS(curr_node);
      const std::size_t child_idx{ curr_node.children[0] };
      last_valid_node = curr_node;
      curr_node = _estimated_tree.nodes[child_idx];
      const ml4kp_bridge::Plan parent_plan{ _estimated_tree.edges[curr_node.parent_edge].plan };

      curr_time += ml4kp_bridge::duration(parent_plan);
    }
    if (curr_node.point.point.size() > 0)
    {
      node = curr_node;
      valid = true;
    }

    if (not valid and last_valid_node.point.point.size() > 0)
    {
      node = last_valid_node;
      valid = true;
      // LOG_VARS(timestamp)
      // LOG_VARS(node)
      // LOG_VARS(curr_node)
      // LOG_VARS(last_valid_node)
    }
    return valid;
  }

  void log_isam_graph()
  {
    std::streambuf* coutbuf = std::cout.rdbuf();       // save old buf
    std::cout.rdbuf(dbg::variables::ofs_log.rdbuf());  // redirect std::cout to out.txt!
    const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
        printCondition = [&](const gtsam::Factor* f, double err, size_t) { return f != nullptr; };

    const gtsam::Values current_estimate{ _isam.calculateBestEstimate() };
    current_estimate.print("--- VALUES ---\n", SF::formatter);
    _isam.getFactorsUnsafe().print("--- GRAPH ---\n", SF::formatter);
    _isam.getFactorsUnsafe().printErrors(current_estimate, "--- Problem graph ---\n", SF::formatter, printCondition);

    std::cout.rdbuf(coutbuf);
  }

  void log_graph(const std::string msg, const gtsam::NonlinearFactorGraph& graph,
                 const gtsam::Values& current_estimate) const
  {
    std::streambuf* coutbuf = std::cout.rdbuf();       // save old buf
    std::cout.rdbuf(dbg::variables::ofs_log.rdbuf());  // redirect std::cout to out.txt!
    const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
        printCondition = [&](const gtsam::Factor* f, double err, size_t) { return f != nullptr; };

    current_estimate.print("--- VALUES: " + msg + " ---\n", SF::formatter);
    graph.print("--- GRAPH: " + msg + " ---\n", SF::formatter);
    graph.printErrors(current_estimate, "--- Problem graph: " + msg + " ---\n", SF::formatter, printCondition);

    std::cout.rdbuf(coutbuf);
  }

  void update_estimated_tree()
  {
    change_status(stela_thread_t::ISAM, interface::StelaStatus::UPDATE_ESTIMATION);
    double dt{ 0.0 };
    Control ui{};
    // LOG_MSG("--- Estimation tree update ---")
    // const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
    // printCondition = [&](const gtsam::Factor* f, double err, size_t) { return f != nullptr; };
    // _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter, printCondition);
    // _isam.calculateEstimate().print("update_estimated_tree", SF::formatter);
    // DEBUG_VARS(_estimated_tree);
    // DEBUG_VARS(_x_queue);
    // DEBUG_VARS(_x_curr, _x_next);
    // DEBUG_VARS(_estimated_tree)
    // log_isam_graph();
    for (auto& node_pair : _estimated_tree.nodes)
    {
      const std::size_t node_idx{ node_pair.second.index };
      // DEBUG_VARS(_x_curr, _x_next, node_idx)

      //
      // DEBUG_VARS(node_idx);
      const StateKeys node_keys{ _robot->keyState(1, node_idx) };
      //
      // _fg_mutex.lock();

      update_estimates<0>(_node_estimates, _isam, node_keys);

      if (node_idx == _current_replanning_root)
      {
        _replanning_root_estimates = _node_estimates;
        compute_covariances<0>(_replanning_root_covariances, _isam, node_keys);
      }

      _robot->copy_estimates(node_pair.second.point, _node_estimates);

      if (_estimated_tree.nodes[node_idx].children.size() > 0)
      {
        const std::size_t node_next_idx{ _estimated_tree.nodes[node_idx].children[0] };
        const gtsam::Key key_dt{ _robot->keyT(node_idx, node_next_idx) };
        const gtsam::Key key_u01{ _robot->keyU(node_idx, node_next_idx) };

        // DEBUG_VARS(node_idx, node_next_idx);
        calculate_estimate_safe(dt, key_dt);
        calculate_estimate_safe(ui, key_u01);

        // if (std::fabs(ui[0]) > 0.5)
        // {
        //   LOG_MSG("update_estimated_tree")
        //   log_isam_graph();
        //   LOG_KEY(key_u01)
        //   LOG_VARS(ui.transpose())
        //   LOG_VARS(_x_curr, _x_next)

        //   prx_throw("[update_estimated_tree] Velocity too high!")
        // }

        const std::size_t edge_idx{ _estimated_tree.nodes[node_next_idx].parent_edge };
        // DEBUG_VARS(dt, node_idx, edge_idx, node_next_idx);
        // DEBUG_VARS(dt, ui.transpose());
        // DEBUG_VARS(_estimated_tree.edges[edge_idx])
        _estimated_tree.edges[edge_idx].plan.steps[0].duration.data = ros::Duration(dt);
        // _estimated_tree.edges[edge_idx].plan.steps[0].control.point = ros::Duration(dt);
        ml4kp_bridge::copy(_estimated_tree.edges[edge_idx].plan.steps[0].control, ui);
        _robot->bound(_estimated_tree.edges[edge_idx].plan);

        // DEBUG_VARS(node_idx, node_next_idx, dt)
      }
    }
    _estimated_tree.root = _x_curr;
    change_status(stela_thread_t::ISAM, interface::StelaStatus::IDLE);
    // LOG_MSG("--- Estimation tree update FINISHED ---")
  }

  void add_idle_fg(const std::size_t x_prev, const std::size_t x_next)
  {
    // _x0_start_time = finish_time;

    // _next_node_time = _x0_start_time + ros::Duration(_dt01);
    // DEBUG_VARS(x_prev, x_next)
    // const std::size_t x_prev{ _x_curr };
    // print_factors_to_remove();
    // _x_curr = _x_next;
    // _x_next++;

    // DEBUG_PRINT
    GraphValues graph_values{ _robot->idle_state_to_fg(x_prev, x_next, _time_as_variable) };
    // print_factors_to_remove();
    // DEBUG_PRINT
    // remove_factors();

    safe_fg_update(graph_values.first, graph_values.second);

    _values.insert_or_assign(graph_values.second);
    // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),
    //                                   _isam2_result.newFactorsIndices.begin(),  // no-lint
    //                                   _isam2_result.newFactorsIndices.end());

    update_dt();

    _future_factors_queue.push_back(x_prev);
    insert_factors(x_prev, x_next);

    // _next_node_index = _x_queue.back();
    // DEBUG_VARS(_x_queue);
    // DEBUG_VARS(x_prev, x_next)
    std::size_t next_node_index{ x_next };
    // DEBUG_VARS(next_node_index);
    // DEBUG_VARS(_estimated_tree);

    EdgeNodePair edge_node{ motion_planning::create_edge_node(_estimated_tree.nodes[x_prev], next_node_index) };

    edge_node.first.plan.steps.emplace_back();
    _robot->plan_step(edge_node.first.plan.steps.back(), _robot->idle_control(), _robot->idle_dt());

    _tree.edges[edge_node.first.index] = edge_node.first;
    _tree.nodes[edge_node.second.index] = edge_node.second;
    // _tree.nodes[edge_node.second.parent] = _estimated_tree.nodes.back();

    _estimated_tree.edges[edge_node.first.index] = edge_node.first;
    _estimated_tree.nodes[edge_node.second.index] = edge_node.second;

    // DEBUG_VARS(x_prev, x_next)
    // DEBUG_VARS(_estimated_tree);
    // _estimated_tree.edges.push_back(edge_node.first);
    // _estimated_tree.nodes.push_back(edge_node.second);

    // _next_tree_edge = edge_node.first.index;
    // _tree.edges.push_back(edge_node.first);
    // _tree.nodes.push_back(edge_node.second);
    // DEBUG_VARS(_estimated_tree)

    // _past_factors_queue.push_back(x_prev);
    // check_factor_removal();
    // remove_node_edge(_estimated_tree, x_prev);
  }

  // Assuming we are currently somewhere along edge E0: N0--E0-->N1--E2-->N2, check if we need to change to E2
  void update_next_goal()
  {
    const ros::Time now{ ros::Time::now() };
    _x0_start_time = _x0_start_time.isZero() ? now : _x0_start_time;  // Only update the first time
    const ros::Time finish_time{ _x0_start_time + ros::Duration(_dt01) };

    const double nowdt{ (now - _start_time).toSec() };
    const double finishdt{ (finish_time - _start_time).toSec() };

    const std::string now_str{ utils::time_to_string(now) };
    const std::string finish_time_str{ utils::time_to_string(finish_time) };

    // DEBUG_VARS(finish_time_str, now_str);
    if (now >= finish_time)
    {
      change_status(stela_thread_t::ISAM, interface::StelaStatus::GRAPH_UPDATE);

      // if (not _tree_received and _idle_initialized)
      // Add another idle edge
      if (_state == stela_state_t::IDLE)
      {
        // DEBUG_VARS(_x_queue)

        add_idle_fg(_x_queue.back(), _x_queue.back() + 1);

        _past_factors_queue.push_back(_x_curr);

        check_factor_removal();

        // DEBUG_VARS(_x_curr)
        // DEBUG_VARS(_estimated_tree)
        _estimated_tree.erase_node(_x_curr);

        _x_queue.pop_front();

        _x0_start_time = finish_time;

        _x_curr = _x_queue[0];
        _x_next = _x_queue[1];
        _x_queue.push_back(_x_queue.back() + 1);
      }
      if (_state == stela_state_t::TREE_RECEIVED)
      {
        std::pair<std::vector<std::size_t>, std::size_t> res_found{ find_root_placement_in_fg() };
        std::vector<std::size_t>& idx_to_remove{ res_found.first };
        std::size_t& last_valid{ res_found.second };
        if (idx_to_remove.size() > 0)
        {
          LOG_MSG("TREE_EXECUTING: Node found");
          // DEBUG_VARS(_x_queue)
          _past_factors_queue.push_back(_x_curr);
          // DEBUG_PRINT
          check_factor_removal();
          // DEBUG_VARS(_isam2_update_params.removeFactorIndices);
          // DEBUG_VARS(_past_factors_queue);
          for (auto idx : idx_to_remove)
          {
            const gtsam::FactorIndices& indices{ _inserted_factors[idx] };
            // DEBUG_VARS(idx, indices)

            // const std::size_t idx_parent{ _estimated_tree.nodes[idx].parent };
            gtsam::FactorIndices more_factors{ remove_common_factors(idx, last_valid) };
            // DEBUG_VARS(more_factors);

            _isam2_update_params.removeFactorIndices.insert(_isam2_update_params.removeFactorIndices.end(),  // no-lint
                                                            indices.begin(), indices.end());

            _isam2_update_params.removeFactorIndices.insert(_isam2_update_params.removeFactorIndices.end(),  // no-lint
                                                            more_factors.begin(), more_factors.end());

            _estimated_tree.erase_node(idx);
            const auto it = std::find(_x_queue.begin(), _x_queue.end(), idx);
            if (it != _x_queue.end())
            {
              _x_queue.erase(it);
            }
          }
          // print_factors_to_remove();
          // Values calculateEstimate() const;
          // _isam.calculateEstimate().print("VALUES", SF::formatter);

          // remove_factors();
          // DEBUG_PRINT
          add_idle_fg(last_valid, _tree.root);

          _past_factors_queue.push_back(_x_curr);

          // DEBUG_PRINT
          check_factor_removal();

          _estimated_tree.erase_node(_x_curr);
          _x_queue.pop_front();

          _x0_start_time = finish_time;

          _x_curr = _x_queue[0];
          _x_next = _x_queue[1];
          _x_queue.push_back(_tree.root);

          const std::size_t old_next_tree_edge{ _next_tree_edge };
          const std::size_t child_idx{ _tree.nodes[_tree.root].children[0] };
          _next_tree_edge = _tree.nodes[child_idx].parent_edge;
          _state = stela_state_t::TREE_EXECUTING;

          LOG_VARS(old_next_tree_edge, _next_tree_edge)
        }
        else if (idx_to_remove.size() == 0)
        {
          LOG_MSG("TREE_RECEIVED: Adding idle");
          // DEBUG_VARS(_x_queue)

          add_idle_fg(_x_queue.back(), _x_queue.back() + 1);

          _past_factors_queue.push_back(_x_curr);

          check_factor_removal();

          // DEBUG_VARS(_x_curr)
          // DEBUG_VARS(_x_curr, _estimated_tree.nodes.size())
          // DEBUG_VARS(_estimated_tree)
          _estimated_tree.erase_node(_x_curr);
          _x_queue.pop_front();

          // remove_node_edge(_estimated_tree, _x_curr);

          _x0_start_time = finish_time;

          _x_curr = _x_queue[0];
          _x_next = _x_queue[1];
          _x_queue.push_back(_x_queue.back() + 1);
        }
      }
      else if (_state == stela_state_t::TREE_EXECUTING)
      {
        // LOG_MSG("TREE_EXECUTING: Adding tree node");
        const std::size_t x_prev{ _x_queue[0] };
        _x_queue.pop_front();

        _x_curr = _x_queue[0];
        _x_next = _x_queue[1];
        // _x_curr = _x_next;

        // const prx_models::Node& node_current{ _tree.nodes[_x_curr] };
        // const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
        // _x_next = node_child.index;

        _selected_nodes.push_back(_x_curr);
        _past_factors_queue.push_back(x_prev);

        // if (_past_factors_queue.size() == 0 or _past_factors_queue.back() != node_current.parent)
        // {
        // _past_factors_queue.push_back(node_current.parent);

        // }
        _future_factors_queue.pop_front();
        _estimated_tree.erase_node(x_prev);

        update_dt();

        _x0_start_time = finish_time;
        _next_node_time = _x0_start_time + ros::Duration(_dt01);

        add_tree_node();

        // DEBUG_VARS(_x_queue)
      }

      change_status(stela_thread_t::ISAM, interface::StelaStatus::IDLE);
    }
  }

  void update_dt()
  {
    _key_dt = _robot->keyT(_x_curr, _x_next);
    if (_time_as_variable)
    {
      // DEBUG_PRINT
      calculate_estimate_safe(_dt01, _key_dt);
    }
    else
    {
      _dt01 = _values.at<double>(_key_dt);
    }
    // DEBUG_VARS(_dt01);
  }

  template <typename Estimate>
  void calculate_estimate_safe(Estimate& variable, const gtsam::Key& key)
  {
    try
    {
      // DEBUG_PRINT
      // PRINT_KEY(key);
      // _fg_mutex.lock();
      variable = _isam.calculateEstimate<Estimate>(key);
      // _fg_mutex.unlock();
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      PRINT_MSG("calculate_estimate_safe");
      const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
      failure_to_file(msg + e.what());
      std::cout << msg << std::string(e.what()) << std::endl;
      throw e;
      // failure_to_file(e.what());

      // gtsam::Values current_estimate{ _isam.getLinearizationPoint() };
      // estimates_to_file<0>(std::cout, current_estimate, key, false);
      // const std::string msg{ "Clique of " + SF::formatter(key) };
      // _isam[key]->print(msg, SF::formatter);
    }
    catch (std::out_of_range e)
    {
      PRINT_ERROR("[calculate_estimate_safe] Error: std::out_of_range. ")
      ERROR_VARS(_x_curr, _x_next);
      PRINT_KEY_ERROR(key);
      ERROR_VARS(e.what());
      throw e;
    }
    // DEBUG_PRINT
  }

  void print_covariance(const std::size_t id, const std::string message)
  {
    if (_x_curr < 11 or _x_curr > 30)
    {
      return;
    }
    const StateKeys keys{ RobotInterface::keyState(1, id) };
    const gtsam::Key kx{ std::get<0>(keys) };
    const gtsam::Key kxd{ std::get<1>(keys) };
    const std::string x{ SF::formatter(kx) };
    const std::string xdot{ SF::formatter(kxd) };
    auto Xcov = _isam.marginalCovariance(kx);
    auto XDcov = _isam.marginalCovariance(kxd);

    PRINT_MSG(message)

    DEBUG_VARS(_x_curr, id, x, xdot);
    DEBUG_VARS(Xcov);
    DEBUG_VARS(XDcov);
  }
  void log_isam_result()
  {
    LOG_MSG("--- ISAM RESULT ---")
    const double error_before{ _isam2_result.getErrorBefore() };
    const double error_after{ _isam2_result.getErrorAfter() };
    LOG_VARS(error_before, error_after)
    gtsam::ISAM2Result::DetailedResults* results{ _isam2_result.details() };
    const std::size_t variables_relinearized{ _isam2_result.getVariablesRelinearized() };
    const std::size_t variables_reeliminated{ _isam2_result.getVariablesReeliminated() };
    const std::size_t cliques{ _isam2_result.getCliques() };
    LOG_VARS(variables_relinearized, variables_reeliminated, cliques);

    std::streambuf* coutbuf = std::cout.rdbuf();       // save old buf
    std::cout.rdbuf(dbg::variables::ofs_log.rdbuf());  // redirect std::cout to out.txt!

    // const gtsam::Values current_estimate{ _isam.calculateBestEstimate() };
    for (auto var_pair : results->variableStatus)
    {
      const std::string& key{ SF::formatter(var_pair.first) };
      const bool reeliminated{ var_pair.second.isReeliminated };
      const bool above_relin_threshold{ var_pair.second.isAboveRelinThreshold };
      const bool is_relinearize_involved{ var_pair.second.isRelinearizeInvolved };
      const bool is_relinearized{ var_pair.second.isRelinearized };
      const bool is_observed{ var_pair.second.isObserved };
      const bool is_new{ var_pair.second.isNew };
      const bool in_root_clique{ var_pair.second.inRootClique };

      // current_estimate.find(var_pair.first)->value.print("updated: ");
      LOG_MSG("-- " + key)
      if (_isam.valueExists(var_pair.first))
      {
        _isam.calculateEstimate(var_pair.first).print("\tupdated: ");
      }
      else
      {
        LOG_MSG("\t Value does not exists")
      }

      LOG_VARS(key, reeliminated, above_relin_threshold, is_relinearize_involved, is_relinearized, is_observed, is_new,
               in_root_clique)
    }
    std::cout.rdbuf(coutbuf);
    LOG_MSG("--- FINISHED RESULT ---")

    // print_segment();
  }

  void add_observations()
  {
    if (_state == stela_state_t::IDLE or _state == stela_state_t::TREE_RECEIVED or
        _state == stela_state_t::TREE_EXECUTING)
    {
      change_status(stela_thread_t::ISAM, interface::StelaStatus::OBSERVATION);

      // DEBUG_VARS(_x_curr, _x_next, _x0_start_time)

      const GraphValues graph_values_z{ _robot->add_observation_factor(_x_curr, _x_next, _x0_start_time) };

      try
      {
        // if (not _fg_initialized)
        // {
        //   initialize_graph();
        // }
        // print_segment();

        // _fg_mutex.lock();

        _isam2_result = _isam.update(graph_values_z.first, graph_values_z.second);

        // log_isam_result();
        // const double error_before{ _isam2_result.getErrorBefore() };
        // const double error_after{ _isam2_result.getErrorAfter() };
        // LOG_VARS(error_before, error_after)

        insert_factors(_x_curr, _x_next);

        // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),
        // _isam2_result.newFactorsIndices.begin(),  // no-lint
        // _isam2_result.newFactorsIndices.end());
        _isam2_result.newFactorsIndices.clear();

        _key_u01 = _robot->keyU(_x_curr, _x_next);
        // _fg_mutex.lock();
        _u01 = _isam.calculateEstimate<Control>(_key_u01);
        // LOG_VARS(_x_curr, _x_next, _u01.transpose())
        // graph_values_z.first.print("Observation Graph", SF::formatter);
        // LOG_VARS(graph_values_z.first.size())
        // if (std::fabs(_u01[0]) > 0.8)
        // {
        //   // print_s
        //   // log_isam_graph();
        //   LOG_MSG("add_observations")
        //   LOG_KEY(_key_u01)
        //   LOG_VARS(_x_curr, _x_next, _u01.transpose())
        //   prx_throw("Velocity too high!");
        // }
        // _fg_mutex.unlock();

        update_dt();
        _observations_added = true;
        // _key_dt = RobotInterface::keyT(_x_curr, _x_next);
        // _dt01 = _isam.calculateEstimate<double>(_key_dt);
        change_status(stela_thread_t::ISAM, interface::StelaStatus::IDLE);
      }
      catch (gtsam::IndeterminantLinearSystemException e)
      {
        DEBUG_PRINT
        PRINT_MSG("add_observations");
        const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
        failure_to_file(msg + e.what());
        std::cout << msg << std::string(e.what()) << std::endl;
      }
    }
  }

  void publish_control()
  {
    change_status(stela_thread_t::ISAM, interface::StelaStatus::PUBLISH_CONTROL);
    _next_node_time = _x0_start_time + ros::Duration(_dt01);

    ml4kp_bridge::copy(_control_stamped.space_point, _u01);
    _robot->bound(_control_stamped);

    _control_stamped.header.seq++;
    _control_stamped.header.stamp = ros::Time::now();
    _stamped_control_publisher.publish(_control_stamped);

    change_status(stela_thread_t::ISAM, interface::StelaStatus::IDLE);
    // LOG_MSG("Published control")
    // LOG_VARS(_x_curr, _x_next, _u01.transpose())
  }

  void print_segment()
  {
    LOG_MSG("--- SEGMENT --- ")
    double dt{ 0.0 };
    Control ui{};
    std::size_t idx{ _x_curr };
    if (idx < 2)
      return;
    StateEstimates estimates;

    gtsam::Key key_dt{ _robot->keyT(idx, idx + 1) };
    gtsam::Key key_u01{ _robot->keyU(idx, idx + 1) };
    StateKeys node_keys{ _robot->keyState(1, idx) };

    State x0, x1;
    Eigen::RowVector3d xdot0, xdot1, xdot1_pred;

    update_estimates<0>(estimates, _isam, node_keys);
    x0 = std::get<0>(estimates);
    xdot0 = std::get<1>(estimates).transpose();

    node_keys = _robot->keyState(1, idx + 1);

    while (_isam.valueExists(key_dt) and _isam.valueExists(key_u01))
    {
      update_estimates<0>(estimates, _isam, node_keys);
      calculate_estimate_safe(dt, key_dt);
      calculate_estimate_safe(ui, key_u01);

      x1 = std::get<0>(estimates);
      xdot1 = std::get<1>(estimates).transpose();
      auto ctrl = ui.transpose();

      xdot1_pred = prx_models::mushr_CtrlAccel_t<>::predict(xdot0.transpose(), ui, dt, RobotInterface::default_params,
                                                            _robot->default_poly)
                       .transpose();
      const auto xdot1_error{ xdot1_pred - xdot1 };

      LOG_MSG("****")
      LOG_KEYS(node_keys[0], node_keys[1], key_u01, key_dt)
      LOG_VARS(idx);
      LOG_VARS(x0, x1);
      LOG_VARS(xdot0, xdot1);
      LOG_VARS(dt, ctrl)
      LOG_VARS(xdot1_pred)
      LOG_VARS(xdot1_error)

      x0 = x1;
      xdot0 = xdot1;
      idx++;
      key_dt = _robot->keyT(idx, idx + 1);
      key_u01 = _robot->keyU(idx, idx + 1);
      node_keys = _robot->keyState(1, idx + 1);
      // xdot_prev = xdot;
    }
    LOG_MSG("--- FINISHED SEGMENT --- ")

    // nodes_to_print.push_back(_x_curr);

    // for (int i = 0; i < segment; ++i)
    // {
    //   const std::size_t parent{ _tree.nodes[nodes_to_print.front()].parent };
    //   nodes_to_print.push_front(parent);
    // }
    // for (int i = 0; i < segment; ++i)
    // {
    //   const std::size_t child{ _tree.nodes[nodes_to_print.back()].children[0] };
    //   nodes_to_print.push_back(child);
    // }
    // StateEstimates estimates;
    // PRINT_MSG("-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~");
    // const gtsam::Values current_estimate{ _isam.calculateBestEstimate() };

    // for (auto id : nodes_to_print)
    // {
    //   if (_tree.nodes[id].parent != id)
    //   {
    //     const StateKeys state_keys{ RobotInterface::keyState(1, id) };
    //     update_estimates<0>(estimates, _isam, state_keys);
    //     const State x{ std::get<0>(estimates) };
    //     const Eigen::RowVectorXd xdot{ std::get<1>(estimates).transpose() };
    //     const gtsam::Key ku{ RobotInterface::keyU(_tree.nodes[id].parent, id) };
    //     const ControlTranspose u_fg{ _isam.calculateEstimate<Control>(ku).transpose() };

    //     const std::uint64_t parent_edge{ _tree.nodes[id].parent_edge };
    //     const ml4kp_bridge::Plan& plan{ _tree.edges[parent_edge].plan };
    //     auto u_plan = plan.steps[0].control.point;
    //     auto sbmp_node = _tree.nodes[id].point.point;

    //     PRINT_MSG("---");
    //     DEBUG_VARS(id, x, xdot);
    //     DEBUG_VARS(sbmp_node);
    //     DEBUG_VARS(u_fg, u_plan);

    //     insert_factors(_x_curr, _x_next);

    //     for (auto factor_id : _inserted_factors[id])
    //     {
    //       const double error{ _isam.getFactorsUnsafe()[factor_id]->error(current_estimate) };
    //       _isam.getFactorsUnsafe()[factor_id]->print("Factor", SF::formatter);
    //       DEBUG_VARS(error);
    //     }
    //   }
    // }
  }

  void obstacle_factors(gtsam::NonlinearFactorGraph& graph, const ml4kp_bridge::SpacePoint& point, const int x_id) const
  {
    // if (_obstacle_mode == "distance")
    // {
    //   const gtsam::Key keyX{ _robot->keyX(1, x_id) };
    //   _robot->copy_state(_x, point);
    //   for (auto obstacle_info : _obstacle_collision_infos)
    //   {
    //     if (ObstacleFactor::close_enough(_x, _obstacle_factor_include_distance, obstacle_info, _robot_collision_ptr,
    //                                      _config_from_state, _obstacle_tolerance_result))
    //     {
    //       graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX,
    //       _obstacle_distance_tolerance,
    //                                            0.1, _obstacle_noise);

    //       _obstacles_marker.points.emplace_back();
    //       _obstacles_marker.points.back().x = _x[0];
    //       _obstacles_marker.points.back().y = _x[1];
    //       _obstacles_marker.points.back().z = 0;
    //       _obstacles_marker.points.emplace_back();
    //       _obstacles_marker.points.back().x = obstacle_info->pose.position()[0];
    //       _obstacles_marker.points.back().y = obstacle_info->pose.position()[1];
    //       _obstacles_marker.points.back().z = 0;
    //     }
    //   }
    // }
    // if (_obstacle_mode == "all")
    // {
    //   const gtsam::Key keyX{ _robot->keyX(1, x_id) };
    //   _robot->copy_state(_x, point);
    //   for (auto obstacle_info : _obstacle_collision_infos)
    //   {
    //     // ObstacleFactor::close_enough(_state, _obstacle_factor_include_distance, obstacle_info,
    //     // _robot_collision_ptr,
    //     //                              _config_from_state, _obstacle_tolerance_result))
    //     graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX, _obstacle_distance_tolerance,
    //                                          0.1, _obstacle_noise);

    //     // _obstacles_marker.points.emplace_back();
    //     // _obstacles_marker.points.back().x = _state[0];
    //     // _obstacles_marker.points.back().y = _state[1];
    //     // _obstacles_marker.points.back().z = 0;
    //     // _obstacles_marker.points.emplace_back();
    //     // _obstacles_marker.points.back().x = obstacle_info->pose.position()[0];
    //     // _obstacles_marker.points.back().y = obstacle_info->pose.position()[1];
    //     // _obstacles_marker.points.back().z = 0;
    //   }
    // }
    if (_obstacle_mode == "sdf")
    {
      PRINT_MSG_ONCE("Using SDF Factors")
      const gtsam::Key keyX{ _robot->keyX(1, x_id) };
      // RobotInterface::state(_state, point);
      graph.emplace_shared<SdfFactor>(keyX, _obstacle_distance_tolerance, _sdf, _obstacle_noise);
    }
    else if (_obstacle_mode == "none")
    {
      PRINT_MSG_ONCE("No obstacle factors")
    }
    else
    {
      prx_throw("unsupported obstacle mode: " << _obstacle_mode);
    }
  }

  void remove_node_edge(prx_models::Tree& tree, const std::size_t idx)
  {
    // Can be done better (faster) with std::map or smth similar
    prx_models::Edge edge_removed;
    for (auto iter = tree.nodes.begin(); iter < tree.nodes.end(); iter++)
    {
      if (iter->index == idx)
      {
        tree.nodes.erase(iter);
      }
    }
    for (auto iter = tree.edges.begin(); iter < tree.edges.end(); iter++)
    {
      if (iter->source == idx)
      {
        tree.edges.erase(iter);
      }
      else if (iter->target == idx)
      {
        tree.edges.erase(iter);
      }
    }
  }

  void initialize_graph()
  {
    _x_curr = 0;
    _x_next = 1;
    // const prx_models::Node& root_node{ _tree.nodes[_tree.root] };
    const GraphValues root_graph_values{ _robot->idle_root(_x_curr) };
    _values.insert(root_graph_values.second);
    _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);

    insert_factors(_x_curr, _x_next);
    // insert_factors(_x_curr, _x_next);

    // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),  // no-lint
    // _isam2_result.newFactorsIndices.begin(), _isam2_result.newFactorsIndices.end());

    GraphValues graph_values{ _robot->idle_state_to_fg(_x_curr, _x_next, _time_as_variable) };

    _values.insert(graph_values.second);

    _isam2_result = _isam.update(graph_values.first, graph_values.second);

    insert_factors(_x_curr, _x_next);

    // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),  // no-lint
    // _isam2_result.newFactorsIndices.begin(), _isam2_result.newFactorsIndices.end());

    std::size_t next_node_index{ _x_curr };
    _active_nodes.insert(_x_curr);
    _active_nodes.insert(_x_next);
    _estimated_tree.nodes.clear();
    _estimated_tree.edges.clear();

    _tree.clear();

    _estimated_tree.root = next_node_index;
    // _estimated_tree.nodes[next_node_index]=
    _estimated_tree.nodes[next_node_index].index = next_node_index;
    _estimated_tree.nodes[next_node_index].parent = next_node_index;
    _estimated_tree.nodes[next_node_index].parent_edge = next_node_index;

    next_node_index++;
    EdgeNodePair edge_node{ motion_planning::create_edge_node(_estimated_tree.nodes[next_node_index - 1],
                                                              next_node_index) };

    edge_node.first.plan.steps.emplace_back();
    _robot->plan_step(edge_node.first.plan.steps.back(), _robot->idle_control(), _robot->idle_dt());

    _estimated_tree.edges[edge_node.first.index] = edge_node.first;
    _estimated_tree.nodes[edge_node.second.index] = edge_node.second;

    // _tree.copy(_estimated_tree);  // copy only the initialization
    _current_future_nodes = 1;
    // DEBUG_VARS(_estimated_tree.nodes);
    // safe_fg_update(graph_values.first, graph_values.second);

    _future_factors_queue.push_back(_x_curr);
    // insert_factors(_x_curr, _x_next);

    _x_queue.push_back(_x_curr);

    int idx{ 1 };
    for (; idx < _total_future_nodes; ++idx)
    {
      add_idle_fg(idx, idx + 1);
      insert_factors(idx, idx + 1);
      _x_queue.push_back(idx);
      _current_future_nodes++;
    }
    _x_queue.push_back(idx);
    // DEBUG_VARS(_x_queue);
    // DEBUG_VARS(_estimated_tree)
    _start_time = ros::Time::now();

    _state = stela_state_t::IDLE;

    // DEBUG_VARS(_estimated_tree);
    PRINT_MSG("Stela Windowed IDLE Initialized");
  }

  void insert_factors(const std::size_t id0, const std::size_t id1)
  {
    const StateKeys state0_keys{ _robot->keyState(1, id0) };
    // const StateKeys state1_keys{ _robot->keyState(1, id1) };
    const ControlKeys ctrl01_keys{ _robot->keyControl(id0, id1) };
    const TimeKeys time01_keys{ _robot->keyTime(id0, id1) };

    std::vector<gtsam::Key> all0_keys;
    all0_keys.insert(all0_keys.end(), state0_keys.begin(), state0_keys.end());
    all0_keys.insert(all0_keys.end(), ctrl01_keys.begin(), ctrl01_keys.end());
    all0_keys.insert(all0_keys.end(), time01_keys.begin(), time01_keys.end());

    // std::set<std::size_t> inserted;
    for (auto iter = _isam2_result.newFactorsIndices.begin(); iter != _isam2_result.newFactorsIndices.end(); iter++)
    {
      const std::size_t factor_id{ *iter };

      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor_id]->keys() };

      std::size_t id_to_insert{ id1 };
      for (auto key : factor_keys)
      {
        auto k0_res = std::find(all0_keys.begin(), all0_keys.end(), key);
        // auto k1_res = std::find(keys1.begin(), keys1.end(), key);
        if (k0_res != all0_keys.end())
        {
          id_to_insert = id0;
          break;
        }
      }

      if (_inserted_factors_sets[id_to_insert].count(factor_id) > 0)
      {
        continue;
      }
      _inserted_factors_sets[id_to_insert].insert(factor_id);
      // inserted.insert(factor_id);
      // if (id0 == 0 or id1 == 1)
      // {
      //   PRINT_KEYS_CONTAINER(factor_keys);
      //   DEBUG_VARS(id_to_insert, _inserted_factors[id_to_insert]);
      // }
      _inserted_factors[id_to_insert].insert(_inserted_factors[id_to_insert].end(), factor_id);
    }

    // _isam.getFactorsUnsafe().print("Current Graph", SF::formatter);
  }

  void add_tree_node()
  {
    // LOG_MSG("--- New node ---")

    // const prx_models::Node next_node{ _tree.nodes[_next_tree_node] };

    // prx_models::Edge edge;
    const prx_models::Edge& edge{ _tree.edges[_next_tree_edge] };
    // DEBUG_VARS(edge.index, edge.source, edge.target);
    const prx_models::Node& node_current{ _tree.nodes[edge.target] };
    const std::size_t total_children{ node_current.children.size() };

    if (_tree.root != edge.source)
    {
      const prx_models::Node& node_parent{ _tree.nodes[edge.source] };
      _active_nodes.insert(node_parent.index);
    }

    // DEBUG_VARS(edge.plan);
    // GraphValues graph_values{ _robot->node_edge_to_fg(edge.source, edge.target, node_current.point, edge.plan,
    // _time_as_variable) };
    GraphValues graph_values{ _robot->node_edge_to_fg(node_current, edge) };

    // graph_values.second.print("add_tree_node", SF::formatter);
    obstacle_factors(graph_values.first, node_current.point, edge.target);

    check_factor_removal();

    // print_factors_to_remove();
    // LOG_VARS(node_current)
    // LOG_VARS(edge)
    // graph_values.first.print("New Graph", SF::formatter);
    // DEBUG_VARS(_isam2_update_params.removeFactorIndices)
    // LOG_VARS(_x_curr, _x_next, _next_tree_edge)
    // LOG_VARS(edge.source, edge.target, node_current.point, edge.plan)
    safe_fg_update(graph_values.first, graph_values.second);

    LOG_VARS(_x_curr, _x_next)
    LOG_VARS(edge.source, edge.target);

    _future_factors_queue.push_back(edge.source);
    insert_factors(edge.source, edge.target);

    _estimated_tree.edges[edge.index] = edge;
    _estimated_tree.nodes[edge.target] = node_current;
    _estimated_tree.nodes[edge.source].children.push_back(edge.target);
    _estimated_tree.nodes[edge.target].children.clear();

    const gtsam::Key key_u01{ _robot->keyU(edge.source, edge.target) };
    Control ui;
    calculate_estimate_safe(ui, key_u01);

    // LOG_VARS(edge)
    // LOG_KEY(key_u01);
    // LOG_VARS(ui.transpose());

    // DEBUG_VARS(_estimated_tree.edges[edge.index]);
    _x_queue.push_back(node_current.index);

    if (total_children > 0)
    {
      const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
      const std::size_t old_next_tree_edge{ _next_tree_edge };
      _next_tree_edge = node_child.parent_edge;
      _tree_valid = true;

      // LOG_VARS(old_next_tree_edge, _next_tree_edge);
    }
    else
    {
      const std::string msg{ "Changing to IDLE" };
      // DEBUG_VARS(msg, _x_curr, node_current.index)
      _tree_valid = false;
      _state = stela_state_t::IDLE;
      // _goal_id = node_current.index;
    }
    // LOG_MSG("--- New node FINISHED ---")
  }

  void remove_factors()
  {
    // LOG_MSG("Removing factors")
    try
    {
      // _fg_mutex.lock();
      // print_factors_to_remove();
      // DEBUG_PRINT
      FactorGraph fg;
      Values values;
      _isam2_result = _isam.update(fg, values, _isam2_update_params);
      // DEBUG_PRINT
      _isam2_update_params.removeFactorIndices.clear();
      // _fg_mutex.unlock();
      // DEBUG_PRINT
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      PRINT_MSG("remove_factors");
      DEBUG_VARS(e.what());
      // failure_to_file(e.what());
      // prx::fg::indeterminant_linear_system_helper(graph, _values);
      std::cout << "[EXCEPTION] Var: " << SF::formatter(e.nearbyVariable()) << std::endl;
      // _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter);
      // graph.printErrors(_values, "Problem graph", SF::formatter);
      // _values.print("Values", SF::formatter);
      throw e;
    }
  }

  void safe_fg_update(const FactorGraph& graph, const Values& values)
  {
    // LOG_MSG("safe_fg_update")
    try
    {
      // SF::symbols_to_file("/Users/Gary/pracsys/catkin_ws/factor_graph_symbols.txt");
      // _fg_mutex.lock();
      // _values.insert(new_values);
      _values.insert_or_assign(values);

      // graph.printErrors(_values, "Problem graph", SF::formatter);
      // _isam2_update_params.force_relinearize = true;
      _isam2_result = _isam.update(graph, values, _isam2_update_params);
      // log_isam_result();
      _isam2_update_params.removeFactorIndices.clear();
      // _fg_mutex.unlock();
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      // _fg_mutex.unlock();
      DEBUG_PRINT
      PRINT_MSG("safe_fg_update");
      PRINT_KEYS(e.nearbyVariable())
      prx::fg::indeterminant_linear_system_helper(graph, _values);
      // std::cout << "[EXCEPTION] Var: " << SF::formatter(e.nearbyVariable()) << std::endl;
      // graph.printErrors(_values, "Problem graph", SF::formatter);
      const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
          printCondition = [&](const gtsam::Factor* f, double err, size_t) { return f != nullptr; };
      _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter, printCondition);
      failure_to_file(e.what());

      throw e;
    }
    catch (gtsam::ValuesKeyAlreadyExists e)
    {
      // _fg_mutex.unlock();
      DEBUG_PRINT
      PRINT_MSG("safe_fg_update");
      PRINT_KEYS(e.key())
      DEBUG_VARS(e.what())
      failure_to_file(e.what());
      throw e;
      // _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter);
    }
    catch (gtsam::ValuesKeyDoesNotExist e)
    {
      graph.print("Error Graph: ", SF::formatter);
      values.print("Error values: ", SF::formatter);
      DEBUG_PRINT
      PRINT_MSG("safe_fg_update");
      PRINT_KEYS(e.key())
      DEBUG_VARS(e.what())
      // log_isam_graph();

      failure_to_file(e.what());
      // std::cout << "[EXCEPTION] Not found: " << SF::formatter(e.key()) << std::endl;
      // graph.print("Problem graph", SF::formatter);
      // _fg_mutex.unlock();
      // values.print("Problem values", SF::formatter);
      throw e;
    }
  }

  void check_factor_removal()
  {
    // DEBUG_VARS(_past_factors_queue.size(), _total_past_nodes);

    if (_past_factors_queue.size() > _total_past_nodes)
    {
      // SF::symbols_to_file("/Users/Gary/pracsys/catkin_ws/factor_graph_symbols.txt");
      // DEBUG_VARS(_past_factors_queue);
      const std::size_t past_factor_id{ _past_factors_queue.front() };
      const gtsam::FactorIndices& indices{ _inserted_factors[past_factor_id] };
      // DEBUG_VARS(_past_factors_queue);
      // DEBUG_VARS(past_factor_id, indices);

      // DEBUG_PRINT
      node_info_to_file(past_factor_id);
      // DEBUG_VARS(indices);

      _isam2_update_params.removeFactorIndices.insert(_isam2_update_params.removeFactorIndices.end(),  // no-lint
                                                      indices.begin(), indices.end());

      _past_factors_queue.pop_front();
      _active_nodes.erase(past_factor_id);

      const StateKeys state_keys{ _robot->keyState(1, _past_factors_queue.front()) };

      StateEstimates estimates;
      std::vector<Eigen::MatrixXd> covariances;

      // DEBUG_PRINT
      update_estimates<0>(estimates, _isam, state_keys);
      compute_covariances<0>(covariances, _isam, state_keys);

      const GraphValues graph_values_priors{ _robot->estimate_to_prior(_past_factors_queue.front(), estimates,
                                                                       covariances) };
      // _isam2_result = _isam.update(graph_values_priors.first, graph_values_priors.second);
      // remove_factors();
      safe_fg_update(graph_values_priors.first, Values());

      insert_factors(past_factor_id, _past_factors_queue.front());

      const prx_models::Node node_to_remove{ _tree.nodes[past_factor_id] };

      auto iter_edge_to_remove = _tree.edges.find(node_to_remove.parent_edge);
      auto iter_node_to_remove = _tree.nodes.find(past_factor_id);
      if (iter_edge_to_remove != _tree.edges.end())
      {
        _tree.edges.erase(iter_edge_to_remove);
      }
      if (iter_node_to_remove != _tree.nodes.end())
      {
        _tree.nodes.erase(iter_node_to_remove);
      }
      // remove_node_edge(_estimated_tree, past_factor_id);

      // DEBUG_VARS(past_factor_id);
      // print_factors_to_remove();
      // node_info_to_file(past_factor_id);
      // _current_past_nodes--;
    }
  }

  gtsam::FactorIndices remove_common_factors(const std::size_t x_curr, const std::size_t x_other)
  {
    const StateKeys state_keys{ _robot->keyState(1, x_curr) };
    const ControlKeys ctrl_keys{ _robot->keyControl(x_other, x_curr) };

    std::vector<gtsam::Key> all_keys;
    all_keys.push_back(_robot->keyT(x_other, x_curr));
    all_keys.insert(all_keys.end(), state_keys.begin(), state_keys.end());
    all_keys.insert(all_keys.end(), ctrl_keys.begin(), ctrl_keys.end());

    const gtsam::FactorIndices& indices{ _inserted_factors[x_other] };

    DEBUG_VARS(x_curr, x_other)
    PRINT_KEYS_(all_keys);
    gtsam::FactorIndices result;
    for (auto factor : indices)
    {
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor]->keys() };
      // DEBUG_VARS(factor);
      // PRINT_KEYS(factor_keys);

      if (match_factor_lists(factor_keys, all_keys))
      {
        result.push_back(factor);
      }
    }
    return result;
  }

  bool match_factor_lists(const gtsam::KeyVector& factor_keys, const std::vector<gtsam::Key> all_keys)
  {
    for (auto k : factor_keys)
    {
      const auto it_x = std::find(all_keys.begin(), all_keys.end(), k);
      if (it_x != all_keys.end())
      {
        PRINT_MSG("Removing factor!")
        PRINT_KEY(k);
        // result.push_back(factor);
        return true;
      }
    }
    return false;
  }

  void print_factors_to_remove() const
  {
    // PRINT_MSG("Factors to remove");
    for (auto factor_to_remove : _isam2_update_params.removeFactorIndices)
    {
      // DEBUG_VARS(factor_to_remove);
      // const gtsam::FactorIndices& indices{ _inserted_factors[factor_to_remove] };

      // for (auto factor_id : indices)
      // {
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor_to_remove]->keys() };
      // LOG_VARS(_x_curr, factor_to_remove);
      // PRINT_KEYS_CONTAINER(factor_keys);
      // }
    }
  }

  void node_info_to_file(const std::size_t node_id)
  {
    _ofs << node_id << " ";
    const StateKeys keys{ _robot->keyState(1, node_id) };
    // PRINT_KEYS(keys[0], keys[1])
    // _values.print("node_info_to_file", SF::formatter);
    // _fg_mutex.lock();
    update_values<0, StateEstimates>(_values, _isam, keys);
    // PRINT_KEYS_CONTAINER(keys);
    // DEBUG_VARS(_x_curr, _x_next);
    estimates_to_file<StateEstimates, 0>(_ofs, _values, keys, _isam, true);
    // _fg_mutex.unlock();
  }

  std::pair<std::vector<std::size_t>, std::size_t> find_root_placement_in_fg()
  {
    // DEBUG_PRINT
    const ros::Time root_stamp{ _tree.nodes[_tree.root].stamp };

    ros::Time curr_stamp{ ros::Time::now() };
    double dt{ _dt01 };

    std::size_t node_idx_prev{ _estimated_tree.root };
    std::size_t node_idx_next{ _estimated_tree.root };
    std::size_t node_last_valid{ _estimated_tree.root };

    std::vector<std::size_t> idx_to_remove;
    // while (curr_stamp < root_stamp)
    while (_estimated_tree.nodes[node_idx_prev].children.size() > 0)
    {
      if (curr_stamp > root_stamp)
      {
        idx_to_remove.push_back(node_idx_prev);
      }
      else
      {
        node_last_valid = node_idx_prev;
      }
      node_idx_next =
          _estimated_tree.nodes[node_idx_prev].children[0];  //_estimated_tree.get_child_edge(node_idx_prev, 0);
      const gtsam::Key key_dt{ _robot->keyT(node_idx_prev, node_idx_next) };

      DEBUG_PRINT
      calculate_estimate_safe(dt, key_dt);
      node_idx_prev = node_idx_next;
      curr_stamp += ros::Duration(dt);
    }
    if (curr_stamp > root_stamp)
    {
      idx_to_remove.push_back(node_idx_prev);
    }
    // DEBUG_VARS(curr_stamp, root_stamp);
    // DEBUG_VARS(node_idx_prev, node_idx_next);
    // DEBUG_VARS(idx_to_remove);

    return { idx_to_remove, node_last_valid };
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    _tree.clear();
    _tree.copy(msg);

    if (_state == stela_state_t::IDLE or _state == stela_state_t::TREE_RECEIVED)
    {
      // find_root_placement_in_fg();
      _state = stela_state_t::TREE_RECEIVED;
      // _x0_start_time = _tree.nodes[msg->root].stamp;
      // const std::size_t child_idx{ _tree.nodes[msg->root].children[0] };
    }
    else if (_state == stela_state_t::TREE_EXECUTING)
    {
      // Merge the new tree with the current executing FG
    }

    // _state = stela_state_t::TREE;

    // DEBUG_VARS(ros::Time::now());
    // LOG_VARS(ros::Time::now());
    // LOG_VARS(*msg);
    // // std::pair<bool, prx_models::Node&> curr_node{ motion_planning::get_node_safe(_tree, msg->root) };
    // const bool node_exists{ _tree.nodes.count(msg->root) > 0 };
    // const bool valid_tree{ msg->root > _x_curr };
    // // DEBUG_VARS(_x_curr, msg->root, msg->t0, ros::Time::now())
    // // LOG_VARS(_x_curr, msg->root, valid_tree, node_exists)
    // if (valid_tree and node_exists)
    // {
    //   _tree.merge(*msg);

    //   // curr_node.second = motion_planning::get_root(*msg);
    //   // curr_node = motion_planning::get_node_safe(_tree, curr_node.second.index);

    //   // _tree.nodes.insert(_tree.nodes.end(), msg->nodes);
    //   // _tree.edges.insert(_tree.edges.end(), msg->edges);

    //   if (_tree.nodes[msg->root].children.size() > 0)
    //   {
    //     const std::size_t child_idx{ _tree.nodes[msg->root].children[0] };
    //     _next_tree_edge = _tree.nodes[child_idx].parent_edge;
    //     DEBUG_VARS(_current_future_nodes, _total_future_nodes);
    //     for (; _current_future_nodes < _total_future_nodes; ++_current_future_nodes)
    //     {
    //       add_tree_node();
    //       DEBUG_VARS(_current_future_nodes, _total_future_nodes);
    //     }
    //   }
    //   {
    //     // else add idle / contingency
    //   }
    //   LOG_VARS(_tree)
    // }
  }

private:
  Values _values;
  // FactorGraph _factor_graph; _past_factors_queue

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;
  gtsam::ISAM2UpdateParams _isam2_update_params;

  StateKeys _state_keys;
  StateEstimates _state_estimates;
  StateEstimates _node_estimates;
  StateEstimates _sbmp_nodes;

  gtsam::Key _key_last_esimated_state;
  gtsam::Key _key_u01;
  gtsam::Key _key_dt;

  ml4kp_bridge::SpacePointStamped _control_stamped;
  // prx_models::Tree _tree;

  bool _new_tree_available;
  prx_models::tree_msg_wrapper_t _new_tree;
  // prx_models::Tree _new_tree;

  prx_models::tree_msg_wrapper_t _tree;
  prx_models::tree_msg_wrapper_t _prev_tree;

  ros::Subscriber _tree_subscriber;
  ros::Subscriber _collision_subscriber;

  ros::Publisher _control_publisher;
  ros::Publisher _stamped_control_publisher;
  ros::Publisher _finish_publisher;
  ros::Publisher _viz_obstacles_publisher;
  ros::Publisher _estimated_tree_publisher;
  ros::Publisher _estimated_traj_publisher;
  ros::Publisher _planner_clock_publisher;

  ros::Timer _control_timer;
  ros::Timer _control_frequency_timer;
  ros::Timer _estimation_timer;
  ros::Timer _observations_freq_timer;
  ros::Timer _clock_timer;
  ros::Timer _replan_timer;

  ros::ServiceClient _planner_service_client;

  bool _use_contingency;
  prx_models::StelaKraft _planner_service_call;

  // std::unique_ptr<StelaActionServer> _stela_action_server;

  interface::PlannerClock _planner_clock_msg;

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  // std::size_t _id_x_hat;

  State _x;
  Control _u01;
  // Control _u_plan;
  Observation _z_new;
  double _dt01;

  bool _call_replanner;
  // double _time_remaining;

  // motion_planning::StelaGraphTraversalGoal _goal;

  std::size_t _x_curr;
  std::size_t _x_next;
  std::deque<std::size_t> _x_queue;

  // std::vector<std::uint64_t> _selected_nodes;
  // std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;

  bool _tree_valid;
  motion_planning::tree_manager_t _tree_manager;
  prx_models::tree_msg_wrapper_t _estimated_tree;
  ros::Time _next_node_time;
  ros::Time _start_time;
  ros::Time _x0_start_time;
  bool _last_local_goal;

  // File/output
  bool _files_created;
  std::string _output_dir;
  std::string _experiment_id;

  // Obstacle-relates stuff
  std::string _obstacle_mode;
  // gtsam::NonlinearFactorGraph _obstacle_graph;
  double _obstacle_distance_tolerance;
  double _obstacle_factor_include_distance;
  SdfPtr _sdf;

  typename RobotInterface::ConfigFromState _config_from_state;
  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  typename ObstacleFactor::ToleranceResult _obstacle_tolerance_result;
  typename ObstacleFactor::DistanceResult _obstacle_distance_result;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;
  visualization_msgs::Marker _obstacles_marker;

  const std::string _name{};

  std::size_t _next_tree_node;
  std::size_t _next_tree_edge;
  int _total_future_nodes, _current_future_nodes;

  std::size_t _freq_counter;
  std::size_t _total_calls;
  double _freq_accum;

  std::size_t _goal_id;
  bool _goal_reached;
  bool _visualize;

  // std::size_t _current_past_nodes;_inserted_factors
  int _total_past_nodes;

  std::set<std::size_t> _active_nodes;
  std::vector<std::size_t> _selected_nodes;

  // Map of id -> factors, in order of insertion in the FG.
  std::unordered_map<std::size_t, std::set<std::size_t>> _inserted_factors_sets;
  std::unordered_map<std::size_t, gtsam::FactorIndices> _inserted_factors;
  std::list<std::size_t> _past_factors_queue;
  std::list<std::size_t> _future_factors_queue;
  ml4kp_bridge::StelaTrajectory _estimated_trajectory;

  bool _using_stepper;

  std::string _timestamp;
  std::ofstream _ofs;

  ros::Duration _max_observation_delay;
  double _control_frequency;
  std::size_t _total_z_calls;

  // std::size_t _next_idle_idx;
  // std::size_t _trees_received;
  // bool _idle_initialized;

  stela_state_t _state;

  bool _time_as_variable;
  bool _observations_added;
  bool _fg_initialized;

  int _replanning_calls;

  // std::size_t _next_node_index;

  std::shared_ptr<RobotInterface> _robot;

  mutable std::mutex _fg_mutex;

  gtsam::LevenbergMarquardtParams _lm_params;

  ros::NodeHandle _nh;

  utils::time_profiler_t _profiler;

  std::size_t _current_replanning_root;
  StateEstimates _replanning_root_estimates;
  std::vector<Eigen::MatrixXd> _replanning_root_covariances;

  interface::StelaStatus _status;
  ros::Publisher _replanning_status_publisher, _isam_status_publisher;

#ifdef GTSAM_USE_TBB
  tbb::global_control _tbb_control;
#endif
};
}  // namespace motion_planning
