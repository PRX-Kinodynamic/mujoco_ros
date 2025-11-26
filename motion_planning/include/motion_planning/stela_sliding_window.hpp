#include <chrono>
#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <utils/std_utils.hpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <motion_planning/utils.hpp>
#include <motion_planning/sdf_factor.hpp>
#include <motion_planning/tree_bridge.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <ml4kp_bridge/StelaTrajectory.h>
#include <prx_models/tree_msg_wrapper.hpp>
#include <utils/time_profiler.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif
namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename RobotInterface, typename Base>
class stela_windowed_t : public Base
{
  enum stela_state_t
  {
    INITIALIZING = 0,  // Starting and setting up
    IDLE,              // Adding idle state (i.e. stay in place)
    TREE_RECEIVED,     // Got a tree but the start is in the future
    TREE_EXECUTING,    // Following the tree
    FINISHING          // Closing files and the like
  };

  using Derived = stela_windowed_t<RobotInterface, Base>;

  using SF = prx::fg::symbol_factory_t;

  using Control = typename RobotInterface::Control;
  using State = typename RobotInterface::State;

  // using StateDot = typename RobotInterface::StateDot;
  using Observation = typename RobotInterface::Observation;

  using StateKeys = typename RobotInterface::StateKeys;
  using ControlKeys = typename RobotInterface::ControlKeys;

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
    , _tree_received(false)
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
    , _next_node_index(0)
    , _state(stela_state_t::INITIALIZING)
#ifdef GTSAM_USE_TBB
    , _tbb_control(tbb::global_control::max_allowed_parallelism, 8)
#endif
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
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

    std::vector<double> plant_parameters{};

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
    PARAM_SETUP_WITH_DEFAULT(private_nh, visualize, visualize)
    PARAM_SETUP_WITH_DEFAULT(private_nh, time_as_variable, time_as_variable)
    PARAM_SETUP_WITH_DEFAULT(private_nh, obstacle_sigma, obstacle_sigma)
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id)
    PARAM_SETUP_WITH_DEFAULT(private_nh, plant_parameters, plant_parameters)
    PARAM_SETUP_WITH_DEFAULT(private_nh, report_control_frequency, report_control_frequency)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_future_nodes, total_future_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_past_nodes, total_past_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)
    PARAM_SETUP_WITH_DEFAULT(private_nh, using_stepper, using_stepper)
    PARAM_SETUP_WITH_DEFAULT(private_nh, estimation_pub_freq, estimation_pub_freq);
    PARAM_SETUP_WITH_DEFAULT(private_nh, observation_frquency, observation_frquency);

    DEBUG_PRINT
    if (plant_parameters.size() > 0)
    {
      RobotInterface::set_params(plant_parameters);
    }
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

    _robot = std::make_shared<RobotInterface>(private_nh);

    RobotInterface::print_params();
    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };
    const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    const ros::Duration control_timer(1.0 / control_frequency);
    const ros::Duration estimation_timer(1.0 / estimation_pub_freq);
    const ros::Duration observation_timer(1.0 / observation_frquency);

    _control_timer = private_nh.createTimer(control_timer, &Derived::main_timer_callback, this);
    _estimation_timer = private_nh.createTimer(estimation_timer, &Derived::estimation_timer_callback, this);

    // How much time can it pass between observations before declaring failure
    _observations_freq_timer = private_nh.createTimer(observation_timer, &Derived::observation_timer_callback, this);

    if (report_control_frequency)
    {
      const ros::Duration control_freq_timer(1.0);
      _control_frequency_timer = private_nh.createTimer(control_freq_timer, &Derived::check_frequency, this);
    }

    _tree_subscriber = private_nh.subscribe(input_tree_topic_name, 1, &Derived::tree_callback, this);
    _collision_subscriber = private_nh.subscribe(collision_topic, 1, &Derived::collision_callback, this);

    _finish_publisher = private_nh.advertise<std_msgs::Bool>(finish_topic, 1, true);
    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);

    _viz_obstacles_publisher = private_nh.advertise<visualization_msgs::Marker>(obstacle_viz_topic, 1, false);

    _estimated_tree_publisher = private_nh.advertise<prx_models::Tree>(estimated_tree_topic, 1, true);
    _estimated_traj_publisher =
        private_nh.advertise<ml4kp_bridge::StelaTrajectory>(estimated_trajectory_topic, 1, true);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "StelaControl";

    _prev_header.stamp = ros::Time::now();
    _next_node_time = ros::Time::now();

    auto obstacles = prx::load_obstacles(environment);
    // _obstacle_list = obstacles.second;
    _obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

    _robot_collision_ptr = RobotInterface::collision_geometry();
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
  }

  ~stela_windowed_t()
  {
    to_file();
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    if (msg->data)
    {
      to_file(true);
    }
  }

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

    ofs_data << "Initialized: " << (_tree_received ? "true" : "false") << "\n";
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

    _tree_received = false;

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
    if (_tree_received)
    {
      // _isam.calculateBestEstimate()
      // const double current_error{ _isam.error(_isam.getDelta()) };
      // const gtsam::Values values{ _isam.calculateBestEstimate() };
      // const double current_error{ _isam.getFactorsUnsafe().error(values) };

      const double dt{ (event.current_real - event.last_real).toSec() };
      const double stela_frequency{ _freq_counter / dt };
      const double& target_frequency{ _control_frequency };
      DEBUG_VARS(stela_frequency, target_frequency);
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
    }
  }

  void observation_timer_callback(const ros::TimerEvent& event)
  {
    // if (_tree_received)
    // DEBUG_PRINT
    add_observations();
    _total_z_calls++;
    // DEBUG_PRINT
    // const ros::Duration dt{ ros::Time::now() - _tf.header.stamp };
    // if (dt > _max_observation_delay)
    // {
    //   to_file(false, false, true);
    // }
    // }
  }

  void estimation_timer_callback(const ros::TimerEvent& event)
  {
    // DEBUG_VARS(_tree_received, _observations_added, _idle_initialized)
    // if (_tree_received)
    // {
    update_estimated_tree();

    // if (_visualize)
    // {
    // _viz_obstacles_publisher.publish(_obstacles_marker);
    // }
    _estimated_tree_publisher.publish(_estimated_tree.to_msg());
    _estimated_traj_publisher.publish(_estimated_trajectory);
    // }
    // else if (_observations_added and _idle_initialized)
    // {
    // DEBUG_PRINT
    // update_estimated_tree();
    // _estimated_tree_publisher.publish(_estimated_tree);
    // _estimated_traj_publisher.publish(_estimated_trajectory);
    // PRINT_MSG("[Stela] estimated tree sent ");
    // }
  }

  void main_timer_callback(const ros::TimerEvent& event)
  {
    // if (_tree_received)
    // {
    _profiler.start();
    update_next_goal();
    // _profiler.checkpoint();
    // print_error("After updating goal");
    // const bool valid_observations{ add_observations() };
    // _profiler.checkpoint();
    // print_error("After adding observations");
    // if (not _goal_reached and valid_observations)
    // {

    publish_control();

    // }
    _profiler.end();
    _freq_counter++;
    _total_calls++;
    // }
  }

  void update_estimated_tree()
  {
    double dt{ 0.0 };
    remove_factors();
    for (auto& node_pair : _estimated_tree.nodes)
    {
      // DEBUG_VARS(_x_curr, _x_next, node.index)
      std::size_t node_idx{ node_pair.second.index };

      // DEBUG_PRINT
      // DEBUG_VARS(node_idx);
      const StateKeys node_keys{ RobotInterface::keyState(1, node_idx) };
      update_estimates<0>(_node_estimates, _isam, node_keys);
      RobotInterface::copy(node_pair.second.point, _node_estimates);

      if (_estimated_tree.nodes[node_idx].children.size() > 0)
      {
        const std::size_t node_next_idx{ _estimated_tree.nodes[node_idx].children[0] };
        const gtsam::Key key_dt{ RobotInterface::keyT(node_idx, node_next_idx) };
        calculate_estimate_safe(dt, key_dt);
        const std::size_t edge_idx{ _estimated_tree.nodes[node_next_idx].parent_edge };
        _estimated_tree.edges[edge_idx].plan.steps[0].duration.data = ros::Duration(dt);
        // DEBUG_VARS(node_idx, node_next_idx, dt)
      }
    }
    _estimated_tree.root = _x_curr;
  }

  void add_idle_fg(const std::size_t x_prev, const std::size_t x_next)
  {
    // _x0_start_time = finish_time;

    // _next_node_time = _x0_start_time + ros::Duration(_dt01);

    // const std::size_t x_prev{ _x_curr };
    // print_factors_to_remove();
    // _x_curr = _x_next;
    // _x_next++;

    GraphValues graph_values{ _robot->idle_state_to_fg(x_prev, x_next, _time_as_variable) };
    safe_fg_update(graph_values.first, graph_values.second);

    _values.insert_or_assign(graph_values.second);
    // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),
    //                                   _isam2_result.newFactorsIndices.begin(),  // no-lint
    //                                   _isam2_result.newFactorsIndices.end());

    update_dt();

    _future_factors_queue.push_back(x_prev);
    insert_factors(x_prev, x_next);

    EdgeNodePair edge_node{ motion_planning::create_edge_node(_estimated_tree.nodes[x_prev], _next_node_index) };
    // DEBUG_VARS(_next_node_index);

    edge_node.first.plan.steps.emplace_back();
    RobotInterface::plan_step(edge_node.first.plan.steps.back(), _robot->idle_control(), _robot->idle_dt());

    // _tree.edges[edge_node.first.index] = edge_node.first;
    // _tree.nodes[edge_node.second.index] = edge_node.second;
    // _tree.nodes[edge_node.second.parent] = _estimated_tree.nodes.back();

    _estimated_tree.edges[edge_node.first.index] = edge_node.first;
    _estimated_tree.nodes[edge_node.second.index] = edge_node.second;

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
      // if (not _tree_received and _idle_initialized)
      // Add another idle edge
      if (_state == stela_state_t::IDLE)
      {
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
        // _x_next++;
        // _next_idle_idx++;
        // _tree.nodes[msg->root].stamp
      }
      if (_state == stela_state_t::TREE_RECEIVED)
      {
        std::pair<std::vector<std::size_t>, std::size_t> res_found{ find_root_placement_in_fg() };
        std::vector<std::size_t>& idx_to_remove{ res_found.first };
        std::size_t& last_valid{ res_found.second };
        if (idx_to_remove.size() > 0)
        {
          // DEBUG_VARS(_x_queue)
          _past_factors_queue.push_back(_x_curr);
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
          print_factors_to_remove();
          // Values calculateEstimate() const;
          // _isam.calculateEstimate().print("VALUES", SF::formatter);

          DEBUG_PRINT
          remove_factors();

          add_idle_fg(last_valid, _tree.root);
          _past_factors_queue.push_back(_x_curr);

          check_factor_removal();
          DEBUG_PRINT

          _estimated_tree.erase_node(_x_curr);
          _x_queue.pop_front();

          _x0_start_time = finish_time;

          _x_curr = _x_queue[0];
          _x_next = _x_queue[1];
          _x_queue.push_back(_tree.root);
          DEBUG_PRINT

          const std::size_t child_idx{ _tree.nodes[_tree.root].children[0] };
          _next_tree_edge = _tree.nodes[child_idx].parent_edge;
          _state = stela_state_t::TREE_EXECUTING;
          DEBUG_PRINT
        }
        else if (idx_to_remove.size() == 0)
        {
          PRINT_MSG("TREE_RECEIVED: Adding idle");
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
        DEBUG_PRINT
        const std::size_t x_prev{ _x_queue[0] };
        _x_queue.pop_front();

        _x_curr = _x_queue[0];
        _x_next = _x_queue[1];
        // _x_curr = _x_next;
        DEBUG_PRINT

        // const prx_models::Node& node_current{ _tree.nodes[_x_curr] };
        // const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
        // _x_next = node_child.index;

        DEBUG_PRINT
        _selected_nodes.push_back(_x_curr);
        _past_factors_queue.push_back(x_prev);
        DEBUG_PRINT
        // if (_past_factors_queue.size() == 0 or _past_factors_queue.back() != node_current.parent)
        // {
        // _past_factors_queue.push_back(node_current.parent);
        DEBUG_PRINT
        // }
        _future_factors_queue.pop_front();
        DEBUG_PRINT

        update_dt();

        DEBUG_PRINT
        _x0_start_time = finish_time;
        _next_node_time = _x0_start_time + ros::Duration(_dt01);

        DEBUG_PRINT
        add_tree_node();
        DEBUG_PRINT

        // DEBUG_VARS(_x_queue)
      }
    }
  }

  void update_dt()
  {
    _key_dt = RobotInterface::keyT(_x_curr, _x_next);
    if (_time_as_variable)
    {
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
      variable = _isam.calculateEstimate<Estimate>(key);
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      PRINT_MSG("calculate_estimate_safe");
      const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
      failure_to_file(msg + e.what());
      std::cout << msg << std::string(e.what()) << std::endl;

      // failure_to_file(e.what());

      // gtsam::Values current_estimate{ _isam.getLinearizationPoint() };
      // estimates_to_file<0>(std::cout, current_estimate, key, false);
      // const std::string msg{ "Clique of " + SF::formatter(key) };
      // _isam[key]->print(msg, SF::formatter);
    }
    catch (std::out_of_range e)
    {
      PRINT_MSG("calculate_estimate_safe");
      PRINT_KEYS(key);
      DEBUG_VARS(e.what());
    }
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

  void add_observations()
  {
    // const bool new_observation{ query_tf() };
    // const bool header_updated{ _tf.header.stamp > _prev_header.stamp };
    // // DEBUG_VARS(new_observation, header_updated, _last_local_goal);

    // bool add_observation{ new_observation and header_updated };
    // // DEBUG_VARS(new_observation, header_updated, !_last_local_goal);
    // // DEBUG_VARS(add_observation);
    // // if (new_observation and header_updated and not _last_local_goal)
    // if (add_observation)
    // {
    // _prev_header = _tf.header;
    // const double dt{ (_tf.header.stamp - _x0_start_time).toSec() };
    // if (dt < 0)
    //   return;

    // RobotInterface::copy(_z_new, _tf);
    // _time_remaining = (_next_node_time - ros::Time::now()).toSec();
    //
    // DEBUG_VARS(_idle_initialized, _tree_received)
    // if (_idle_initialized or _tree_received)
    if (_state == stela_state_t::IDLE or _state == stela_state_t::TREE_RECEIVED or
        _state == stela_state_t::TREE_EXECUTING)
    {
      const GraphValues graph_values_z{ _robot->add_observation_factor(_x_curr, _x_next, _x0_start_time) };
      try
      {
        // DEBUG_VARS(_x_curr, _x_next);

        _isam2_result = _isam.update(graph_values_z.first, graph_values_z.second);

        // insert_factors(_x_curr, _x_next);

        insert_factors(_x_curr, _x_next);

        // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),
        // _isam2_result.newFactorsIndices.begin(),  // no-lint
        // _isam2_result.newFactorsIndices.end());
        _isam2_result.newFactorsIndices.clear();

        _key_u01 = RobotInterface::keyU(_x_curr, _x_next);
        _u01 = _isam.calculateEstimate<Control>(_key_u01);

        update_dt();
        _observations_added = true;
        // _key_dt = RobotInterface::keyT(_x_curr, _x_next);
        // _dt01 = _isam.calculateEstimate<double>(_key_dt);
      }
      catch (gtsam::IndeterminantLinearSystemException e)
      {
        PRINT_MSG("add_observations");
        const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
        failure_to_file(msg + e.what());
        std::cout << msg << std::string(e.what()) << std::endl;
      }
    }

    // const StateKeys state_keys{ RobotInterface::keyState(1, _x_curr) };
    // update_estimates<0>(_state_estimates, state_keys);
    // update_estimates<0>(_node_estimates, _isam, state_keys);

    // RobotInterface::copy(_feedback.xhat, _state_estimates);
    // }
    // return add_observation;
  }

  void publish_control()
  {
    _next_node_time = _x0_start_time + ros::Duration(_dt01);

    ml4kp_bridge::copy(_control_stamped.space_point, _u01);
    _control_stamped.header.seq++;
    _control_stamped.header.stamp = ros::Time::now();
    _stamped_control_publisher.publish(_control_stamped);

    // DEBUG_VARS(_u01.transpose());
    // print_segment();
    // update_estimates<0>(_state_estimates, state_keys);
    // update_estimates<0>(_node_estimates, _isam, state_keys);
  }

  void print_segment()
  {
    const int segment{ 3 };
    std::list<std::size_t> nodes_to_print;

    nodes_to_print.push_back(_x_curr);

    for (int i = 0; i < segment; ++i)
    {
      const std::size_t parent{ _tree.nodes[nodes_to_print.front()].parent };
      nodes_to_print.push_front(parent);
    }
    for (int i = 0; i < segment; ++i)
    {
      const std::size_t child{ _tree.nodes[nodes_to_print.back()].children[0] };
      nodes_to_print.push_back(child);
    }
    StateEstimates estimates;
    PRINT_MSG("-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~");
    const gtsam::Values current_estimate{ _isam.calculateBestEstimate() };

    for (auto id : nodes_to_print)
    {
      if (_tree.nodes[id].parent != id)
      {
        const StateKeys state_keys{ RobotInterface::keyState(1, id) };
        update_estimates<0>(estimates, _isam, state_keys);
        const State x{ std::get<0>(estimates) };
        const Eigen::RowVectorXd xdot{ std::get<1>(estimates).transpose() };
        const gtsam::Key ku{ RobotInterface::keyU(_tree.nodes[id].parent, id) };
        const ControlTranspose u_fg{ _isam.calculateEstimate<Control>(ku).transpose() };

        const std::uint64_t parent_edge{ _tree.nodes[id].parent_edge };
        const ml4kp_bridge::Plan& plan{ _tree.edges[parent_edge].plan };
        auto u_plan = plan.steps[0].control.point;
        auto sbmp_node = _tree.nodes[id].point.point;

        PRINT_MSG("---");
        DEBUG_VARS(id, x, xdot);
        DEBUG_VARS(sbmp_node);
        DEBUG_VARS(u_fg, u_plan);

        insert_factors(_x_curr, _x_next);

        for (auto factor_id : _inserted_factors[id])
        {
          const double error{ _isam.getFactorsUnsafe()[factor_id]->error(current_estimate) };
          _isam.getFactorsUnsafe()[factor_id]->print("Factor", SF::formatter);
          DEBUG_VARS(error);
        }
      }
    }
  }

  void obstacle_factors(gtsam::NonlinearFactorGraph& graph, const ml4kp_bridge::SpacePoint& point, int x_id)
  {
    if (_obstacle_mode == "distance")
    {
      const gtsam::Key keyX{ RobotInterface::keyX(1, x_id) };
      RobotInterface::state(_x, point);
      for (auto obstacle_info : _obstacle_collision_infos)
      {
        if (ObstacleFactor::close_enough(_x, _obstacle_factor_include_distance, obstacle_info, _robot_collision_ptr,
                                         _config_from_state, _obstacle_tolerance_result))
        {
          graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX, _obstacle_distance_tolerance,
                                               0.1, _obstacle_noise);

          _obstacles_marker.points.emplace_back();
          _obstacles_marker.points.back().x = _x[0];
          _obstacles_marker.points.back().y = _x[1];
          _obstacles_marker.points.back().z = 0;
          _obstacles_marker.points.emplace_back();
          _obstacles_marker.points.back().x = obstacle_info->pose.position()[0];
          _obstacles_marker.points.back().y = obstacle_info->pose.position()[1];
          _obstacles_marker.points.back().z = 0;
        }
      }
    }
    if (_obstacle_mode == "all")
    {
      const gtsam::Key keyX{ RobotInterface::keyX(1, x_id) };
      RobotInterface::state(_x, point);
      for (auto obstacle_info : _obstacle_collision_infos)
      {
        // ObstacleFactor::close_enough(_state, _obstacle_factor_include_distance, obstacle_info,
        // _robot_collision_ptr,
        //                              _config_from_state, _obstacle_tolerance_result))
        graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX, _obstacle_distance_tolerance,
                                             0.1, _obstacle_noise);

        // _obstacles_marker.points.emplace_back();
        // _obstacles_marker.points.back().x = _state[0];
        // _obstacles_marker.points.back().y = _state[1];
        // _obstacles_marker.points.back().z = 0;
        // _obstacles_marker.points.emplace_back();
        // _obstacles_marker.points.back().x = obstacle_info->pose.position()[0];
        // _obstacles_marker.points.back().y = obstacle_info->pose.position()[1];
        // _obstacles_marker.points.back().z = 0;
      }
    }
    if (_obstacle_mode == "sdf")
    {
      PRINT_MSG_ONCE("Using SDF Factors")
      const gtsam::Key keyX{ RobotInterface::keyX(1, x_id) };
      // RobotInterface::state(_state, point);
      graph.emplace_shared<SdfFactor>(keyX, _obstacle_distance_tolerance, _sdf, _obstacle_noise);
    }
  }

  void set_next_node()
  {
    if (_tree_received)
    {
      const prx_models::Node& node_current{ _tree.nodes[_x_curr] };
      // DEBUG_VARS(_x_curr, node_current);
      if (node_current.index == _goal_id)
      {
        _goal_reached = true;
        _selected_nodes.push_back(_goal_id);
        PRINT_MSG("[StelaWindowed] Goal Reached");
        to_file(false, false);
      }
      else
      {
        const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
        // DEBUG_VARS(node_child);
        _x_next = node_child.index;
        _selected_nodes.push_back(node_current.index);
        if (_past_factors_queue.size() == 0 or _past_factors_queue.back() != node_current.parent)
        {
          _past_factors_queue.push_back(node_current.parent);
          // _past_factors_queue.push_back(_future_factors_queue.front());
        }
        _future_factors_queue.pop_front();
      }
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

    // insert_factors(_x_curr, _x_next);

    // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),  // no-lint
    // _isam2_result.newFactorsIndices.begin(), _isam2_result.newFactorsIndices.end());

    // _next_tree_edge = _tree.nodes[root_node.children[0]].parent_edge;
    GraphValues graph_values{ _robot->idle_state_to_fg(_x_curr, _x_next, _time_as_variable) };
    _values.insert(graph_values.second);
    // GraphValues graph_values{ _robot->node_edge_to_fg(_x_curr, _x_next, _time_as_variable) };

    _isam2_result = _isam.update(graph_values.first, graph_values.second);

    insert_factors(_x_curr, _x_next);

    // _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),  // no-lint
    // _isam2_result.newFactorsIndices.begin(), _isam2_result.newFactorsIndices.end());

    _active_nodes.insert(_x_curr);
    _active_nodes.insert(_x_next);
    _next_node_index = _x_curr;
    _estimated_tree.nodes.clear();
    _estimated_tree.edges.clear();

    _tree.clear();

    _estimated_tree.root = _next_node_index;
    // _estimated_tree.nodes[_next_node_index]=
    _estimated_tree.nodes[_next_node_index].index = _next_node_index;
    _estimated_tree.nodes[_next_node_index].parent = _next_node_index;
    _estimated_tree.nodes[_next_node_index].parent_edge = _next_node_index;

    _next_node_index++;
    EdgeNodePair edge_node{ motion_planning::create_edge_node(_estimated_tree.nodes[_next_node_index - 1],
                                                              _next_node_index) };

    edge_node.first.plan.steps.emplace_back();
    RobotInterface::plan_step(edge_node.first.plan.steps.back(), _robot->idle_control(), _robot->idle_dt());

    _estimated_tree.edges[edge_node.first.index] = edge_node.first;
    _estimated_tree.nodes[edge_node.second.index] = edge_node.second;

    // _tree.copy(_estimated_tree);  // copy only the initialization
    _current_future_nodes = 1;
    // DEBUG_VARS(_estimated_tree.nodes);
    // safe_fg_update(graph_values.first, graph_values.second);

    _future_factors_queue.push_back(_x_curr);
    insert_factors(_x_curr, _x_next);

    _x_queue.push_back(_x_curr);

    int idx{ _current_future_nodes };
    for (; idx < _total_future_nodes; ++idx)
    {
      // DEBUG_VARS(idx, _total_future_nodes)
      add_idle_fg(idx, idx + 1);
      _x_queue.push_back(idx);
    }
    _x_queue.push_back(idx);
    DEBUG_VARS(_x_queue);
    // DEBUG_VARS(_estimated_tree)
    _start_time = ros::Time::now();

    _state = stela_state_t::IDLE;

    PRINT_MSG("Stela Windowed IDLE Initialized");
  }

  void insert_factors(const std::size_t id0, const std::size_t id1)
  {
    const StateKeys keys0{ RobotInterface::keyState(1, id0) };
    const StateKeys keys1{ RobotInterface::keyState(1, id1) };

    for (auto iter = _isam2_result.newFactorsIndices.begin(); iter != _isam2_result.newFactorsIndices.end(); iter++)
    {
      const std::size_t factor_id{ *iter };
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor_id]->keys() };

      std::size_t id_to_insert{ id1 };
      for (auto key : factor_keys)
      {
        auto k0_res = std::find(keys0.begin(), keys0.end(), key);
        // auto k1_res = std::find(keys1.begin(), keys1.end(), key);
        if (k0_res != keys0.end())
        {
          id_to_insert = id0;
          break;
        }
      }
      // PRINT_KEYS(factor_keys);
      // DEBUG_VARS(factor_id, id_to_insert);_inserted_factors
      _inserted_factors[id_to_insert].insert(_inserted_factors[id_to_insert].end(), factor_id);
    }
    // _isam.getFactorsUnsafe().print("Current Graph", SF::formatter);
  }

  void add_tree_node()
  {
    DEBUG_VARS(_next_tree_edge);

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

    GraphValues graph_values{ RobotInterface::node_edge_to_fg(edge.source, edge.target, node_current.point, edge.plan,
                                                              _time_as_variable) };

    obstacle_factors(graph_values.first, node_current.point, edge.target);

    check_factor_removal();

    // print_factors_to_remove();

    DEBUG_VARS(_isam2_update_params.removeFactorIndices)
    safe_fg_update(graph_values.first, graph_values.second);

    _future_factors_queue.push_back(edge.source);
    insert_factors(edge.source, edge.target);

    _estimated_tree.edges[edge.index] = edge;
    _estimated_tree.nodes[edge.target] = node_current;

    _x_queue.push_back(node_current.index);

    if (total_children > 0)
    {
      const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
      _next_tree_edge = node_child.parent_edge;

      // DEBUG_VARS(node_child, _next_tree_edge);
    }
    else
    {
      _tree_received = false;
      // _goal_id = node_current.index;
    }
  }

  void remove_factors()
  {
    // DEBUG_PRINT
    try
    {
      // DEBUG_PRINT
      _isam2_result = _isam.update(FactorGraph(), Values(), _isam2_update_params);
      // DEBUG_PRINT
      _isam2_update_params.removeFactorIndices.clear();
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
    try
    {
      // SF::symbols_to_file("/Users/Gary/pracsys/catkin_ws/factor_graph_symbols.txt");
      _values.insert_or_assign(values);

      // graph.printErrors(_values, "Problem graph", SF::formatter);
      // _isam2_update_params.force_relinearize = true;
      _isam2_result = _isam.update(graph, values, _isam2_update_params);

      _isam2_update_params.removeFactorIndices.clear();
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      PRINT_MSG("safe_fg_update");
      failure_to_file(e.what());
      prx::fg::indeterminant_linear_system_helper(graph, _values);
      std::cout << "[EXCEPTION] Var: " << SF::formatter(e.nearbyVariable()) << std::endl;
      // graph.printErrors(_values, "Problem graph", SF::formatter);
      // _values.print("Values", SF::formatter);
      throw e;
    }
    catch (gtsam::ValuesKeyAlreadyExists e)
    {
      PRINT_MSG("safe_fg_update");
      PRINT_KEYS(e.key())
      DEBUG_VARS(e.what())
      failure_to_file(e.what());
      _isam.getFactorsUnsafe().printErrors(_values, "Problem graph", SF::formatter);
    }
    catch (gtsam::ValuesKeyDoesNotExist e)
    {
      PRINT_MSG("safe_fg_update");
      PRINT_KEYS(e.key())
      DEBUG_VARS(e.what())
      failure_to_file(e.what());
      // std::cout << "[EXCEPTION] Not found: " << SF::formatter(e.key()) << std::endl;
      // graph.print("Problem graph", SF::formatter);
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

      node_info_to_file(past_factor_id);

      _isam2_update_params.removeFactorIndices.insert(_isam2_update_params.removeFactorIndices.end(),  // no-lint
                                                      indices.begin(), indices.end());

      _past_factors_queue.pop_front();
      _active_nodes.erase(past_factor_id);

      // remove_node_edge(_estimated_tree, past_factor_id);

      // DEBUG_VARS(past_factor_id);
      // print_factors_to_remove();
      // node_info_to_file(past_factor_id);
      // _current_past_nodes--;
    }
  }

  gtsam::FactorIndices remove_common_factors(const std::size_t x_curr, const std::size_t x_other)
  {
    const StateKeys state_keys{ RobotInterface::keyState(1, x_curr) };
    const ControlKeys ctrl_keys{ RobotInterface::keyControl(x_other, x_curr) };

    std::vector<gtsam::Key> all_keys;
    all_keys.push_back(RobotInterface::keyT(x_other, x_curr));
    all_keys.insert(all_keys.end(), state_keys.begin(), state_keys.end());
    all_keys.insert(all_keys.end(), ctrl_keys.begin(), ctrl_keys.end());

    const gtsam::FactorIndices& indices{ _inserted_factors[x_other] };

    PRINT_KEYS_(all_keys);
    gtsam::FactorIndices result;
    for (auto factor : indices)
    {
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor]->keys() };
      DEBUG_VARS(factor);
      PRINT_KEYS(factor_keys);

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
        PRINT_KEYS(k);
        // result.push_back(factor);
        return true;
      }
    }
    return false;
  }

  void print_factors_to_remove() const
  {
    PRINT_MSG("Factors to remove");
    for (auto factor_to_remove : _isam2_update_params.removeFactorIndices)
    {
      // DEBUG_VARS(factor_to_remove);
      // const gtsam::FactorIndices& indices{ _inserted_factors[factor_to_remove] };

      // for (auto factor_id : indices)
      // {
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor_to_remove]->keys() };
      DEBUG_VARS(_x_curr, factor_to_remove);
      PRINT_KEYS(factor_keys);
      // }
    }
  }

  void node_info_to_file(const std::size_t node_id)
  {
    _ofs << node_id << " ";
    const StateKeys keys{ RobotInterface::keyState(1, node_id) };
    // PRINT_KEYS(keys[0], keys[1])
    // _values.print("node_info_to_file", SF::formatter);
    update_values<0, StateEstimates>(_values, _isam, keys);
    estimates_to_file<StateEstimates, 0>(_ofs, _values, keys, _isam, true);
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
      const gtsam::Key key_dt{ RobotInterface::keyT(node_idx_prev, node_idx_next) };
      calculate_estimate_safe(dt, key_dt);
      node_idx_prev = node_idx_next;
      curr_stamp += ros::Duration(dt);
    }
    if (curr_stamp > root_stamp)
    {
      idx_to_remove.push_back(node_idx_prev);
    }
    DEBUG_VARS(curr_stamp, root_stamp);
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

    //   _tree_received = true;

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

  ros::Timer _control_timer;
  ros::Timer _control_frequency_timer;
  ros::Timer _estimation_timer;
  ros::Timer _observations_freq_timer;

  // std::unique_ptr<StelaActionServer> _stela_action_server;

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

  // double _time_remaining;

  // motion_planning::StelaGraphTraversalGoal _goal;

  std::size_t _x_curr;
  std::size_t _x_next;
  std::deque<std::size_t> _x_queue;

  // std::vector<std::uint64_t> _selected_nodes;
  // std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;

  bool _tree_received;
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

  std::size_t _next_node_index;

  std::shared_ptr<RobotInterface> _robot;

  utils::time_profiler_t _profiler;
#ifdef GTSAM_USE_TBB
  tbb::global_control _tbb_control;
#endif
};
}  // namespace motion_planning
