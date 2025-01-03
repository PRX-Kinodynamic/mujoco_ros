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

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <ml4kp_bridge/StelaTrajectory.h>
#include <prx_models/tree_msg_wrapper.hpp>

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class stela_windowed_t : public Base
{
  using Derived = stela_windowed_t<SystemInterface, Base>;

  using SF = prx::fg::symbol_factory_t;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;

  // using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;

  using StateKeys = typename SystemInterface::StateKeys;
  using ControlKeys = typename SystemInterface::ControlKeys;

  using StateEstimates = typename SystemInterface::StateEstimates;
  using ControlEstimates = typename SystemInterface::ControlEstimates;

  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename SystemInterface::ConfigFromState,
                                                    prx::fg::collision_info_t::CollisionErrorType::STEP>;

  using Sdf = utils::signed_distance_field_t;
  using SdfPtr = std::shared_ptr<Sdf>;
  using SdfFactor = motion_planning::sdf_factor_t<State, typename SystemInterface::ConfigFromState>;

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
    , _tree_recevied(false)
    , _last_local_goal(true)
    , _goal_received(false)
    , _experiment_id("test")
    , _files_created(false)
    , _name("STELA_SW")
    , _total_future_nodes(10)
    , _total_past_nodes(10)
    , _goal_id(std::numeric_limits<std::size_t>::max())
    , _goal_reached(false)
    , _visualize(false)
    , _using_stepper(false)
    , _trees_received(0)
    , _max_observation_delay(1.0)
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    PRINT_MSG("Starting Stela Windowed");

    std::string tree_topic_name;
    std::string control_topic;
    std::string collision_topic;
    std::string environment;
    std::string estimated_tree_topic;
    double obstacle_sigma{ 0.1 };
    double control_frequency;
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
    int& total_future_nodes{ _total_future_nodes };
    int& total_past_nodes{ _total_past_nodes };
    int estimation_pub_freq{ 30 };
    // ROS_PARAM_SETUP(private_nh, random_seed);
    // ROS_PARAM_SETUP(private_nh, plant_config_file);
    // ROS_PARAM_SETUP(private_nh, planner_config_file);
    PARAM_SETUP(private_nh, estimated_tree_topic);
    PARAM_SETUP(private_nh, tree_topic_name);
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
    PARAM_SETUP_WITH_DEFAULT(private_nh, obstacle_sigma, obstacle_sigma)
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id)
    PARAM_SETUP_WITH_DEFAULT(private_nh, plant_parameters, plant_parameters)
    PARAM_SETUP_WITH_DEFAULT(private_nh, report_control_frequency, report_control_frequency)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_future_nodes, total_future_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_past_nodes, total_past_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)
    PARAM_SETUP_WITH_DEFAULT(private_nh, using_stepper, using_stepper)
    PARAM_SETUP_WITH_DEFAULT(private_nh, estimation_pub_freq, estimation_pub_freq);

    if (plant_parameters.size() > 0)
    {
      SystemInterface::set_params(plant_parameters);
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
    SystemInterface::print_params();
    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };
    const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    const ros::Duration control_timer(1.0 / control_frequency);
    const ros::Duration estimation_timer(1.0 / estimation_pub_freq);

    _control_timer = private_nh.createTimer(control_timer, &Derived::main_timer_callback, this);
    _estimation_timer = private_nh.createTimer(estimation_timer, &Derived::estimation_timer_callback, this);

    // How much time can it pass between observations before declaring failure
    const ros::Duration observation_timer(10);
    _observations_freq_timer = private_nh.createTimer(observation_timer, &Derived::observation_timer_callback, this);

    if (report_control_frequency)
    {
      const ros::Duration control_freq_timer(1.0);
      _control_frequency_timer = private_nh.createTimer(control_freq_timer, &Derived::check_frequency, this);
    }

    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);
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

    _robot_collision_ptr = SystemInterface::collision_geometry();
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
    ofs_data << "Initialized: " << (_tree_recevied ? "true" : "false") << "\n";
    ofs_data << "ElapsedTime: " << elapsed_time << "\n";
    ofs_data << "Collision: " << (collision ? "true" : "false") << "\n";
    ofs_data << "ObstacleDistanceTolerance: " << _obstacle_distance_tolerance << "\n";
    ofs_data << "ObstacleMode: " << _obstacle_mode << "\n";
    ofs_data << "ExceptionRaised: " << (rasied_exception ? "true" : "false") << "\n";
    ofs_data << "NetworkProblem: " << (network_problem ? "true" : "false") << "\n";
    ofs_data.close();

    // PRINT_MSG("[TODO] Data files for STELA_SW not implemented.");
    // gtsam::Values estimate{ _isam.calculateEstimate() };
    ofs_branch << "# id point[...]\n";
    for (auto node_id : _selected_nodes)
    {
      // ofs << node_id << " ";
      // const StateKeys keys{ SystemInterface::keyState(1, node_id) };
      // estimates_to_file<StateEstimates, 0>(ofs, _values, keys, _isam, false);
      // ofs_covariance
      ofs_branch << node_id << " ";
      ml4kp_bridge::to_file(_tree.nodes[node_id].point, ofs_branch);
      ofs_branch << "\n";
    }

    while (not _past_factors_queue.empty())
    {
      const std::size_t id{ _past_factors_queue.front() };

      // _isam2_update_params.removeFactorIndices = _inserted_factors[id];
      _past_factors_queue.pop_front();
      _active_nodes.erase(id);
      node_info_to_file(id);
    }

    _ofs.close();
    ofs_branch.close();

    _files_created = true;

    _tree_recevied = false;

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

  void check_frequency(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      // const double current_error{ _isam.error(_isam.getDelta()) };

      const double dt{ (event.current_real - event.last_real).toSec() };
      const double stela_frequency{ _freq_counter / dt };

      DEBUG_VARS(stela_frequency);
      _freq_counter = 0;
    }
  }

  void observation_timer_callback(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      const ros::Duration dt{ ros::Time::now() - _tf.header.stamp };
      if (dt > _max_observation_delay)
      {
        to_file(false, false, true);
      }
    }
  }

  void estimation_timer_callback(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      update_estimated_tree();
      if (_visualize)
      {
        _estimated_tree_publisher.publish(_estimated_tree);
        _viz_obstacles_publisher.publish(_obstacles_marker);
      }
      _estimated_traj_publisher.publish(_estimated_trajectory);
    }
  }

  void main_timer_callback(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      update_next_goal();
      const bool valid_observations{ add_observations() };
      if (not _goal_reached and valid_observations)
      {
        publish_control();
      }
    }
    _freq_counter++;
  }

  void update_estimated_tree()
  {
    _tree_manager.reset();
    _estimated_tree.nodes.clear();
    _estimated_tree.edges.clear();
    _estimated_tree.root = 0;
    _estimated_tree.nodes.resize(_active_nodes.size() * 2);

    int node_idx{ 0 };
    using NodeSourceTarget = std::pair<std::size_t, std::size_t>;
    std::vector<NodeSourceTarget> nodes_edges;
    std::map<std::size_t, std::size_t> nodes_map;
    for (auto idx : _active_nodes)
    {
      const StateKeys node_keys{ SystemInterface::keyState(1, idx) };
      update_estimates<0>(_node_estimates, _isam, node_keys);

      _estimated_tree.nodes[node_idx] = _tree_manager.create_node();
      SystemInterface::copy(_estimated_tree.nodes[node_idx].point, _node_estimates);

      nodes_map[idx] = node_idx;
      node_idx++;
    }

    // int past_nodes{ static_cast<int>(_current_past_nodes) };

    _estimated_trajectory.data.clear();
    _estimated_trajectory.ids.clear();
    for (auto id : _past_factors_queue)
    {
      // if (past_nodes >= 0)
      // {
      _estimated_trajectory.data.emplace_back();
      _estimated_trajectory.data.back().space_point = _estimated_tree.nodes[nodes_map[id]].point;
      _estimated_trajectory.ids.push_back(id);
      // DEBUG_VARS(id, _estimated_trajectory.data.back().space_point.point);
      // past_nodes--;
      // }
    }

    // for (int idx = 0; idx < node_idx; ++idx)
    for (auto idx : _active_nodes)
    {
      const std::size_t new_idx{ nodes_map[idx] };
      // DEBUG_VARS(idx, new_idx);
      if (new_idx != 0)
      {
        prx_models::Node& target_node{ _estimated_tree.nodes[new_idx] };
        prx_models::Node& parent_node{ _estimated_tree.nodes[target_node.parent] };
        const prx_models::Edge edge{ _tree_manager.create_edge(parent_node, target_node) };
        _estimated_tree.edges.push_back(edge);
      }
    }
  }

  bool query_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
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
    if (query_tf() and now >= finish_time)
    {
      _x_curr = _x_next;
      set_next_node();
      if (not _goal_reached)
      {
        const std::uint64_t parent_edge{ _tree.nodes[_x_next].parent_edge };
        const ml4kp_bridge::Plan& plan{ _tree.edges[parent_edge].plan };
        const double next_duration{ ml4kp_bridge::duration(plan).toSec() };

        SystemInterface::copy(_u_plan, plan.steps[0].control);  // DBG

        _key_dt = SystemInterface::keyT(_x_curr, _x_next);
        calculate_estimate_safe(_dt01, _key_dt);

        // const ros::Duration extra{};
        _x0_start_time = finish_time;
        _next_node_time = _x0_start_time + ros::Duration(_dt01);

        if (not _using_stepper)
        {
          add_tree_node();
        }

        // _current_past_nodes++;
      }
    }
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
      const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
      failure_to_file(msg + e.what());
      std::cout << msg << std::string(e.what()) << std::endl;

      // failure_to_file(e.what());

      // gtsam::Values current_estimate{ _isam.getLinearizationPoint() };
      // estimates_to_file<0>(std::cout, current_estimate, key, false);
      // const std::string msg{ "Clique of " + SF::formatter(key) };
      // _isam[key]->print(msg, SF::formatter);
    }
  }

  void print_covariance(const std::size_t id, const std::string message)
  {
    if (_x_curr < 11 or _x_curr > 30)
    {
      return;
    }
    const StateKeys keys{ SystemInterface::keyState(1, id) };
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

  bool add_observations()
  {
    const bool new_observation{ query_tf() };
    const bool header_updated{ _tf.header.stamp > _prev_header.stamp };
    // DEBUG_VARS(new_observation, header_updated, _last_local_goal);

    bool add_observation{ new_observation and header_updated };
    // DEBUG_VARS(new_observation, header_updated, !_last_local_goal);
    // DEBUG_VARS(add_observation);
    // if (new_observation and header_updated and not _last_local_goal)
    if (add_observation)
    {
      _prev_header = _tf.header;
      const double dt{ (_tf.header.stamp - _x0_start_time).toSec() };
      // if (dt < 0)
      //   return;

      SystemInterface::copy(_z_new, _tf);
      _time_remaining = (_next_node_time - ros::Time::now()).toSec();

      const GraphValues graph_values_z{ SystemInterface::add_observation_factor(_x_curr,
                                                                                _x_next,                 // no-lint
                                                                                _state_estimates, _u01,  // no-lint
                                                                                _z_new, dt, 0.01) };
      try
      {
        _isam2_result = _isam.update(graph_values_z.first, graph_values_z.second);

        // DEBUG_VARS(_x_curr, _isam2_result.newFactorsIndices);
        _inserted_factors[_x_curr].insert(_inserted_factors[_x_curr].end(),
                                          _isam2_result.newFactorsIndices.begin(),  // no-lint
                                          _isam2_result.newFactorsIndices.end());
        _isam2_result.newFactorsIndices.clear();

        _key_u01 = SystemInterface::keyU(_x_curr, _x_next);
        _key_dt = SystemInterface::keyT(_x_curr, _x_next);

        _u01 = _isam.calculateEstimate<Control>(_key_u01);
        _dt01 = _isam.calculateEstimate<double>(_key_dt);
      }
      catch (gtsam::IndeterminantLinearSystemException e)
      {
        const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
        failure_to_file(msg + e.what());
        std::cout << msg << std::string(e.what()) << std::endl;
      }

      // const StateKeys state_keys{ SystemInterface::keyState(1, _x_curr) };
      // update_estimates<0>(_state_estimates, state_keys);
      // update_estimates<0>(_node_estimates, _isam, state_keys);

      // SystemInterface::copy(_feedback.xhat, _state_estimates);
    }
    return add_observation;
  }

  void publish_control()
  {
    _next_node_time = _x0_start_time + ros::Duration(_dt01);

    ml4kp_bridge::copy(_control_stamped.space_point, _u01);
    _control_stamped.header.seq++;
    _control_stamped.header.stamp = ros::Time::now();
    _stamped_control_publisher.publish(_control_stamped);

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
        const StateKeys state_keys{ SystemInterface::keyState(1, id) };
        update_estimates<0>(estimates, _isam, state_keys);
        const State x{ std::get<0>(estimates) };
        const Eigen::RowVectorXd xdot{ std::get<1>(estimates).transpose() };
        const gtsam::Key ku{ SystemInterface::keyU(_tree.nodes[id].parent, id) };
        const ControlTranspose u_fg{ _isam.calculateEstimate<Control>(ku).transpose() };

        const std::uint64_t parent_edge{ _tree.nodes[id].parent_edge };
        const ml4kp_bridge::Plan& plan{ _tree.edges[parent_edge].plan };
        auto u_plan = plan.steps[0].control.point;
        auto sbmp_node = _tree.nodes[id].point.point;

        PRINT_MSG("---");
        DEBUG_VARS(id, x, xdot);
        DEBUG_VARS(sbmp_node);
        DEBUG_VARS(u_fg, u_plan);

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
      const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
      SystemInterface::state(_state, point);
      for (auto obstacle_info : _obstacle_collision_infos)
      {
        if (ObstacleFactor::close_enough(_state, _obstacle_factor_include_distance, obstacle_info, _robot_collision_ptr,
                                         _config_from_state, _obstacle_tolerance_result))
        {
          graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX, _obstacle_distance_tolerance,
                                               0.1, _obstacle_noise);

          _obstacles_marker.points.emplace_back();
          _obstacles_marker.points.back().x = _state[0];
          _obstacles_marker.points.back().y = _state[1];
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
      const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
      SystemInterface::state(_state, point);
      for (auto obstacle_info : _obstacle_collision_infos)
      {
        // ObstacleFactor::close_enough(_state, _obstacle_factor_include_distance, obstacle_info, _robot_collision_ptr,
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
      const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
      SystemInterface::state(_state, point);
      graph.emplace_shared<SdfFactor>(keyX, _obstacle_distance_tolerance, _sdf, _obstacle_noise);
    }
  }

  void set_next_node()
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

  void initialize()
  {
    const prx_models::Node& root_node{ _tree.nodes[_tree.root] };
    const GraphValues root_graph_values{ SystemInterface::root_to_fg(_tree.root, root_node.point) };
    _values.insert(root_graph_values.second);
    _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);

    _inserted_factors[_tree.root].insert(_inserted_factors[_tree.root].end(),  // no-lint
                                         _isam2_result.newFactorsIndices.begin(),
                                         _isam2_result.newFactorsIndices.end());

    _next_tree_edge = _tree.nodes[root_node.children[0]].parent_edge;
    for (int i = 0; i < _total_future_nodes; ++i)
    {
      add_tree_node();
    }
    _x_curr = _tree.root;
    set_next_node();
    PRINT_MSG("Stela Windowed Initialized");
    _tree_recevied = true;
    _key_dt = SystemInterface::keyT(_x_curr, _x_next);
    calculate_estimate_safe(_dt01, _key_dt);
    _start_time = ros::Time::now();
  }

  void insert_factors(const std::size_t id0, const std::size_t id1)
  {
    const StateKeys keys0{ SystemInterface::keyState(1, id0) };
    const StateKeys keys1{ SystemInterface::keyState(1, id1) };

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
      // DEBUG_VARS(factor_id, id_to_insert);
      _inserted_factors[id_to_insert].insert(_inserted_factors[id_to_insert].end(), factor_id);
    }
    // _isam.getFactorsUnsafe().print("Current Graph", SF::formatter);
  }

  void add_tree_node()
  {
    // DEBUG_VARS(_next_tree_edge);
    const prx_models::Edge& edge{ _tree.edges[_next_tree_edge] };
    // DEBUG_VARS(edge.index, edge.source, edge.target);
    const prx_models::Node& node_current{ _tree.nodes[edge.target] };
    const std::size_t total_children{ node_current.children.size() };

    if (_goal_id == node_current.index)
    {
      return;
    }

    if (_tree.root != edge.source)
    {
      const prx_models::Node& node_parent{ _tree.nodes[edge.source] };
      _active_nodes.insert(node_parent.index);
    }

    GraphValues graph_values{ SystemInterface::node_edge_to_fg(edge.source, edge.target, node_current.point,
                                                               edge.plan) };
    obstacle_factors(graph_values.first, node_current.point, edge.target);

    check_factor_removal();

    // print_factors_to_remove();
    safe_fg_update(graph_values.first, graph_values.second);

    _future_factors_queue.push_back(edge.source);
    insert_factors(edge.source, edge.target);

    // DEBUG_VARS(node_current);
    if (total_children > 0)
    {
      const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
      _next_tree_edge = node_child.parent_edge;
      // DEBUG_VARS(node_child, _next_tree_edge);
    }
    else
    {
      _goal_id = node_current.index;
    }
  }

  void safe_fg_update(const FactorGraph& graph, const Values& values)
  {
    try
    {
      SF::symbols_to_file("/Users/Gary/pracsys/catkin_ws/factor_graph_symbols.txt");
      _values.insert(values);
      _isam2_result = _isam.update(graph, values, _isam2_update_params);
      _isam2_update_params.removeFactorIndices.clear();
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      failure_to_file(e.what());
      prx::fg::indeterminant_linear_system_helper(graph, _values);
      std::cout << "[EXCEPTION] Var: " << SF::formatter(e.nearbyVariable()) << std::endl;
      // graph.printErrors(_values, "Problem graph", SF::formatter);
      // _values.print("Values", SF::formatter);
      throw e;
    }
    catch (gtsam::ValuesKeyDoesNotExist e)
    {
      failure_to_file(e.what());
      std::cout << "[EXCEPTION] Not found: " << SF::formatter(e.key()) << std::endl;
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
      const std::size_t past_factor_id{ _past_factors_queue.front() };
      const gtsam::FactorIndices& indices{ _inserted_factors[past_factor_id] };
      _isam2_update_params.removeFactorIndices.insert(_isam2_update_params.removeFactorIndices.end(),  // no-lint
                                                      indices.begin(), indices.end());

      _past_factors_queue.pop_front();
      _active_nodes.erase(past_factor_id);

      // DEBUG_VARS(past_factor_id);
      // print_factors_to_remove();
      node_info_to_file(past_factor_id);
      // _current_past_nodes--;
    }
  }

  void print_factors_to_remove() const
  {
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

  void remove_future_graph()
  {
    for (auto id_to_remove : _future_factors_queue)
    {
      if (id_to_remove == _x_curr)
      {
        continue;
      }
      // DEBUG_VARS(id_to_remove);
      const gtsam::FactorIndices& indices{ _inserted_factors[id_to_remove] };
      _isam2_update_params.removeFactorIndices.insert(_isam2_update_params.removeFactorIndices.end(), indices.begin(),
                                                      indices.end());

      _active_nodes.erase(id_to_remove);

      for (auto factor_id : indices)
      {
        const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor_id]->keys() };
        // PRINT_KEYS(factor_keys);
      }
    }

    const prx_models::Node& current_node{ _prev_tree.nodes[_x_curr] };
    const std::size_t child_id{ static_cast<std::size_t>(current_node.children[0]) };
    const StateKeys keys_child{ SystemInterface::keyState(1, child_id) };

    gtsam::FactorIndices& curr_indices{ _inserted_factors[_x_curr] };
    gtsam::FactorIndices new_indices{};
    // DEBUG_VARS(curr_indices);
    for (auto factor_id : curr_indices)
    {
      // const std::size_t factor_id{ *iter };
      // DEBUG_VARS(_trees_received, _x_curr, factor_id);
      const gtsam::KeyVector& factor_keys{ _isam.getFactorsUnsafe()[factor_id]->keys() };
      bool factor_found{ false };
      for (auto key : factor_keys)
      {
        auto kchild_res = std::find(keys_child.begin(), keys_child.end(), key);
        if (kchild_res != keys_child.end())
        {
          // const gtsam::FactorIndices& indices{ _inserted_factors[id_to_remove] };
          _isam2_update_params.removeFactorIndices.push_back(factor_id);
          // const gtsam::KeyVector& found_factor_keys{ _isam.getFactorsUnsafe()[factor_id]->keys() };
          // PRINT_KEYS(found_factor_keys);
          factor_found = true;
          // idx_iterators_to_remove(factor_id);
        }
      }
      if (not factor_found)
      {
        new_indices.push_back(factor_id);
      }
    }
    std::swap(new_indices, _inserted_factors[_x_curr]);
    // DEBUG_VARS(new_indices);
    // DEBUG_VARS(_inserted_factors[_x_curr]);

    _future_factors_queue.clear();
    // node_info_to_file(id);
  }

  void add_new_tree()
  {
    prx_models::Node& root_node{ _tree.nodes[_tree.root] };
    prx_models::Edge& edge{ _tree.edges[root_node.parent_edge] };
    // const prx_models::Node& child_node{ _tree.nodes[root_node.children[0]] };
    // const prx_models::Edge& edge{ _tree.edges[child_node.parent_edge] };
    // const GraphValues root_graph_values{ SystemInterface::root_to_fg(_tree.root, root_node.point) };
    // _values.insert(root_graph_values.second);
    // _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);

    // _inserted_factors[_tree.root].insert(_inserted_factors[_tree.root].end(),  // no-lint
    //                                      _isam2_result.newFactorsIndices.begin(),
    //                                      _isam2_result.newFactorsIndices.end());

    // DEBUG_VARS(_x_curr);
    // DEBUG_VARS(root_node);
    // // // DEBUG_VARS(child_node);
    // DEBUG_VARS(edge);
    // GraphValues graph_values{ SystemInterface::node_edge_to_fg(_x_curr, edge.target, root_node.point, edge.plan) };
    // obstacle_factors(graph_values.first, root_node.point, edge.target);

    _tree.nodes[_x_curr] = _prev_tree.nodes[_x_curr];
    _tree.nodes[_x_curr].children[0] = _tree.root;
    edge.source = _x_curr;
    _next_tree_edge = edge.index;
    root_node.parent = _x_curr;
    // _tree.nodes[_x_curr] = root_node;
    // DEBUG_VARS(edge);

    // _tree.edges[current_node.parent_edge] = _prev_tree.edges[current_node.parent_edge];
    // safe_fg_update(graph_values.first, graph_values.second);
    // // _future_factors_queue.push_back(_x_curr);
    // // DEBUG_VARS(_x_curr, edge.target)
    // insert_factors(_x_curr, edge.target);
    // _future_factors_queue.push_back(edge.target);

    // _next_tree_edge = _tree.nodes[root_node.children[0]].parent_edge;

    const std::size_t prev_goal_id{ _goal_id };
    while (_goal_id == prev_goal_id)
    {
      // DEBUG_VARS(prev_goal_id, _goal_id);
      add_tree_node();
    }
    _x_next = root_node.index;
  }

  void node_info_to_file(const std::size_t node_id)
  {
    _ofs << node_id << " ";
    const StateKeys keys{ SystemInterface::keyState(1, node_id) };
    update_values<0, StateEstimates>(_values, _isam, keys);
    estimates_to_file<StateEstimates, 0>(_ofs, _values, keys, _isam, true);
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    _prev_tree.copy(msg);
    std::swap(_tree, _prev_tree);

    // _tree.root = msg->root;
    // _tree.nodes = msg->nodes;
    // _tree.edges = msg->edges;

    if (not _tree_recevied)
    {
      const prx_models::Node& node_current{ _tree.nodes[_tree.root] };
      const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
      _next_tree_edge = node_child.parent_edge;
      initialize();
    }
    else
    {
      // DEBUG_VARS(_trees_received, _x_curr);
      // DEBUG_VARS(_trees_received, _x_curr);
      remove_future_graph();
      // DEBUG_VARS(_trees_received, _x_curr);
      add_new_tree();

      // set_next_node();
    }

    _prev_tree.clear();
    _trees_received++;
  }

private:
  Values _values;
  // FactorGraph _factor_graph;

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

  State _state;
  Control _u01;
  Control _u_plan;
  Observation _z_new;
  double _dt01;

  double _time_remaining;

  bool _goal_received;
  motion_planning::StelaGraphTraversalGoal _goal;

  std::uint64_t _x_curr;
  std::uint64_t _x_next;

  // std::vector<std::uint64_t> _selected_nodes;
  // std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;

  bool _tree_recevied;
  motion_planning::tree_manager_t _tree_manager;
  prx_models::Tree _estimated_tree;
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

  typename SystemInterface::ConfigFromState _config_from_state;
  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  typename ObstacleFactor::ToleranceResult _obstacle_tolerance_result;
  typename ObstacleFactor::DistanceResult _obstacle_distance_result;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;
  visualization_msgs::Marker _obstacles_marker;

  const std::string _name{};

  std::size_t _next_tree_edge;
  int _total_future_nodes;

  std::size_t _freq_counter;
  std::size_t _freq_total;
  double _freq_accum;

  std::size_t _goal_id;
  bool _goal_reached;
  bool _visualize;

  // std::size_t _current_past_nodes;
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

  std::size_t _trees_received;

  ros::Duration _max_observation_delay;
};
}  // namespace motion_planning
