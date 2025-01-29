#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <actionlib/server/simple_action_server.h>
#include <motion_planning/StelaGraphTraversalAction.h>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/factor_graphs/factors/obstacle_factor.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>

#include <motion_planning/sdf_factor.hpp>

#include <motion_planning/utils.hpp>
#include <utils/std_utils.hpp>
#include <utils/time_profiler.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class scate_t : public Base
{
  using Derived = scate_t<SystemInterface, Base>;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;
  using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;
  using StateKeys = typename SystemInterface::StateKeys;
  using StateEstimates = typename SystemInterface::StateEstimates;
  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename SystemInterface::ConfigFromState>;
  using LessThanCmp = prx::fg::VectorLessThanCmp<Control>;
  using GreaterThanCmp = prx::fg::VectorGreaterThanCmp<Control>;
  using MaxControlLimitFactor = prx::fg::constraint_factor_t<Control, GreaterThanCmp>;
  using MinControlLimitFactor = prx::fg::constraint_factor_t<Control, LessThanCmp>;
  using Sdf = utils::signed_distance_field_t;
  using SdfPtr = std::shared_ptr<Sdf>;
  using SdfFactor = motion_planning::sdf_factor_t<State, typename SystemInterface::ConfigFromState>;

  using SF = prx::fg::symbol_factory_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  scate_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, true, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _tf_listener(_tf_buffer)
    , _isam(_isam_params)
    , _isam_initialized(false)
    , _current_node(-1)
    , _fix_sigmas(1.0)
    , _obstacle_activation_distance(1.0)
    , _files_created(false)
    , _experiment_id("test")
    , _lm_params(prx::fg::default_levenberg_marquardt_parameters())
    , _init_lm_params(prx::fg::default_levenberg_marquardt_parameters())
    , _sim_clock(false)
    , _profiler()
    , _total_calls(0)
    , _total_z_calls(0)
    , _total_added_observations(0)
    , _tree_received(false)  
    , _is_verbose(false)
    , _no_obs(false)
  #ifdef GTSAM_USE_TBB
    , _tbb_control(tbb::global_control::max_allowed_parallelism, 8)
  #endif
    {
    }

  ~scate_t()
  {
    to_file();
  }

  virtual void onInit()
  {

    #ifdef GTSAM_USE_TBB
     std::cout << "Scate: Initializing with TBB control" << std::endl;
    #endif
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    std::string graph_topic_name{ "" };
    std::string control_topic;
    std::string collision_topic;
    double control_frequency, ctrl_computation_buffer, observation_frequency;
    double& obstacle_activation_distance{ _obstacle_activation_distance };
    double& fix_sigmas{ _fix_sigmas };
    double& obstacle_sigma{ _obstacle_sigma };
    std::vector<double> min_ctrl_limit;
    std::vector<double> max_ctrl_limit;
    std::string environment, solution_tree_topic, sbmp_tree_topic, replan_scate_topic;
    bool& time_factor{ _time_factor };
    bool& limit_ctrl{ _limit_controls };
    bool& verbose{ _is_verbose };
    bool& no_obs{ _no_obs };
    bool naive_guess{ false };
    int fg_iterations{ 100 };
    bool& sim_clock{ _sim_clock };

    std::string sdf_params;
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& obstacle_mode{ _obstacle_mode };
    std::string& output_dir{ _output_dir };
    std::string& experiment_id{ _experiment_id };
    bool lm_fixed_lambda;

    PARAM_SETUP(private_nh, solution_tree_topic);
    PARAM_SETUP(private_nh, ctrl_computation_buffer);
    PARAM_SETUP(private_nh, sbmp_tree_topic);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, control_frequency);
    PARAM_SETUP(private_nh, observation_frequency);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, obstacle_sigma)
    PARAM_SETUP(private_nh, obstacle_activation_distance)
    PARAM_SETUP(private_nh, environment);
    PARAM_SETUP(private_nh, obstacle_mode)
    PARAM_SETUP(private_nh, time_factor);
    PARAM_SETUP(private_nh, collision_topic)
    PARAM_SETUP(private_nh, limit_ctrl);
    PARAM_SETUP(private_nh, output_dir);
    PARAM_SETUP(private_nh, min_ctrl_limit);
    PARAM_SETUP(private_nh, max_ctrl_limit);
    PARAM_SETUP(private_nh, replan_scate_topic);
    PARAM_SETUP(private_nh, lm_fixed_lambda);
    PARAM_SETUP_WITH_DEFAULT(private_nh, verbose, verbose);
    PARAM_SETUP_WITH_DEFAULT(private_nh, no_obs, no_obs);
    PARAM_SETUP_WITH_DEFAULT(private_nh, sim_clock, sim_clock);
    PARAM_SETUP_WITH_DEFAULT(private_nh, fg_iterations, fg_iterations);
    PARAM_SETUP_WITH_DEFAULT(private_nh, naive_guess, naive_guess);
    PARAM_SETUP_WITH_DEFAULT(private_nh, fix_sigmas, fix_sigmas);
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id);
    PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)

    DEBUG_VARS(obstacle_mode);

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

    _is_sbmp_init = !naive_guess;
    _max_ctrl_comp_duration = (1.0 / control_frequency) + ctrl_computation_buffer;

    DEBUG_VARS(_is_sbmp_init);

    _min_ctrl_limit = Eigen::Map<Control>(min_ctrl_limit.data(), min_ctrl_limit.size());
    _max_ctrl_limit = Eigen::Map<Control>(max_ctrl_limit.data(), max_ctrl_limit.size());

    _lm_params.setUseFixedLambdaFactor(lm_fixed_lambda);
    _init_lm_params.setUseFixedLambdaFactor(lm_fixed_lambda);
    _lm_params.setMaxIterations(fg_iterations);
    _init_lm_params.setMaxIterations(400);
  
    if (!verbose) {
      _lm_params.setVerbosityLM("SILENT");
    }
    
    _obstacle_noise = gtsam::noiseModel::Isotropic::Sigma(1, obstacle_sigma);

    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };

    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _finish_publisher = private_nh.advertise<std_msgs::Bool>(finish_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);

    _tree_publisher = private_nh.advertise<prx_models::Tree>(solution_tree_topic, 1, true);

    if (_is_sbmp_init) {
      DEBUG_VARS(sbmp_tree_topic);
      _tree_subscriber = private_nh.subscribe(sbmp_tree_topic, 1, &Derived::init_from_sbmp, this);
      _name = "scate_sbmp"; 
    }
    else
    {
      int& total_states{ _total_states };
      std::vector<double>& init_ctrl{ _init_ctrl };
      PARAM_SETUP(private_nh, total_states);
      PARAM_SETUP(private_nh, init_ctrl);
      _name = "scate_naive";
    }
      
    _collision_subscriber = private_nh.subscribe(collision_topic, 1, &Derived::collision_callback, this);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "ScateControl";

    _prev_header.stamp = ros::Time::now();

    auto obstacles = prx::load_obstacles(environment);
    _obstacle_list = obstacles.second;

    const std::string plant_name{ SystemInterface::plant_name };
    _plant = prx::system_factory_t::create_system(plant_name, plant_name);
    _robot_collision_ptr = SystemInterface::collision_geometry();
    _obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

    double& delta_t{ _delta_t };
    PARAM_SETUP(private_nh, delta_t);

    utils::get_param_and_check(private_nh, "/Plant/start_state", _start_state);
    DEBUG_VARS("Start state: ", _start_state);
    
    utils::get_param_and_check(private_nh, "/Plant/goal/state", _goal_state);
    DEBUG_VARS("Goal state: ", _goal_state);

    _current_node = -1;
    
    DEBUG_VARS(observation_frequency);
    const ros::Duration observation_timer(1.0 / observation_frequency);
    DEBUG_VARS(observation_timer.toSec());
    _observation_timer = private_nh.createTimer(observation_timer, &Derived::observation_callback, this);

    if (!sim_clock) {
      DEBUG_VARS(control_frequency);
      const ros::Duration control_timer(1.0 / control_frequency);
      _control_timer = private_nh.createTimer(control_timer, &Derived::action_function, this);  
    }
    else {
      _replan_subscriber = private_nh.subscribe(replan_scate_topic, 1, &Derived::replan_callback, this);
      _control_stamped.space_point.point = std::vector<double>{0, 0};
      _control_stamped.header.seq++;
      _control_stamped.header.stamp = ros::Time::now();

      _stamped_control_publisher.publish(_control_stamped);
    }

    _timestamp = utils::timestamp();
    const std::string path{ _output_dir + "/" + _name };

    _profiler.set_filename(path + "_freq_" + _experiment_id + "_" + _timestamp + ".txt");
    _profiler.start();
    _profiler.checkpoint("Start");
    _profiler.end("End");
  }

  void observation_callback(const ros::TimerEvent& event)
  {
    if (_is_verbose) {
      std::cout << "Observation callback" << std::endl;
    }
    if (!is_initialized()) return;

    _total_z_calls++;
    bool added_new_obs = add_observations();

    if (added_new_obs) {
      _total_added_observations++;
    }

    if (_is_verbose and added_new_obs) {
      std::cout << "Added new observations" << std::endl;
    }
    
    _added_observations = _added_observations or added_new_obs;
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    ROS_WARN("Collision Detected! Creating file");
    if (msg->data)
    {
      to_file(true);
    }
  }

  void replan_callback(const std_msgs::EmptyConstPtr& msg)
  {
    if (_is_verbose) {
      std::cout << "Replan callback" << std::endl;
    }
    
    double computation_duration = update_fg_and_publish_controls(ros::Time::now());

    DEBUG_VARS(computation_duration);
  }

  void action_function(const ros::TimerEvent& event)
  {
    if (_is_verbose) {
      std::cout << "Action function" << std::endl;
    }

    double computation_duration = update_fg_and_publish_controls(event.current_real);

    // if (computation_duration > _max_ctrl_comp_duration)
    // {
    //   ROS_WARN("Computation duration: %f exceeded computation duration limit. Quitting ", computation_duration);
    //   to_file(false, true);
    // }

    if (_is_verbose) {
      DEBUG_VARS(computation_duration);
    }
  }

  double update_fg_and_publish_controls(const ros::Time event_timestamp) {
    _profiler.start();

    if (!is_initialized() && !_is_sbmp_init) {
      init_from_naive_guess();
      _isam_initialized = true;
    }

    if (is_initialized()) {
      bool is_node_updated = update_current_node(event_timestamp);

      if (_current_node < _total_states - 1) {
        _profiler.checkpoint();

        if (_added_observations) {
          optimize_fg();
        }
        bool should_publish_control = _sim_clock or (_added_observations or is_node_updated);
        publish_tree();

        if (should_publish_control){
          publish_control();
        }
      }

      _total_calls++;
    }

    _added_observations = false;

    ros::WallDuration computation_duration = _profiler.end();

    return computation_duration.toSec();
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

  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    using StateType = typename std::tuple_element<I, StateEstimates>::type;
    const gtsam::Key key{ keys[I] };
    const StateType state{ estimate.at<StateType>(key) };
    

    // ofs << i << " ";
    ofs << SF::formatter(key) << " ";
    for (int i = 0; i < state.size(); ++i)
    {
      ofs << state[i] << " ";
    }

    // try 
    // {
    //   const Eigen::MatrixXd cov{ _isam.marginalCovariance(key) };
    //   const Eigen::VectorXd diagonal{ cov.diagonal() };
      
    //   for (int i = 0; i < diagonal.size(); ++i)
    //   {
    //     ofs << diagonal[i] << " ";
    //   }
    // }
    // catch (std::out_of_range e)
    // {
    //   const std::string problem_key{ SF::formatter(key) };
    //   PRINT_MSG_VARS("Can't compute covariance", problem_key);
    //   DEBUG_VARS(e.what());
    // }

    estimates_to_file<I + 1>(ofs, estimate, keys);
    // ofs << "\n";
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    ofs << "\n";
  }

  void to_file(const bool collision = false, const bool ctrl_time_limit_exceeded = false)
  {
    if (_files_created)
      return;

    _collision_subscriber.shutdown();
    
    const std::string filename_data{ _output_dir + "/" + _name + "_data_" + _experiment_id + "_" + _timestamp + ".txt" };

    
    std::ofstream ofs_data(filename_data);

    const double elapsed_time{ (ros::Time::now() - _start_time).toSec() };
    const double avg_freq{ _total_calls / elapsed_time };
    const double avg_observation_freq{ static_cast<double>(_total_z_calls) / elapsed_time };
    const double avg_added_observation_freq{ static_cast<double>(_total_added_observations) / elapsed_time };

    ofs_data << "Initialized: " << (is_initialized() ? "true" : "false") << "\n";
    ofs_data << "ElapsedTime: " << elapsed_time << "\n";
    ofs_data << "Collision: " << (collision ? "true" : "false") << "\n";
    ofs_data << "ObstacleDistanceTolerance: " << _obstacle_activation_distance << "\n";
    ofs_data << "ObstacleMode: " << _obstacle_mode << "\n";
    ofs_data << "ExceptionRaised: " << "false" << "\n";
    ofs_data << "NetworkProblem: " << "false" << "\n";
    ofs_data << "AverageFrequency: " << avg_freq << "\n";
    ofs_data << "ObservationFrequency: " << avg_observation_freq << "\n";
    ofs_data << "SuccessfulObservationFrequency: " << avg_added_observation_freq << "\n";

    ofs_data.close();

    const std::string filename{ _output_dir + "/" + _name  + "_" + _experiment_id + "_" + _timestamp + ".txt" };
    std::ofstream ofs(filename);

    ofs << "# id key_x x[...] key_xdot xdot[...]\n";
    for (int node_id = 0; node_id< _total_states; ++node_id)
    {
      ofs << node_id << " ";
      const StateKeys keys{ SystemInterface::keyState(1, node_id) };
      estimates_to_file<0>(ofs, _current_estimate, keys);
    }

    ofs.close();

    if (_is_sbmp_init) {
      const std::string filename_branch_gt{ _output_dir + "/" + _name + "_branch_gt_" + _experiment_id + "_" + _timestamp + ".txt" };

      std::ofstream ofs_branch(filename_branch_gt);

      ofs_branch << "# id point[...]\n";
      for (int node_id = 0; node_id < _total_states; ++node_id)
      {
        // ofs << node_id << " ";
        // const StateKeys keys{ SystemInterface::keyState(1, node_id) };
        // estimates_to_file<StateEstimates, 0>(ofs, _values, keys, _isam, false);
        // ofs_covariance
        ofs_branch << node_id << " ";
        ml4kp_bridge::to_file(_sbmp_tree.nodes[node_id].point, ofs_branch);
        ofs_branch << "\n";
      }

      ofs_branch.close();
    }

    _files_created = true;

    _tree_received = false;

    std_msgs::Bool msg;
    msg.data = true;
    _finish_publisher.publish(msg);

    PRX_DBG_VARS(collision);

    ros::Rate rate(1);
    rate.sleep();
    ros::shutdown();
  }

  bool is_initialized() const { return _is_sbmp_init ? _tree_received : _isam_initialized; }

  bool update_current_node(const ros::Time& event_timestamp)
  {
    if (_current_node == -1)
    {
      _next_node_time_stamp = event_timestamp + ros::Duration(_delta_t);
      _current_node = 0;
      return true;
    }
    else if (_current_node >= _total_states - 1){
      ROS_WARN("Finished! Creating file");
      to_file();
      return false;
    }

    if (event_timestamp >= _next_node_time_stamp)
    {
      _current_node++;
      _next_node_time_stamp += ros::Duration(_delta_t);

      if (_is_verbose) {
        DEBUG_VARS(_current_node);
      }

      return true;
    }

    return false;
  }

  bool add_observations()
  {
    if (_current_node == -1)
      return false;

    const bool new_observation{ query_tf() };
    const bool header_updated{ _tf.header.stamp > _prev_header.stamp };

    if (new_observation and header_updated and !_no_obs)
    {
      _prev_header = _tf.header;

      Observation z_new;
      SystemInterface::copy(z_new, _tf);

      ros::Time current_node_time_stamp = _next_node_time_stamp - ros::Duration(_delta_t);

      double duration = (_tf.header.stamp - current_node_time_stamp).toSec();

      const GraphValues graph_values_z{ SystemInterface::add_observation_factor(_current_node, _current_node + 1, z_new, duration, 0.01) };

      _factor_graph += graph_values_z.first;
      _current_estimate.insert(graph_values_z.second);
      
      return true;
    }
    return false;
  }

  void optimize_fg()
  {
      gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _lm_params);

      _current_estimate = optimizer.optimize();
  }

  void publish_control()
  {
    const gtsam::Key uKey{ SystemInterface::keyU(_current_node, _current_node + 1) };
    const gtsam::Key xkey{ SystemInterface::keyX(1, _current_node) };
    const gtsam::Key xDotKey{ SystemInterface::keyXdot(1, _current_node) };
  

    const Control u{ _current_estimate.at<Control>(uKey) };
    
    
    const State x{ _current_estimate.at<State>(xkey) };
    const State xdot{ _current_estimate.at<StateDot>(xDotKey) };

    ml4kp_bridge::copy(_control_stamped.space_point, u);

    _control_stamped.header.seq++;
    _control_stamped.header.stamp = ros::Time::now();

    _stamped_control_publisher.publish(_control_stamped);

    if (_is_verbose) {
      DEBUG_VARS(u);
    }
  }

  void obstacle_factors(const ml4kp_bridge::SpacePoint& point, const int x_id)
  {
    if (_obstacle_mode == "all") {
      const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
      for (auto obstacle_info : _obstacle_collision_infos)
      {
        _obstacle_graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX,
                                                      _obstacle_activation_distance, 0.1, _obstacle_noise);
      }
    }

    if (_obstacle_mode == "sdf")
    {
      PRINT_MSG_ONCE("Using SDF Factors")
      const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
      _obstacle_graph.emplace_shared<SdfFactor>(keyX, _obstacle_activation_distance, _sdf, _obstacle_noise);
    }
  }

  void control_limit_factors(int i)
  {
    const gtsam::Key uKey{ SystemInterface::keyU(i, i + 1) };
    _control_limit_graph.emplace_shared<MinControlLimitFactor>(uKey, _min_ctrl_limit);
    _control_limit_graph.emplace_shared<MaxControlLimitFactor>(uKey, _max_ctrl_limit);
  }

  prx_models::Tree normalize_sbmp_tree(const prx_models::TreeConstPtr& orig_tree)
  {
    prx_models::Tree new_tree;

    // Get the original root node
    const prx_models::Node& orig_root_node{ orig_tree->nodes[orig_tree->root] };

    // Set the root node of the new tree
    new_tree.root = 0;
    prx_models::Node new_root_node;
    new_root_node.point = ml4kp_bridge::SpacePoint(orig_root_node.point);
    new_tree.nodes.emplace_back(new_root_node);
    
    // Get the first child that is not the root node    
    auto orig_child_id = orig_root_node.children[0];
    int node_idx = 0;
    while (orig_child_id == orig_tree->root and node_idx < orig_root_node.children.size()) {
      orig_child_id = orig_root_node.children[++node_idx];
    }
    if (orig_child_id == orig_tree->root) {
      throw std::runtime_error("Root node has no children");
    }

    std::uint64_t orig_edge_id = orig_tree->nodes[orig_child_id].parent_edge;

    node_idx = 1;
    int edge_idx = 0;
    bool add_edge = false;
    double previous_edge_duration = _delta_t, remaining_edge_time = 0;
    ml4kp_bridge::Plan new_edge_plan;
    ml4kp_bridge::SpacePoint new_node_point;
    Eigen::Vector2d accumulated_control = Eigen::Vector2d::Zero();


    while(true) {
      const prx_models::Edge& edge{ orig_tree->edges[orig_edge_id] };
      const prx_models::Node& node_parent{ orig_tree->nodes[edge.source] };
      const prx_models::Node& node_current{ orig_tree->nodes[edge.target] };

      const Eigen::Vector2d edge_control{edge.plan.steps[0].control.point.data()};
      double edge_duration = edge.plan.steps[0].duration.data.toSec();

      // std::cout << edge_control[0] << ";" << edge_control[1] << ";" << edge_duration << std::endl;
      
      prx_assert(edge_duration == _delta_t or previous_edge_duration == _delta_t, "Edge duration mismatch. At least one edge duration in consecutive edges should be equal to delta_t");
      prx_assert(edge_duration <= _delta_t, "Edge duration should be less than or equal to delta_t");

      if (remaining_edge_time == 0) {
        if (edge_duration == _delta_t) {
          add_edge = true;
          new_edge_plan = ml4kp_bridge::Plan(edge.plan);
          new_node_point = ml4kp_bridge::SpacePoint(node_current.point);

          remaining_edge_time = 0;
          accumulated_control = Eigen::Vector2d::Zero();
        }
        else {
          accumulated_control += edge_duration * edge_control;
          remaining_edge_time = _delta_t - edge_duration;
        }
      }
      else {
        Eigen::Vector2d new_edge_control;

        if (edge_duration < remaining_edge_time) {
          remaining_edge_time -= edge_duration;
          accumulated_control += edge_duration * edge_control;
        }
        else if (edge_duration == remaining_edge_time) {
          add_edge = true;
          new_edge_control = (accumulated_control + (remaining_edge_time * edge_control))/ _delta_t;
          ml4kp_bridge::PlanStep plan_step;

          new_node_point = ml4kp_bridge::SpacePoint(node_current.point);

          remaining_edge_time = 0;
          accumulated_control = Eigen::Vector2d::Zero();
        }
        else{
          add_edge = true;
          double unused_duration = edge_duration - remaining_edge_time;
    
          new_edge_control = (accumulated_control + (remaining_edge_time * edge_control))/ _delta_t;

          // Compute the new node position
          Eigen::Vector4d parent_node_vector, current_node_vector, new_node_vector;
          parent_node_vector << node_parent.point.point[0], node_parent.point.point[1], node_parent.point.point[2], node_parent.point.point[3];
          current_node_vector << node_current.point.point[0], node_current.point.point[1], node_current.point.point[2], node_current.point.point[3];
          new_node_vector = ((remaining_edge_time * parent_node_vector) + (unused_duration * current_node_vector)) / edge_duration;

          // Set the new node point
          new_node_point = ml4kp_bridge::SpacePoint();
          new_node_point.point.insert(new_node_point.point.end(), new_node_vector.data(), new_node_vector.data() + 4);
          new_node_point.point.push_back(0);

          // For the next edge
          remaining_edge_time = _delta_t - unused_duration;          
          // accumulated_control = unused_duration * edge_control;
          accumulated_control = edge_control * unused_duration;
        }

        if (add_edge) {
          new_edge_plan = ml4kp_bridge::Plan();
          ml4kp_bridge::PlanStep plan_step;
          plan_step.duration.data = ros::Duration(_delta_t);
          plan_step.control.point.insert(plan_step.control.point.end(), new_edge_control.data(), new_edge_control.data() + 2);
          new_edge_plan.steps.push_back(plan_step);
        }
        
      }

      if (add_edge) {
        prx_models::Edge edge;
        edge.index = edge_idx;
        edge.source = node_idx - 1;
        edge.target = node_idx;
        edge.plan = new_edge_plan;

        prx_models::Node node;
        node.index = node_idx;
        node.parent = node_idx - 1;
        node.parent_edge = edge_idx;
        node.point = new_node_point;

        new_tree.nodes[node_idx - 1].children.emplace_back(node_idx);
        new_tree.nodes.emplace_back(node);
        new_tree.edges.emplace_back(edge);

        node_idx++;
        edge_idx++;
        add_edge = false;
      }

      const std::size_t total_children{ node_current.children.size() };
      if (total_children == 0) {
        // Add the final edge with the remaining edge time
        if (remaining_edge_time > 0) {
          Eigen::Vector2d new_edge_control = (accumulated_control + (remaining_edge_time * edge_control)) / _delta_t;
          ml4kp_bridge::PlanStep plan_step;
          plan_step.duration.data = ros::Duration(_delta_t);
          plan_step.control.point.insert(plan_step.control.point.end(), new_edge_control.data(), new_edge_control.data() + 2);
          ml4kp_bridge::Plan new_edge_plan;
          new_edge_plan.steps.push_back(plan_step);

          prx_models::Edge edge;
          edge.index = edge_idx;
          edge.source = node_idx - 1;
          edge.target = node_idx;
          edge.plan = new_edge_plan;

          prx_models::Node node;
          node.index = node_idx;
          node.parent = node_idx - 1;
          node.parent_edge = edge_idx;
          node.point.point = _goal_state;

          new_tree.nodes[node_idx - 1].children.emplace_back(node_idx);
          new_tree.nodes.emplace_back(node);
          new_tree.edges.emplace_back(edge);
        }

        break;
      }

      const prx_models::Node& next_node_child{ orig_tree->nodes[node_current.children[0]] };
      orig_edge_id = next_node_child.parent_edge;
      previous_edge_duration = edge_duration;
    }

    return new_tree;
  }

  void init_from_sbmp(const prx_models::TreeConstPtr& msg)
  {
    std::cout << "Received SBMP tree" << std::endl;

    _sbmp_tree = normalize_sbmp_tree(msg);

    const prx_models::Node& root_node{ _sbmp_tree.nodes[_sbmp_tree.root] };
    _values = gtsam::Values();

    GraphValues root_graph_values{ SystemInterface::root_to_fg(_sbmp_tree.root, root_node.point) };


    auto child_id = root_node.children[0];

    // Start and goal states do not count as states in the FG
    int current_node = 0;
    _total_states = 1;

    const prx_models::Node& node_child{ _sbmp_tree.nodes[child_id] };
    std::uint64_t edge_id = _sbmp_tree.nodes[child_id].parent_edge;

    while(true) {
      const prx_models::Edge& edge{ _sbmp_tree.edges[edge_id] };
      const prx_models::Node& node_parent{ _sbmp_tree.nodes[edge.source] };
      const prx_models::Node& node_current{ _sbmp_tree.nodes[edge.target] };

      const std::size_t total_children{ node_current.children.size() };

      GraphValues graph_values;

      graph_values = SystemInterface::node_edge_to_fg(current_node, current_node+1, node_current.point, edge.plan, false);

      if (_sbmp_tree.root != node_current.index)
      {
        obstacle_factors(node_current.point, current_node+1);

        if (_limit_controls and total_children > 0) control_limit_factors(current_node+1);
      }

      root_graph_values.first += graph_values.first;
      root_graph_values.second.insert(graph_values.second);

      if (total_children == 0) {
        break;
      }

      const prx_models::Node& next_node_child{ _sbmp_tree.nodes[node_current.children[0]] };
      edge_id = next_node_child.parent_edge;

      current_node++;
    } 

    _total_states = current_node + 1;

    DEBUG_VARS(_total_states);

    SF::symbols_to_file("/Users/htnamus/All_Stuff/Programming_Stuff/ros_workspace/data/symbols.txt");

    // Adding goal state
    prx::space_t* ss{ _plant->get_state_space() };
    ml4kp_bridge::SpacePoint goal_state{};
    goal_state.point = _goal_state;
    GraphValues graph_values{ SystemInterface::fix_cost(_total_states-1, goal_state, _fix_sigmas) };
    root_graph_values.first += graph_values.first;
    root_graph_values.second.insert_or_assign(graph_values.second);

    // Updating with obstacle and control limit factors
    root_graph_values.first += _obstacle_graph;
    root_graph_values.first += _control_limit_graph;

    _factor_graph = root_graph_values.first;
    _current_estimate = root_graph_values.second;

    gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _init_lm_params);
    _current_estimate = optimizer.optimize();

    const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
        printCondition = [&](const gtsam::Factor*, double err, size_t) { return err > 0.1; };

    // _current_estimate.print("Initial estimate: ", SF::formatter);

    // _factor_graph.printErrors(_current_estimate, "Problem graph", SF::formatter, printCondition);

    _tree_received = true;
    _start_time = ros::Time::now();
  }

  void init_from_naive_guess()
  {
    prx::space_t* ss{ _plant->get_state_space() };
    ml4kp_bridge::SpacePoint state{};

    ml4kp_bridge::Plan plan;
    plan.steps.emplace_back();
    plan.steps[0].control.point = _init_ctrl;
    plan.steps[0].duration.data = ros::Duration(_delta_t);

    state.point = _start_state;
    GraphValues root_graph_values{ SystemInterface::fix_cost(0, state, _fix_sigmas) };
    if (_limit_controls) control_limit_factors(0);

    for (int i = 1; i < _total_states; ++i)
    {
      const gtsam::Key xkey{ SystemInterface::keyX(1, i) };
      const gtsam::Key xdotkey{ SystemInterface::keyXdot(1, i) };

      const double t{ i / static_cast<double>(_total_states) };
      ss->interpolate(_start_state, _goal_state, t, state.point);
      GraphValues graph_values{ SystemInterface::scate_fg(i - 1, i, state, plan, _time_factor) };

      root_graph_values.first += graph_values.first;
      root_graph_values.second.insert(graph_values.second);
      obstacle_factors(state, i);
      if (_limit_controls and i < _total_states-1) control_limit_factors(i);
    }

    ss->copy(state.point, _goal_state);
    GraphValues graph_values{ SystemInterface::fix_cost(_total_states-1, state, _fix_sigmas) };
    root_graph_values.first += graph_values.first;
    root_graph_values.first += _obstacle_graph;
    root_graph_values.first += _control_limit_graph;

    _factor_graph = root_graph_values.first;
    _current_estimate = root_graph_values.second;

    gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _init_lm_params);
    _current_estimate = optimizer.optimize();

    // const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
    //     printCondition = [&](const gtsam::Factor*, double err, size_t) { return err > 0.1; };

    // _current_estimate.print("Initial estimate: ", SF::formatter);

    // _factor_graph.printErrors(_current_estimate, "Problem graph", SF::formatter, printCondition);
    _start_time = ros::Time::now();
  }

  void publish_tree()
  {
    _tree_manager.reset();
    _tree.nodes.clear();
    _tree.edges.clear();
    _tree.root = 0;

    for (int i = _current_node; i < _total_states-1; ++i)
    {
      const gtsam::Key xkey{ SystemInterface::keyX(1, i) };
      const gtsam::Key xdotkey{ SystemInterface::keyXdot(1, i) };
      const State xi{ _current_estimate.at<State>(xkey) };
      const State xdoti{ _current_estimate.at<StateDot>(xdotkey) };

      if (i > _current_node)
      {
        prx_models::Node parent_node{ _tree.nodes.back() };
        _tree.nodes.push_back(std::move(_tree_manager.create_node()));
        prx_models::Edge edge{ _tree_manager.create_edge(parent_node, _tree.nodes.back()) };
        _tree.edges.push_back(edge);
      }
      else
      {
        _tree.nodes.push_back(std::move(_tree_manager.create_node()));
      }

      SystemInterface::copy(_tree.nodes.back().point, xi, xdoti);
    }

    _tree_publisher.publish(_tree);
  }

private:
  Values _values;
  FactorGraph _factor_graph, _obstacle_graph, _control_limit_graph;
  SdfPtr _sdf;

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;

  ml4kp_bridge::SpacePointStamped _control_stamped;
  prx_models::Tree _tree;

  ros::Publisher _control_publisher;
  ros::Publisher _stamped_control_publisher;
  ros::Publisher _tree_publisher;
  ros::Publisher _finish_publisher;
  ros::Subscriber _tree_subscriber;
  ros::Subscriber _collision_subscriber;
  ros::Subscriber _replan_subscriber;

  ros::Timer _control_timer;
  ros::Timer _observation_timer;

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  bool _isam_initialized;
  bool _is_sbmp_init;
  bool _is_verbose, _no_obs;
  bool _tree_received;
  bool _added_observations;
  motion_planning::tree_manager_t _tree_manager;
  int _current_node;

  // File/output
  bool _files_created;
  std::string _output_dir;
  std::string _experiment_id;

  std::size_t _total_calls, _total_z_calls, _total_added_observations;

  prx_models::Tree _sbmp_tree;

  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;

  double _obstacle_sigma;
  double _obstacle_activation_distance;
  double _max_ctrl_comp_duration;
  bool _limit_controls;
  Control _min_ctrl_limit;
  Control _max_ctrl_limit;
  gtsam::Values _current_estimate;
  gtsam::LevenbergMarquardtParams _lm_params, _init_lm_params;

  int _total_states;
  std::vector<double> _start_state;
  std::vector<double> _goal_state;
  std::vector<double> _init_ctrl;
  double _delta_t;
  ros::Time _next_node_time_stamp;
  ros::Time _start_time;
  
  bool _time_factor;
  bool _sim_clock;
  double _fix_sigmas;

  std::shared_ptr<prx::system_t> _plant;

  std::string _obstacle_mode;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;

  std::string _timestamp;
  std::string _name;
  utils::time_profiler_t _profiler;

  #ifdef GTSAM_USE_TBB
    tbb::global_control _tbb_control;
  #endif
};
}  // namespace motion_planning
