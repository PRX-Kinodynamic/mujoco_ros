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
#include <motion_planning/utils.hpp>
#include <motion_planning/sdf_factor.hpp>

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
    , _sim_clock(false)
    , _tree_received(false)  {};

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    std::string graph_topic_name{ "" };
    std::string control_topic;
    std::string collision_topic;
    double control_frequency;
    double& obstacle_activation_distance{ _obstacle_activation_distance };
    double& fix_sigmas{ _fix_sigmas };
    double& obstacle_sigma{ _obstacle_sigma };
    std::vector<double> min_control_limit;
    std::vector<double> max_control_limit;
    std::string environment, solution_tree_topic, sbmp_tree_topic, replan_scate_topic;
    bool& time_factor{ _time_factor };
    bool& limit_ctrl{ _limit_controls };
    bool naive_guess{ false };
    int fg_iterations{ 100 };
    bool& sim_clock{ _sim_clock };

    std::string sdf_params;
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& obstacle_mode{ _obstacle_mode };
    std::string& output_dir{ _output_dir };
    std::string& experiment_id{ _experiment_id };

    PARAM_SETUP(private_nh, solution_tree_topic);
    PARAM_SETUP(private_nh, sbmp_tree_topic);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, control_frequency);
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
    PARAM_SETUP(private_nh, min_control_limit);
    PARAM_SETUP(private_nh, max_control_limit);
    PARAM_SETUP(private_nh, replan_scate_topic);
    PARAM_SETUP_WITH_DEFAULT(private_nh, sim_clock, sim_clock);
    PARAM_SETUP_WITH_DEFAULT(private_nh, fg_iterations, fg_iterations);
    PARAM_SETUP_WITH_DEFAULT(private_nh, naive_guess, naive_guess);
    PARAM_SETUP_WITH_DEFAULT(private_nh, fix_sigmas, fix_sigmas);
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id);
    PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)

    DEBUG_VARS(obstacle_mode);

    if (obstacle_mode == "sdf")
    {
      // if (sdf_params == "")
      //   prx_throw("No SDF params!");
      // prx::param_loader sdf_param_loader{};
      // sdf_param_loader = Sdf::default_parameters();
      // sdf_param_loader.add_file(sdf_params);
      // sdf_param_loader["environment"].set(environment);
      // // ml4kp_bridge::check_for_ros_params(sdf_param_loader, private_nh);
      // _sdf = Sdf::create(sdf_param_loader);
    }

    _is_sbmp_init = !naive_guess;

    DEBUG_VARS(_is_sbmp_init);

    _min_control_limit = Eigen::Map<Control>(min_control_limit.data(), min_control_limit.size());
    _max_control_limit = Eigen::Map<Control>(max_control_limit.data(), max_control_limit.size());

    _lm_params.setUseFixedLambdaFactor(true);
    _lm_params.setMaxIterations(fg_iterations);
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

    double& init_duration{ _init_duration };
    PARAM_SETUP(private_nh, init_duration);

    utils::get_param_and_check(private_nh, "/Plant/start_state", _start_state);
    DEBUG_VARS("Start state: ", _start_state);
    
    utils::get_param_and_check(private_nh, "/Plant/goal/state", _goal_state);
    DEBUG_VARS("Goal state: ", _goal_state);
    
    if (naive_guess)
    {
      int& total_states{ _total_states };
      std::vector<double>& init_ctrl{ _init_ctrl };
      PARAM_SETUP(private_nh, total_states);
      PARAM_SETUP(private_nh, init_ctrl);
    }

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
  }

  void replan_callback(const std_msgs::EmptyConstPtr& msg)
  {
    update_fg_and_publish_controls(ros::Time::now());
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    std::cout << "Collision Detected" << std::endl;
    if (msg->data)
    {
      to_file(true);
    }
  }

  void action_function(const ros::TimerEvent& event)
  {
    update_fg_and_publish_controls(event.current_real);    
  }

  void update_fg_and_publish_controls(const ros::Time event_timestamp) {
    if (!is_initialized())
    {
      if (_is_sbmp_init){
        return;
      }
      else {
        init_from_naive_guess();
        _isam_initialized = true;
      }
    }
    
    bool is_node_updated = update_current_node(event_timestamp);
    if (_current_node >= _total_states - 1) return;

    bool added_observations = add_observations();
    bool should_publish_control = _sim_clock or (added_observations or is_node_updated);

    publish_tree();

    if (should_publish_control){
      publish_control();
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

  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    using StateType = typename std::tuple_element<I, StateEstimates>::type;
    const gtsam::Key key{ keys[I] };
    const StateType state{ estimate.at<StateType>(key) };
    const Eigen::MatrixXd cov{ _isam.marginalCovariance(key) };
    const Eigen::VectorXd diagonal{ cov.diagonal() };

    // ofs << i << " ";
    ofs << SF::formatter(key) << " ";
    for (int i = 0; i < diagonal.size(); ++i)
    {
      ofs << state[i] << " ";
    }
    for (int i = 0; i < diagonal.size(); ++i)
    {
      ofs << diagonal[i] << " ";
    }
    estimates_to_file<I + 1>(ofs, estimate, keys);
    // ofs << "\n";
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    ofs << "\n";
  }

  void to_file(const bool collision = false, const bool raised_exception = false)
  {
    if (_files_created)
      return;

    // const std::string filename{ _output_dir + "/scate_" + _experiment_id + "_" + utils::timestamp() + ".txt" };
    // const std::string filename_branch_gt{ _output_dir + "/scate_branch_gt_" + _experiment_id + "_" +
    //                                       utils::timestamp() + ".txt" };
    // const std::string filename_data{ _output_dir + "/scate_data_" + _experiment_id + "_" + utils::timestamp() +
    //                                  ".txt" };

    // // DEBUG_VARS(filename);
    // // DEBUG_VARS(filename_branch_gt);
    // // DEBUG_VARS(filename_data);
    // std::ofstream ofs(filename);
    // std::ofstream ofs_branch(filename_branch_gt);
    // std::ofstream ofs_data(filename_data);
    // const double elapsed_time{ (ros::Time::now() - _start_time).toSec() };
    // ofs_data << "ElapsedTime: " << elapsed_time << "\n";
    // ofs_data << "Collision: " << (collision ? "true" : "false") << "\n";
    // ofs_data << "ObstacleMode: " << _obstacle_mode << "\n";
    // ofs_data << "ExceptionRaised: " << (raised_exception ? "true" : "false") << "\n";

    // gtsam::Values estimate{ _isam.calculateEstimate() };
    // ofs << "# id key_x x[...] xCov[...] key_xdot xdot[...] xdotCov[...]\n";
    // ofs_branch << "# id point[...]\n";
    // for (int node_id = 0; node_id< _total_states; ++node_id)
    // {
    //   ofs << node_id << " ";
    //   const StateKeys keys{ SystemInterface::keyState(1, node_id) };
    //   estimates_to_file<0>(ofs, _current_estimate, keys);
    //   // const ml4kp_bridge::SpacePoint& {};
    //   ofs_branch << node_id << " ";
    //   ofs_branch << "\n";
    // }

    // ofs.close();
    // ofs_branch.close();
    // ofs_data.close();

    // _files_created = true;

    
    // const Control u {Control::Zero() };
    
    // ml4kp_bridge::copy(_control_stamped.space_point, u);

    // _control_stamped.header.seq++;
    // _control_stamped.header.stamp = ros::Time::now();

    // _stamped_control_publisher.publish(_control_stamped);

    // std_msgs::Bool msg;
    // msg.data = true;
    // _finish_publisher.publish(msg);

    // PRX_DBG_VARS(collision);

    ros::Rate rate(1);
    rate.sleep();
    ros::shutdown();
  }

  bool is_initialized() const { return _is_sbmp_init ? _tree_received : _isam_initialized; }

  bool update_current_node(const ros::Time& event_timestamp)
  {
    if (_current_node == -1)
    {
      _next_node_time_stamp = event_timestamp + ros::Duration(_init_duration);
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
      _next_node_time_stamp += ros::Duration(_init_duration);
      return true;
    }

    return false;
  }

  bool add_observations()
  {
    const bool new_observation{ query_tf() };
    const bool header_updated{ _tf.header.stamp > _prev_header.stamp };
    if (new_observation and header_updated)
    {
      _prev_header = _tf.header;

      Observation z_new;
      SystemInterface::copy(z_new, _tf);

      ros::Time current_node_time_stamp = _next_node_time_stamp - ros::Duration(_init_duration);

      double duration = (_tf.header.stamp - current_node_time_stamp).toSec();

      const GraphValues graph_values_z{ SystemInterface::add_observation_factor(_current_node, _current_node + 1, z_new, duration, 0.01) };

      _factor_graph += graph_values_z.first;
      _current_estimate.insert(graph_values_z.second);

      gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _lm_params);

      _current_estimate = optimizer.optimize();
      return true;
    }
    return false;
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
  }

  void obstacle_factors(const ml4kp_bridge::SpacePoint& point, const int x_id)
  {
    // if (_obstacle_mode == "all") {
    //   const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
    //   for (auto obstacle_info : _obstacle_collision_infos)
    //   {
    //     _obstacle_graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX,
    //                                                   _obstacle_activation_distance, 0.1, _obstacle_noise);
    //   }
    // }

    // if (_obstacle_mode == "sdf")
    // {
    //   PRINT_MSG_ONCE("Using SDF Factors")
    //   const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
    //   _obstacle_graph.emplace_shared<SdfFactor>(keyX, _obstacle_activation_distance, _sdf, _obstacle_noise);
    // }
  }

  void control_limit_factors(int i)
  {
    const gtsam::Key uKey{ SystemInterface::keyU(i, i + 1) };
    _control_limit_graph.emplace_shared<MinControlLimitFactor>(uKey, _min_control_limit);
    _control_limit_graph.emplace_shared<MaxControlLimitFactor>(uKey, _max_control_limit);
  }

  void init_from_sbmp(const prx_models::TreeConstPtr& msg)
  {
    std::cout << "Received SBMP tree" << std::endl;

    const ros::Time start{ ros::Time::now() };
    const std::size_t total_tree_nodes{ msg->nodes.size() };

    const prx_models::Node& root_node{ msg->nodes[msg->root] };
    _values = gtsam::Values();

    GraphValues root_graph_values{ SystemInterface::root_to_fg(msg->root, root_node.point) };


    auto child_id = root_node.children[0];
    int idx = 0;

    while (child_id == msg->root and idx < root_node.children.size()) {
      child_id = root_node.children[++idx];
    }

    if (child_id == msg->root) {
      throw std::runtime_error("Root node has no children");
    }

    // Start and goal states do not count as states in the FG
    _total_states = 1;

    const prx_models::Node& node_child{ msg->nodes[child_id] };
    std::uint64_t edge_id = node_child.parent_edge;

    while(true) {
      _total_states++;
      const prx_models::Edge& edge{ msg->edges[edge_id] };
      const prx_models::Node& node_parent{ msg->nodes[edge.source] };
      const prx_models::Node& node_current{ msg->nodes[edge.target] };

      const std::size_t total_children{ node_current.children.size() };

      GraphValues graph_values;

      graph_values = SystemInterface::node_edge_to_fg(edge.source, edge.target, node_current.point, edge.plan);

      if (msg->root != node_current.index)
      {
        obstacle_factors(node_current.point, edge.target);

        if (_limit_controls and total_children > 0) control_limit_factors(edge.target);
      }

      root_graph_values.first += graph_values.first;
      root_graph_values.second.insert(graph_values.second);

      if (total_children == 0) {
        break;
      }

      const prx_models::Node& next_node_child{ msg->nodes[node_current.children[0]] };
      edge_id = next_node_child.parent_edge;
    }

    DEBUG_VARS(_total_states);

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


    gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _lm_params);
    _current_estimate = optimizer.optimize();

    const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
        printCondition = [&](const gtsam::Factor*, double err, size_t) { return err > 0.1; };

    // _current_estimate.print("Initial estimate: ", SF::formatter);

    // _factor_graph.printErrors(_current_estimate, "Problem graph", SF::formatter, printCondition);

    _tree_received = true;
  }

  void init_from_naive_guess()
  {
    prx::space_t* ss{ _plant->get_state_space() };
    ml4kp_bridge::SpacePoint state{};

    ml4kp_bridge::Plan plan;
    plan.steps.emplace_back();
    plan.steps[0].control.point = _init_ctrl;
    plan.steps[0].duration.data = ros::Duration(_init_duration);

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

    gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _lm_params);
    _current_estimate = optimizer.optimize();

    // const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
    //     printCondition = [&](const gtsam::Factor*, double err, size_t) { return err > 0.1; };

    // _current_estimate.print("Initial estimate: ", SF::formatter);

    // _factor_graph.printErrors(_current_estimate, "Problem graph", SF::formatter, printCondition);
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

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  bool _isam_initialized;
  bool _is_sbmp_init;
  bool _tree_received;
  motion_planning::tree_manager_t _tree_manager;
  int _current_node;

  // File/output
  bool _files_created;
  std::string _output_dir;
  std::string _experiment_id;

  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;

  double _obstacle_sigma;
  double _obstacle_activation_distance;
  bool _limit_controls;
  Control _min_control_limit;
  Control _max_control_limit;
  gtsam::Values _current_estimate;
  gtsam::LevenbergMarquardtParams _lm_params;

  int _total_states;
  std::vector<double> _start_state;
  std::vector<double> _goal_state;
  std::vector<double> _init_ctrl;
  double _init_duration;
  ros::Time _next_node_time_stamp;
  ros::Time _start_time;
  
  bool _time_factor;
  bool _sim_clock;
  double _fix_sigmas;

  std::shared_ptr<prx::system_t> _plant;

  std::string _obstacle_mode;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;
};
}  // namespace motion_planning
