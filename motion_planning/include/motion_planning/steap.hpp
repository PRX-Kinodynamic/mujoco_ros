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

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class steap_t : public Base
{
  using Derived = steap_t<SystemInterface, Base>;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;
  using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;
  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename SystemInterface::configuration_from_state>;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  steap_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, true, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _tf_listener(_tf_buffer)
    , _isam(_isam_params)
    , _isam_initialized(false)
    , _obstacle_distance_tolerance(1){};

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string tree_topic_name;
    std::string graph_topic_name{ "" };
    std::string control_topic;
    double control_frequency;
    double& obstacle_distance_tolerance{ _obstacle_distance_tolerance };
    double& obstacle_sigma{ _obstacle_sigma };

    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    // ROS_PARAM_SETUP(private_nh, random_seed);
    // ROS_PARAM_SETUP(private_nh, plant_config_file);
    // ROS_PARAM_SETUP(private_nh, planner_config_file);
    // PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, control_frequency);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, obstacle_sigma)
    PARAM_SETUP(private_nh, obstacle_distance_tolerance)
    PARAM_SETUP_WITH_DEFAULT(private_nh, graph_topic_name, graph_topic_name);

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };

    const ros::Duration control_timer(1.0 / control_frequency);
    _control_timer = private_nh.createTimer(control_timer, &Derived::action_function, this);
    // _ol_timer = private_nh.createTimer(control_timer, &Derived::control_timer_callback, this);

    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);
    if (graph_topic_name != "")
      _graph_subscriber = private_nh.subscribe(graph_topic_name, 1, &Derived::graph_callback, this);

    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);

    const std::string steap_tree_topic_name{ ros::this_node::getNamespace() + "/steap/tree" };
    _tree_publisher = private_nh.advertise<prx_models::Tree>(steap_tree_topic_name, 1);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "SteapControl";

    _prev_header.stamp = ros::Time::now();

    bool naive_guess{ false };
    PARAM_SETUP_WITH_DEFAULT(private_nh, naive_guess, naive_guess);

    std::string environment;
    PARAM_SETUP(private_nh, environment);
    auto obstacles = prx::load_obstacles(environment);
    _obstacle_list = obstacles.second;
    DEBUG_VARS(environment, _obstacle_list.size());

    if (naive_guess)
    {
      int& total_states{ _total_states };
      std::vector<double> start;
      std::vector<double> goal;
      std::vector<double> init_ctrl;
      double init_duration;

      PARAM_SETUP(private_nh, total_states);
      PARAM_SETUP(private_nh, start);
      PARAM_SETUP(private_nh, goal);
      PARAM_SETUP(private_nh, init_ctrl);
      PARAM_SETUP(private_nh, init_duration);

      init_from_naive_guess(total_states, start, goal, init_ctrl, init_duration);
      _isam_initialized = true;
    }
  }

  void action_function(const ros::TimerEvent& event)
  {
    if (_isam_initialized)
    {
      // add_observations();
      // publish_control();
      publish_tree();
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
  void update_next_goal(const double dt)
  {
    _time_remaining = _time_remaining - dt;
    if (_time_remaining < 0.00001)
    {
      // _time_remaining = _goal.durations[_local_goal_id];
      // _local_goal_id++;
      // _x_next = _goal.selected_branch[_local_goal_id];
      // DEBUG_VARS(_time_remaining, _x_next);
    }
  }

  void add_observations()
  {
    const bool new_observation{ query_tf() };
    if (new_observation and _tf.header.stamp > _prev_header.stamp)
    {
      const double dt{ (_tf.header.stamp - _prev_header.stamp).toSec() };
      SystemInterface::copy(_z_new, _tf);
      update_next_goal(dt);  // A big (negative) number to force updates

      // const std::uint64_t x1{ _local_goal.selected_branch[1] };
      const GraphValues graph_values_z{ SystemInterface::add_observation_factor(_id_x_hat,
                                                                                _id_x_hat + 1,            // no-lint
                                                                                _x_hat, _xdot_hat, _u01,  // no-lint
                                                                                _z_new, dt, 0.01) };
      const GraphValues graph_values_1{ SystemInterface::local_adaptation(_id_x_hat + 1, _x_next, _u01,
                                                                          _time_remaining) };
      prx::fg::symbol_factory_t::symbols_to_file();

      _isam2_result = _isam.update(graph_values_z.first, graph_values_z.second);
      _isam2_result = _isam.update(graph_values_1.first, graph_values_1.second);

      _id_x_hat++;

      _key_x_hat = SystemInterface::keyX(-1, _id_x_hat);
      _key_xdot_hat = SystemInterface::keyXdot(-1, _id_x_hat);
      _key_u01 = SystemInterface::keyU(-1 * _id_x_hat, _x_next);
      // DEBUG_VARS(_key_x_hat, prx::fg::symbol_factory_t::formatter(_key_x_hat))
      // DEBUG_VARS(_key_xdot_hat, prx::fg::symbol_factory_t::formatter(_key_xdot_hat))
      // DEBUG_VARS(_key_u01, prx::fg::symbol_factory_t::formatter(_key_u01))

      _x_hat = _isam.calculateEstimate<State>(_key_x_hat);
      _xdot_hat = _isam.calculateEstimate<StateDot>(_key_xdot_hat);

      // SystemInterface::copy(_feedback.xhat, _x_hat, _xdot_hat);

      _prev_header = _tf.header;
    }
  }

  void publish_control()
  {
    _u01 = _isam.calculateEstimate<Control>(_key_u01);

    ml4kp_bridge::copy(_control_stamped.space_point, _u01);
    _control_stamped.header.seq++;
    _control_stamped.header.stamp = ros::Time::now();
    _stamped_control_publisher.publish(_control_stamped);
  }

  void add_obstacle_factor(FactorGraph& graph, int state_id)
  {
    using CollisionInfoPtr = std::shared_ptr<prx::fg::collision_info_t>;

    const gtsam::Key xkey{ SystemInterface::keyX(1, state_id) };
    using SF = prx::fg::symbol_factory_t;

    // DEBUG_VARS(SF::formatter(xkey));

    gtsam::noiseModel::Base::shared_ptr obstacle_noise{ gtsam::noiseModel::Isotropic::Sigma(2, _obstacle_sigma) };
    for (auto obstacle : _obstacle_list)
    {
      prx::movable_object_t::Geometries geometries{ obstacle->get_geometries() };
      prx::movable_object_t::Configurations configurations{ obstacle->get_configurations() };

      const std::size_t total_geoms{ geometries.size() };
      // DEBUG_VARS(total_geoms);

      for (int i = 0; i < total_geoms; ++i)
      {
        std::shared_ptr<prx::geometry_t> g{ geometries[i].second };
        std::shared_ptr<prx::transform_t> tf{ configurations[i].second };

        const prx::geometry_type_t g_type{ g->get_geometry_type() };
        const std::vector<double> g_params{ g->get_geometry_params() };

        const Eigen::Matrix3d rot{ tf->rotation() };
        const Eigen::Vector3d t{ tf->translation() };

        // CollisionInfoPtr obstacle_ptr{ std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t) };

        // DEBUG_VARS(t.transpose());

        // graph.emplace_shared<ObstacleFactor>(obstacle_ptr, _robot_collision_ptr, xkey);
        graph.emplace_shared<ObstacleFactor>(g_type, g_params, rot, t,  // no-lint
                                             _robot_collision_ptr->geom_type, _robot_collision_ptr->params, xkey,
                                             _obstacle_distance_tolerance, 0.01, obstacle_noise);
      }
    }
    // _isam2_result = _isam.update(graph, values);
  }

  void init_from_naive_guess(const std::size_t total_states, std::vector<double> start, std::vector<double> goal,
                             const std::vector<double> init_ctrl, const double init_duration)
  {
    ml4kp_bridge::Plan plan;
    ml4kp_bridge::SpacePoint state;

    state.point = start;

    const std::string plant_name{ SystemInterface::plant_name };
    std::shared_ptr<prx::plant_t> plant{ prx::system_factory_t::create_system_as<prx::plant_t>(plant_name,
                                                                                               plant_name) };
    prx_assert(plant != nullptr, "Plant " << plant_name << " couldn't be constructed!");

    prx::movable_object_t::Geometries geometries{ plant->get_geometries() };
    prx::movable_object_t::Configurations configurations{ plant->get_configurations() };

    const std::size_t total_geoms{ geometries.size() };
    prx_assert(total_geoms == 1, "More than 1 geometry not supported");
    std::shared_ptr<prx::geometry_t> g{ geometries[0].second };
    std::shared_ptr<prx::transform_t> tf{ configurations[0].second };

    const prx::geometry_type_t g_type{ g->get_geometry_type() };
    const std::vector<double> g_params{ g->get_geometry_params() };

    const Eigen::Matrix3d rot{ tf->rotation() };
    const Eigen::Vector3d t{ tf->translation() };

    // CollisionInfoPtr obstacle_ptr{ std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t) };
    // graph.emplace_shared<ObstacleFactor>(obstacle_ptr, robot, kr);

    _robot_collision_ptr = std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t);

    prx::space_t* space{ plant->get_state_space() };
    GraphValues root_graph_values{ SystemInterface::root_to_fg(0, state) };

    _factor_graph += root_graph_values.first;
    _values.insert(root_graph_values.second);
    add_obstacle_factor(root_graph_values.first, 0);
    // _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);

    plan.steps.emplace_back();
    plan.steps[0].control.point = init_ctrl;
    plan.steps[0].duration.data = ros::Duration(init_duration);

    state.point = start;
    root_graph_values.first += SystemInterface::add_fix_cost(0, state).first;
    // _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);

    state.point = start;
    for (int i = 1; i <= total_states; ++i)
    {
      const double ti = i / static_cast<double>(total_states);
      space->interpolate(start, goal, ti, state.point);

      GraphValues graph_values{ SystemInterface::trajectory_to_fg(i - 1, i, state, plan) };
      // GraphValues graph_values{ SystemInterface::node_edge_to_fg(i - 1, i, state, plan) };
      // DEBUG_VARS(i, ti, state.point);
      add_obstacle_factor(graph_values.first, i);
      root_graph_values.first += graph_values.first;
      root_graph_values.second.insert(graph_values.second);
      // _isam2_result = _isam.update(graph_values.first, graph_values.second);

      // _feedback.lookahead.edges.push_back(std::move(edge));
    }

    state.point = goal;
    // GraphValues graph_values{ SystemInterface::add_fix_cost(total_states, state) };
    root_graph_values.first += SystemInterface::add_fix_cost(total_states, state).first;
    // _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);
    // SystemInterface::add_fix_cost(total_states, state);

    gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
    lm_params.setUseFixedLambdaFactor(true);

    int lm_iters{ 0 };
    PARAM_SETUP(Base::getPrivateNodeHandle(), lm_iters)
    lm_params.setMaxIterations(lm_iters);

    gtsam::LevenbergMarquardtOptimizer optimizer(root_graph_values.first, root_graph_values.second, lm_params);

    _current_estimate = optimizer.optimize();
    // std::ofstream ofs_map("/Users/Gary/pracsys/catkin_ws/steap.txt");
    // for (auto factor : root_graph_values.first)
    // {
    //   auto factor_obstacle = boost::dynamic_pointer_cast<ObstacleFactor>(factor);
    //   if (factor_obstacle)
    //   {
    //     factor_obstacle->to_stream(ofs_map, root_graph_values.second);
    //     factor_obstacle->to_stream(ofs_map, _current_estimate);
    //   }
    // }
    // DEBUG_VARS("LevenbergMarquardtOptimizer finished");
    // _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);
    _current_node = 0;
  }

  void publish_tree()
  {
    // _current_estimate = _isam.calculateEstimate();

    _tree_manager.reset();
    _tree.nodes.clear();
    _tree.edges.clear();
    _tree.root = 0;

    for (int i = _current_node; i < _total_states; ++i)
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
      // _tree.nodes.back().point.point = state.point;
    }

    _tree_publisher.publish(_tree);
  }

  void branch_to_traj(const prx_models::TreeConstPtr msg, const std::uint64_t edge_id)
  {
    const prx_models::Edge& edge{ msg->edges[edge_id] };
    const prx_models::Node& node_parent{ msg->nodes[edge.source] };
    const prx_models::Node& node_current{ msg->nodes[edge.target] };

    // const ml4kp_bridge::SpacePoint& node_state{};

    const GraphValues graph_values{ SystemInterface::node_edge_to_fg(edge.source, edge.target, node_current.point,
                                                                     edge.plan) };
    _factor_graph += graph_values.first;
    _values.insert(graph_values.second);

    _isam2_result = _isam.update(graph_values.first, graph_values.second);
    // DEBUG_VARS(_isam2_result.getErrorBefore(), _isam2_result.getErrorAfter());

    const std::size_t total_children{ node_current.children.size() };

    if (total_children >= 1)  // This is not a leaf
    {
      for (int i = 0; i < total_children; ++i)
      {
        const prx_models::Node& node_child{ msg->nodes[node_current.children[i]] };

        branch_to_traj(msg, node_child.parent_edge);
      }
      // branch_to_traj(msg, node.children[0], traj_id);
    }
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    _isam.clear();
    _values = gtsam::Values();
    _factor_graph = gtsam::NonlinearFactorGraph();
    const prx_models::Node& node{ msg->nodes[msg->root] };

    for (auto child_id : node.children)
    {
      if (child_id == msg->root)  // The root node is its own parent :/
        continue;
      const prx_models::Node& node_child{ msg->nodes[child_id] };

      const GraphValues graph_values{ SystemInterface::root_to_fg(msg->root, node.point) };

      _factor_graph += graph_values.first;
      _values.insert(graph_values.second);
      _isam2_result = _isam.update(graph_values.first, graph_values.second);

      // _key_last_esimated_state = SystemInterface::keyX(1, msg->root);
      // DEBUG_VARS(_isam2_result.getErrorBefore(), _isam2_result.getErrorAfter());

      branch_to_traj(msg, node_child.parent_edge);
    }
    // const Values values{ _isam.calculateBestEstimate() };
    // const FactorGraph& factor_graph{ _isam.getFactorsUnsafe() };
    // const NonlinearFactorGraph& getFactorsUnsafe() const {
    // SystemInterface::to_file("/Users/Gary/pracsys/catkin_ws/fg_isam.txt", factor_graph, values,
    // std::ofstream::trunc); SystemInterface::to_file("/Users/Gary/pracsys/catkin_ws/fg_raw.txt", _factor_graph,
    // _values, std::ofstream::trunc);

    _id_x_hat = msg->root;
    const GraphValues graph_values{ SystemInterface::root_to_fg(_id_x_hat, node.point, true) };
    _isam2_result = _isam.update(graph_values.first, graph_values.second);

    _key_x_hat = SystemInterface::keyX(-1, _id_x_hat);
    _key_xdot_hat = SystemInterface::keyXdot(-1, _id_x_hat);

    _x_hat = _isam.calculateEstimate<State>(_key_x_hat);
    _xdot_hat = _isam.calculateEstimate<StateDot>(_key_xdot_hat);

    // _isam.print("isam", prx::fg::symbol_factory_t::formatter);
    // get_current_control();
    // std::ofstream ofs("/Users/Gary/pracsys/catkin_ws/bayes_tree.dot");
    // _isam.dot(ofs, prx::fg::symbol_factory_t::formatter);
    //
    _tree.root = msg->root;
    _tree.nodes = msg->nodes;
    _tree.edges = msg->edges;
    // _stela_action_server->start();
    _isam_initialized = true;
    ROS_INFO("Action server started");
  }

  void graph_callback(const prx_models::GraphConstPtr msg)
  {
    _isam.clear();
    _values = gtsam::Values();
    _factor_graph = gtsam::NonlinearFactorGraph();

    for (auto edge : msg->edges)
    {
      const GraphValues graph_values{ SystemInterface::graph_edge_to_factor_graph(edge.source, edge.target,
                                                                                  edge.plan) };
      _factor_graph += graph_values.first;
      _values.insert(graph_values.second);
    }

    for (auto node : msg->nodes)
    {
      const GraphValues graph_values{ SystemInterface::graph_node_to_factor_graph(node.index, node.point) };
      _factor_graph += graph_values.first;
      _values.insert(graph_values.second);
    }

    prx::fg::symbol_factory_t::symbols_to_file("/Users/Gary/pracsys/catkin_ws/symbols.txt");

    // SystemInterface::to_file("/Users/Gary/pracsys/catkin_ws/fg_isam.txt", factor_graph, values,
    // std::ofstream::trunc);
    SystemInterface::to_file("/Users/Gary/pracsys/catkin_ws/fg_raw.txt", _factor_graph, _values, std::ofstream::trunc);
    _isam2_result = _isam.update(_factor_graph, _values);
    const double ErrorBefore{ _isam2_result.getErrorBefore() };
    const double ErrorAfter{ _isam2_result.getErrorAfter() };
    DEBUG_VARS(ErrorBefore, ErrorAfter);
    // while (not query_tf())
    // {
    //   _id_x_hat = 0;
    // }

    // const GraphValues graph_values{ SystemInterface::root_to_fg(_id_x_hat, node.point, true) };
    // _isam2_result = _isam.update(graph_values.first, graph_values.second);

    // _key_x_hat = SystemInterface::keyX(-1, _id_x_hat);
    // _key_xdot_hat = SystemInterface::keyXdot(-1, _id_x_hat);

    // _x_hat = _isam.calculateEstimate<State>(_key_x_hat);
    // _xdot_hat = _isam.calculateEstimate<StateDot>(_key_xdot_hat);

    // // _isam.print("isam", prx::fg::symbol_factory_t::formatter);
    // // get_current_control();
    // // std::ofstream ofs("/Users/Gary/pracsys/catkin_ws/bayes_tree.dot");
    // // _isam.dot(ofs, prx::fg::symbol_factory_t::formatter);
    // //
    // _stela_action_server->start();
    ROS_INFO("Action server started");
  }

private:
  Values _values;
  FactorGraph _factor_graph;

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;

  gtsam::Key _key_last_esimated_state;
  gtsam::Key _key_x_hat;
  gtsam::Key _key_xdot_hat;
  gtsam::Key _key_u01;

  ml4kp_bridge::SpacePointStamped _control_stamped;
  prx_models::Tree _tree;

  ros::Subscriber _tree_subscriber;
  ros::Subscriber _graph_subscriber;

  ros::Publisher _control_publisher;
  ros::Publisher _stamped_control_publisher;
  ros::Publisher _tree_publisher;

  ros::Timer _control_timer;

  ros::ServiceServer _x_key_query_service;
  ros::ServiceServer _xdot_key_query_service;

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  std::size_t _id_x_hat;
  std::size_t _id_x1;
  State _x_hat;
  StateDot _xdot_hat;
  Control _u01;
  Observation _z01;
  Observation _z_new;
  double _time_remaining;

  bool _used_x2;

  std::uint64_t _x_next;
  std::size_t _local_goal_id;

  bool _isam_initialized;
  motion_planning::tree_manager_t _tree_manager;
  int _current_node;
  int _total_states;

  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;

  double _obstacle_sigma;
  double _obstacle_distance_tolerance;
  gtsam::Values _current_estimate;
};
}  // namespace motion_planning