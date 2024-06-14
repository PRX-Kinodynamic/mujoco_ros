#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <utils/std_utils.cpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_planning/StelaGraphTraversalAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
namespace motion_planning
{

namespace stela_types
{
// Function pointers: define the signature of the function to be taken as template parameter
using RootToFGFn = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> (*)(const std::size_t,
                                                                             const ml4kp_bridge::SpacePoint&);
using NodeEdgeToFGFn = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> (*)(const std::size_t, const std::size_t,
                                                                                 const ml4kp_bridge::SpacePoint&,
                                                                                 const ml4kp_bridge::Plan&);
using ToFileFn = void (*)(const std::string, const gtsam::NonlinearFactorGraph&, const gtsam::Values&,
                          const std::ios_base::openmode);

}  // namespace stela_types

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class stela_t : public Base
{
  using Derived = stela_t<SystemInterface, Base>;
  using StelaActionServer = actionlib::SimpleActionServer<motion_planning::StelaGraphTraversalAction>;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;
  using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  stela_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, true, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _tf_listener(_tf_buffer)
    , _isam(_isam_params)
    , _tree_recevied(false)
    , _last_local_goal(true)
  {
    for (int i = 0; i < 36; ++i)
    {
      _pose_with_cov.pose.covariance[i] = 0.0;
    }
    _pose_with_cov.header.frame_id = "world";
  };
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string tree_topic_name;
    std::string graph_topic_name{ "" };
    std::string control_topic;
    double control_frequency;

    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& output_dir{ _output_dir };
    // ROS_PARAM_SETUP(private_nh, random_seed);
    // ROS_PARAM_SETUP(private_nh, plant_config_file);
    // ROS_PARAM_SETUP(private_nh, planner_config_file);
    PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, control_frequency);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, output_dir)
    PARAM_SETUP_WITH_DEFAULT(private_nh, graph_topic_name, graph_topic_name);

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string pose_with_cov_topic{ ros::this_node::getNamespace() + "/pose_with_cov" };

    const ros::Duration control_timer(1.0 / control_frequency);
    _control_timer = private_nh.createTimer(control_timer, &Derived::action_function, this);
    // _ol_timer = private_nh.createTimer(control_timer, &Derived::control_timer_callback, this);

    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);
    if (graph_topic_name != "")
      _graph_subscriber = private_nh.subscribe(graph_topic_name, 1, &Derived::graph_callback, this);

    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);
    _pose_with_cov_publisher =
        private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_with_cov_topic, 1, true);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "StelaControl";

    // _x_key_query_service = private_nh.advertiseService("/stela/x_key_query", &Derived::service_callback, this);
    // _xdot_key_query_service =
    //     private_nh.advertiseService("/stela/xdot_key_query", &Derived::xdot_service_callback, this);

    _stela_action_server = std::make_unique<StelaActionServer>(private_nh, "action", false);
    // std::make_unique<StelaActionServer>(private_nh, "action", boost::bind(&Derived::stela_action, this, _1), false);

    _prev_header.stamp = ros::Time::now();
    _local_goal_id = 0;
    _next_node_time = ros::Time::now();
  }

  void to_file()
  {
    using SF = prx::fg::symbol_factory_t;
    gtsam::Values estimate{ _isam.calculateEstimate() };
    const std::string filename{ _output_dir + "/stela_" + utils::timestamp() + ".txt" };
    std::ofstream ofs(filename);
    ofs << "# id key_x x[...] key_xdot xdot[...]\n";
    for (int i = 0; i < _id_x_hat; ++i)
    {
      const gtsam::Key key_x{ SystemInterface::keyX(-1, i) };
      const State x{ estimate.at<State>(key_x) };
      const Eigen::MatrixXd cov{ _isam.marginalCovariance(key_x) };

      ofs << i << " ";
      ofs << SF::formatter(key_x) << " " << x.transpose() << " ";
      ofs << cov.diagonal().transpose() << " ";
      ofs << "\n";
    }
    ofs.close();
  }

  void action_function(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      // ROS_INFO("Action server is active!");
      if (_stela_action_server->isNewGoalAvailable())  // and
      {
        auto new_goal = _stela_action_server->acceptNewGoal();
        if (new_goal->header.seq > _goal.header.seq)
        {
          // ROS_INFO("Goal received");
          _goal = *new_goal;

          // _local_goal_id = 0;
          // DEBUG_VARS(_local_goal_id, _selected_nodes.size());
          // DEBUG_VARS(_goal.selected_branch);
          for (int i = 0; i < _selected_nodes.size(); ++i)
          {
            // DEBUG_VARS(_goal.selected_branch[0], _selected_nodes[i]);
            if (_goal.selected_branch[0] == _selected_nodes[i])
            {
              _selected_nodes.erase(_selected_nodes.begin() + i + 1, _selected_nodes.end());
              _edge_durations.erase(_edge_durations.begin() + i + 1, _edge_durations.end());
              _selected_nodes.insert(_selected_nodes.end(), _goal.selected_branch.begin() + 1,
                                     _goal.selected_branch.end());
              _edge_durations.insert(_edge_durations.end(), _goal.durations.begin() + 1, _goal.durations.end());
            }
          }
          if (_selected_nodes.size() == 0)
          {
            _selected_nodes.insert(_selected_nodes.end(), _goal.selected_branch.begin(), _goal.selected_branch.end());
            _edge_durations.insert(_edge_durations.end(), _goal.durations.begin(), _goal.durations.end());
            _x_next = _selected_nodes[0];
          }
          // DEBUG_VARS(_selected_nodes);

          _stela_action_server->setPreempted();
          build_feedback_lookahead_tree();
        }
      }
      if (_goal.stop or _tree.nodes[_x_next].children.size() == 0)
      {
        to_file();
        ros::shutdown();
      }
      if (_goal.selected_branch.size() < 1)
      {
        ROS_WARN("selected_branch is too short, finished?");
        return;
      }
      // ROS_INFO("Updating next goal");
      update_next_goal();
      add_observations();
      publish_control();

      _feedback.current_root = _x_next;
      _stela_action_server->publishFeedback(_feedback);
    }
  }

  template <typename NodesMap>
  void traverse_lookahead_tree(const std::uint64_t node_id, NodesMap& nodes_ids_map, const bool root,
                               const std::size_t& remaining_nodes)
  {
    if (remaining_nodes == 0 or nodes_ids_map.count(node_id) > 0)
    {
      return;
    }
    const gtsam::Key x_key{ SystemInterface::keyX(1, node_id) };
    const gtsam::Key xdot_key{ SystemInterface::keyXdot(1, node_id) };

    const State x{ _isam.calculateEstimate<State>(x_key) };
    const StateDot xdot{ _isam.calculateEstimate<StateDot>(xdot_key) };

    prx_models::Node new_node{ std::move(_tree_manager.create_node()) };
    SystemInterface::copy(new_node.point, x, xdot);
    // gtsam::VectorValues vector_vals;
    // vector_vals.insert(x_key, x);
    // vector_vals.insert(xdot_key, xdot);

    // _factor_graph
    const State x_sbmp{ _values.at<State>(x_key) };
    const StateDot xdot_sbmp{ _values.at<StateDot>(xdot_key) };

    // using SF = prx::fg::symbol_factory_t;
    // DEBUG_VARS(SF::formatter(x_key), x_key);
    // DEBUG_VARS(x.transpose(), x_sbmp.transpose());
    // DEBUG_VARS(SF::formatter(xdot_key), xdot_key);
    // _isam[x_key]->print(" ", SF::formatter);
    const Eigen::MatrixXd cov{ _isam.marginalCovariance(x_key) };
    const Eigen::MatrixXd S{ cov.inverse() };
    const State diff{ x - x_sbmp };
    const double updated_cost{ diff.transpose() * S * diff };
    // new_node.cost = updated_cost;
    // DEBUG_VARS(new_node.index, new_node.cost);

    nodes_ids_map[node_id] = new_node.index;

    if (not root)
    {
      const std::uint64_t original_parent_id{ _tree.nodes[node_id].parent };
      prx_models::Node parent_node{ _feedback.lookahead.nodes[nodes_ids_map[original_parent_id]] };
      // _feedback.lookahead_costs.push_back(parent_node);
      prx_models::Edge edge{ _tree_manager.create_edge(parent_node, new_node) };
      _feedback.lookahead.edges.push_back(std::move(edge));
    }
    else
    {
      for (int i = 0; i < 2; ++i)
      {
        for (int j = 0; j < 2; ++j)
        {
          _pose_with_cov.pose.pose.position.x = x[0];
          _pose_with_cov.pose.pose.position.y = x[1];
          _pose_with_cov.pose.covariance[6 * i + j] = cov(i, j);
        }
      }
      _pose_with_cov_publisher.publish(_pose_with_cov);
    }

    // DEBUG_VARS(_feedback.lookahead.nodes.size(), new_node.index);
    if (_feedback.lookahead.nodes.size() <= new_node.index)
    {
      _feedback.lookahead.nodes.resize(new_node.index * 2);
    }
    if (_feedback.lookahead_costs.size() <= node_id)
    {
      _feedback.lookahead_costs.resize(node_id * 2, 0.0);
    }

    _feedback.lookahead_costs[node_id] = updated_cost;
    _feedback.lookahead.nodes[new_node.index] = std::move(new_node);

    for (auto child_id : _tree.nodes[node_id].children)
    {
      traverse_lookahead_tree(child_id, nodes_ids_map, false, remaining_nodes - 1);
    }
    // visited.insert(node_id);

    // traverse_lookahead_tree(const std::uint64_t node_id, nodes_ids_map, remaining_nodes - 1);
  }

  void build_feedback_lookahead_tree()
  {
    // _local_goal_id = 0;
    // update_next_goal(10000);  // A big number to force updates
    _tree_manager.reset();

    std::queue<std::uint64_t> nodes_to_visit;
    std::queue<std::uint64_t> other_branches;

    const std::size_t total_lookahead_nodes{ _goal.selected_branch.size() };
    // DEBUG_VARS(total_lookahead_nodes);

    _feedback.lookahead.nodes.clear();
    _feedback.lookahead.edges.clear();
    _feedback.lookahead.root = 0;
    _feedback.lookahead.nodes.resize(total_lookahead_nodes * 2);
    _feedback.lookahead_costs.resize(total_lookahead_nodes * 2);
    // SystemInterface::copy(_feedback.xhat, _x_hat, _xdot_hat);

    std::uint64_t node_id;
    for (auto node_to_visit : _goal.selected_branch)
    {
      nodes_to_visit.push(node_to_visit);
    }

    std::unordered_set<std::uint64_t> visited;
    // nodes_ids_map[old_id] = new_id
    std::unordered_map<std::uint64_t, std::uint64_t> nodes_ids_map;

    traverse_lookahead_tree(_goal.selected_branch.front(), nodes_ids_map, true, total_lookahead_nodes);
  }

  void stela_action(const motion_planning::StelaGraphTraversalGoalConstPtr& goal)
  {
    // DEBUG_VARS(goal->x0, goal->x1, goal->x2, goal->edge_duration_01, goal->edge_duration_12);
    if (_goal.header.seq != goal->header.seq)
    {
      ros::Rate rate(10);
      _prev_header.stamp = ros::Time::now();

      // const State x_current{ _isam.calculateEstimate<State>(_x_current) };
      // const StateDot xdot_current{ _isam.calculateEstimate<StateDot>(_xdot_current) };
      // const gtsam::Key x1_key{ SystemInterface::keyX(1, goal->x1) };
      // const gtsam::Key xdot1_key{ SystemInterface::keyXdot(1, goal->x1) };

      // const gtsam::Key x2_key{ SystemInterface::keyX(1, goal->x1) };
      // const gtsam::Key xdot2_key{ SystemInterface::keyXdot(1, goal->x1) };

      // const State x1{ _isam.calculateEstimate<State>(x1_key) };
      // const StateDot xdot1{ _isam.calculateEstimate<StateDot>(xdot1_key) };

      // const State x2{ _isam.calculateEstimate<State>(x2_key) };
      // const StateDot xdot2{ _isam.calculateEstimate<StateDot>(xdot2_key) };

      // SystemInterface::copy(_feedback.x1, x1, xdot1);
      // SystemInterface::copy(_feedback.x2, x2, xdot2);
      // _key_u01 = SystemInterface::keyU(goal->x0, goal->x1);

      _goal = *goal;
      // _feedback.duration_so_far = 0;
      // _time_remaining = goal->edge_duration_01;
      while (ros::ok())
      {
        // DEBUG_VARS(_stela_action_server->isNewGoalAvailable());
        if (_stela_action_server->isNewGoalAvailable())
        {
          auto new_goal = _stela_action_server->acceptNewGoal();
          _goal = *new_goal;
          // if (_used_x2)
          // {
          //   _feedback.duration_so_far = _feedback.duration_so_far - _local_goal.edge_duration_01;
          // }
          // else
          // {
          //   _feedback.duration_so_far = 0;
          //   _time_remaining = new_goal->edge_duration_01;
          // }
          // DEBUG_VARS(_feedback.duration_so_far, _time_remaining);
          // DEBUG_VARS(_local_goal);
        }
        add_observations();
        rate.sleep();
      }
    }
    // if(success)
    // {
    // result_.sequence = feedback_.sequence;
    // ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    // assuming the state can be obtained from x, xdot exclusively
    // SystemInterface::copy(_result.xfinal, x1, xdot1);
    // _stela_action_server->setSucceeded(_result);
    // }
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
    if (now >= _next_node_time)
    {
      if (_local_goal_id < _selected_nodes.size() - 1)
      {
        _local_goal_id++;

        // const double next_duration{ _edge_durations[_local_goal_id] };

        _feedback.current_root = _x_next;
        _x_next = _selected_nodes[_local_goal_id];
        _last_local_goal = false;

        const std::uint64_t parent_edge{ _tree.nodes[_x_next].parent_edge };
        const ml4kp_bridge::Plan& plan{ _tree.edges[parent_edge].plan };
        const double next_duration{ ml4kp_bridge::duration(plan).toSec() };
        _next_node_time = now + ros::Duration(next_duration);

        // DEBUG_VARS(_x_next, next_duration, now);
      }
      else  // last available goal
      {
        _last_local_goal = true;
      }
    }
  }

  void add_observations()
  {
    const bool new_observation{ query_tf() };
    if (new_observation and _tf.header.stamp > _prev_header.stamp and not _last_local_goal)
    {
      const double dt{ (_tf.header.stamp - _prev_header.stamp).toSec() };
      SystemInterface::copy(_z_new, _tf);
      _time_remaining = (_next_node_time - ros::Time::now()).toSec();

      const GraphValues graph_values_z{ SystemInterface::add_observation_factor(_id_x_hat,
                                                                                _id_x_hat + 1,            // no-lint
                                                                                _x_hat, _xdot_hat, _u01,  // no-lint
                                                                                _z_new, dt, 0.01) };
      // prx::fg::symbol_factory_t::symbols_to_file();

      // if (_last_local_goal)

      const GraphValues graph_values_1{ SystemInterface::local_adaptation(_id_x_hat + 1, _x_next, _u01,
                                                                          _time_remaining) };
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

      SystemInterface::copy(_feedback.xhat, _x_hat, _xdot_hat);

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
    _stela_action_server->start();
    _tree_recevied = true;
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
    // DEBUG_VARS(ErrorBefore, ErrorAfter);
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
  ros::Publisher _pose_with_cov_publisher;
  geometry_msgs::PoseWithCovarianceStamped _pose_with_cov;

  ros::Timer _control_timer;

  ros::ServiceServer _x_key_query_service;
  ros::ServiceServer _xdot_key_query_service;
  std::unique_ptr<StelaActionServer> _stela_action_server;

  motion_planning::StelaGraphTraversalFeedback _feedback;
  motion_planning::StelaGraphTraversalResult _result;

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
  motion_planning::StelaGraphTraversalGoal _goal;

  std::uint64_t _x_next;
  std::size_t _local_goal_id;

  std::vector<std::uint64_t> _selected_nodes;
  std::vector<double> _edge_durations;

  bool _tree_recevied;
  motion_planning::tree_manager_t _tree_manager;
  ros::Time _next_node_time;
  bool _last_local_goal;

  std::string _output_dir;
};
}  // namespace motion_planning