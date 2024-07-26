#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <utils/std_utils.cpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_planning/StelaGraphTraversalAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class stela_t : public Base
{
  using Derived = stela_t<SystemInterface, Base>;
  using StelaActionServer = actionlib::SimpleActionServer<motion_planning::StelaGraphTraversalAction>;

  using SF = prx::fg::symbol_factory_t;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;
  // using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;

  using StateKeys = typename SystemInterface::StateKeys;
  using ControlKeys = typename SystemInterface::ControlKeys;

  using StateEstimates = typename SystemInterface::StateEstimates;
  using ControlEstimates = typename SystemInterface::ControlEstimates;

  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename SystemInterface::ConfigFromState>;

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
    , _goal_received(false)
    , _mode("None")
    , _experiment_id("test")
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
    std::string control_topic;
    std::string collision_topic;
    std::string environment;
    double obstacle_sigma{ 0.1 };
    double control_frequency;
    double& obstacle_distance_tolerance{ _obstacle_distance_tolerance };

    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& output_dir{ _output_dir };
    std::string& obstacle_mode{ _obstacle_mode };
    std::string& experiment_id{ _experiment_id };

    // ROS_PARAM_SETUP(private_nh, random_seed);
    // ROS_PARAM_SETUP(private_nh, plant_config_file);
    // ROS_PARAM_SETUP(private_nh, planner_config_file);
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
    PARAM_SETUP_WITH_DEFAULT(private_nh, obstacle_sigma, obstacle_sigma)
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id)

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };
    const std::string pose_with_cov_topic{ ros::this_node::getNamespace() + "/pose_with_cov" };
    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };
    const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    const ros::Duration control_timer(1.0 / control_frequency);
    _control_timer = private_nh.createTimer(control_timer, &Derived::action_function, this);
    // _ol_timer = private_nh.createTimer(control_timer, &Derived::control_timer_callback, this);

    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);
    _collision_subscriber = private_nh.subscribe(collision_topic, 1, &Derived::collision_callback, this);

    _finish_publisher = private_nh.advertise<std_msgs::Bool>(finish_topic, 1, true);
    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);
    _pose_with_cov_publisher =
        private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_with_cov_topic, 1, true);

    _viz_obstacles_publisher = private_nh.advertise<visualization_msgs::Marker>(obstacle_viz_topic, 1, false);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "StelaControl";

    _stela_action_server = std::make_unique<StelaActionServer>(private_nh, "action", false);

    _prev_header.stamp = ros::Time::now();
    _local_goal_id = 0;
    _next_node_time = ros::Time::now();

    auto obstacles = prx::load_obstacles(environment);
    // _obstacle_list = obstacles.second;
    _obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

    _robot_collision_ptr = SystemInterface::collision_geometry();
    _obstacle_noise = gtsam::noiseModel::Isotropic::Sigma(2, obstacle_sigma);

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
    _obstacles_marker.scale.x = 0.1;
    _obstacles_marker.scale.y = 0.1;
    _obstacles_marker.scale.z = 0.1;

    _obstacles_marker.color.a = 1.0;  // Don't forget to set the alpha!
    _obstacles_marker.color.r = 0.98;
    _obstacles_marker.color.g = 0.55;
    _obstacles_marker.color.b = 0.02;
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    if (msg->data)
    {
      to_file(true);
    }
  }

  void to_file(const bool collision = false)
  {
    gtsam::Values estimate{ _isam.calculateEstimate() };
    const std::string filename{ _output_dir + "/stela_" + _experiment_id + "_" + utils::timestamp() + ".txt" };
    const std::string filename_branch_gt{ _output_dir + "/stela_branch_gt_" + _experiment_id + "_" +
                                          utils::timestamp() + ".txt" };
    const std::string filename_data{ _output_dir + "/stela_data_" + _experiment_id + "_" + utils::timestamp() +
                                     ".txt" };

    DEBUG_VARS(filename);
    DEBUG_VARS(filename_branch_gt);
    DEBUG_VARS(filename_data);
    std::ofstream ofs(filename);
    std::ofstream ofs_branch(filename_branch_gt);
    std::ofstream ofs_data(filename_data);
    const double elapsed_time{ (ros::Time::now() - _start_time).toSec() };
    ofs_data << "ElapsedTime: " << elapsed_time << "\n";
    ofs_data << "Collision: " << (collision ? "true" : "false") << "\n";
    ofs_data << "ObstacleDistanceTolerance: " << _obstacle_distance_tolerance << "\n";
    ofs_data << "ObstacleMode: " << _mode << "\n";

    ofs << "# id key_x x[...] xCov[...] key_xdot xdot[...] xdotCov[...]\n";
    for (int i = 0; i < _id_x_hat; ++i)
    {
      ofs << i << " ";
      const StateKeys keys{ SystemInterface::keyState(-1, i) };
      estimates_to_file<0>(ofs, estimate, keys);
    }
    ofs_branch << "# id point[...]\n";
    for (auto node_id : _selected_nodes)
    {
      // const ml4kp_bridge::SpacePoint& {};
      ofs_branch << node_id << " ";
      ml4kp_bridge::to_file(_tree.nodes[node_id].point, ofs_branch);
    }

    ofs.close();
    ofs_branch.close();
    ofs_data.close();

    std_msgs::Bool msg;
    msg.data = true;
    _finish_publisher.publish(msg);
    _tree_recevied = false;

    ros::Rate rate(1);
    rate.sleep();
    ros::shutdown();
  }

  void action_function(const ros::TimerEvent& event)
  {
    if (_tree_recevied)
    {
      ROS_INFO_ONCE("Action server is active!");
      if (_stela_action_server->isNewGoalAvailable())  // and
      {
        auto new_goal = _stela_action_server->acceptNewGoal();
        if (new_goal->header.seq > _goal.header.seq)
        {
          ROS_INFO_ONCE("Goal received");
          _goal_received = true;
          _goal = *new_goal;

          for (int i = 0; i < _selected_nodes.size(); ++i)
          {
            // DEBUG_VARS(_goal.selected_branch[0], _selected_nodes[i]);
            if (_goal.selected_branch[0] == _selected_nodes[i])
            {
              _selected_nodes.erase(_selected_nodes.begin() + i + 1, _selected_nodes.end());
              _selected_nodes.insert(_selected_nodes.end(), _goal.selected_branch.begin() + 1,
                                     _goal.selected_branch.end());
            }
          }
          if (_selected_nodes.size() == 0)
          {
            _selected_nodes.insert(_selected_nodes.end(), _goal.selected_branch.begin(), _goal.selected_branch.end());
            _x_next = _selected_nodes[0];
          }
          // DEBUG_VARS(_selected_nodes);

          _stela_action_server->setPreempted();
          build_feedback_lookahead_tree();
        }
      }
      if (not _goal_received)
        return;
      if (_goal.stop or _tree.nodes[_x_next].children.size() == 0)
      {
        ROS_WARN("Finished! Creating file");
        to_file();
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

    const StateKeys node_keys{ SystemInterface::keyState(1, node_id) };
    update_estimates<0>(_node_estimates, node_keys);

    // const gtsam::Key x_key{ SystemInterface::keyX(1, node_id) };
    // const gtsam::Key xdot_key{ SystemInterface::keyXdot(1, node_id) };

    // const State x{ _isam.calculateEstimate<State>(x_key) };
    // const StateDot xdot{ _isam.calculateEstimate<StateDot>(xdot_key) };

    prx_models::Node new_node{ std::move(_tree_manager.create_node()) };
    SystemInterface::copy(new_node.point, _node_estimates);

    // _factor_graph
    const double updated_cost{ compute_error<0>(node_keys) };
    // for (std::size_t i = 0; i < node_keys.size(); ++i)
    // {
    //   using StateType = std::tuple_element<i, StateEstimates>::type;
    //   const gtsam::Key& key{ node_keys[i] };
    //   const StateType x{ std::get<i>(_node_estimates) };
    //   const StateType x_sbmp{ _values.at<StateType>(key) };

    //   const const Eigen::MatrixXd cov{ _isam.marginalCovariance(key) };
    //   const Eigen::MatrixXd S{ cov.inverse() };
    //   const State diff{ x - x_sbmp };
    //   updated_cost = diff.transpose() * S * diff;
    // }

    nodes_ids_map[node_id] = new_node.index;

    if (not root)
    {
      const std::uint64_t original_parent_id{ _tree.nodes[node_id].parent };
      prx_models::Node parent_node{ _feedback.lookahead.nodes[nodes_ids_map[original_parent_id]] };
      prx_models::Edge edge{ _tree_manager.create_edge(parent_node, new_node) };
      _feedback.lookahead.edges.push_back(std::move(edge));
    }

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
  }

  void build_feedback_lookahead_tree()
  {
    _tree_manager.reset();

    std::queue<std::uint64_t> nodes_to_visit;
    std::queue<std::uint64_t> other_branches;

    const std::size_t total_lookahead_nodes{ _goal.selected_branch.size() };

    _feedback.lookahead.nodes.clear();
    _feedback.lookahead.edges.clear();
    _feedback.lookahead.root = 0;
    _feedback.lookahead.nodes.resize(total_lookahead_nodes * 2);
    _feedback.lookahead_costs.resize(total_lookahead_nodes * 2);

    std::uint64_t node_id;
    for (auto node_to_visit : _goal.selected_branch)
    {
      nodes_to_visit.push(node_to_visit);
    }

    std::unordered_set<std::uint64_t> visited;
    std::unordered_map<std::uint64_t, std::uint64_t> nodes_ids_map;

    traverse_lookahead_tree(_goal.selected_branch.front(), nodes_ids_map, true, total_lookahead_nodes);
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

        _feedback.current_root = _x_next;
        _x_next = _selected_nodes[_local_goal_id];
        _last_local_goal = false;

        const std::uint64_t parent_edge{ _tree.nodes[_x_next].parent_edge };
        const ml4kp_bridge::Plan& plan{ _tree.edges[parent_edge].plan };
        const double next_duration{ ml4kp_bridge::duration(plan).toSec() };
        _next_node_time = now + ros::Duration(next_duration);
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
                                                                                _id_x_hat + 1,           // no-lint
                                                                                _state_estimates, _u01,  // no-lint
                                                                                _z_new, dt, 0.01) };

      const GraphValues graph_values_1{ SystemInterface::local_adaptation(_id_x_hat + 1, _x_next, _u01,
                                                                          _time_remaining) };
      _isam2_result = _isam.update(graph_values_z.first, graph_values_z.second);
      _isam2_result = _isam.update(graph_values_1.first, graph_values_1.second);

      _id_x_hat++;

      _key_u01 = SystemInterface::keyU(-1 * _id_x_hat, _x_next);

      const StateKeys state_keys{ SystemInterface::keyState(-1, _id_x_hat) };
      update_estimates<0>(_state_estimates, state_keys);

      SystemInterface::copy(_feedback.xhat, _state_estimates);

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

  void obstacle_factors(const ml4kp_bridge::SpacePoint& point, int x_id)
  {
    if (_obstacle_mode == "distance")
    {
      _mode = "distance";
      Eigen::Vector3d p1, p2;
      const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
      // DEBUG_VARS(SF::formatter(keyX));
      SystemInterface::state(_state, point);
      for (auto obstacle_info : _obstacle_collision_infos)
      {
        if (ObstacleFactor::close_enough(_state, _obstacle_distance_tolerance, obstacle_info, _robot_collision_ptr,
                                         _config_from_state, _obstacle_tolerance_result))
        // {
        // if (ObstacleFactor::distances(_state, p1, p2, obstacle_info, _robot_collision_ptr, _config_from_state,
        //                               _obstacle_distance_result) < _obstacle_distance_tolerance)
        {
          _obstacle_graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX,
                                                         _obstacle_distance_tolerance, 0.1, _obstacle_noise);
          _obstacles_marker.points.emplace_back();
          _obstacles_marker.points.back().x = _state[0];
          _obstacles_marker.points.back().y = _state[1];
          _obstacles_marker.points.back().z = 0;
          _obstacles_marker.points.emplace_back();
          _obstacles_marker.points.back().x = p1[0];
          _obstacles_marker.points.back().y = p1[1];
          _obstacles_marker.points.back().z = 0;
          // DEBUG_VARS(x_id, _state.transpose());
        }
        // gtsam::NonlinearFactorGraph obstacle_graph{ ObstacleFactor::collision_factors(
        // keyX, obstacle, _robot_collision_ptr, _obstacle_distance_tolerance, _obstacle_noise) };
        // for (auto factor : obstacle_graph)
        // {
        //   auto obstacle_factor = boost::dynamic_pointer_cast<ObstacleFactor>(factor);
        //   const bool valid_factor(obstacle_factor != nullptr);
        //   prx_assert(valid_factor, "Obstacle factor is not valid!");
        //   if (obstacle_factor->close_enough(_state, _obstacle_distance_tolerance))
        //   {
        //     // graph_values.first.add(obstacle_factor);
        //     DEBUG_VARS(x_id, _state.transpose());
        //   }
        // }
      }
    }
  }

  void branch_to_traj(const prx_models::TreeConstPtr msg, const std::uint64_t edge_id)
  {
    const prx_models::Edge& edge{ msg->edges[edge_id] };
    const prx_models::Node& node_parent{ msg->nodes[edge.source] };
    const prx_models::Node& node_current{ msg->nodes[edge.target] };

    GraphValues graph_values{ SystemInterface::node_edge_to_fg(edge.source, edge.target, node_current.point,
                                                               edge.plan) };
    obstacle_factors(node_current.point, edge.target);

    _values.insert(graph_values.second);

    // ROS_DEBUG_STREAM_NAMED("STELA", "N0: " << edge.source << " E01: " << edge_id << " N1: " << edge.target);

    // if (edge.source == 928 or edge.source == 5017)
    // {
    //   // void printErrors(const Values& values, const std::string& str = "NonlinearFactorGraph: ",
    //   // const KeyFormatter& keyFormatter = DefaultKeyFormatter);
    //   graph_values.first.print("Graph: ", SF::formatter);
    //   // void print(const std::string& str = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
    //   graph_values.second.print("Values: ", SF::formatter);
    //   graph_values.first.printErrors(_values, "Graph: ", SF::formatter);
    // }
    _isam2_result = _isam.update(graph_values.first, graph_values.second);
    // ROS_DEBUG_STREAM_NAMED("STELA", "cliques: " << _isam2_result.cliques);
    const std::size_t total_children{ node_current.children.size() };

    if (total_children >= 1)  // This is not a leaf
    {
      for (int i = 0; i < total_children; ++i)
      {
        const prx_models::Node& node_child{ msg->nodes[node_current.children[i]] };

        branch_to_traj(msg, node_child.parent_edge);
      }
    }
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    const ros::Time start{ ros::Time::now() };
    ROS_DEBUG_STREAM_NAMED("STELA", "Tree received: " << msg->nodes.size());
    _isam.clear();
    _values = gtsam::Values();
    const prx_models::Node& node{ msg->nodes[msg->root] };

    const GraphValues root_graph_values{ SystemInterface::root_to_fg(msg->root, node.point) };
    _values.insert(root_graph_values.second);
    _isam2_result = _isam.update(root_graph_values.first, root_graph_values.second);
    for (auto child_id : node.children)
    {
      if (child_id == msg->root)  // The root node is its own parent :/
        continue;
      const prx_models::Node& node_child{ msg->nodes[child_id] };

      branch_to_traj(msg, node_child.parent_edge);
    }

    ROS_DEBUG_STREAM_NAMED("STELA", "Finished traversing tree ");
    _id_x_hat = msg->root;
    const GraphValues graph_values{ SystemInterface::root_to_fg(_id_x_hat, node.point, true) };
    _isam2_result = _isam.update(graph_values.first, graph_values.second);

    ROS_DEBUG_STREAM_NAMED("STELA", "Adding obstacle graph: " << _obstacle_graph.size());
    if (_obstacle_graph.size() > 0)
    {
      _viz_obstacles_publisher.publish(_obstacles_marker);
      _isam2_result = _isam.update(_obstacle_graph, gtsam::Values());
    }

    const StateKeys state_keys{ SystemInterface::keyState(-1, _id_x_hat) };
    update_estimates<0>(_state_estimates, state_keys);

    _tree.root = msg->root;
    _tree.nodes = msg->nodes;
    _tree.edges = msg->edges;
    ROS_DEBUG_STREAM_NAMED("STELA", "Starting action server");
    _stela_action_server->start();
    _tree_recevied = true;
    ROS_DEBUG_STREAM_NAMED("STELA", "Action server started");
    const double elapsed_time{ (ros::Time::now() - start).toSec() };
    std::cout << "Elapsed time: " << elapsed_time << std::endl;

    _start_time = ros::Time::now();
  }

private:
  // template <std::size_t I = 0, typename FuncT, typename... Tp>
  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  inline void update_estimates(StateEstimates& state_estimates, const StateKeys& keys)
  {
    using EstimateType = typename std::tuple_element<I, StateEstimates>::type;
    std::get<I>(state_estimates) = _isam.calculateEstimate<EstimateType>(keys[I]);
    update_estimates<I + 1>(state_estimates, keys);
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>
  inline void update_estimates(StateEstimates& state_estimates, const StateKeys& keys)
  {
  }

  template <std::size_t I, std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  double compute_error(const StateKeys& keys)
  {
    using StateType = typename std::tuple_element<I, StateEstimates>::type;
    const gtsam::Key& key{ keys[I] };
    const StateType& x{ std::get<I>(_node_estimates) };
    const StateType x_sbmp{ _values.at<StateType>(key) };

    const Eigen::MatrixXd cov{ _isam.marginalCovariance(key) };
    const Eigen::MatrixXd S{ cov.inverse() };
    const StateType diff{ x - x_sbmp };
    return diff.transpose() * S * diff + compute_error<I + 1>(keys);
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  double compute_error(const StateKeys& keys)
  {
    return 0;
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
    for (auto e : state)
    {
      ofs << e << " ";
    }
    for (auto e : diagonal)
    {
      ofs << e << " ";
    }
    estimates_to_file<I + 1>(ofs, estimate, keys);
    // ofs << "\n";
  }

  template <std::size_t I, std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
  void estimates_to_file(std::ofstream& ofs, const gtsam::Values& estimate, const StateKeys& keys)
  {
    ofs << "\n";
  }

  Values _values;
  // FactorGraph _factor_graph;

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;

  StateKeys _state_keys;
  StateEstimates _state_estimates;
  StateEstimates _node_estimates;
  StateEstimates _sbmp_nodes;

  gtsam::Key _key_last_esimated_state;
  gtsam::Key _key_u01;

  ml4kp_bridge::SpacePointStamped _control_stamped;
  prx_models::Tree _tree;

  ros::Subscriber _tree_subscriber;
  ros::Subscriber _collision_subscriber;

  ros::Publisher _control_publisher;
  ros::Publisher _stamped_control_publisher;
  ros::Publisher _pose_with_cov_publisher;
  ros::Publisher _finish_publisher;
  ros::Publisher _viz_obstacles_publisher;

  geometry_msgs::PoseWithCovarianceStamped _pose_with_cov;

  ros::Timer _control_timer;

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

  State _state;
  Control _u01;
  Observation _z_new;

  double _time_remaining;

  bool _goal_received;
  motion_planning::StelaGraphTraversalGoal _goal;

  std::uint64_t _x_next;
  std::size_t _local_goal_id;

  std::vector<std::uint64_t> _selected_nodes;
  // std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;

  bool _tree_recevied;
  motion_planning::tree_manager_t _tree_manager;
  ros::Time _next_node_time;
  ros::Time _start_time;
  bool _last_local_goal;

  // File/output
  std::string _output_dir;
  std::string _experiment_id;

  // Obstacle-relates stuff
  std::string _obstacle_mode, _mode;
  gtsam::NonlinearFactorGraph _obstacle_graph;
  double _obstacle_distance_tolerance;
  typename SystemInterface::ConfigFromState _config_from_state;
  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  typename ObstacleFactor::ToleranceResult _obstacle_tolerance_result;
  typename ObstacleFactor::DistanceResult _obstacle_distance_result;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;
  visualization_msgs::Marker _obstacles_marker;
};
}  // namespace motion_planning
