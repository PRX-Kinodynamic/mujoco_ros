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
#include <ml4kp_bridge/StelaTrajectory.h>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <prx_models/tree_msg_wrapper.hpp>

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class sbmp_stepper_t : public Base
{
  using Derived = sbmp_stepper_t<SystemInterface, Base>;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;

  // using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;

  using StateKeys = typename SystemInterface::StateKeys;
  using ControlKeys = typename SystemInterface::ControlKeys;

  using StateEstimates = typename SystemInterface::StateEstimates;
  using ControlEstimates = typename SystemInterface::ControlEstimates;

  static constexpr Eigen::Index XDim{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index UDim{ gtsam::traits<Control>::dimension };

  using ControlTranspose = Eigen::RowVector<double, UDim>;

  using NodeIdx = std::size_t;

public:
  sbmp_stepper_t()
    : _tree_recevied(false)
    , _experiment_id("test")
    , _files_created(false)
    , _name("STELA_SW")
    , _total_future_nodes(10)
    , _total_past_nodes(10)
    , _current_past_nodes(0)
    , _goal_id(std::numeric_limits<std::size_t>::max())
    , _goal_reached(false)
    , _visualize(false)
    , _next_idx(0) {};

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    PRINT_MSG("Starting Stela Windowed");

    std::string tree_topic_name;
    std::string estimation_topic;

    std::string sdf_params;
    std::string tree_ahead_topic;
    std::string& output_dir{ _output_dir };
    std::string& experiment_id{ _experiment_id };

    std::vector<double> plant_parameters{};

    // bool report_control_frequency{ true };
    bool& visualize{ _visualize };
    int& total_future_nodes{ _total_future_nodes };
    int& total_past_nodes{ _total_past_nodes };

    PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP(private_nh, output_dir)
    PARAM_SETUP(private_nh, estimation_topic)
    PARAM_SETUP(private_nh, tree_ahead_topic)
    PARAM_SETUP_WITH_DEFAULT(private_nh, visualize, visualize)
    PARAM_SETUP_WITH_DEFAULT(private_nh, experiment_id, experiment_id)
    // PARAM_SETUP_WITH_DEFAULT(private_nh, report_control_frequency, report_control_frequency)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_future_nodes, total_future_nodes)
    PARAM_SETUP_WITH_DEFAULT(private_nh, total_past_nodes, total_past_nodes)

    const std::string finish_topic{ ros::this_node::getNamespace() + "/finished" };
    const std::string obstacle_viz_topic{ ros::this_node::getNamespace() + "/obstacle_edges" };

    // const ros::Duration control_timer(1.0 / control_frequency);
    // _control_timer = private_nh.createTimer(control_timer, &Derived::main_timer_callback, this);

    // if (report_control_frequency)
    // {
    //   const ros::Duration control_freq_timer(1.0);
    //   _control_frequency_timer = private_nh.createTimer(control_freq_timer, &Derived::check_frequency, this);
    // }

    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);
    _estimation_subscriber = private_nh.subscribe(estimation_topic, 1, &Derived::estimation_callback, this);

    _tree_ahead_publisher = private_nh.advertise<prx_models::Tree>(tree_ahead_topic, 1, true);
    _finish_publisher = private_nh.advertise<std_msgs::Bool>(finish_topic, 1, true);

    _next_node_time = ros::Time::now();

    _x0_start_time = ros::Time::ZERO;

    _timestamp = utils::timestamp();
    const std::string path{ _output_dir + "/" + _name };
    const std::string filename{ path + "_" + _experiment_id + "_" + _timestamp + ".txt" };
    // _ofs.open(filename);
    // _ofs << "# id key_x x[...] xCov[...] key_xdot xdot[...] xdotCov[...]\n";
  }

  ~sbmp_stepper_t()
  {
    to_file();
  }

  void collision_callback(const std_msgs::BoolConstPtr& msg)
  {
    if (msg->data)
    {
      to_file();
    }
  }

  void to_file()
  {
  }

  void main_timer_callback(const ros::TimerEvent& event)
  {
  }

  double distance_to_node(const ml4kp_bridge::SpacePoint& estimated, const std::size_t node_id)
  {
    const prx_models::Node& node{ _tree.nodes[node_id] };
    return SystemInterface::DistanceFunction(estimated, node.point);
  }

  double find_closest_child(const ml4kp_bridge::SpacePoint& estimated, const std::size_t current_idx,
                            std::size_t& child_id)
  {
    const prx_models::Node& current_node{ _tree.nodes[current_idx] };
    // DEBUG_VARS(estimated.point, current_idx);
    // DEBUG_VARS(current_node.point.point);
    double min_dist{ std::numeric_limits<double>::max() };
    for (auto child : current_node.children)
    {
      const double dist_child{ distance_to_node(estimated, child) };
      // min_dist = std::min(min_dist, dist_child);
      if (dist_child < min_dist)
      {
        min_dist = dist_child;
        child_id = child;
        // DEBUG_VARS(min_dist, child_id);
      }
    }
    // DEBUG_VARS(estimated.point, min_dist);
    // DEBUG_VARS(_tree.nodes[child_id].point);
    return min_dist;
  }

  NodeIdx find_closest(const ml4kp_bridge::SpacePoint& estimated, const std::size_t current_idx)
  {
    NodeIdx child_id{ 0 };
    NodeIdx closest{ current_idx };
    const NodeIdx parent_id{ static_cast<std::size_t>(_tree.nodes[current_idx].parent) };
    const double dist_children{ find_closest_child(estimated, current_idx, child_id) };
    const double dist_curr{ distance_to_node(estimated, current_idx) };
    const double dist_past{ distance_to_node(estimated, parent_id) };

    if (dist_children < dist_curr)
    {
      closest = child_id;
    }
    else if (dist_curr > dist_past)
    {
      closest = parent_id;
    }
    // DEBUG_VARS(estimated.point);
    // DEBUG_VARS(current_idx, dist_children, dist_curr, dist_past);
    // DEBUG_VARS(closest, child_id, parent_id);
    return closest;
  }

  void get_future_tree(const NodeIdx& node_id, const std::size_t remaining)
  {
    // if (_ids_map.count(node_id) > 0 or _ids_revmap.count(node_id) > 0)
    // if (_ids_map.count(node_id) > 0 or _tree.nodes.count(node_id) > 0)
    // {
    const std::size_t new_node_id{ get_next_node_id() };
    _ids_map[node_id] = new_node_id;
    _ids_revmap[new_node_id] = node_id;
    // _ids.insert(node_id);
    // }
    // else
    // {
    // _ids_map[node_id] = node_id;
    // _ids_revmap[node_id] = node_id;
    // _ids.insert(node_id);
    // }
    // DEBUG_VARS(_ids_map.count(node_id), _tree.nodes.count(node_id));
    // DEBUG_VARS(node_id, _ids_map[node_id], _ids_revmap[_ids_map[node_id]]);
    const prx_models::Node& node{ _tree.nodes[node_id] };
    const prx_models::Edge& edge{ _tree.edges[node.parent_edge] };
    _tree_ahead.nodes.push_back(_tree.nodes[node_id]);
    _tree_ahead.edges.push_back(_tree.edges[node.parent_edge]);

    const std::size_t idx{ _tree_ahead.nodes.size() - 1 };
    // prx_models::Node& new_node{ _tree_ahead.nodes[idx] };
    // prx_models::Edge& new_edge{ _tree_ahead.edges[idx] };

    _tree_ahead.nodes[idx].index = _ids_map[node.index];
    _tree_ahead.nodes[idx].parent = _ids_map[node.parent];

    _tree_ahead.edges[idx].source = _ids_map[edge.source];

    _tree_ahead.nodes[idx].children.clear();
    if (remaining > 0)
    {
      // DEBUG_VARS(edge.index, edge.source, edge.target);
      // _tree_ahead.nodes[idx].children.reserve(node.children.size());
      // DEBUG_VARS(node_id, _ids_map[node_id], remaining);
      // DEBUG_VARS(node.children, _tree_ahead.nodes[idx].children);
      for (int i = 0; i < node.children.size(); ++i)
      {
        const NodeIdx child{ _tree.nodes[node_id].children[i] };
        get_future_tree(child, remaining - 1);
        // DEBUG_VARS(i, child, remaining);
        // DEBUG_VARS(_ids_map[child]);
        // DEBUG_VARS(new_node.children);

        _tree_ahead.nodes[idx].children.push_back(_ids_map[child]);
      }
      // DEBUG_VARS(_tree_ahead.nodes[idx].children);
    }

    _tree_ahead.edges[idx].target = _ids_map[edge.target];
    // DEBUG_VARS(remaining, node_id, edge.source, edge.target, _ids_map[edge.source], _ids_map[edge.target]);
  }

  void estimation_callback(const ml4kp_bridge::StelaTrajectoryConstPtr msg)
  {
    const ml4kp_bridge::SpacePointStamped& xt{ msg->data.back() };
    const NodeIdx node_id{ static_cast<NodeIdx>(msg->ids.back()) };
    // DEBUG_VARS(msg->ids);
    // DEBUG_VARS(msg->data);
    compute_and_publish_tree(xt, node_id, false);
  }

  void compute_and_publish_tree(const ml4kp_bridge::SpacePointStamped& xt, const NodeIdx& node_id, const bool first)
  {
    // const NodeIdx current_idx{ _ids_map[node_id] };  // node_id
    const NodeIdx current_idx{ _ids_revmap[node_id] };
    // DEBUG_VARS(node_id, current_idx)

    const NodeIdx closest{ find_closest(xt.space_point, current_idx) };
    const NodeIdx next{ first ? closest : _tree.nodes[closest].children[0] };
    // DEBUG_VARS()

    get_future_tree(next, _total_future_nodes);
    _tree_ahead.root = _ids_map[next];
    // _tree_ahead.edges[_tree_ahead.nodes[next].parent_edge] = _ids_map[next];

    // DEBUG_VARS(_tree_ahead);
    _tree_ahead_publisher.publish(_tree_ahead);
    _tree_ahead.nodes.clear();
    _tree_ahead.edges.clear();
  }

  NodeIdx get_next_node_id()
  {
    const NodeIdx node_id{ _next_idx };
    _next_idx++;
    if (_ids_revmap.count(node_id) == 0)
      return node_id;
    else
      return get_next_node_id();
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    // _tree.root = msg->root;
    // _tree.nodes = msg->nodes;
    // _tree.edges = msg->edges;

    _tree.clear();
    _tree.copy(msg);

    for (auto node : _tree.nodes)
    {
      _ids_map[node.first] = node.first;
      _ids_revmap[node.first] = node.first;
    }

    const prx_models::Node& node_current{ _tree.nodes[_tree.root] };
    const prx_models::Node& node_child{ _tree.nodes[node_current.children[0]] };
    _next_tree_edge = node_child.parent_edge;

    PRINT_MSG("SBMP Tree stepper Initialized");

    _tree_recevied = true;
    _start_time = ros::Time::now();

    ml4kp_bridge::SpacePointStamped xt{};
    xt.space_point = _tree.nodes[_tree.root].point;
    compute_and_publish_tree(xt, _tree.root, true);
  }

private:
  std::size_t _next_idx;
  prx_models::Tree _tree_ahead;
  ros::Publisher _tree_ahead_publisher;

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

  ros::Subscriber _tree_subscriber;
  ros::Subscriber _estimation_subscriber;

  ros::Publisher _control_publisher;
  ros::Publisher _stamped_control_publisher;
  ros::Publisher _finish_publisher;
  ros::Publisher _viz_obstacles_publisher;
  ros::Publisher _estimated_tree_publisher;

  ros::Timer _control_timer;
  ros::Timer _control_frequency_timer;

  std::string _world_frame;
  std::string _robot_frame;

  State _state;
  Control _u01;
  Control _u_plan;
  Observation _z_new;
  double _dt01;

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

  const std::string _name{};

  std::size_t _next_tree_edge;
  int _total_future_nodes;

  std::size_t _freq_counter;
  std::size_t _freq_total;
  double _freq_accum;

  std::size_t _goal_id;
  bool _goal_reached;
  bool _visualize;

  std::size_t _current_past_nodes;
  int _total_past_nodes;

  std::unordered_map<std::size_t, std::size_t> _ids_map;
  std::unordered_map<std::size_t, std::size_t> _ids_revmap;
  std::set<std::size_t> _ids;

  std::string _timestamp;
  std::ofstream _ofs;
};
}  // namespace motion_planning
