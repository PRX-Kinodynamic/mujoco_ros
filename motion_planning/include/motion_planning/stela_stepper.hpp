#include <thread>
#include <fstream>
#include <ml4kp_bridge/defs.h>
#include <prx_models/MushrPlanner.h>
#include <prx_models/mj_mushr.hpp>
#include <motion_planning/single_shot_planner_service.hpp>
#include <motion_planning/planner_client.hpp>

#include <utils/std_utils.cpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

#include <interface/SetInt.h>
#include <interface/SetDuration.h>

#include <actionlib/client/simple_action_client.h>
#include <motion_planning/StelaGraphTraversalAction.h>

#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_planning/StelaGraphTraversalAction.h>
namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class stela_stepper_t : public Base
{
  using Derived = stela_stepper_t<SystemInterface, Base>;
  using StelaActionClient = actionlib::SimpleActionClient<motion_planning::StelaGraphTraversalAction>;
  using CostTrajectoryPair = std::pair<double, std::vector<std::uint64_t>>;

  enum BranchSelection
  {
    MIN_COST = 0,
    RANDOM
  };

public:
  stela_stepper_t() : _node_epsilon(0.01){};
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    // double frequency{ 0.0 };
    std::string action_topic{ "" };
    std::string tree_topic_name{ "" };
    std::string branch_selection{ "min_cost" };
    double& node_epsilon_radius{ _node_epsilon };
    double& lookahead_duration{ _lookahead_duration };
    double& cost_threashold{ _cost_threashold };
    int random_seed{ 112992 };

    // PARAM_SETUP(private_nh, frequency);
    PARAM_SETUP(private_nh, action_topic);
    PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP(private_nh, lookahead_duration);
    PARAM_SETUP(private_nh, cost_threashold);
    PARAM_SETUP_WITH_DEFAULT(private_nh, random_seed, random_seed);
    PARAM_SETUP_WITH_DEFAULT(private_nh, branch_selection, branch_selection);
    PARAM_SETUP_WITH_DEFAULT(private_nh, node_epsilon_radius, node_epsilon_radius);

    if (branch_selection == "min_cost")
    {
      _branch_selection = BranchSelection::MIN_COST;
    }
    else if (branch_selection == "random")
    {
      _branch_selection = BranchSelection::RANDOM;
    }
    DEBUG_VARS(branch_selection, _branch_selection);
    // DEBUG_VARS(frequency);
    const std::string stela_tree_topic_name{ ros::this_node::getNamespace() + "/lookahead" };

    _action_client = std::make_unique<StelaActionClient>(action_topic, true);

    // _stepper_timer = private_nh.createTimer(ros::Duration(1.0 / frequency), &Derived::send_goal, this);
    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);

    _stela_tree_publisher = private_nh.advertise<prx_models::Tree>(stela_tree_topic_name, 1);

    DEBUG_VARS(action_topic);
    DEBUG_VARS(tree_topic_name);

    _goal.header.seq = 0;
    prx::init_random(random_seed);

    // _action_client->waitForServer();
    // DEBUG_VARS("");
  }

  template <typename TreePtr>
  double select_branch(const TreePtr msg, const std::uint64_t edge_id, const std::size_t traj_id, const double cost)
  {
    const prx_models::Edge& edge{ msg->edges[edge_id] };
    const prx_models::Node& node_parent{ msg->nodes[edge.source] };
    const prx_models::Node& node_current{ msg->nodes[edge.target] };

    const std::size_t total_children{ node_current.children.size() };

    const double current_cost{ cost + node_current.cost };
    // DEBUG_VARS(traj_id, total_children);
    _node_trajectories[traj_id].second.push_back(edge.target);
    // DEBUG_VARS(traj_id, _node_trajectories[traj_id].second.size());
    // DEBUG_VARS(node_current.index, node_current.children);
    double min_cost{ std::numeric_limits<double>::max() };
    if (total_children == 0)
    {
      _node_trajectories[traj_id].first = current_cost;
      min_cost = current_cost;
    }
    else if (total_children >= 1)  // This is not a leaf
    {
      for (int i = total_children - 1; i >= 0; --i)
      {
        const prx_models::Node& node_child{ msg->nodes[node_current.children[i]] };

        std::size_t new_traj_id{ traj_id };
        if (i >= 1)
        {
          // const std::vector<std::uint64_t>& curr_traj{ _node_trajectories[traj_id].second };
          _node_trajectories.emplace_back();
          new_traj_id = _node_trajectories.size() - 1;
          // DEBUG_VARS(new_traj_id, _node_trajectories[traj_id].second.size());
          // DEBUG_VARS(_node_trajectories[new_traj_id].second.size());
          _node_trajectories[new_traj_id].second = _node_trajectories[traj_id].second;
        }
        const double cost_fwd{ select_branch(msg, node_child.parent_edge, new_traj_id, current_cost) };
        min_cost = std::min(min_cost, cost_fwd);
      }
    }
    _cost_to_go[node_current.index] = min_cost;
    return min_cost;
  }

  bool compute_goal(const motion_planning::StelaGraphTraversalFeedbackConstPtr& feedback, const std::uint64_t edge_id,
                    const double cost, const double total_duration)
  {
    const prx_models::Edge& edge{ _tree.edges[edge_id] };
    const prx_models::Node& node_parent{ _tree.nodes[edge.source] };
    prx_models::Node& node_current{ _tree.nodes[edge.target] };

    // DEBUG_VARS(node_current.index, cost, total_duration);
    if (total_duration >= _lookahead_duration)
    {
      _goal.selected_branch.push_back(node_current.index);
      return true;
    }

    const double estimated_cost{ feedback == nullptr ? 0.0 : std::fabs(feedback->lookahead_costs[node_current.index]) };

    const double accum_cost{ cost + estimated_cost };

    std::vector<std::pair<double, std::uint64_t>> children_costs;
    // if (accum_cost <= _cost_threashold)
    // {
    if (node_current.children.size() == 0)
    {
      _goal.selected_branch.push_back(node_current.index);
      return true;
    }
    for (auto child_id : node_current.children)
    {
      if (child_id == node_current.index)
        continue;
      const double child_cost{ _cost_to_go[child_id] };
      children_costs.push_back(std::make_pair(child_cost, child_id));
      // DEBUG_VARS(child_id, child_cost);
    }
    if (_branch_selection == BranchSelection::MIN_COST)
    {
      std::sort(children_costs.begin(), children_costs.end(), _cost_id_cmp);
      node_current.children.clear();
      for (auto cost_id_pair : children_costs)
      {
        node_current.children.push_back(cost_id_pair.second);
      }
      // std::swap(node_current.children[0], node_current.children[children_costs[0].second]);
    }
    else if (_branch_selection == BranchSelection::RANDOM and children_costs.size() > 1)
    {
      if (_random_selection.count(node_current.index) == 0)
      {
        std::shuffle(children_costs.begin(), children_costs.end(), prx::global_generator);

        node_current.children.clear();
        for (auto cost_id_pair : children_costs)
        {
          node_current.children.push_back(cost_id_pair.second);
        }
        _random_selection[node_current.index] = 1;
        // DEBUG_VARS(node_current.index, node_current.children);
      }
    }

    const ml4kp_bridge::Plan& plan{ edge.plan };
    const double next_duration{ ml4kp_bridge::duration(plan).toSec() };
    const double accum_duration{ total_duration + next_duration };

    for (auto child_cost_id_pair : children_costs)
    {
      const std::uint64_t child_id{ child_cost_id_pair.second };
      const prx_models::Node& node_child{ _tree.nodes[child_id] };

      const bool valid_branch{ compute_goal(feedback, node_child.parent_edge, accum_cost, accum_duration) };
      if (valid_branch)
      {
        _goal.selected_branch.push_back(node_current.index);
        return true;
      }
      else
      {
        _goal.selected_branch.pop_back();
      }
    }
    if (accum_cost > _cost_threashold)
    {
      // ROS_INFO("Saving alternative:");
      _current_selected_branch.clear();
      // DEBUG_VARS(_goal.selected_branch);
      // std::copy(_goal.selected_branch.begin(), _goal.selected_branch.end(),
      //           std::back_inserter(_current_selected_branch));
      // _current_selected_branch = _goal.selected_branch;
      // DEBUG_VARS(_current_selected_branch);
    }

    return false;
  }

  void populate_goal()
  {
    double total_duration{ 0.0 };
    _goal.selected_branch.clear();
    _goal.durations.clear();

    const std::size_t trajectory_size{ _node_trajectories[0].second.size() };
    // std::size_t state_id{ 0 };
    std::size_t state_id{ _current_state_id };
    while (total_duration < _lookahead_duration and state_id + 1 < trajectory_size)
    {
      const std::uint64_t node_id{ _node_trajectories[0].second[state_id] };
      const std::uint64_t next_node_id{ _node_trajectories[0].second[state_id + 1] };

      const ml4kp_bridge::Plan& plan{ _tree.edges[_tree.nodes[next_node_id].parent_edge].plan };
      const double current_duration{ ml4kp_bridge::duration(plan).toSec() };

      _goal.selected_branch.push_back(node_id);

      _goal.durations.push_back(current_duration);
      total_duration += current_duration;
      state_id++;
    }

    _current_state_id++;
  }

  void select_branch()
  {
    _node_trajectories.clear();
    const prx_models::Node& node{ _tree.nodes[_current_root] };
    double min_cost{ std::numeric_limits<double>::max() };
    for (auto child_id : node.children)
    {
      if (child_id == _current_root)  // The root node is its own parent :/
        continue;
      const prx_models::Node& node_child{ _tree.nodes[child_id] };
      _node_trajectories.emplace_back();
      _node_trajectories.back().second.push_back(_current_root);

      min_cost =
          std::min(select_branch(&_tree, node_child.parent_edge, _node_trajectories.size() - 1, node.cost), min_cost);
    }
    _cost_to_go[_current_root] = min_cost;

    std::sort(_node_trajectories.begin(), _node_trajectories.end(), _trajectory_cmp);
  }

  template <typename FeedbackPtr>
  void compute_goal(FeedbackPtr feedback)
  {
    bool valid_branch{ false };
    // DEBUG_VARS(_current_root, _tree.nodes[_current_root].children);
    for (auto child_id : _tree.nodes[_current_root].children)
    {
      if (child_id == _current_root)
        continue;
      // DEBUG_VARS(_current_root, child_id);
      const std::uint64_t edge_id{ _tree.nodes[child_id].parent_edge };
      valid_branch = compute_goal(feedback, edge_id, 0.0, 0.0);
      if (valid_branch)
      {
        ROS_INFO("Valid branch found!");
        _goal.selected_branch.push_back(_current_root);
        break;
      }
    }
    if (not valid_branch)
    {
      ROS_INFO("Using backup traj");
      std::copy(_current_selected_branch.begin(), _current_selected_branch.end(),
                std::back_inserter(_goal.selected_branch));
    }
    std::reverse(_goal.selected_branch.begin(), _goal.selected_branch.end());
    // DEBUG_VARS(_goal.selected_branch);
    _goal.header.seq++;
    _goal.header.stamp = ros::Time::now();
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    const prx_models::Node& node{ msg->nodes[msg->root] };
    // DEBUG_VARS(msg->root);
    _tree.root = msg->root;
    _tree.nodes = msg->nodes;
    _tree.edges = msg->edges;
    _current_root = msg->root;
    select_branch();

    _current_traj = 0;
    _current_state_id = 0;
    // populate_goal();
    compute_goal(nullptr);

    _action_client->waitForServer();
    ROS_INFO("Action client connected to server");
    _goal.header.seq++;
    _goal.header.stamp = ros::Time::now();
    // _next_goal_check = ros::Time::now() + ros::Duration(_goal.durations.front());
    _action_client->sendGoal(_goal,                                    // no-lint
                             StelaActionClient::SimpleDoneCallback(),  // no-lint
                             StelaActionClient::SimpleActiveCallback(),
                             boost::bind(&Derived::action_feedback_callback, this, _1));
    ROS_INFO("Goal sent");
  }

private:
  // Called every time feedback is received for the goal
  void action_feedback_callback(const motion_planning::StelaGraphTraversalFeedbackConstPtr& feedback)
  {
    const ros::Time now{ ros::Time::now() };

    if (now >= _next_goal_check)
    {
      ROS_INFO("Action callback");
      _current_root = feedback->current_root;

      _goal.selected_branch.clear();
      _goal.durations.clear();

      compute_goal(feedback);

      _next_goal_check = now + ros::Duration(0.2);

      _action_client->sendGoal(_goal,                                    // no-lint
                               StelaActionClient::SimpleDoneCallback(),  // no-lint
                               StelaActionClient::SimpleActiveCallback(),
                               boost::bind(&Derived::action_feedback_callback, this, _1));
      // DEBUG_VARS(_goal.selected_branch.front(), _next_goal_check);
    }
    _stela_tree_publisher.publish(feedback->lookahead);
  }

  template <typename PairToCompare>
  struct pair_comparison_t
  {
    bool operator()(const PairToCompare& a, const PairToCompare& b) const
    {
      return a.first < b.first;
    }
  };

  pair_comparison_t<CostTrajectoryPair> _trajectory_cmp;
  pair_comparison_t<std::pair<double, std::uint64_t>> _cost_id_cmp;
  std::unique_ptr<StelaActionClient> _action_client;

  std::vector<CostTrajectoryPair> _node_trajectories;

  std::unordered_map<std::uint64_t, double> _cost_to_go;
  ros::Timer _stepper_timer;
  ros::Subscriber _tree_subscriber;
  ros::Publisher _stela_tree_publisher;

  motion_planning::StelaGraphTraversalGoal _goal;

  prx_models::Tree _tree;

  std::size_t _x2_id;
  std::size_t _current_state_id;

  double _node_epsilon;
  double _lookahead_duration;
  double _cost_threashold;

  ros::Time _next_goal_check;
  std::size_t _current_traj;
  std::size_t _current_root;

  std::vector<std::uint64_t> _current_selected_branch;
  BranchSelection _branch_selection;

  std::unordered_map<std::uint64_t, std::uint64_t> _random_selection;
};

}  // namespace motion_planning