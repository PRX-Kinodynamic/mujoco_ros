#include <utils/dbg_utils.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <motion_planning/motion_planning_types.hpp>
#include <motion_planning/tree_bridge.hpp>
#include <motion_planning/planner_setup.hpp>
#include <motion_planning/planner_postprocessing.hpp>

#include <analytical/fg_ltv_sde.hpp>

#include <interface/SetInt.h>

namespace motion_planning
{

template <class Base>
class tree_to_trajectories_t : public Base
{
  using Derived = tree_to_trajectories_t<Base>;

public:
  tree_to_trajectories_t()
    : Base()
    , _traj_plan_pub_service_name("/sbmp/tree_to_trajectory")
    , _plan_topic_name("/sbmp/plan")
    , _traj_topic_name("/sbmp/trajectory")
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string tree_topic_name;

    PARAM_SETUP(private_nh, tree_topic_name);
    // PARAM_SETUP_WITH_DEFAULT(private_nh, stela_params, stela_params);

    _traj_plan_pub_service_name = ros::this_node::getNamespace() + _traj_plan_pub_service_name;
    _plan_topic_name = ros::this_node::getNamespace() + _plan_topic_name;
    _traj_topic_name = ros::this_node::getNamespace() + _traj_topic_name;

    // subscriber
    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);

    // publishers
    _plan_publisher = private_nh.advertise<ml4kp_bridge::PlanStamped>(_plan_topic_name, 1, true);
    _traj_publisher = private_nh.advertise<ml4kp_bridge::Trajectory>(_traj_topic_name, 1, true);

    _traj_plan_service = private_nh.advertiseService(_traj_plan_pub_service_name, &Derived::service_callback, this);
    // _goal_marker_publisher = private_nh.advertise<visualization_msgs::Marker>(_goal_marker_topic_name, 1, true);
  }

  bool service_callback(interface::SetInt::Request& req, interface::SetInt::Response& res)
  {
    const std::int64_t value{ req.value };
    res.success = false;
    if (value < _trajs.size())
    {
      const ros::Duration duration{ ml4kp_bridge::duration(_plans[value].plan) };
      _traj_publisher.publish(_trajs[value]);
      _plan_publisher.publish(_plans[value]);
      res.answer = prx::utilities::convert_to<std::string>(duration.toSec());
      res.success = true;
    }
    return res.success;
  }

  void branch_to_traj(const prx_models::TreeConstPtr msg, const std::uint64_t node_id, const std::size_t traj_id)
  {
    const prx_models::Node& node{ msg->nodes[node_id] };
    const prx_models::Edge& edge{ msg->edges[node.parent_edge] };

    const std::size_t total_children{ node.children.size() };

    _trajs[traj_id].data.push_back(node.point);
    // std::copy(edge.plan.steps.begin(), edge.plan.steps.end(), std::back_inserter(_plans[traj_id].steps));
    ml4kp_bridge::append(_plans[traj_id].plan, edge.plan);

    // DEBUG_VARS(node_id, total_children);
    if (total_children >= 1)  // This is not a leaf
    {
      for (int i = 1; i < total_children; ++i)
      {
        const std::size_t child_id{ node.children[i] };
        // DEBUG_VARS(node_id, total_children, i, child_id);

        _trajs.emplace_back();
        _plans.emplace_back();
        // _plans.back().plan.emplace_back();
        ml4kp_bridge::copy(_trajs.back(), _trajs[traj_id]);
        ml4kp_bridge::copy(_plans.back().plan, _plans[traj_id].plan);

        branch_to_traj(msg, child_id, _trajs.size() - 1);
      }
      branch_to_traj(msg, node.children[0], traj_id);
    }
    // DEBUG_VARS(node_id, _trajs.size());
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    const prx_models::Node& node{ msg->nodes[msg->root] };

    for (auto child_id : node.children)
    {
      if (child_id == msg->root)  // The root node is its own parent :/
        continue;
      _trajs.emplace_back();
      _plans.emplace_back();
      // _plans.back().plan.emplace_back();
      _trajs.back().data.push_back(node.point);

      branch_to_traj(msg, child_id, _trajs.size() - 1);
    }
    const std::size_t total_trajectories{ _trajs.size() };
    DEBUG_VARS(total_trajectories);
  }

  // Topic names
  std::string _traj_plan_pub_service_name;
  std::string _plan_topic_name;
  std::string _traj_topic_name;

  std::vector<ml4kp_bridge::PlanStamped> _plans;
  std::vector<ml4kp_bridge::Trajectory> _trajs;

  // Subscribers
  ros::Subscriber _tree_subscriber;

  // Publishers
  ros::Publisher _traj_publisher;
  ros::Publisher _plan_publisher;
  // ros::Publisher _goal_marker_publisher;

  // Services
  ros::ServiceServer _traj_plan_service;
};
}  // namespace motion_planning