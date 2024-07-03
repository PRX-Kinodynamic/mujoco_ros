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
class random_branch_t : public Base
{
  using Derived = random_branch_t<Base>;

public:
  random_branch_t() : Base(), _traj_plan_pub_service_name("/sbmp/tree_to_trajectory")
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    int random_seed{ 112992 };

    std::string tree_topic_name;
    std::string plan_topic_name;
    std::string traj_topic_name;
    std::string shutdown_topic{ "" };

    PARAM_SETUP(private_nh, plan_topic_name);
    PARAM_SETUP(private_nh, traj_topic_name);
    PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, shutdown_topic, shutdown_topic);
    PARAM_SETUP_WITH_DEFAULT(private_nh, random_seed, random_seed);
    // PARAM_SETUP_WITH_DEFAULT(private_nh, stela_params, stela_params);
    prx::init_random(random_seed);

    _traj_plan_pub_service_name = ros::this_node::getNamespace() + _traj_plan_pub_service_name;
    // _plan_topic_name = ros::this_node::getNamespace() + _plan_topic_name;
    // _traj_topic_name = ros::this_node::getNamespace() + _traj_topic_name;

    // subscriber
    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::tree_callback, this);

    // publishers
    _plan_publisher = private_nh.advertise<ml4kp_bridge::PlanStamped>(plan_topic_name, 1, true);
    _traj_publisher = private_nh.advertise<ml4kp_bridge::Trajectory>(traj_topic_name, 1, true);

    _traj_plan_service = private_nh.advertiseService(_traj_plan_pub_service_name, &Derived::service_callback, this);

    if (shutdown_topic != "")
      _shutdown_subscriber = private_nh.subscribe(shutdown_topic, 1, &utils::shutdown_callback<std_msgs::Bool>);
  }

  // bool service_callback(interface::SetInt::Request& req, interface::SetInt::Response& res)
  bool service_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    _traj_publisher.publish(_traj);
    _plan_publisher.publish(_plan);
    return true;
  }

  void branch_to_traj(const prx_models::TreeConstPtr msg, const std::uint64_t node_id)
  {
    const prx_models::Node& node{ msg->nodes[node_id] };
    const prx_models::Edge& edge{ msg->edges[node.parent_edge] };

    const std::size_t total_children{ node.children.size() };

    _traj.data.push_back(node.point);
    // std::copy(edge.plan.steps.begin(), edge.plan.steps.end(), std::back_inserter(_plans[traj_id].steps));
    ml4kp_bridge::append(_plan.plan, edge.plan);

    if (total_children >= 1)  // This is not a leaf
    {
      const int idx{ prx::uniform_int_random(0, total_children - 1) };
      const std::size_t child_id{ node.children[idx] };

      // _trajs.emplace_back();
      // _plans.emplace_back();
      // _plans.back().plan.emplace_back();
      // ml4kp_bridge::copy(_trajs.back(), _trajs[traj_id]);
      // ml4kp_bridge::copy(_plans.back().plan, _plans[traj_id].plan);

      branch_to_traj(msg, child_id);
      // branch_to_traj(msg, node.children[0], traj_id);
    }
    // DEBUG_VARS(node_id, _trajs.size());
  }

  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    const prx_models::Node& node{ msg->nodes[msg->root] };

    const std::size_t total_children{ node.children.size() };
    // for (auto child_id : node.children)
    while (true)
    {
      const int idx{ prx::uniform_int_random(0, total_children - 1) };
      const std::size_t child_id{ node.children[idx] };
      if (child_id == msg->root)  // The root node is its own parent :/
        continue;
      // _trajs.emplace_back();
      // _plans.emplace_back();
      // _plans.back().plan.emplace_back();
      _traj.data.push_back(node.point);

      branch_to_traj(msg, child_id);
      break;
    }
    // const std::size_t total_trajectories{ _trajs.size() };
    // DEBUG_VARS(total_trajectories);
  }

  // Topic names
  std::string _traj_plan_pub_service_name;

  ml4kp_bridge::PlanStamped _plan;
  ml4kp_bridge::Trajectory _traj;

  // Subscribers
  ros::Subscriber _tree_subscriber;
  ros::Subscriber _shutdown_subscriber;

  // Publishers
  ros::Publisher _traj_publisher;
  ros::Publisher _plan_publisher;
  // ros::Publisher _goal_marker_publisher;

  // Services
  ros::ServiceServer _traj_plan_service;
};
}  // namespace motion_planning