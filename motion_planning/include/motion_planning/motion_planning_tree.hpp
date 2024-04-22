#include <utils/dbg_utils.hpp>
#include <std_msgs/String.h>

#include <motion_planning/motion_planning_types.hpp>

namespace motion_planning
{

template <class Base>
class mp_tree_t : public Base
{
  using Derived = mp_tree_t<Base>;

public:
  mp_tree_t() : _add_node_topic_name("/mp_tree/add_node"), _tree_topic_name("/mp_tree/tree")
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;
    _add_node_topic_name = ros::this_node::getNamespace() + _add_node_topic_name;

    // subscribers
    _add_node_subscriber = private_nh.subscribe(_add_node_topic_name, 100, &Derived::add_node, this);

    // publishers
    _tree_publisher = private_nh.advertise<prx_models::Graph>(_tree_topic_name, 1);

    // timers
    _tree_timer = private_nh.createTimer(ros::Duration(1), &Derived::publish_graph, this);

    allocate_memory(1000);
  }

  inline void allocate_memory(const std::size_t& new_size)
  {
    if (_tree.nodes.capacity() <= new_size)  // using a buffer of 10 (randomly)
    {
      _tree.nodes.resize(new_size);
      _tree.edges.resize(new_size);  // Technically this should be nodes-1, one extra because why not...
    }
  }

  // Copy an edge into another:
  // Edge_to <- Edge_from
  inline void copy_edge(prx_models::Edge& to, const prx_models::Edge& from)
  {
    to.index = static_cast<std::size_t>(from.index);
    to.source = static_cast<std::size_t>(from.source);
    to.target = static_cast<std::size_t>(from.target);
    to.plan = from.plan;
  }

  void add_node(const prx_models::NodeEdgeConstPtr msg)
  {
    // DEBUG_PRINT;
    const std::size_t node_id{ static_cast<std::size_t>(msg->node.index) };
    const std::size_t edge_id{ static_cast<std::size_t>(msg->edge.index) };

    allocate_memory(node_id);

    _tree.edges[edge_id] = msg->edge;
    _tree.nodes[node_id] = msg->node;

    _tree.edges[edge_id].status = Types::NodeEdgeStatus::ADDED;
    _tree.nodes[node_id].status = Types::NodeEdgeStatus::ADDED;
  }

  void publish_graph(const ros::TimerEvent& event)
  {
    // DEBUG_VARS(ros::Time::now(), _add_node_subscriber.getNumPublishers());

    _tree_publisher.publish(_tree);
  }

  // Topic names
  std::string _add_node_topic_name;
  std::string _tree_topic_name;

  // Subscribers
  ros::Subscriber _add_node_subscriber;

  // Publishers
  ros::Publisher _tree_publisher;

  // Timers
  ros::Timer _tree_timer;

  // Graph
  prx_models::Graph _tree;
};
}  // namespace motion_planning