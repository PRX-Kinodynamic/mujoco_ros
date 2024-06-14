#pragma once

#include <prx/utilities/data_structures/tree.hpp>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/plan_step_bridge.hpp>
#include <ml4kp_bridge/plan_bridge.hpp>
#include <ml4kp_bridge/space_bridge.hpp>
#include <ml4kp_bridge/trajectory_bridge.hpp>

#include <prx_models/Graph.h>
#include <prx_models/NodeEdge.h>
#include <prx_models/Tree.h>
#include <motion_planning/motion_planning_types.hpp>

namespace motion_planning
{
template <typename Node>
void copy(prx_models::Node& ros_node, const Node& mlkp_node)
{
  ros_node.children.clear();

  ros_node.index = mlkp_node.get_index();
  ros_node.parent = mlkp_node.get_parent();
  ros_node.parent_edge = mlkp_node.get_parent_edge();
  ros_node.status = Types::NodeEdgeStatus::ADDED;
  ros_node.cost = mlkp_node.cost();
  const std::list<prx::node_index_t>& children{ mlkp_node.get_children() };
  ros_node.children.resize(children.size());
  std::copy(children.begin(), children.end(), ros_node.children.begin());

  ml4kp_bridge::copy(ros_node.point, mlkp_node.point);
}

template <typename Edge>
void copy(prx_models::Edge& ros_edge, const Edge& mlkp_edge)
{
  ros_edge.index = mlkp_edge.get_index();
  ros_edge.source = mlkp_edge.get_source();
  ros_edge.target = mlkp_edge.get_target();
  ros_edge.status = Types::NodeEdgeStatus::ADDED;
  ml4kp_bridge::copy(ros_edge.plan, mlkp_edge.plan);
}

template <typename Node, typename Edge>
void copy(prx_models::Tree& ros_tree, const prx::tree_t& mlkp_tree)
{
  auto vertex_iters_pairs = mlkp_tree.vertices();
  ros_tree.nodes.resize(mlkp_tree.num_vertices());

  for (auto iter = vertex_iters_pairs.first; iter != vertex_iters_pairs.second; iter++)
  {
    const std::shared_ptr<Node> node{ std::dynamic_pointer_cast<Node>(*iter) };
    prx_assert(node != nullptr, "Node is null!");
    // int i = ros_tree.nodes.size();
    const prx::node_index_t index{ node->get_index() };
    if (ros_tree.nodes.size() <= index)
    {
      ros_tree.nodes.resize(index + 1);
    }
    copy(ros_tree.nodes[index], *node);
  }
  auto edges_iters_pairs = mlkp_tree.edges();
  ros_tree.edges.resize(mlkp_tree.num_edges());
  // DEBUG_VARS((*edges_iters_pairs.first)->get_index());
  // DEBUG_VARS((*edges_iters_pairs.second)->get_index());
  for (auto iter = edges_iters_pairs.first; iter != edges_iters_pairs.second; iter++)
  {
    // *_tree.get_edge_as<rrt_edge_t>((*iter)->get_index())->traj;
    const prx::edge_index_t index{ (*iter)->get_index() };
    const std::shared_ptr<Edge> edge{ mlkp_tree.get_edge_as<Edge>(index) };
    prx_assert(edge != nullptr, "Edge is null!");
    if (ros_tree.edges.size() <= index)
    {
      ros_tree.edges.resize(index + 1);
    }
    copy(ros_tree.edges[index], *edge);
  }
}

class tree_manager_t
{
  using EdgeNodePair = std::pair<prx_models::Edge, prx_models::Node>;

public:
  tree_manager_t() : _next_edge_index(0)
  {
    _nodes.resize(100);
    _edges.resize(100);
  }

  prx_models::Edge create_edge(prx_models::Node& from, prx_models::Node& to)
  {
    if (_edges.size() <= 1)
    {
      _edges.resize(100);
    }
    prx_models::Edge edge{ std::move(_edges.front()) };
    _edges.pop_front();

    const std::uint64_t from_id{ from.index };
    const std::uint64_t to_id{ to.index };

    from.children.push_back(to_id);
    to.parent = from_id;

    edge.index = _next_edge_index;
    edge.source = from_id;
    edge.target = to_id;

    to.parent_edge = edge.index;

    _next_edge_index++;
    return edge;
  }

  prx_models::Node create_node()
  {
    if (_nodes.size() <= 1)
    {
      _nodes.resize(100);
    }
    prx_models::Node node{ std::move(_nodes.front()) };
    _nodes.pop_front();

    node.index = _next_node_index;
    node.parent = _next_node_index;
    _next_node_index++;
    return node;
  }

  // Given a node N0, create edge and node child: N0 -- E0 -- N1
  EdgeNodePair create_edge_node(prx_models::Node& n0)
  {
    prx_models::Node n1{ std::move(create_node()) };
    prx_models::Edge e0{ std::move(create_edge(n0, n1)) };
    return { e0, n1 };
  }

  void reset()
  {
    _next_node_index = 0;
    _next_edge_index = 0;
  }

private:
  std::list<prx_models::Node> _nodes;
  std::list<prx_models::Edge> _edges;
  std::uint64_t _next_node_index;
  std::uint64_t _next_edge_index;
};
}  // namespace motion_planning