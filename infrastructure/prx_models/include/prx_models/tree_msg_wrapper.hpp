#pragma once
#include <array>
#include <map>
#include <numeric>
#include <prx_models/Tree.h>

namespace prx_models
{

struct tree_msg_wrapper_t
{
public:
  using EdgeIdx = std::size_t;
  using NodeIdx = std::size_t;

  using Edge = prx_models::Edge;
  using Node = prx_models::Node;

  tree_msg_wrapper_t() {};

  tree_msg_wrapper_t(const prx_models::TreeConstPtr msg)
  {
    copy(msg);
  }

  void copy(const prx_models::TreeConstPtr msg)
  {
    root = msg->root;
    copy(msg->nodes);
    copy(msg->edges);
  }

  void copy(const std::vector<Node>& msg_nodes)
  {
    // DEBUG_VARS(nodes.size(), msg_nodes.size());
    for (auto node : msg_nodes)
    {
      // DEBUG_VARS(node.index);
      nodes[node.index] = node;
    }
    // DEBUG_VARS(nodes.size(), msg_nodes.size());
  }

  void copy(const std::vector<Edge>& msg_edges)
  {
    for (auto edge : msg_edges)
    {
      // DEBUG_VARS(edge.index, edge.source, edge.target);
      edges[edge.index] = edge;
    }
  }

  void clear()
  {
    nodes.clear();
    edges.clear();
  }

  friend void swap(tree_msg_wrapper_t& lhs, tree_msg_wrapper_t& rhs)
  {
    std::swap(lhs.root, rhs.root);
    std::swap(lhs.nodes, rhs.nodes);
    std::swap(lhs.edges, rhs.edges);
  }

  friend std::ostream& operator<<(std::ostream& os, const tree_msg_wrapper_t& obj)
  {
    os << "Root: " << obj.root << "\n";
    os << "Nodes:\n";
    for (auto n : obj.nodes)
    {
      os << "\t" << n.first << ": " << n.second << "\n";
    }
    os << "Edges:\n";
    for (auto e : obj.edges)
    {
      os << "\t" << e.first << ": " << e.second << "\n";
    }
    return os;
  }

  NodeIdx root;
  std::map<NodeIdx, Node> nodes;
  std::map<EdgeIdx, Edge> edges;
};

}  // namespace prx_models
