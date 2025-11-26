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

  prx_models::Tree to_msg()
  {
    prx_models::Tree tree;
    tree.root = root;
    for (auto& node : nodes)
    {
      tree.nodes.push_back(node.second);
    }
    for (auto& edge : edges)
    {
      tree.edges.push_back(edge.second);
    }
    return tree;
  }

  EdgeIdx get_child_edge(const NodeIdx& node_idx, const NodeIdx& child)
  {
    const NodeIdx child_idx{ nodes[node_idx].children[child] };
    return nodes[child_idx].parent_edge;
  }

  void copy(const tree_msg_wrapper_t& other)
  {
    root = other.root;
    nodes.insert(other.nodes.begin(), other.nodes.end());
    edges.insert(other.edges.begin(), other.edges.end());
  }

  void copy(const prx_models::TreeConstPtr msg)
  {
    copy(*msg);
  }
  void copy(const prx_models::Tree& other_tree)
  {
    root = other_tree.root;
    copy(other_tree.nodes);
    copy(other_tree.edges);
  }

  // template <typename Container>
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

  // template <typename Container>
  void copy(const std::vector<Edge>& msg_edges)
  {
    for (auto edge : msg_edges)
    {
      edges[edge.index] = edge;
    }
  }

  void clear()
  {
    nodes.clear();
    edges.clear();
  }

  void merge(const prx_models::Tree& other_tree)
  {
    for (auto& other_node : other_tree.nodes)
    {
      nodes[other_node.index] = other_node;
    }

    for (auto& other_edge : other_tree.edges)
    {
      edges[other_edge.index] = other_edge;
    }
  }

  void erase_node(const NodeIdx idx)
  {
    if (nodes.size() == 0)
      return;
    // DEBUG_VARS(idx);
    const auto node_iter = nodes.find(idx);

    if (idx == root)
    {
      // DEBUG_VARS(node_iter->second.children.size())
      // DEBUG_VARS(node_iter->second)

      root = node_iter->second.children[0];
    }
    else
    {
      const EdgeIdx parent_edge{ node_iter->second.parent_edge };

      const auto edge_iter = edges.find(parent_edge);

      edges.erase(edge_iter);
    }

    // DEBUG_PRINT
    nodes.erase(node_iter);
    // DEBUG_PRINT
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
