#pragma once
#include <array>
#include <map>
#include <numeric>
#include <prx_models/Tree.h>
#include <utils/dbg_utils.hpp>

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

  tree_msg_wrapper_t(const prx_models::Tree& msg)
  {
    copy(msg);
  }

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
    for (auto node : other.nodes)
    {
      nodes[node.first] = node.second;
    }
    for (auto edge : other.edges)
    {
      edges[edge.first] = edge.second;
    }
    // nodes.insert(other.nodes.begin(), other.nodes.end());
    // edges.insert(other.edges.begin(), other.edges.end());
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

  void erase_child(const NodeIdx node_idx, const NodeIdx idx_to_remove)
  {
    Node& node{ nodes[node_idx] };
    for (auto iter = node.children.begin(); iter != node.children.end(); iter++)
    {
      if ((*iter) == idx_to_remove)
      {
        node.children.erase(iter);
        return;
      }
    }
  }

  void erase_edge(const EdgeIdx idx)
  {
    const auto edge_iter = edges.find(idx);
    edges.erase(edge_iter);
  }

  void erase_node(const NodeIdx idx)
  {
    if (nodes.size() == 0)
      return;
    // PRINT_MSG("erase_node")
    // DEBUG_VARS(idx);
    const auto node_iter = nodes.find(idx);
    // EdgeIdx parent_edge;

    if (idx == root)
    {
      // PRINT_MSG("Removing root");
      // DEBUG_VARS(node_iter->second.children.size())
      // DEBUG_VARS(node_iter->second)

      root = node_iter->second.children[0];
      nodes[root].parent = root;
      erase_edge(nodes[root].parent_edge);
      // parent_edge = nodes[root].parent_edge;
    }
    else
    {
      // parent_edge = node_iter->second.parent_edge;
      const EdgeIdx parent_edge{ node_iter->second.parent_edge };
      const NodeIdx parent_node{ edges[parent_edge].source };
      erase_child(parent_node, idx);

      // DEBUG_VARS(parent_edge, parent_node)
      // nodes[edges[parent_edge].source].children.erase(idx);
      erase_edge(parent_edge);
    }
    // const auto edge_iter = edges.find(parent_edge);
    // edges.erase(edge_iter);

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
    std::regex nl_re("\\n");
    for (auto n : obj.nodes)
    {
      std::stringstream strstr;
      strstr << n.second;
      const std::string str{ strstr.str() };
      os << "\t" << n.first << ":\n\t\t";
      std::regex_replace(std::ostreambuf_iterator<char>(os), str.begin(), str.end() - 1, nl_re, "\n\t\t");
      os << str.back();
    }
    os << "Edges:\n";
    for (auto e : obj.edges)
    {
      std::stringstream strstr;
      strstr << e.second;
      const std::string str{ strstr.str() };
      os << "\t" << e.first << ":\n\t\t";
      std::regex_replace(std::ostreambuf_iterator<char>(os), str.begin(), str.end() - 1, nl_re, "\n\t\t");
      os << str.back();
    }
    return os;
  }

  NodeIdx root;
  std::map<NodeIdx, Node> nodes;
  std::map<EdgeIdx, Edge> edges;
};

}  // namespace prx_models
