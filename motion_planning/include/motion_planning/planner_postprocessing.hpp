#pragma once

#include <queue>
#include <stack>
#include <unordered_set>

#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/dirt.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>

#include <prx_models/Graph.h>
#include <prx_models/NodeEdge.h>
#include <prx_models/Tree.h>

namespace motion_planning
{

template <typename Node, typename Edge>
std::shared_ptr<prx::tree_t> build_solution_tree(const std::vector<prx::proximity_node_t*>& goal_nodes,
                                                 const prx::tree_t& tree, prx::space_t* state_space,
                                                 std::shared_ptr<Node> root)
{
  std::stack<Node*> solution_nodes;
  std::unordered_set<prx::node_index_t> visited;
  std::queue<prx::proximity_node_t*> to_visit{};

  for (auto node : goal_nodes)
  {
    to_visit.push(node);
  }

  visited.insert(root->get_index());
  while (to_visit.size() > 0)
  {
    Node* curr_node{ dynamic_cast<Node*>(to_visit.front()) };
    const prx::node_index_t parent{ curr_node->get_parent() };

    if (visited.count(parent) == 0)
    {
      to_visit.push(tree[parent].get());
      visited.insert(parent);
    }
    solution_nodes.push(curr_node);
    to_visit.pop();
  }
  // solution_nodes.push(root);

  std::shared_ptr<prx::tree_t> sln_tree{ std::make_shared<prx::tree_t>() };

  // [ original_index ] -> new_index
  std::unordered_map<prx::node_index_t, prx::node_index_t> new_index_map;

  const prx::node_index_t start_vertex{ sln_tree->add_vertex<Node, Edge>() };
  std::shared_ptr<Node> new_root_node{ sln_tree->get_vertex_as<Node>(start_vertex) };
  new_root_node->point = state_space->make_point();
  state_space->copy(new_root_node->point, root->point);

  new_index_map[root->get_index()] = start_vertex;
  while (not solution_nodes.empty())
  {
    Node* node{ solution_nodes.top() };
    // add node
    const prx::node_index_t node_index{ sln_tree->add_vertex<Node, Edge>() };
    std::shared_ptr<Node> new_tree_node{ sln_tree->get_vertex_as<Node>(node_index) };
    new_tree_node->point = state_space->make_point();
    state_space->copy(new_tree_node->point, node->point);
    new_index_map[node->get_index()] = node_index;

    const std::shared_ptr<Edge> old_edge{ tree.get_edge_as<Edge>(node->get_parent_edge()) };

    const prx::node_index_t parent_index{ new_index_map[node->get_parent()] };
    const prx::edge_index_t edge_index{ sln_tree->add_edge(parent_index, node_index) };
    std::shared_ptr<Edge> new_edge{ sln_tree->get_edge_as<Edge>(edge_index) };
    new_edge->plan = std::make_shared<prx::plan_t>(*(old_edge->plan));
    new_edge->traj = std::make_shared<prx::trajectory_t>(*(old_edge->traj));
    new_edge->edge_cost = old_edge->edge_cost;
    new_tree_node->cost_to_come = node->cost_to_come;

    new_tree_node->duration = node->duration;

    solution_nodes.pop();
  }
  return sln_tree;
}

// Get the tree of solutions from the SBMP tree
template <typename PlannerPtr, typename PlannerQueryPtr>
std::shared_ptr<prx::tree_t> fulfill_stela_query(const prx::param_loader& params, PlannerPtr& planner,
                                                 PlannerQueryPtr& query)
{
  // Assuming params["stela"]
  if (params.exists("goal"))
  {
    query->init(params);

    DEBUG_VARS(*query);

    return planner->tree_of_solutions();
  }
  prx_throw("No valid parameters for fulfill_stela_query ");
  return nullptr;
}

}  // namespace motion_planning