#pragma once

#include <ml4kp_bridge/defs.h>

#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/playback/trajectory.hpp>
#include <prx_models/Tree.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace prx_models
{

std::pair<prx_models::Edge, prx_models::Node> create_edge_node(prx_models::Node& parent, std::size_t& next_index)
{
  // DEBUG_VARS(next_index, parent)
  prx_models::Edge edge;
  prx_models::Node node;

  edge.index = parent.index;
  edge.source = parent.index;
  edge.target = next_index;

  node.index = next_index;
  node.parent = parent.index;
  node.parent_edge = edge.index;

  parent.children.push_back(node.index);

  next_index++;
  return { edge, node };
}

void tree_from_plan_traj(prx_models::Tree& sln_tree, const prx::plan_t& plan, const prx::trajectory_t& traj,
                         const double max_edge_duration)
{
  using EdgeNodePair = std::pair<prx_models::Edge, prx_models::Node>;

  if (traj.duration() < plan.duration())
  {
    PRINT_MSG("[Replanner::tree_from_plan_traj] Trajectory shorter than plan");
    DEBUG_VARS(traj.duration(), plan.duration());
    // DEBUG_VARS(plan)
    // DEBUG_VARS(traj)
    return;
  }

  std::size_t current_idx{ sln_tree.root + 1 };
  // sln_tree.root = current_idx;
  double ti{ 0.0 };
  double curr_cost{ 0.0 };

  for (std::size_t i = 0; i < plan.size(); ++i)
  {
    const prx::plan_step_t ps_i{ plan[i] };

    double dt_remaining{ ps_i.duration };
    while (dt_remaining > 0.0001)  // small epsilon
    {
      // DEBUG_VARS(dt_remaining)
      EdgeNodePair edge_node{ create_edge_node(sln_tree.nodes.back(), current_idx) };

      const double dt_curr{ std::min(max_edge_duration, dt_remaining) };
      // DEBUG_VARS(ti, dt_curr)
      const prx::space_point_t xi{ traj.at(ti + dt_curr, false) };
      edge_node.first.plan.steps.emplace_back();

      ml4kp_bridge::copy(edge_node.first.plan.steps.back(), ps_i);
      edge_node.first.plan.steps.back().duration.data = ros::Duration(dt_curr);

      ml4kp_bridge::copy(edge_node.second.point, xi);

      sln_tree.edges.push_back(edge_node.first);
      sln_tree.nodes.push_back(edge_node.second);

      dt_remaining = dt_remaining - max_edge_duration;
      ti += dt_curr;
    }
    // ti += ps_i.duration;
  }
}
}  // namespace prx_models
