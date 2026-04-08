#pragma once

namespace motion_planning
{
template <typename RobotInterface>
struct tree_validation_params_t
{
  using StateEstimates = typename RobotInterface::StateEstimates;

  std::shared_ptr<RobotInterface> robot;
  StateEstimates estimates;
  std::vector<Eigen::MatrixXd> covariances;

  gtsam::LevenbergMarquardtParams lm_params;
};

template <typename RobotInterface>
bool check_new_tree(prx_models::tree_msg_wrapper_t& new_tree, const tree_validation_params_t<RobotInterface> params)
{
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  Values proposed_values;
  FactorGraph proposed_graph;

  prx_models::tree_msg_wrapper_t::NodeIdx curr_node_idx{ new_tree.root };

  GraphValues graph_values_0{ params.robot->estimate_to_prior(curr_node_idx, params.estimates, params.covariances) };

  proposed_graph.push_back(graph_values_0.first);
  proposed_values.insert(graph_values_0.second);

  while (new_tree.nodes[curr_node_idx].children.size() > 0)
  {
    const prx_models::tree_msg_wrapper_t::NodeIdx child_idx{ new_tree.nodes[curr_node_idx].children[0] };
    const prx_models::Node& node{ new_tree.nodes[child_idx] };
    const prx_models::Edge& edge{ new_tree.edges[node.parent_edge] };

    GraphValues graph_values{ params.robot->node_edge_to_fg(node, edge) };

    proposed_graph.push_back(graph_values.first);
    proposed_values.insert(graph_values.second);

    curr_node_idx = child_idx;
  }

  try
  {
    gtsam::LevenbergMarquardtOptimizer optimizer(proposed_graph, proposed_values, params.lm_params);
    const Values result{ optimizer.optimize() };
    const double validation_error{ optimizer.error() };

    const bool accept_tree{ validation_error < 1.0 };

    // const ros::WallTime optimization_stamp{ ros::WallTime::now() };

    LOG_VARS(validation_error, accept_tree);
    return accept_tree;
  }
  catch (gtsam::ValuesKeyDoesNotExist e)
  {
    PRINT_MSG("check_new_tree");
    PRINT_MSG("[check_new_tree] Problem with test factor graph");
    PRINT_KEYS(e.key())
    DEBUG_VARS(e.what())
    LOG_CLOSE

    throw e;
  }
  catch (gtsam::IndeterminantLinearSystemException exception)
  {
    const std::string exception_nearby_variable{ SF::formatter(exception.nearbyVariable()) };
    LOG_VARS(exception_nearby_variable);
    LOG_VARS(exception.what());
    LOG_CLOSE
    throw;
  }
  return false;
}
}  // namespace motion_planning