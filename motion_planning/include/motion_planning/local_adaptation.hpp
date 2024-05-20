#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_copy.hpp>
#include <ros/ros.h>

namespace mj_ros
{
// template <typename Model, typename IntegrationFactor, typename DynamicsFactor, typename ObservationFactor>
template <typename Nodelet>
class local_adaptation_t : public Nodelet
{
  using LocalAdapter = local_adaptation_t<Nodelet>;
  using NoiseModel = typename gtsam::noiseModel::Base::shared_ptr;
  using Observation = typename Model::Observation;
  using State = typename Model::State;
  using StateDot = typename Model::StateDot;
  using Control = typename Model::Control;

  using TrajectoryEstimation = estimation::fg::trajectory_estimation;

  using SF = prx::fg::symbol_factory_t;

  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;

  using GraphValues = std::pair<FactorGraph, Values>;

public:
  local_adaptation_t()
  {
  }
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string tree_topic;
    PARAM_SETUP(private_nh, tree_topic);

    _tree_subscriber = private_nh.subscribe(tree_topic, 1, &LocalAdapter::tree_callback, this);
  }

  ~local_adaptation_t(){};

  struct tree_operations_t
  {
    using Node = prx_models::NodeConstPtr;
    using Edge = prx_models::EdgeConstPtr;

    tree_operations_t(const prx_models::TreeConstPtr tree) : _tree(tree){};
    Node node(const int index) const
    {
      return _tree->nodes[index];
    }

    Edge parent_edge(const int node_index) const
    {
      return _tree->nodes[index].parent_edge;
    }

    Eigen::VectorXd node_state(const int node_index)
    {
      const ml4kp_bridge::SpacePoint state{ _tree->nodes[node_index].point };
      Eigen::VectorXd vec(state.size());
      for (int i = 0; i < count; ++i)
      {
        vec[i] = state[i];
      }
      return vec;
    }
    Eigen::VectorXd control(const int edge_index)
    {
      const ml4kp_bridge::Plan plan{ _tree->edges[edge_index].plan };
      prx_assert(plan.steps.size() == 1, "Expected plan size to be 1 but got" << plan.size());

      const ml4kp_bridge::SpacePoint control{ plan.steps.front().control };
      Eigen::VectorXd vec(control.size());
      for (int i = 0; i < count; ++i)
      {
        vec[i] = control[i];
      }
      return vec;
    }
    double duration(const int edge_index)
    {
      const ml4kp_bridge::Plan plan{ _tree->edges[edge_index].plan };
      prx_assert(plan.steps.size() == 1, "Expected plan size to be 1 but got" << plan.size());

      const ros::Duration d(plan.steps.front().duration.data);
      return d.toSec();
    }
    const prx_models::TreeConstPtr _tree;
  };
  void tree_callback(const prx_models::TreeConstPtr msg)
  {
    GraphValues graph_values{ sbmp_tree_to_fg(0, Tree::node(msg->root)) };
  }

  template <typename Node, typename Tree>
  static GraphValues sbmp_tree_to_fg(const int level, const Node& node)
  {
    GraphValues graph_values{};
    const auto& children{ Tree::children(node) };
    std::size_t step_c{ 0 };
    for (auto child : children)
    {
      const GraphValues graph_values_child{ sbmp_tree_to_fg(level + 1, step_c, Tree::parent_edge(child),
                                                            Tree::node(child)) };
      graph_values.first += graph_values_child.first;
      graph_values.second += graph_values_child.second;
      step_c++;
    }
    return graph_values;
  }

  template <typename Edge, typename Node, typename SbmpToFg>
  static GraphValues sbmp_tree_to_fg(const int level, const std::size_t step, const Node& node, const Edge& edge)
  {
    GraphValues graph_values{ Tree::SbmpToFg(level, step, Tree::node_state(node), Tree::control(edge),
                                             Tree::duration(edge)) };

    const GraphValues graph_values_child{ sbmp_tree_to_fg(level + 1, Tree::node(node)) };
    graph_values.first += graph_values_child.first;
    graph_values.second += graph_values_child.second;
    return graph_values;
  }

private:
  ros::Subscriber _tree_subscriber;
};
}  // namespace mj_ros