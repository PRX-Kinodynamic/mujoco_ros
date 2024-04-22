#include <ros/ros.h>
#include <ml4kp_bridge/defs.h>

namespace motion_planning
{

// Class to provide an interface to the tree of a SBMP tree-planner.
// It needs to do bookkeeping on the status of the nodes: Idle, New, Active, Update, Remove
template <typename TreePlanner>
class tree_interface_t : public TreePlanner
{
public:
  enum node_status_t
  {
    NEW = 0,
    ACTIVE,
    UPDATE,
    REMOVE
  };

  template <typename... Targs>
  tree_interface_t(Targs... Fargs) : TreePlanner(Fargs...), grow_condition_checker("iterations", 1)
  {
  }

  void set_node_to_remove(const prx::node_index_t& idx)
  {
    _node_status[idx] = node_status_t::REMOVE;
    bnb(node_index_t v, 0, true);
  }

  node_status_t node_state(const prx::node_index_t& idx)
  {
    // Need to check if node has been removed by the planner itself (e.g. BNB)
    if (not tree.is_node_valid(idx))
    {
      _node_status[idx] = node_status_t::REMOVE;
    }

    return _node_status[idx];
  }

  void grow_tree()
  {
    const std::size_t prev_total_nodes{ tree.num_vertices() };

    resolve_query(grow_condition_checker);

    const std::size_t new_total_nodes{ tree.num_vertices() };
    for (int i = prev_total_nodes; i < new_total_nodes; ++i)
    {
      _node_status[i] = node_status_t::NEW;
      _new_nodes.emplace_back(i);
    }
  }

  void remove_nodes()
  {
    tree.remove_vertices();
  }

protected:
  prx::condition_check_t grow_condition_checker;

  std::unordered_map<prx::node_index_t, node_status_t> _node_status;
  std::deque<prx::node_index_t> _new_nodes;
  std::deque<prx::node_index_t> _updated_nodes;
}
