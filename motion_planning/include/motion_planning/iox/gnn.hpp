#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <iceoryx_hoofs/cxx/vector.hpp>
// #include <iceoryx_hoofs/cxx/pair.hpp>
#include <functional>
#include <cmath>

#include <utils/dbg_utils.hpp>

namespace motion_planning
{
namespace nearest_neighbors
{
using NodeDistancePair = std::pair<std::size_t, double>;

struct single_query_result_t
{
  single_query_result_t(const double dist, const std::size_t id, const bool status) noexcept
    : distance(dist), index(id), success(status){};
  single_query_result_t() noexcept = default;
  // single_query_result_t() noexcept : distance(-1), index(std::numeric_limits<std::size_t>::max()), success(false){};
  ~single_query_result_t(){};

  double distance{ -1 };
  std::size_t index{ std::numeric_limits<std::size_t>::max() };
  bool success{ false };
};
/**
 * Sorts a list of proximity_node_t's. Performed using a quick sort operation.
 * @param close_nodes The list to sort.
 * @param distances The distances that determine the ordering.
 */

struct node_comparison_function_t
{
  bool operator()(const NodeDistancePair& a, const NodeDistancePair& b) const
  {
    const double a_dist{ a.second };
    const double b_dist{ b.second };
    return a_dist < b_dist;
  }
};

/**
 * Performs sorting over a list of nodes. Assumes all nodes before index are sorted.
 * @param close_nodes The list to sort.
 * @param index The index to start from.
 */
template <typename NodeDistanceVector>
int resort_nodes(NodeDistanceVector& close_nodes, int index)
{
  double temp_dist;
  std::size_t temp_node;

  while (index > 0 && close_nodes[index].second < close_nodes[index - 1].second)
  {
    temp_dist = close_nodes[index].second;
    close_nodes[index].second = close_nodes[index - 1].second;
    close_nodes[index - 1].second = temp_dist;

    temp_node = close_nodes[index].first;
    close_nodes[index].first = close_nodes[index - 1].first;
    close_nodes[index - 1].first = temp_node;

    index--;
  }
  return index;
}

template <typename NodeDistanceVector>
void sort_nodes(NodeDistanceVector& close_nodes)
{
  const node_comparison_function_t cmp{};
  std::sort(close_nodes.begin(), close_nodes.end(), cmp);
}

/**
 * @brief <b> Proximity node for the graph-based distance metric.</b>
 *
 * Proximity node for the graph-based distance metric.
 *
 * @author Edgar Granados, Kostas Bekris
 */
template <typename State, std::size_t MAX_NN = 200>
class node_t
{
public:
  using Node = node_t<State, MAX_NN>;
  using Neighbors = iox::cxx::vector<std::size_t, MAX_NN>;
  /**
   * @brief Constructor
   */
  node_t(){};

  node_t(const State state) : _state(state){};

  virtual ~node_t(){};

  /**
   * Deletes a node index from this node's neighbor list.
   * @brief Deletes a node index from this node's neighbor list.
   * @param node The index to delete.
   */
  // void delete_neighbor(long unsigned node);

  /**
   * Replaces a node index from this node's neighbor list.
   * @brief Replaces a node index from this node's neighbor list.
   * @param prev The index to look for.
   * @param new_index The index to replace with.
   */
  // void replace_neighbor(long unsigned prev, long unsigned new_index);

  void remove_all_neighbors()
  {
    // nr_neighbors = 0;
    added_index = 0;
    _neighbors.clear();
  }

  static void add_neighbor(Node& new_node, const std::size_t& n_index)
  {
    new_node._neighbors.emplace_back(n_index);
  }

  static inline Neighbors& neighbors(Node& node)
  {
    return node._neighbors;
  }

  static inline const Neighbors& neighbors(const Node& node)
  {
    return node._neighbors;
  }

  static inline std::size_t& index(Node& node)
  {
    return node._index;
  }
  static inline std::size_t index(const Node& node)
  {
    return node._index;
  }

  static inline bool& added_to_metric(Node& node)
  {
    return node._added_to_metric;
  }
  static inline bool added_to_metric(const Node& node)
  {
    return node._added_to_metric;
  }

  static std::size_t& added_index(Node& node)
  {
    return node._added_index;
  }
  static std::size_t added_index(const Node& node)
  {
    return node._added_index;
  }

  static State& state(Node& node)
  {
    return node._state;
  }
  static State state(const Node& node)
  {
    return node._state;
  }

protected:
  bool _added_to_metric;

  std::size_t _added_index;
  /**
   * @brief Index in the data structure. Serves as an identifier to other nodes.
   */
  std::size_t _index;

  /**
   * @brief The neighbor list for this node.
   */
  Neighbors _neighbors;

  State _state;
};

/**
 * A proximity structure based on graph literature. Each node maintains a list of neighbors.
 * When performing queries, the graph is traversed to determine other locally close nodes.
 * @brief <b> A proximity structure based on graph literature. </b>
 * @author Edgar Granados
 */
template <typename Container, typename Metric>
class graph_t
{
public:
  using Graph = graph_t<Container, Metric>;

  graph_t() : _metric(Metric()), _current_nodes(0), _added_node_id(0){};
  graph_t(const Metric metric_) : _metric(metric_), _current_nodes(0), _added_node_id(0){};
  inline graph_t(Graph&& rhs) noexcept
  {
    *this = std::move(rhs);
  }

  template <typename... Targs>
  void emplace_back(Targs... args) noexcept
  {
    _container.emplace_back(args...);
    _current_nodes++;
  }

  auto& back()
  {
    return _container.back();
  }
  const auto& back() const
  {
    return _container.back();
  }

  auto& operator[](const std::size_t i)
  {
    return _container[i];
  }
  const auto& operator[](const std::size_t i) const
  {
    return _container[i];
  }

  static std::size_t current_nodes(const Graph& container)
  {
    return container._current_nodes;
  }

  static void clear_added(Graph& container)
  {
    container._added_node_id++;
  }

  static std::size_t added_index(const Graph& container)
  {
    return container._added_node_id;
  }

  template <typename State>
  static double metric(const Graph& container, const State& a, const State& b)
  {
    return container._metric(a, b);
  }

protected:
  const Metric _metric;

  Container _container;
  std::size_t _current_nodes;
  std::size_t _added_node_id;
};

/**
 * Queries for the GNN. Each node maintains a list of neighbors.
 * When performing queries, the graph is traversed to determine other locally close nodes.
 * @brief <b> A proximity structure based on graph literature. </b>
 * @author Edgar Granados, Kostas Bekris
 */
template <typename Node, std::size_t MAX_KK = 2000>
class queries_t
{
  // Id of the node and auxiliary distance value. Mostly used for temporary storage
  // using NodeDistancePair = iox::cxx::pair<std::size_t, double>;

public:
  /**
   * @brief Constructor
   * @param state The first node to add to the structure.
   */
  queries_t() : _second_nodes_distances(MAX_KK), _query_node()
  {
    // query_node = new abstract_node_t();
  }

  ~queries_t(){};

  /**
   * Adds a node to the proximity structure
   * @brief Adds a node to the proximity structure
   * @param node The node to insert.
   */
  template <typename Graph>
  void add_node(Node& new_node, Graph& graph)
  {
    const std::size_t new_node_idx{ Node::index(new_node) };
    if (Node::added_to_metric(new_node))
    {
      prx_throw("Trying to add a node [ " << new_node_idx << " ] that is already added ");
    }
    const std::size_t k{ percolation_threshold(graph) };

    const std::size_t new_k{ find_k_close(new_node, _second_nodes_distances, graph, k) };

    for (int i = 0; i < new_k; i++)
    {
      const std::size_t second_index{ index(_second_nodes_distances[i]) };
      Node::add_neighbor(new_node, second_index);
      Node::add_neighbor(graph[second_index], new_node_idx);
      // graph_node->add_neighbor(second_nodes[i]->get_prox_index());
    }
    Node::added_to_metric(new_node) = true;
  }

  /**
   * @brief Removes a node from the structure.
   * @param node
   */
  void remove_node(Node& node)
  {
  }

  template <typename State, typename Graph>
  single_query_result_t single_query(const State& state_in, const Graph& graph)
  {
    Node::state(_query_node) = state_in;
    // double distance;
    // return find_closest(_query_node, &distance);
    return basic_closest_search(_query_node, graph);
  }

  // std::vector<proximity_node_t*> multi_query(const space_point_t& point, int k);

  // std::vector<proximity_node_t*> radius_and_closest_query(const space_point_t& point, double rad);

protected:
  /**
   * Returns the closest node in the data structure.
   * @brief Returns the closest node in the data structure.
   * @param state The query point.
   * @param distance The resulting distance between the closest point and the query point.
   * @return The closest point.
   */
  // single_query_result_t find_closest(const Node node, const Container& container)
  // {
  //   // long unsigned min_index = -1;
  //   // return basic_closest_search(state, the_distance, &min_index);
  //   return basic_closest_search(node, container);
  // }

  /**
   * Find the k closest nodes to the query point.
   * @brief Find the k closest nodes to the query point.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param k The number to return.
   * @return The number of nodes actually returned.
   */
  template <typename NodeDistanceVector, typename Graph>
  std::size_t find_k_close(const Node& node, NodeDistanceVector& close_nodes_distances, Graph& graph, std::size_t k)
  {
    const std::size_t current_nodes{ Graph::current_nodes(graph) };
    if (current_nodes == 0)
    {
      return 0;
    }

    if (k > MAX_KK)
    {
      k = MAX_KK;
    }
    else if (k >= current_nodes)
    {
      for (int i = 0; i < current_nodes; i++)
      {
        close_nodes_distances[i].first = Node::index(graph[i]);  //_nodes[i];
        close_nodes_distances[i].second = Graph::metric(graph, Node::state(graph[i]), Node::state(node));
      }
      sort_nodes(close_nodes_distances);
      return current_nodes;
    }

    Graph::clear_added(graph);

    // double& curr_distance{ distance(close_nodes_distances[0]) };
    // index(close_nodes_distances[0]) = basic_closest_search(node, curr_distance, min_index);
    const single_query_result_t result{ basic_closest_search(node, graph) };
    // close_nodes[0] = basic_closest_search(state, &(distances[0]), &min_index);
    index(close_nodes_distances[0]) = result.index;
    distance(close_nodes_distances[0]) = result.distance;
    std::size_t min_index{ result.index };

    Node::added_index(graph[min_index]) = Graph::added_index(graph);

    min_index = 0;
    std::size_t current_node_neighbors{ 1 };
    double max_distance{ result.distance };

    /* Find the neighbors of the closest node if they are not already in the set of k-closest nodes.
    If the distance to any of the neighbors is less than the distance to the k-th closest element,
    then replace the last element with the neighbor and resort the list. In order to decide the next
    node to pivot about, it is either the next node on the list of k-closest
    */
    do
    {
      auto neighbors = Node::neighbors(graph[min_index]);
      current_node_neighbors = neighbors.size();

      int lowest_replacement = current_node_neighbors;

      for (int j = 0; j < current_node_neighbors; j++)
      {
        Node& the_neighbor{ graph[neighbors[j]] };
        if (does_node_exist(the_neighbor, graph) == false)
        {
          Node::added_index(the_neighbor) = Graph::added_index(graph);

          const double current_distance{ Graph::metric(graph, Node::state(the_neighbor), Node::state(node)) };
          bool to_resort{ false };
          if (current_node_neighbors < k)
          {
            index(close_nodes_distances[current_node_neighbors]) = Node::index(the_neighbor);
            distance(close_nodes_distances[current_node_neighbors]) = current_distance;
            current_node_neighbors++;
            to_resort = true;
          }
          else if (current_distance < distance(close_nodes_distances[k - 1]))
          {
            index(close_nodes_distances[current_node_neighbors]) = Node::index(the_neighbor);
            distance(close_nodes_distances[current_node_neighbors]) = current_distance;
            to_resort = true;
          }

          if (to_resort)
          {
            const int test{ resort_nodes(close_nodes_distances, current_node_neighbors - 1) };
            lowest_replacement = (test < lowest_replacement ? test : lowest_replacement);
          }
        }
      }

      /* In order to decide the next node to pivot about,
      it is either the next node on the list of k-closest (min_index)
      or one of the new neighbors in the case that it is closer than nodes already checked.
      */
      if (min_index < lowest_replacement)
      {
        min_index++;
      }
      else
      {
        min_index = lowest_replacement;
      }
    } while (min_index < current_node_neighbors);

    return current_node_neighbors;
  }

  /**
   * Find all nodes within a radius and the closest node.
   * @brief Find all nodes within a radius and the closest node.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param delta The radius to search within.
   * @return The number of nodes returned.
   */
  // int find_delta_close_and_closest(proximity_node_t* state, proximity_node_t** close_nodes, double* distances,
  //                                  double delta);

  /**
   * Find all nodes within a radius.
   * @brief Find all nodes within a radius.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param delta The radius to search within.
   * @return The number of nodes returned.
   */
  // int find_delta_close(proximity_node_t* state, proximity_node_t** close_nodes, double* distances, double delta);

  /**
   * Determine the number of nodes to sample for initial populations in queries.
   * @brief Determine the number of nodes to sample for initial populations in queries.
   * @return The number of random nodes to initially select.
   */

  template <typename Container>
  inline std::size_t sampling_function(const Container& container) const
  {
    const std::size_t current_nodes{ Container::current_nodes(container) };
    if (current_nodes < 200)
      return current_nodes;
    else
      return 200;
  }

  /**
   * Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
   * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
   * @return
   */
  template <typename Container>
  inline std::size_t percolation_threshold(const Container& container) const
  {
    const std::size_t current_nodes{ Container::current_nodes(container) };
    if (current_nodes > 12)
      return (2.0 * std::log(current_nodes));
    else
      return current_nodes;
  }

  /**
   * Helper function for determining existance in a list.
   * @brief Helper function for determining existance in a list.
   * @param query_node The node to search for.
   * @param node_list The list to search.
   * @param list_size The size of the list.
   * @return If query_node exists in node_list.
   */
  template <typename Container>
  inline bool does_node_exist(const Node& query_node, const Container& container) const
  {
    return Node::added_index(query_node) == Container::added_index(container);
  }

  /**
   * The basic search process for finding the closest node to the query state.
   * @brief Find the closest node to the query state.
   * @param state The query state.
   * @param distance The corresponding distance to the query point.
   * @param node_index The index of the returned node.
   * @return The closest node.
   */
  template <typename Graph>
  single_query_result_t basic_closest_search(const Node& node, const Graph& graph) const
  {
    using Neighbors = typename Node::Neighbors;
    const std::size_t current_nodes{ Graph::current_nodes(graph) };
    if (current_nodes == 0)
    {
      return single_query_result_t();
    }

    const std::size_t num_samples{ sampling_function(graph) };
    double min_distance{ std::numeric_limits<double>::max() };
    std::size_t min_index{ std::numeric_limits<std::size_t>::max() };

    for (std::size_t i = 0; i < num_samples; i++)
    {
      const std::size_t index{ static_cast<std::size_t>(
          prx::uniform_int_random(0, static_cast<int>(current_nodes - 1))) };
      const double distance{ Graph::metric(graph, Node::state(graph[index]), Node::state(node)) };

      if (distance < min_distance)
      {
        min_distance = distance;
        min_index = index;
      }
    }

    std::size_t old_min_index{ min_index };
    do
    {
      old_min_index = min_index;
      // neighbors is a vector of std::size_t's of unknown size to this function
      // (although size has to be known at compile time)
      const Neighbors& neighbors{ Node::neighbors(graph[min_index]) };
      std::size_t current_node_neighbors{ neighbors.size() };
      // long unsigned* neighbors = nodes[min_index]->get_neighbors(&nr_neighbors);
      for (std::size_t j = 0; j < current_node_neighbors; j++)
      {
        const double distance{ Graph::metric(graph, Node::state(graph[neighbors[j]]), Node::state(node)) };
        if (distance < min_distance)
        {
          min_distance = distance;
          min_index = neighbors[j];
        }
      }
    } while (old_min_index != min_index);

    return single_query_result_t(min_distance, min_index, true);
  }

  double& distance(NodeDistancePair& pair)
  {
    return pair.second;
  }
  double distance(const NodeDistancePair& pair) const
  {
    return pair.second;
  }
  std::size_t& index(NodeDistancePair& pair)
  {
    return pair.first;
  }
  std::size_t index(const NodeDistancePair& pair) const
  {
    return pair.first;
  }

  /**
   * @brief The nodes being stored.
   */
  // iox::cxx::vector<Node, MAX_NODES> _nodes;

  /**
   * @brief The current number of nodes being stored.
   */
  // std::size_t _current_nodes;

  /**
   * @brief The maximum number of nodes that can be stored.
   */
  // std::size_t cap_nodes;

  /**
   * @brief Temporary storage for query functions.
   */
  iox::cxx::vector<NodeDistancePair, MAX_KK> _second_nodes_distances;

  // std::vector<proximity_node_t*> added_nodes;
  // std::size_t _added_node_id;

  Node _query_node;
};
}  // namespace nearest_neighbors

}  // namespace motion_planning