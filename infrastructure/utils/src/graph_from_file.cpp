#include <thread>
#include <unordered_map>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/SetDuration.h>
#include <prx/utilities/general/csv_reader.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx_models/Graph.h>

using csv_reader_t = prx::utilities::csv_reader_t;
using Block = csv_reader_t::Block<std::string>;
using prx::utilities::convert_to;

int main(int argc, char** argv)
{
  const std::string node_name{ "GraphFromFile" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string filename;
  std::string graph_topic_name;

  PARAM_SETUP(nh, filename);
  PARAM_SETUP(nh, graph_topic_name);

  DEBUG_VARS(filename);

  prx_models::Node node;
  prx_models::Graph graph;

  std::size_t label_id{ 0 };
  std::unordered_map<std::string, std::size_t> labels;
  // Assumed format: Two blocks, first N lines with one point each, second block M lines with edges. Block separated by
  // new line
  csv_reader_t reader(filename);
  prx::constants::separating_value = ' ';
  // First block: Nodes
  Block block_nodes{ reader.next_block() };
  for (auto line : block_nodes)
  {
    if (line.size() == 0)
      break;
    const std::string label{ line[0] };
    labels[label] = label_id;

    node.index = label_id;
    node.parent = label_id;  // Setting to itself, in a graph it doesn't make sense to have a parent
    node.parent_edge = label_id;
    node.cost = 0;  // For graph the cost is in the edge
    node.children.clear();
    node.point.point.clear();
    for (int i = 1; i < line.size(); ++i)
    {
      node.point.point.push_back(convert_to<double>(line[i]));
    }
    graph.nodes.push_back(node);
    label_id++;
  }
  DEBUG_VARS("Edges");
  // Second block: edges
  std::size_t edge_id{ 0 };
  prx_models::Edge edge;

  Block block_edges{ reader.next_block() };
  for (auto line : block_edges)
  {
    const std::string label_A{ line[0] };
    const std::string label_B{ line[1] };
    const double cost{ convert_to<double>(line[2]) };
    const double duration{ convert_to<double>(line[3]) };

    const std::size_t id_A{ labels[label_A] };
    const std::size_t id_B{ labels[label_B] };
    edge.index = edge_id;
    edge.source = id_A;
    edge.target = id_B;
    edge.cost = cost;

    edge.plan.steps.clear();
    edge.plan.steps.emplace_back();
    edge.plan.steps.back().duration.data = ros::Duration(duration);
    for (int i = 4; i < line.size(); ++i)
    {
      edge.plan.steps.back().control.point.push_back(convert_to<double>(line[i]));
    }

    // In a graph, instead of nodes as children, it contains edges
    graph.nodes[id_A].children.push_back(edge_id);
    graph.nodes[id_B].children.push_back(edge_id);

    edge_id++;
    graph.edges.push_back(edge);
  }

  const std::string MSG("Graph read and publishing");
  const std::size_t total_nodes{ graph.nodes.size() };
  const std::size_t total_edges{ graph.edges.size() };
  DEBUG_VARS(MSG, graph_topic_name);
  DEBUG_VARS(total_nodes, total_edges);

  ros::Rate rate(1);  // sleep for 1 sec
  ros::Publisher graph_publisher{ nh.advertise<prx_models::Graph>(graph_topic_name, 1, true) };
  graph_publisher.publish(graph);

  rate.sleep();

  return 0;
}