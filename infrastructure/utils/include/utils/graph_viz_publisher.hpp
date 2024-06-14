#include <unordered_set>
#include <prx_models/Graph.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
namespace utils
{

template <class Base>
class graph_viz_publisher_t : public Base
{
  using Derived = graph_viz_publisher_t<Base>;

public:
  graph_viz_publisher_t()
    : _viz_edges_topic_name("/edges/marker")
    , _viz_nodes_topic_name("/nodes/marker")
    , _x_idx(0)
    , _y_idx(1)
    , _z_idx(2)
    , _z_missing_value(0)
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string graph_topic_name{};
    std::vector<double> color{};

    int& x_idx{ _x_idx };
    int& y_idx{ _y_idx };
    int& z_idx{ _z_idx };
    double& z_missing_value{ _z_missing_value };

    PARAM_SETUP(private_nh, graph_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, x_idx, x_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, y_idx, y_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_idx, z_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_missing_value, z_missing_value);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, 1.0, 0.0 }));

    _viz_edges_topic_name = graph_topic_name + _viz_edges_topic_name;
    _viz_nodes_topic_name = graph_topic_name + _viz_nodes_topic_name;

    // subscribers
    _graph_subscriber = private_nh.subscribe(graph_topic_name, 1, &Derived::get_graph, this);

    // publishers
    _viz_edges_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_edges_topic_name, 0);
    _viz_nodes_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_nodes_topic_name, 0);

    _nodes_marker.header.frame_id = "world";
    _nodes_marker.header.stamp = ros::Time();
    _nodes_marker.ns = "graph/nodes";
    _nodes_marker.id = 0;
    _nodes_marker.type = visualization_msgs::Marker::POINTS;
    _nodes_marker.action = visualization_msgs::Marker::ADD;
    _nodes_marker.pose.position.x = 0;
    _nodes_marker.pose.position.y = 0;
    _nodes_marker.pose.position.z = 0;
    _nodes_marker.pose.orientation.x = 0.0;
    _nodes_marker.pose.orientation.y = 0.0;
    _nodes_marker.pose.orientation.z = 0.0;
    _nodes_marker.pose.orientation.w = 1.0;
    _nodes_marker.scale.x = 0.1;
    _nodes_marker.scale.y = 0.1;
    _nodes_marker.scale.z = 0.1;

    _nodes_marker.color.a = color[0];  // Don't forget to set the alpha!
    _nodes_marker.color.r = color[1];
    _nodes_marker.color.g = color[2];
    _nodes_marker.color.b = color[3];

    _edges_marker.header.frame_id = "world";
    _edges_marker.header.stamp = ros::Time();
    _edges_marker.ns = "graph/edges";
    _edges_marker.id = 0;
    _edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    _edges_marker.action = visualization_msgs::Marker::ADD;
    _edges_marker.pose.position.x = 0;
    _edges_marker.pose.position.y = 0;
    _edges_marker.pose.position.z = 0;
    _edges_marker.pose.orientation.x = 0.0;
    _edges_marker.pose.orientation.y = 0.0;
    _edges_marker.pose.orientation.z = 0.0;
    _edges_marker.pose.orientation.w = 1.0;
    _edges_marker.scale.x = 0.01;
    _edges_marker.scale.y = 0.01;
    _edges_marker.scale.z = 0.01;
    _edges_marker.color.a = color[0];  // Don't forget to set the alpha!
    _edges_marker.color.r = color[1];
    _edges_marker.color.g = color[2];
    _edges_marker.color.b = color[3];
  }

protected:
  void get_graph(const prx_models::GraphConstPtr msg)
  {
    _nodes_marker.points.clear();
    _edges_marker.points.clear();

    const std::size_t total_nodes{ msg->nodes.size() };
    const std::size_t total_edges{ msg->edges.size() };
    DEBUG_VARS(total_nodes, total_edges);
    // allocate_memory(total_nodes);
    for (std::size_t i = 0; i < total_nodes; ++i)
    {
      const prx_models::Node& node{ msg->nodes[i] };
      // const std::size_t node_id{ static_cast<std::size_t>(node.index) };
      // const std::size_t parent_id{ static_cast<std::size_t>(node.parent) };

      const ml4kp_bridge::SpacePoint& space_point{ node.point };
      const std::size_t pt_dim{ space_point.point.size() };
      _nodes_marker.points.emplace_back();
      _nodes_marker.points.back().x = space_point.point[_x_idx];
      _nodes_marker.points.back().y = space_point.point[_y_idx];
      _nodes_marker.points.back().z = pt_dim > _z_idx ? space_point.point[_z_idx] : _z_missing_value;
    }

    for (std::size_t i = 0; i < total_edges; ++i)
    {
      const prx_models::Edge& edge{ msg->edges[i] };

      const std::uint64_t node_id_A{ edge.source };
      const std::uint64_t node_id_B{ edge.target };

      const prx_models::Node& node_A{ msg->nodes[node_id_A] };
      const prx_models::Node& node_B{ msg->nodes[node_id_B] };
      const ml4kp_bridge::SpacePoint& space_point_A{ node_A.point };
      const ml4kp_bridge::SpacePoint& space_point_B{ node_B.point };
      const std::size_t pt_dim{ space_point_A.point.size() };

      _edges_marker.points.emplace_back();
      _edges_marker.points.back().x = space_point_A.point[_x_idx];
      _edges_marker.points.back().y = space_point_A.point[_y_idx];
      _edges_marker.points.back().z = pt_dim > _z_idx ? space_point_A.point[_z_idx] : _z_missing_value;
      _edges_marker.points.emplace_back();
      _edges_marker.points.back().x = space_point_B.point[_x_idx];
      _edges_marker.points.back().y = space_point_B.point[_y_idx];
      _edges_marker.points.back().z = pt_dim > _z_idx ? space_point_B.point[_z_idx] : _z_missing_value;
    }

    _viz_nodes_publisher.publish(_nodes_marker);
    _viz_edges_publisher.publish(_edges_marker);
  }

  // Topic names
  std::string _viz_edges_topic_name;
  std::string _viz_nodes_topic_name;

  // Subscribers
  ros::Subscriber _graph_subscriber;

  // Publishers
  ros::Publisher _viz_edges_publisher;
  ros::Publisher _viz_nodes_publisher;

  // Timers
  ros::Timer _tree_timer;

  // Viz
  visualization_msgs::Marker _edges_marker;
  visualization_msgs::Marker _nodes_marker;

  // Internal
  int _x_idx;
  int _y_idx;
  int _z_idx;
  double _z_missing_value;
};
}  // namespace utils