#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>
#include <visualization_msgs/Marker.h>

#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <motion_planning/motion_planning_types.hpp>
namespace motion_planning
{

template <class Base>
class mp_tree_viz_publisher_t : public Base
{
  using Derived = mp_tree_viz_publisher_t<Base>;

public:
  mp_tree_viz_publisher_t()
    : _viz_edges_topic_name("/edges/marker")
    , _viz_nodes_topic_name("/nodes/marker")
    , x_idx(0)
    , y_idx(1)
    , z_idx(2)
    , _use_z(false)
    , _z_default(0.0)
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string tree_topic_name{};
    std::vector<double> color{ { 1.0, 0.0, 1.0, 0.0 } };
    bool& use_z{ _use_z };
    double& z_default{ _z_default };
    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;

    double scale{ 0.1 };
    PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, use_z, use_z);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_default, z_default);
    PARAM_SETUP_WITH_DEFAULT(private_nh, x_idx, x_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, y_idx, y_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_idx, z_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, color);
    PARAM_SETUP_WITH_DEFAULT(private_nh, scale, scale);

    prx_assert(color.size() == 4, "Wrong color size");
    _viz_edges_topic_name = tree_topic_name + _viz_edges_topic_name;
    _viz_nodes_topic_name = tree_topic_name + _viz_nodes_topic_name;

    // publishers
    _viz_edges_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_edges_topic_name, 1, true);

    _viz_nodes_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_nodes_topic_name, 1, true);

    // subscribers
    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::get_graph, this);

    _nodes_marker.header.frame_id = "world";
    _nodes_marker.header.stamp = ros::Time::now();
    _nodes_marker.ns = "nodes";
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
    _nodes_marker.scale.x = scale;
    _nodes_marker.scale.y = scale;
    _nodes_marker.scale.z = scale;

    _nodes_marker.color.a = color[0];  // Don't forget to set the alpha!
    _nodes_marker.color.r = color[1];
    _nodes_marker.color.g = color[2];
    _nodes_marker.color.b = color[3];

    _edges_marker.header.frame_id = "world";
    _edges_marker.header.stamp = ros::Time::now();
    _edges_marker.ns = "edges";
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
    _edges_marker.scale.x = scale * 0.1;
    _edges_marker.scale.y = scale * 0.1;
    _edges_marker.scale.z = scale * 0.1;
    _edges_marker.color.a = color[0];  // Don't forget to set the alpha!
    _edges_marker.color.r = color[1];
    _edges_marker.color.g = color[2];
    _edges_marker.color.b = color[3];
  }

protected:
  void get_graph(const prx_models::TreeConstPtr msg)
  {
    const std::size_t total_nodes{ msg->nodes.size() };
    const std::size_t total_edges{ msg->edges.size() };
    _nodes_marker.points.clear();
    _edges_marker.points.clear();

    // DEBUG_VARS(ros::Time::now())
    _nodes_marker.header.stamp = ros::Time::now();
    _edges_marker.header.stamp = ros::Time::now();
    // _nodes_marker.id++;
    // _edges_marker.id++;

    std::unordered_map<std::uint64_t, std::uint64_t> node_map;

    for (auto node : msg->nodes)
    {
      const ml4kp_bridge::SpacePoint& space_point{ node.point };
      _nodes_marker.points.emplace_back();
      if (space_point.point.size() < 2)
        continue;
      _nodes_marker.points.back().x = space_point.point[x_idx];
      _nodes_marker.points.back().y = space_point.point[y_idx];
      _nodes_marker.points.back().z = _use_z ? space_point.point[z_idx] : _z_default;
      node_map[node.index] = _nodes_marker.points.size() - 1;
    }
    for (auto edge : msg->edges)
    {
      if (node_map.count(edge.source) > 0 and node_map.count(edge.target) > 0)
      {
        const std::size_t source_id{ node_map[edge.source] };
        const std::size_t target_id{ node_map[edge.target] };
        _edges_marker.points.push_back(_nodes_marker.points[source_id]);
        _edges_marker.points.push_back(_nodes_marker.points[target_id]);
      }
    }

    _viz_nodes_publisher.publish(_nodes_marker);
    _viz_edges_publisher.publish(_edges_marker);
  }

  // Topic names
  std::string _tree_topic_name;
  std::string _viz_edges_topic_name;
  std::string _viz_nodes_topic_name;

  // Subscribers
  ros::Subscriber _tree_subscriber;

  // Publishers
  ros::Publisher _viz_edges_publisher;
  ros::Publisher _viz_nodes_publisher;

  // Timers
  ros::Timer _tree_timer;

  // Viz
  visualization_msgs::Marker _edges_marker;
  visualization_msgs::Marker _nodes_marker;

  // Internal
  std::unordered_set<std::size_t> added_nodes;
  std::unordered_set<std::size_t> added_edges;
  int x_idx;
  int y_idx;
  int z_idx;

  bool _use_z;
  double _z_default;
};
}  // namespace motion_planning