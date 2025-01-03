#include <unordered_set>
#include <ml4kp_bridge/StelaTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
namespace utils
{

template <class Base>
class stela_trajectory_viz_t : public Base
{
  using Derived = stela_trajectory_viz_t<Base>;

public:
  stela_trajectory_viz_t()
    : _viz_edges_topic_name("/edges/marker")
    , _viz_nodes_topic_name("/nodes/marker")
    , _x_idx(0)
    , _y_idx(1)
    , _z_idx(2)
    , _use_z(false)
    , _z_default(0)
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string topic_name{};
    std::vector<double> color{};

    int& x_idx{ _x_idx };
    int& y_idx{ _y_idx };
    int& z_idx{ _z_idx };
    bool& use_z{ _use_z };
    double& z_default{ _z_default };

    PARAM_SETUP(private_nh, topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, x_idx, x_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, y_idx, y_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_idx, z_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, use_z, use_z);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_default, z_default);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, 1.0, 0.0 }));

    _viz_edges_topic_name = topic_name + _viz_edges_topic_name;
    _viz_nodes_topic_name = topic_name + _viz_nodes_topic_name;

    // subscribers
    _graph_subscriber = private_nh.subscribe(topic_name, 1, &Derived::get_graph, this);

    // publishers
    _viz_edges_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_edges_topic_name, 0);
    _viz_nodes_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_nodes_topic_name, 0);

    _nodes_marker.header.frame_id = "world";
    _nodes_marker.header.stamp = ros::Time();
    _nodes_marker.ns = "StelaTraj";
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
    _edges_marker.ns = "StelaTraj";
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
  void get_graph(const ml4kp_bridge::StelaTrajectoryConstPtr msg)
  {
    _nodes_marker.points.clear();
    _edges_marker.points.clear();

    const std::size_t total_nodes{ msg->data.size() };
    for (std::size_t i = 0; i < total_nodes; ++i)
    {
      // const prx_models::Node& node{ msg->data[i] };
      // const std::size_t node_id{ static_cast<std::size_t>(node.index) };
      // const std::size_t parent_id{ static_cast<std::size_t>(node.parent) };

      const ml4kp_bridge::SpacePoint& space_point{ msg->data[i].space_point };
      _nodes_marker.points.emplace_back();
      _nodes_marker.points.back().x = space_point.point[_x_idx];
      _nodes_marker.points.back().y = space_point.point[_y_idx];
      _nodes_marker.points.back().z = _use_z ? space_point.point[_z_idx] : _z_default;
    }

    for (std::size_t i = 1; i < total_nodes; ++i)
    {
      // const prx_models::Node& node_A{ msg->data[i - 1] };
      // const prx_models::Node& node_B{ msg->data[i] };
      const ml4kp_bridge::SpacePoint& space_point_A{ msg->data[i - 1].space_point };
      const ml4kp_bridge::SpacePoint& space_point_B{ msg->data[i].space_point };

      _edges_marker.points.emplace_back();
      _edges_marker.points.back().x = space_point_A.point[_x_idx];
      _edges_marker.points.back().y = space_point_A.point[_y_idx];
      _edges_marker.points.back().z = _use_z ? space_point_A.point[_z_idx] : _z_default;
      _edges_marker.points.emplace_back();
      _edges_marker.points.back().x = space_point_B.point[_x_idx];
      _edges_marker.points.back().y = space_point_B.point[_y_idx];
      _edges_marker.points.back().z = _use_z ? space_point_B.point[_z_idx] : _z_default;
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
  bool _use_z;
  double _z_default;
};
}  // namespace utils