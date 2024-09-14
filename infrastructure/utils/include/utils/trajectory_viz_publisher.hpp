#include <unordered_set>
#include <ml4kp_bridge/Trajectory.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
namespace utils
{

template <class Base>
class trajectory_viz_publisher_t : public Base
{
  using Derived = trajectory_viz_publisher_t<Base>;

public:
  trajectory_viz_publisher_t() : _viz_traj_topic_name("/viz"), x_idx(0), y_idx(1), z_idx(2), _z_fix(0.0)
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string traj_topic_name{};
    std::vector<double> color{};

    double& z_fix{ _z_fix };
    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;

    PARAM_SETUP(private_nh, traj_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, x_idx, x_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, y_idx, y_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_idx, z_idx);
    PARAM_SETUP_WITH_DEFAULT(private_nh, z_fix, z_fix);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, 1.0, 0.0 }));

    _viz_traj_topic_name = traj_topic_name + _viz_traj_topic_name;

    // subscribers
    _traj_subscriber = private_nh.subscribe(traj_topic_name, 1, &Derived::trajectory_callback, this);

    // publishers
    _viz_traj_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_traj_topic_name, 1, true);

    _traj_marker.header.frame_id = "world";
    _traj_marker.header.stamp = ros::Time();
    _traj_marker.ns = "trajectory";
    _traj_marker.id = 0;
    _traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    _traj_marker.action = visualization_msgs::Marker::ADD;
    _traj_marker.pose.position.x = 0;
    _traj_marker.pose.position.y = 0;
    _traj_marker.pose.position.z = 0;
    _traj_marker.pose.orientation.x = 0.0;
    _traj_marker.pose.orientation.y = 0.0;
    _traj_marker.pose.orientation.z = 0.0;
    _traj_marker.pose.orientation.w = 1.0;
    _traj_marker.scale.x = 0.1;
    _traj_marker.scale.y = 0.1;
    _traj_marker.scale.z = 0.1;

    _traj_marker.color.a = color[0];  // Don't forget to set the alpha!
    _traj_marker.color.r = color[1];
    _traj_marker.color.g = color[2];
    _traj_marker.color.b = color[3];
  }

protected:
  inline void allocate_memory(const std::size_t& new_size)
  {
    // _nodes_marker.points.resize(new_size);
    // // Edges need double the points as it will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
    // _edges_marker.points.resize(new_size * 2);
  }

  // template <typename Graph>
  void trajectory_callback(const ml4kp_bridge::TrajectoryConstPtr msg)
  {
    _traj_marker.action = visualization_msgs::Marker::DELETEALL;
    _viz_traj_publisher.publish(_traj_marker);
    _traj_marker.action = visualization_msgs::Marker::ADD;
    _traj_marker.points.clear();
    for (auto space_point : msg->data)
    {
      _traj_marker.id++;
      _traj_marker.points.emplace_back();
      _traj_marker.points.back().x = space_point.point[x_idx];
      _traj_marker.points.back().y = space_point.point[y_idx];
      _traj_marker.points.back().z = _z_fix;
      // _traj_marker.points.back().z = space_point.point[z_idx];
    }

    _viz_traj_publisher.publish(_traj_marker);
  }

  // Topic names
  std::string _viz_traj_topic_name;

  // Subscribers
  ros::Subscriber _traj_subscriber;

  // Publishers
  ros::Publisher _viz_traj_publisher;

  // Viz
  visualization_msgs::Marker _traj_marker;

  // Internal
  int x_idx;
  int y_idx;
  int z_idx;

  double _z_fix;
};
}  // namespace utils