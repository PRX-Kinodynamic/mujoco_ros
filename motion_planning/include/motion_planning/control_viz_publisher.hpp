#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
#include <motion_planning/motion_planning_types.hpp>
namespace motion_planning
{

template <typename SystemInterface, class Base>
class control_vizualizer_t : public Base
{
  using Derived = control_vizualizer_t<Base>;

public:
  control_vizualizer_t() : _tf_listener(_tf_buffer), _viz_control_name("/marker")
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string control_topic_name{};
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::vector<double> color{};
    // _control_topic_name = ros::this_node::getNamespace() + _control_topic_name;

    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, control_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, .78, 1.0 }));

    _viz_control_name = control_topic_name + _viz_control_name;

    // subscribers
    _control_subscriber = private_nh.subscribe(control_topic_name, 1, &Derived::control_callback, this);

    // publishers
    _viz_control_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_control_name, 0);

    _control_marker.header.frame_id = "world";
    _control_marker.header.stamp = ros::Time();
    _control_marker.ns = "control";
    _control_marker.id = 0;
    _control_marker.type = visualization_msgs::Marker::ARROW;
    _control_marker.action = visualization_msgs::Marker::ADD;
    _control_marker.pose.position.x = 0;
    _control_marker.pose.position.y = 0;
    _control_marker.pose.position.z = 0;
    _control_marker.pose.orientation.x = 0.0;
    _control_marker.pose.orientation.y = 0.0;
    _control_marker.pose.orientation.z = 0.0;
    _control_marker.pose.orientation.w = 1.0;
    _control_marker.scale.x = 0.1;
    _control_marker.scale.y = 0.1;
    _control_marker.scale.z = 0.1;

    _control_marker.color.a = color[0];  // Don't forget to set the alpha!
    _control_marker.color.r = color[1];
    _control_marker.color.g = color[2];
    _control_marker.color.b = color[3];

    _control_marker.points.emplace_back();
    _control_marker.points.emplace_back();
  }

protected:
  void control_callback(const ml4kp_bridge::SpacePoint& msg)
  {
    // geometry_msgs/Point
    if (update_tf)
    {
      SystemInterface::control_vizualization(_control_marker.points[1], msg);

      _control_marker.points[1].x += _control_marker.points[0].x;
      _control_marker.points[1].y += _control_marker.points[0].y;
      _control_marker.points[1].z += _control_marker.points[0].z;
      _viz_control_publisher.publish(_control_marker);
    }
  }
  bool update_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      _control_marker.points[0].x = _tf.transform.translation.x;
      _control_marker.points[0].y = _tf.transform.translation.y;
      _control_marker.points[0].z = _tf.transform.translation.z;
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  // Topic names
  std::string _tree_topic_name;
  std::string _viz_control_name;

  // Subscribers
  ros::Subscriber _control_subscriber;

  // Publishers
  ros::Publisher _viz_control_publisher;

  // Timers
  ros::Timer _tree_timer;

  // Viz
  visualization_msgs::Marker _control_marker;

  // TF
  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  bool _use_z;
  double _z_default;
};
}  // namespace motion_planning