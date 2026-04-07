#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/SensorDataStamped.h>

namespace interface
{
template <class Base>
class spacepoint_to_marker_t : public Base
{
  using Derived = spacepoint_to_marker_t<Base>;

public:
  spacepoint_to_marker_t() : _viz_spacepoint_name("/marker")
  {
  }

  ~spacepoint_to_marker_t() {};
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string spacepoint_topic_name{};
    std::string& world_frame{ _world_frame };
    // std::string& sensor_frame{ _sensor_frame };
    std::vector<double> color{};
    // _control_topic_name = ros::this_node::getNamespace() + _control_topic_name;

    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, spacepoint_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, .78, 1.0 }));

    // const std::string control_stamped_topic_name{ control_topic_name + "_stamped" };
    // _viz_control_stamped_name = control_stamped_topic_name + _viz_spacepoint_name;
    _viz_spacepoint_name = spacepoint_topic_name + _viz_spacepoint_name;

    // subscribers
    _sensor_subscriber = private_nh.subscribe(spacepoint_topic_name, 1, &Derived::sensor_callback, this);

    // publishers
    _viz_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_spacepoint_name, 0);

    _sensor_marker.header.frame_id = _world_frame;
    _sensor_marker.header.stamp = ros::Time();
    _sensor_marker.ns = "spacepoint";
    _sensor_marker.id = 0;
    _sensor_marker.type = visualization_msgs::Marker::CUBE;
    _sensor_marker.action = visualization_msgs::Marker::ADD;
    _sensor_marker.pose.position.x = 0;
    _sensor_marker.pose.position.y = 0;
    _sensor_marker.pose.position.z = 0;
    _sensor_marker.pose.orientation.x = 0.0;
    _sensor_marker.pose.orientation.y = 0.0;
    _sensor_marker.pose.orientation.z = 0.0;
    _sensor_marker.pose.orientation.w = 1.0;

    _sensor_marker.scale.x = 0.42;
    _sensor_marker.scale.y = 0.25;
    _sensor_marker.scale.z = 0.25;

    _sensor_marker.color.a = color[0];  // Don't forget to set the alpha!
    _sensor_marker.color.r = color[1];
    _sensor_marker.color.g = color[2];
    _sensor_marker.color.b = color[3];
  }

protected:
  void sensor_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    const double theta{ msg->space_point.point[2] };
    const Eigen::Quaterniond quat{ Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) };

    // Conversion for Mushr, need to add specializations for other systems
    _sensor_marker.pose.position.x = msg->space_point.point[0];
    _sensor_marker.pose.position.y = msg->space_point.point[1];
    _sensor_marker.pose.position.z = 0.125;
    _sensor_marker.pose.orientation.w = quat.w();
    _sensor_marker.pose.orientation.x = quat.x();
    _sensor_marker.pose.orientation.y = quat.y();
    _sensor_marker.pose.orientation.z = quat.z();

    _viz_publisher.publish(_sensor_marker);
  }

  // Topic names
  std::string _viz_spacepoint_name;
  std::string _viz_control_stamped_name;

  // Subscribers
  ros::Subscriber _sensor_subscriber;
  ros::Subscriber _control_stamped_subscriber;

  // Publishers
  ros::Publisher _viz_publisher;

  // Viz
  visualization_msgs::Marker _sensor_marker;

  // TF
  std::string _world_frame;
  // std::string _sensor_frame;
  // geometry_msgs::TransformStamped _tf;

  // Eigen::Vector3d _pt1;
  // Eigen::Quaternion<double, Eigen::DontAlign> _Tq;
  // Eigen::Vector3d _Tt;
  // Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry, Eigen::DontAlign> _transform;
};
}  // namespace interface