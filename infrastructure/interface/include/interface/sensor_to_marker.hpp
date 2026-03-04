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
class sensor_to_marker_t : public Base
{
  using Derived = sensor_to_marker_t<Base>;

public:
  sensor_to_marker_t() : _viz_sensor_name("/marker")
  {
  }

  ~sensor_to_marker_t() {};
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string sensor_topic_name{};
    std::string& world_frame{ _world_frame };
    // std::string& sensor_frame{ _sensor_frame };
    std::vector<double> color{};
    // _control_topic_name = ros::this_node::getNamespace() + _control_topic_name;

    PARAM_SETUP(private_nh, world_frame);
    // PARAM_SETUP(private_nh, sensor_frame);
    PARAM_SETUP(private_nh, sensor_topic_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, .78, 1.0 }));

    // const std::string control_stamped_topic_name{ control_topic_name + "_stamped" };
    // _viz_control_stamped_name = control_stamped_topic_name + _viz_sensor_name;
    _viz_sensor_name = sensor_topic_name + _viz_sensor_name;

    // subscribers
    _sensor_subscriber = private_nh.subscribe(sensor_topic_name, 1, &Derived::sensor_callback, this);

    // publishers
    _viz_sensor_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_sensor_name, 0);

    _sensor_marker.header.frame_id = _world_frame;
    _sensor_marker.header.stamp = ros::Time();
    _sensor_marker.ns = "sensor";
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

    // _sensor_marker.points.emplace_back();
    // _sensor_marker.points.emplace_back();
  }

protected:
  void sensor_callback(const interface::SensorDataStampedConstPtr& msg)
  {
    _sensor_marker.pose.position.x = msg->raw_sensor_data[0];
    _sensor_marker.pose.position.y = msg->raw_sensor_data[1];
    _sensor_marker.pose.position.z = msg->raw_sensor_data[2];
    _sensor_marker.pose.orientation.w = msg->raw_sensor_data[3];
    _sensor_marker.pose.orientation.x = msg->raw_sensor_data[4];
    _sensor_marker.pose.orientation.y = msg->raw_sensor_data[5];
    _sensor_marker.pose.orientation.z = msg->raw_sensor_data[6];

    _viz_sensor_publisher.publish(_sensor_marker);
    // inline Rotation
    //     prx::euler_to_rotation({msg->raw_sensor_data[]}, "Z");
  }

  // Topic names
  std::string _viz_sensor_name;
  std::string _viz_control_stamped_name;

  // Subscribers
  ros::Subscriber _sensor_subscriber;
  ros::Subscriber _control_stamped_subscriber;

  // Publishers
  ros::Publisher _viz_sensor_publisher;

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