#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>
#include <prx_models/MushrObservation.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace interface
{
template <class Base>
class mushr_observation_to_marker_t : public Base
{
  using Derived = mushr_observation_to_marker_t<Base>;

public:
  mushr_observation_to_marker_t() : _viz_pose_topicname("/marker")
  {
  }

  ~mushr_observation_to_marker_t(){};

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string pose_topicname{};
    std::string& world_frame{ _world_frame };
    std::vector<double> color{};
    // poser_topicname = ros::this_node::getNamespace() + poser_topicname;

    PARAM_SETUP(private_nh, pose_topicname);
    PARAM_SETUP_WITH_DEFAULT(private_nh, world_frame, "world");
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, .78, 1.0 }));

    _viz_pose_topicname = pose_topicname + _viz_pose_topicname;

    // subscribers
    _pose_subscriber = private_nh.subscribe(pose_topicname, 1, &Derived::pose_callback, this);
    // private_nh.subscribe(control_stamped_topic_name, 1, &Derived::control_stamped_callback, this);

    // publishers
    _marker_publisher = private_nh.advertise<visualization_msgs::Marker>(_viz_pose_topicname, 0);

    _marker.header.frame_id = world_frame;
    _marker.header.stamp = ros::Time();
    _marker.ns = "mushr";
    _marker.id = 0;
    _marker.type = visualization_msgs::Marker::CUBE;
    _marker.action = visualization_msgs::Marker::ADD;
    _marker.pose.position.x = 0;
    _marker.pose.position.y = 0;
    _marker.pose.position.z = 0;
    _marker.pose.orientation.x = 0.0;
    _marker.pose.orientation.y = 0.0;
    _marker.pose.orientation.z = 0.0;
    _marker.pose.orientation.w = 1.0;

    // scale.x is the shaft diameter, and scale.y is the head diameter.
    // If scale.z is not zero, it specifies the head length.
    _marker.scale.x = 0.5;
    _marker.scale.y = 0.2965;
    _marker.scale.z = 0.4;

    _marker.color.a = color[0];  // Don't forget to set the alpha!
    _marker.color.r = color[1];
    _marker.color.g = color[2];
    _marker.color.b = color[3];
  }

protected:
  void pose_callback(const prx_models::MushrObservationConstPtr msg)
  {
    _marker.pose = msg->pose;
    _marker_publisher.publish(_marker);
  }

  // Topic names
  std::string _viz_pose_topicname;

  // Subscribers
  ros::Subscriber _pose_subscriber;

  // Publishers
  ros::Publisher _marker_publisher;

  // Viz
  visualization_msgs::Marker _marker;

  // TF
  std::string _world_frame;
};
}  // namespace interface