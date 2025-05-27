#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <ml4kp_bridge/defs.h>

#include <utils/rosparams_utils.hpp>

struct constant_control
{
  // inline void translate_msg(ackermann_msgs::AckermannDriveStamped& ctrl_msg, const ml4kp_bridge::SpacePoint&
  // point_msg)

  constant_control(ros::NodeHandle& nh, std::string control_topic, std::string stop_topic, std::vector<double> ctrl)
    : _publisher(nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true))
    , _publisher_stamped(nh.advertise<ml4kp_bridge::SpacePointStamped>(control_topic + "_stamped", 1, true))
    , _subscriber(nh.subscribe(stop_topic, 1, &constant_control::callback, this))
  {
    for (int i = 0; i < ctrl.size(); ++i)
    {
      ctrl_msg.space_point.point.push_back(ctrl[i]);
    }
    PRX_DEBUG_VARS(control_topic, ctrl_msg)
  }

  void callback(const std_msgs::Bool& msg)
  {
    if (msg.data)
    {
      _publisher.publish(ctrl_msg.space_point);
      _publisher_stamped.publish(ctrl_msg);
    }
  }

  ml4kp_bridge::SpacePointStamped ctrl_msg;

  ros::Publisher _publisher;
  ros::Publisher _publisher_stamped;
  ros::Subscriber _subscriber;
};

// Publish a control when a boolean topic gets a "true"
int main(int argc, char** argv)
{
  const std::string node_name{ "constantControl" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getName() };

  std::string bool_topic{};
  std::string control_topic{};
  std::vector<double> fixed_control{};

  PARAM_SETUP(nh, bool_topic);
  PARAM_SETUP(nh, control_topic);
  PARAM_SETUP(nh, fixed_control);

  constant_control cc(nh, control_topic, bool_topic, fixed_control);

  ros::spin();
  return 0;
}
