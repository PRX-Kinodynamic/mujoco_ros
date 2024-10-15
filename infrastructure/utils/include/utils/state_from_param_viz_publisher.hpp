#include <unordered_set>
#include <ml4kp_bridge/Trajectory.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
namespace utils
{

template <class Base>
class state_from_param_viz_publisher_t : public Base
{
  using Derived = state_from_param_viz_publisher_t<Base>;

public:
  state_from_param_viz_publisher_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::vector<std::string> parameters_to_viz{};
    std::vector<std::string> colors{};
    std::vector<double> radius{};

    // double& z_fix{ _z_fix };
    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;

    PARAM_SETUP(private_nh, parameters_to_viz);
    PARAM_SETUP_WITH_DEFAULT(private_nh, colors, colors);
    PARAM_SETUP_WITH_DEFAULT(private_nh, radius, radius);

    if (colors.size() == 0)
      colors.resize(parameters_to_viz.size(), "1.0 0.0 0.0 0.0");

    if (radius.size() == 0)
      radius.resize(parameters_to_viz.size(), 1.0);

    std::vector<double> state;
    for (auto param_name : parameters_to_viz)
    {
      if (ros::param::get(param_name, state))
      {
        const std::string topic_name{ param_name + "/viz" };
        _viz_state_publisher.push_back(private_nh.advertise<visualization_msgs::Marker>(topic_name, 1, true));

        const visualization_msgs::Marker marker{ create_marker(state, colors.front(), radius.front()) };
        colors.erase(colors.begin());
        radius.erase(radius.begin());
        _viz_state_publisher.back().publish(marker);
      }
      else
      {
        ROS_WARN_STREAM("[StateParamViz]: " << param_name << " not found");
      }
    }
  }

protected:
  inline visualization_msgs::Marker create_marker(const std::vector<double>& position, const std::string& color_str,
                                                  const double rad)
  {
    using prx::utilities::convert_to;
    const std::vector<std::string> color{ prx::split<std::string>(color_str) };

    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "state_param";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = rad;
    marker.scale.y = rad;
    marker.scale.z = rad;

    marker.color.a = convert_to<double>(color[0]);  // Don't forget to set the alpha!
    marker.color.r = convert_to<double>(color[1]);
    marker.color.g = convert_to<double>(color[2]);
    marker.color.b = convert_to<double>(color[3]);

    return marker;
  }

  // Publishers
  std::vector<ros::Publisher> _viz_state_publisher;
};
}  // namespace utils