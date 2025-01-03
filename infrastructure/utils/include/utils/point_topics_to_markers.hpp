#include <unordered_set>
#include <visualization_msgs/MarkerArray.h>
#include <prx/utilities/general/type_conversions.hpp>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/SendString.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>

namespace utils
{

template <bool WithHeader, class Base>
class points_topics_to_markers : public Base
{
  using ThisClass = points_topics_to_markers<WithHeader, Base>;
  using TopicType = typename std::conditional<WithHeader, geometry_msgs::PointStamped, geometry_msgs::Point>::type;
  using TopicTypeConstPtr = boost::shared_ptr<TopicType const>;

public:
  points_topics_to_markers()
  {
  }

  virtual void onInit()
  {
    using namespace prx::utilities;
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string topic, color;
    bool topic_valid{ true };
    std::size_t idx{ 0 };
    while (topic_valid)
    {
      topic_valid = false;
      const std::string topic_name{ "topic_" + convert_to<std::string>(idx) };
      const std::string color_param{ "color_" + convert_to<std::string>(idx) };
      if (private_nh.getParam(topic_name, topic))
      {
        _subscribers.push_back(
            private_nh.subscribe<TopicType>(topic, 1, boost::bind(&ThisClass::callback, this, _1, idx)));
        topic_valid = true;
      }
      if (private_nh.getParam(color_param, color))
      {
        get_color(color, idx);
      }
      idx++;
    }

    double publisher_frequency{ 10.0 };
    double marker_scale{ 1 };
    std::string viz_topic;
    PARAM_SETUP(private_nh, viz_topic)
    PARAM_SETUP_WITH_DEFAULT(private_nh, publisher_frequency, publisher_frequency)
    PARAM_SETUP_WITH_DEFAULT(private_nh, marker_scale, marker_scale)

    const ros::Duration freq_timer(1 / publisher_frequency);
    _pub_timer = private_nh.createTimer(freq_timer, &ThisClass::timer_function, this);
    _publisher = private_nh.advertise<visualization_msgs::Marker>(viz_topic, 1, true);

    _marker.header.frame_id = "world";
    _marker.header.stamp = ros::Time();
    _marker.ns = "points";
    _marker.id = 0;
    _marker.type = visualization_msgs::Marker::POINTS;
    _marker.action = visualization_msgs::Marker::ADD;
    _marker.pose.position.x = 0;
    _marker.pose.position.y = 0;
    _marker.pose.position.z = 0;
    _marker.pose.orientation.x = 0.0;
    _marker.pose.orientation.y = 0.0;
    _marker.pose.orientation.z = 0.0;
    _marker.pose.orientation.w = 1.0;
    _marker.scale.x = marker_scale;
    _marker.scale.y = marker_scale;
    _marker.scale.z = marker_scale;
  }

protected:
  void timer_function(const ros::TimerEvent& event)
  {
    _marker.header.seq++;
    _marker.header.stamp = ros::Time::now();
    _publisher.publish(_marker);
    _marker.points.clear();
    _marker.colors.clear();
  }

  void copy_to_marker(const geometry_msgs::Point msg)
  {
    _marker.points.emplace_back();
    _marker.points.back().x = msg.x;
    _marker.points.back().y = msg.y;
    _marker.points.back().z = msg.z;
  }
  void copy_to_marker(const geometry_msgs::PointStamped msg)
  {
    copy_to_marker(msg.point);
  }

  void callback(const TopicTypeConstPtr msg, int idx)
  {
    copy_to_marker(*msg);
    _marker.colors.push_back(_colors[idx]);
  }

  void get_color(std::string str_color, const int idx)
  {
    // using prx::utilities::convert_to;
    using namespace prx::utilities;
    // Check if color is 0xRRGGBB
    if (str_color.size() == 8)
    {
      str_color = "0xFF" + str_color.substr(2);
    }
    prx_assert(str_color.size() == 10, "Wrong string color, expected '0xRRGGBB' or '0xAARRGGBB'");

    // Color is 0xAARRGGBB
    constexpr double max_val{ 255.0 };
    const double alpha{ convert_to<double>("0x" + str_color.substr(2, 2)) / max_val };
    const double red{ convert_to<double>("0x" + str_color.substr(4, 2)) / max_val };
    const double green{ convert_to<double>("0x" + str_color.substr(6, 2)) / max_val };
    const double blue{ convert_to<double>("0x" + str_color.substr(8, 2)) / max_val };

    _colors.resize(idx + 1);
    _colors[idx].a = alpha;
    _colors[idx].r = red;
    _colors[idx].b = blue;
    _colors[idx].g = green;
  }

  // Subscribers
  std::vector<ros::Subscriber> _subscribers;
  ros::Timer _pub_timer;

  // Publishers
  ros::Publisher _publisher;
  std::vector<std_msgs::ColorRGBA> _colors;

  visualization_msgs::Marker _marker;
};
}  // namespace utils