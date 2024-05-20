#include <unordered_set>
#include <visualization_msgs/MarkerArray.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/SendString.h>

#include <prx/planning/world_model.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/utilities/geometry/basic_geoms/box.hpp>

namespace utils
{

template <class Base>
class environment_publisher_t : public Base
{
  using Derived = environment_publisher_t<Base>;

public:
  environment_publisher_t() : _viz_env_name("/environment_marker_array")
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string environment_file{};
    std::vector<double> color{};

    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;
    _viz_env_name = ros::this_node::getNamespace() + _viz_env_name;

    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, 1.0, 0.0 }));
    PARAM_SETUP_WITH_DEFAULT(private_nh, environment_file, "");

    // publishers
    _environment_publisher = private_nh.advertise<visualization_msgs::MarkerArray>(_viz_env_name, 1, true);

    if (environment_file != "")
    {
      DEBUG_VARS(environment_file);
      read_and_publish_environment(environment_file);
    }
  }

protected:
  bool filename_callback(ml4kp_bridge::SendString::Request& request, ml4kp_bridge::SendString::Response& response)
  {
    const std::string filename{ request.string };
    read_and_publish_environment(filename);
    return true;
  }

  void read_and_publish_environment(const std::string filename)
  {
    visualization_msgs::MarkerArray msg{};
    const prx::PairNameObstacles obstacles{ prx::load_obstacles(filename) };
    const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
    const std::vector<std::string> obstacle_names{ obstacles.first };

    for (int i = 0; i < obstacle_list.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "environment";
      marker.id = i;
      marker.action = visualization_msgs::Marker::ADD;

      const std::shared_ptr<prx::box_t> box{ std::dynamic_pointer_cast<prx::box_t>(obstacle_list[i]) };
      if (box)
      {
        prx::movable_object_t::Geometries geometries{ box->get_geometries() };
        prx::movable_object_t::Configurations configs{ box->get_configurations() };
        for (int j = 0; j < configs.size(); ++j)
        {
          const std::shared_ptr<prx::geometry_t> gi{ geometries[j].second.lock() };
          const std::shared_ptr<prx::transform_t> Rt{ configs[j].second.lock() };
          prx_assert(Rt != nullptr, "Transform is null!");
          const std::string str_color{ gi->get_visualization_color() };
          const Color color{ get_color(str_color) };

          const Eigen::Vector3d t{ Rt->translation() };
          const Eigen::Quaterniond q{ Rt->rotation() };
          const std::vector<double> geom_params{ gi->get_geometry_params() };

          marker.type = visualization_msgs::Marker::CUBE;
          marker.pose.position.x = t[0];
          marker.pose.position.y = t[1];
          marker.pose.position.z = t[2];
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.pose.orientation.w = q.w();
          marker.scale.x = geom_params[0];
          marker.scale.y = geom_params[1];
          marker.scale.z = geom_params[2];
          marker.color.a = color[0];  // Don't forget to set the alpha!
          marker.color.r = color[1];
          marker.color.g = color[2];
          marker.color.b = color[3];
          msg.markers.push_back(marker);
        }
      }
    }
    _environment_publisher.publish(msg);
  }

  using Color = std::array<double, 4>;
  Color get_color(std::string str_color) const
  {
    using namespace prx::utilities;
    // Check if olor is 0xRRGGBB
    if (str_color.size() == 8)
    {
      str_color = "0xFF" + str_color.substr(2);
    }
    prx_assert(str_color.size() == 10, "Wrong string color, expected '0xRRGGBB' or '0xAARRGGBB'");

    // Color is 0xAARRGGBB
    constexpr double max_val{ 255.0 };
    const double alpha{ convert_to<double>(str_color[2] + str_color[3]) / max_val };
    const double red{ convert_to<double>(str_color[4] + str_color[5]) / max_val };
    const double blue{ convert_to<double>(str_color[6] + str_color[7]) / max_val };
    const double green{ convert_to<double>(str_color[8] + str_color[9]) / max_val };
    return Color{ alpha, red, blue, green };
  }

  // Topic names
  std::string _viz_env_name;

  // Subscribers
  ros::Subscriber _tree_subscriber;

  // Publishers
  ros::Publisher _environment_publisher;
};
}  // namespace utils