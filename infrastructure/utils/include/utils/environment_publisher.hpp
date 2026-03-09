#pragma once
#include <visualization_msgs/MarkerArray.h>

#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <ml4kp_bridge/SendString.h>
#include <std_srvs/Empty.h>

#include <prx/planning/world_model.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/utilities/geometry/basic_geoms/box.hpp>
#include <prx/utilities/geometry/basic_geoms/cylinder.hpp>
#include <prx/utilities/geometry/basic_geoms/sphere.hpp>

namespace utils
{

template <class Base>
class environment_publisher_t : public Base
{
  using Derived = environment_publisher_t<Base>;

public:
  environment_publisher_t()
    : _viz_env_name("/environment_marker_array")
    , _reload_service_name("/environment/reload")
    , _bounds_name("/bounds")
    , _msg_valid(false)
    , _bounds_valid(false)
    , _environment_file("")
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::vector<double> color{};
    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;
    _bounds_name = ros::this_node::getNamespace() + _bounds_name;
    _viz_env_name = ros::this_node::getNamespace() + _viz_env_name;
    _reload_service_name = ros::this_node::getNamespace() + _reload_service_name;
    PARAM_SETUP_WITH_DEFAULT(private_nh, color, std::vector<double>({ 1.0, 0.0, 1.0, 0.0 }));

    _timer = private_nh.createTimer(ros::Rate(1.0), &Derived::update, this);

    // publishers
    _bounds_publisher = private_nh.advertise<visualization_msgs::Marker>(_bounds_name, 1, true);
    _environment_publisher = private_nh.advertise<visualization_msgs::MarkerArray>(_viz_env_name, 1, true);
  }

  void update(const ros::TimerEvent& t)
  {
    std::string environment{ "" };
    GLOBAL_PARAM_SETUP_DEFAULT(environment, _environment_file)
    // if (ros::param::has("environment") and ros::param::get("environment", environment))
    // {
    //   PRINT_MSG("Environment not set");
    // }
    // DEBUG_VARS(environment)
    if (environment != _environment_file)
    {
      read_and_publish_environment(environment);
      _environment_file = environment;
    }
    if (_msg_valid)
      _environment_publisher.publish(_msg);
    if (_bounds_valid)
      _bounds_publisher.publish(_bounds_marker);
  }

  // bool reload_environment_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  // {
  //   PRINT_MSG("Environment Publisher service called")
  //   read_and_publish_environment(_environment_file);
  //   return true;
  // }

protected:
  // bool filename_callback(ml4kp_bridge::SendString::Request& request, ml4kp_bridge::SendString::Response& response)
  // {
  //   PRINT_MSG("NOT SUPPORTED");
  //   // const std::string filename{ request.string };
  //   // read_and_publish_environment(filename);
  //   return true;
  // }
  void publish_bounds(prx::param_loader& env_params)
  {
    std::vector<double> max_bounds{ env_params["environment/bounds/max"].template as<std::vector<double>>() };
    std::vector<double> min_bounds{ env_params["environment/bounds/min"].template as<std::vector<double>>() };
    // std::vector<double> position{ _params["environment/root_configuration/position"].as<std::vector<double>>() };
    // std::vector<double> orientation{ _params["environment/root_configuration/orientation"].as<std::vector<double>>()
    // };

    const double x_diff{ max_bounds[0] - min_bounds[0] };
    const double y_diff{ max_bounds[1] - min_bounds[1] };
    const double z_diff{ max_bounds[2] - min_bounds[2] };

    const double x_half{ (max_bounds[0] + min_bounds[0]) / 2.0 };
    const double y_half{ (max_bounds[1] + min_bounds[1]) / 2.0 };
    const double z_half{ (max_bounds[2] + min_bounds[2]) / 2.0 };

    _bounds_marker.header.frame_id = "world";
    _bounds_marker.header.stamp = ros::Time();
    _bounds_marker.ns = "bounds";
    _bounds_marker.id = 0;
    _bounds_marker.action = visualization_msgs::Marker::ADD;

    _bounds_marker.pose.position.x = x_half;
    _bounds_marker.pose.position.y = y_half;
    _bounds_marker.pose.position.z = z_half;
    _bounds_marker.pose.orientation.x = 0.0;
    _bounds_marker.pose.orientation.y = 0.0;
    _bounds_marker.pose.orientation.z = 0.0;
    _bounds_marker.pose.orientation.w = 1.0;
    _bounds_marker.color.a = 0.1;  // Don't forget to set the alpha!
    _bounds_marker.color.r = 1.0;
    _bounds_marker.color.g = 0.0;
    _bounds_marker.color.b = 0.0;

    _bounds_marker.type = visualization_msgs::Marker::CUBE;
    _bounds_marker.scale.x = x_diff;
    _bounds_marker.scale.y = y_diff;
    _bounds_marker.scale.z = z_diff;

    _bounds_valid = true;
  }

  void read_and_publish_environment(std::string& environment_file)
  {
    // std::string& environment{ _environment_file };

    // This assumes "environment" is a string that has the content of the yaml file
    // This is, it was read by ros param as:
    // This setup allows to specify the environment once and be read in multiple programs
    // <param name="environment" textfile="$(find PACKAGE)/PATH/TO/ENVIRONMENT.yaml" />

    prx::param_loader env_params;
    env_params.from_string(environment_file);

    prx_assert(env_params.exists("environment"), "Params: 'environment' needed");
    prx_assert(env_params.exists("environment/name"), "Params: 'environment/name' needed");
    prx_assert(env_params.exists("environment/bounds"), "Params: 'environment/bounds' needed");

    const std::string environment_name{ env_params["environment/name"].template as<std::string>() };

    DEBUG_VARS(environment_name)

    prx::obstacle_loader_t obstacle_loader{ prx::obstacle_loader_t(env_params) };
    publish_bounds(env_params);
    // const prx::PairNameObstacles obstacles{ prx::obstacle_loader_t(pl) };

    const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacle_loader.get_obstacles() };
    const std::vector<std::string> obstacle_names{ obstacle_loader.get_names() };

    for (int i = 0; i < obstacle_list.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "environment";
      marker.id = i;
      marker.action = visualization_msgs::Marker::ADD;

      const std::shared_ptr<prx::box_t> box{ std::dynamic_pointer_cast<prx::box_t>(obstacle_list[i]) };
      const std::shared_ptr<prx::cylinder_t> cylinder{ std::dynamic_pointer_cast<prx::cylinder_t>(obstacle_list[i]) };
      const std::shared_ptr<prx::sphere_t> sphere{ std::dynamic_pointer_cast<prx::sphere_t>(obstacle_list[i]) };
      // DEBUG_VARS((sphere ? "sphere" : "nullptr"));
      if (box or cylinder or sphere)
      {
        prx::movable_object_t::Geometries geometries{ obstacle_list[i]->get_geometries() };
        prx::movable_object_t::Configurations configs{ obstacle_list[i]->get_configurations() };
        // geometries =
        // configs =

        for (int j = 0; j < configs.size(); ++j)
        {
          const std::shared_ptr<prx::geometry_t> gi{ geometries[j].second.lock() };
          const std::shared_ptr<prx::transform_t> Rt{ configs[j].second.lock() };
          prx_assert(Rt != nullptr, "Transform is null!");
          const std::string str_color{ gi->get_visualization_color() };
          const Color color{ get_color(str_color) };

          const Eigen::Vector3d t{ Rt->translation() };
          const Eigen::Matrix3d rot{ Rt->rotation() };
          // Without this, it throws an assertion on unaligned arrays on some linux machines
          const Eigen::Quaternion<double, Eigen::DontAlign> q{ rot };
          const std::vector<double> geom_params{ gi->get_geometry_params() };

          marker.pose.position.x = t[0];
          marker.pose.position.y = t[1];
          marker.pose.position.z = t[2];
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.pose.orientation.w = q.w();
          marker.color.a = color[0];  // Don't forget to set the alpha!
          marker.color.r = color[1];
          marker.color.g = color[2];
          marker.color.b = color[3];

          if (box)
          {
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = geom_params[0];
            marker.scale.y = geom_params[1];
            marker.scale.z = geom_params[2];
          }
          else if (cylinder)
          {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = geom_params[0] * 2.0;  // Ros needs diameter, prx in rad
            marker.scale.y = geom_params[0] * 2.0;  // Ros needs diameter, prx in rad
            marker.scale.z = geom_params[1];
          }
          else if (sphere)
          {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = geom_params[0] * 2.0;  // Ros needs diameter, prx in rad
            marker.scale.y = geom_params[0] * 2.0;  // Ros needs diameter, prx in rad
            marker.scale.z = geom_params[0] * 2.0;
          }
          _msg.markers.push_back(marker);
        }
      }

      // msg.markers.push_back(marker);
    }

    // const Eigen::Vector min_bounds{ obstacle_loader.min_bounds() };
    // const Eigen::Vector max_bounds{ obstacle_loader.max_bounds() };
    _msg_valid = true;
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

  visualization_msgs::Marker _bounds_marker;

  // prx::param_loader _params;

  bool _msg_valid, _bounds_valid;
  std::string _bounds_name;
  visualization_msgs::MarkerArray _msg;

  std::string _environment_file;

  // Topic names
  std::string _viz_env_name;
  std::string _reload_service_name;

  // Subscribers
  ros::Subscriber _tree_subscriber;

  // Publishers
  ros::Publisher _environment_publisher, _bounds_publisher;

  ros::Timer _timer;

  ros::ServiceServer _reload_service;
};
}  // namespace utils