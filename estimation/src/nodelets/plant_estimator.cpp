#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.h>
#include <interface/utils.hpp>
#include <interface/stamped_markers.h>
#include <interface/defs.hpp>
#include <ml4kp_bridge/defs.h>
namespace estimation
{
class plant_estimator_nodelet_t : public nodelet::Nodelet
{
public:
  plant_estimator_nodelet_t()
  {
  }

private:
  virtual void onInit()
  {
    PRX_DEBUG_PRINT;
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    std::string plan_topic;

    PRX_DEBUG_PRINT;
    NODELET_PARAM_SETUP(private_nh, plan_topic);

    _plant_name = "mushr";
    _plant_path = "mushr";
    _plant = prx::system_factory_t::create_system(_plant_name, _plant_path);
    prx_assert(_plant != nullptr, "Failed to create plant");

    PRX_DEBUG_PRINT;
    // const std::vector<prx::system_ptr_t> plants(_plant);
    // _world_model = std::make_shared<prx::worldmodel_t>(plants, {});
    _world_model.reset(new prx::world_model_t({ _plant }, {}));
    _world_model->create_context("estimator_context", { _plant_name }, {});
    auto context = _world_model->get_context("estimator_context");
    _system_group = context.first;
    _collision_group = context.second;
    _state_space.reset(_system_group->get_state_space());
    _control_space.reset(_system_group->get_control_space());
    std::vector<double> min_control_limits = { -1., -1. };
    std::vector<double> max_control_limits = { 1., 1. };
    _control_space->set_bounds(min_control_limits, max_control_limits);
    _plan = std::make_shared<prx::plan_t>(_control_space.get());
    _plan_subscriber = private_nh.subscribe(plan_topic, 1, &plant_estimator_nodelet_t::get_plan, this);
    PRX_DEBUG_PRINT;
  }

  void get_plan(const ml4kp_bridge::PlanStampedConstPtr message)
  {
    PRX_DEBUG_PRINT;
    ml4kp_bridge::copy(_plan, message);
    PRX_DEBUG_VARS(*_plan);
  }
  std::string _plant_name;
  std::string _plant_path;
  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::world_model_t> _world_model;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<prx::space_t> _state_space, _control_space;

  std::shared_ptr<prx::plan_t> _plan;

  ros::Subscriber _plan_subscriber;
};
}  // namespace estimation
PLUGINLIB_EXPORT_CLASS(estimation::plant_estimator_nodelet_t, nodelet::Nodelet);