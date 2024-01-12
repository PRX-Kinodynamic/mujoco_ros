#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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
  using Transform = Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;

public:
  plant_estimator_nodelet_t() : _robot_transform(Transform::Identity())
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    std::string plan_topic;
    std::string trajectory_topic;
    std::string world_frame;
    std::string robot_frame;
    double simulation_step;

    NODELET_PARAM_SETUP(private_nh, plan_topic);
    NODELET_PARAM_SETUP(private_nh, trajectory_topic)
    NODELET_PARAM_SETUP(private_nh, simulation_step)
    NODELET_PARAM_SETUP(private_nh, world_frame);
    NODELET_PARAM_SETUP(private_nh, robot_frame);

    _world_frame = world_frame;
    _robot_frame = robot_frame;

    prx::simulation_step = simulation_step;
    _plant_name = "mushr";
    _plant_path = "mushr";
    _plant = prx::system_factory_t::create_system(_plant_name, _plant_path);
    prx_assert(_plant != nullptr, "Failed to create plant");

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
    _estimated_traj = std::make_shared<prx::trajectory_t>(_state_space.get());
    _current_state = _state_space->make_point();
    Vec(_current_state) = Eigen::VectorXd::Zero(_state_space->size());

    _header.seq = 0;
    _header.stamp = ros::Time::now();
    _header.frame_id = "world";

    _plan_subscriber = private_nh.subscribe(plan_topic, 1, &plant_estimator_nodelet_t::get_plan, this);
    _trajectory_publisher = private_nh.advertise<ml4kp_bridge::TrajectoryStamped>(trajectory_topic, 1);
    _pose_timer = private_nh.createTimer(ros::Duration(0.1), &plant_estimator_nodelet_t::estimate_pose, this);
  }

  void correct_pose()
  {
    // For now, only from transform to state
    _current_state->at(0) = _robot_transform.translation()[0];
    _current_state->at(1) = _robot_transform.translation()[1];
    _current_state->at(2) = _robot_transform.translation()[1];
  }

  void estimate_pose(const ros::TimerEvent& event)
  {
    if (_tf_listener.canTransform(_robot_frame, _world_frame, ros::Time(0)))
    {
      _tf_listener.lookupTransform(_world_frame, _robot_frame, ros::Time(0), _tf_robot);
      tf::transformTFToEigen(_tf_robot, _robot_transform);
    }
    correct_pose();
  }

  void get_plan(const ml4kp_bridge::PlanStampedConstPtr message)
  {
    _plan->clear();
    ml4kp_bridge::copy(_plan, message);

    _system_group->propagate(_current_state, *_plan, *_estimated_traj);

    ml4kp_bridge::TrajectoryStamped traj_msg;
    _header.stamp = ros::Time::now();
    traj_msg.header = _header;
    ml4kp_bridge::copy(traj_msg.trajectory, _estimated_traj);
    _trajectory_publisher.publish(traj_msg);
  }

  std_msgs::Header _header;

  Transform _robot_transform;
  tf::StampedTransform _tf_robot;
  tf::TransformListener _tf_listener;

  std::string _plant_name;
  std::string _plant_path;
  std::string _world_frame;
  std::string _robot_frame;

  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::world_model_t> _world_model;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<prx::space_t> _state_space, _control_space;

  prx::space_point_t _current_state;
  std::shared_ptr<prx::plan_t> _plan;
  std::shared_ptr<prx::trajectory_t> _estimated_traj;

  ros::Subscriber _plan_subscriber;
  ros::Publisher _trajectory_publisher;
  ros::Timer _pose_timer;
};
}  // namespace estimation
PLUGINLIB_EXPORT_CLASS(estimation::plant_estimator_nodelet_t, nodelet::Nodelet);