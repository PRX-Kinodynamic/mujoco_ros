#include <ml4kp_bridge/defs.h>
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "prx_models/mj_mushr.hpp"
#include <ml4kp_bridge/TrajectoryStamped.h>
#include <prx_models/MushrObservation.h>
#include <std_msgs/Empty.h>

namespace control
{
class mushr_feedback_controller_t : public nodelet::Nodelet
{
public:
  mushr_feedback_controller_t()
  {
    std::string params_file;
    ros::param::get("/ml4kp_params", params_file);
    std::cout << "Loading params from: " << params_file << std::endl;
    auto ml4kp_params = prx::param_loader(params_file);
    auto controller_params = prx::param_loader(ml4kp_params["feedback_controller"].as<std::string>());

    std::string plant_name = ml4kp_params["/plant/name"].as<>();
    std::string plant_path = ml4kp_params["/plant/path"].as<>();
    plant = prx::system_factory_t::create_system(plant_name, plant_path);
    ROS_ASSERT(plant != nullptr);

    // Control space limits of the real MuSHR
    std::vector<double> min_control_limits = {-1., -1.};
    std::vector<double> max_control_limits = {1., 1.};
    plant->get_control_space()->set_bounds(min_control_limits, max_control_limits);

    trajectory = std::make_shared<prx::trajectory_t>(plant->get_state_space());
    current_state = plant->get_state_space()->make_point();

    if (controller_params["name"].as<std::string>() == "mushr_pure_pursuit")
    {
      controller = new prx::mushr_pure_pursuit_t(plant, controller_params);
    }
    else if (controller_params["name"].as<std::string>() == "mushr_stanley")
    {
      controller = new prx::mushr_stanley_t(plant, controller_params);
    }
    else
    {
      ROS_ERROR("Unknown controller type: %s", controller_params["name"].as<std::string>().c_str());
      return;
    }
    controller->set_goal(ml4kp_params["goal_state"].as<std::vector<double>>());
  }

  void onInit()
  {
    ros::NodeHandle& nh{ getPrivateNodeHandle() };
    std::string trajectory_topic, pose_topic, control_topic, reset_topic;

    nh.getParam("trajectory_topic", trajectory_topic);
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("control_topic", control_topic);
    nh.getParam("reset_topic", reset_topic);

    ros::Duration _timer_duration = ros::Duration(controller->control_duration());
    _timer = nh.createTimer(_timer_duration, &mushr_feedback_controller_t::control, this);

    _trajectory_subscriber = nh.subscribe(trajectory_topic, 1, &mushr_feedback_controller_t::get_trajectory, this);
    _pose_subscriber = nh.subscribe(pose_topic, 1, &mushr_feedback_controller_t::get_pose, this);
    _reset_subscriber = nh.subscribe(reset_topic, 1, &mushr_feedback_controller_t::reset, this);

    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(plant->get_control_space()->get_dimension());
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStamped& msg)
  {
    ml4kp_bridge::copy(trajectory, msg.trajectory);
    controller->set_points(trajectory);
    _trajectory_received = true;
  }

  void get_pose(const prx_models::MushrObservation& msg)
  {
    prx_models::copy(current_state, msg);
    _pose_received = true;
  }

  void reset(const std_msgs::Empty& msg)
  {
    _trajectory_received = false;
    _pose_received = false;
    controller->reset();
  }

  void control(const ros::TimerEvent& event)
  {
    controller->get_control(current_state, current_control);
    plant->get_control_space()->enforce_bounds(current_control);
    ml4kp_bridge::copy(_ctrl, current_control);
    _control_publisher.publish(_ctrl);
  }

private:
  ros::Subscriber _trajectory_subscriber, _pose_subscriber, _reset_subscriber;
  ros::Publisher _control_publisher;
  ros::Timer _timer;

  prx_models::MushrObservation _pose;
  ml4kp_bridge::SpacePoint _ctrl;
  bool _trajectory_received, _pose_received;

  prx::system_ptr_t plant;
  prx::feedback_controller_t<prx::trajectory_t>* controller;
  std::shared_ptr<prx::trajectory_t> trajectory;
  prx::space_point_t current_state;
  Eigen::VectorXd current_control;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::mushr_feedback_controller_t, nodelet::Nodelet);