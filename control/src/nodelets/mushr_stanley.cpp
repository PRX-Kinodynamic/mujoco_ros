#include <ml4kp_bridge/defs.h>
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ml4kp_bridge/TrajectoryStamped.h>
#include <prx_models/MushrObservation.h>
#include <std_msgs/Empty.h>

namespace control
{
class mushr_stanley_t : public nodelet::Nodelet
{
public:
  mushr_stanley_t() : _trajectory_received(false), _pose_received(false)
  {
    ros::NodeHandle& nh { getPrivateNodeHandle() };
    std::string trajectory_topic, pose_topic, control_topic, reset_topic;
    double frequency;

    nh.getParam("trajectory_topic", trajectory_topic);
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("control_topic", control_topic);
    nh.getParam("frequency", frequency);
    nh.getParam("reset_topic", reset_topic);

    _timer_duration = ros::Duration(1.0 / frequency);
    discretization = 1.0 / (prx::simulation_step * frequency);
    ROS_INFO("Discretization: %d", discretization);
    _timer = nh.createTimer(_timer_duration, &mushr_stanley_t::control, this);
    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(2);
    
    _trajectory_subscriber = nh.subscribe(trajectory_topic, 1, &mushr_stanley_t::get_trajectory, this);
    _pose_subscriber = nh.subscribe(pose_topic, 1, &mushr_stanley_t::get_pose, this);
    _reset_subscriber = nh.subscribe(reset_topic, 1, &mushr_stanley_t::reset, this);
  }

  void control(const ros::TimerEvent& event)
  {
    if (_trajectory_received && _pose_received)
    {
      // regulator(_trajectory, _pose, _ctrl);
    }
    else
    {
      _ctrl.point[0].data = 0;
      _ctrl.point[1].data = 0;
    }
    _control_publisher.publish(_ctrl);
  }

  void reset(const std_msgs::EmptyConstPtr message)
  {
    _trajectory_received = false;
    _pose_received = false;
    discretized_trajectory.clear();
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStampedConstPtr message)
  {
    _trajectory = *message;
    _trajectory_received = true;

    discretized_trajectory.clear();
    for (int i = 0; i < _trajectory.trajectory.data.size(); i += discretization)
    {
      discretized_trajectory.push_back(_trajectory.trajectory.data[i]);
    }

    if (_trajectory.trajectory.data.size() % discretization != 0)
    {
      discretized_trajectory.push_back(_trajectory.trajectory.data.back());
    }

    ROS_INFO("Discretized trajectory size: %d", discretized_trajectory.size());
  }

  void get_pose(const prx_models::MushrObservationConstPtr message)
  {
    _pose = *message;
    _pose_received = true;
  }

protected:
  virtual void onInit()
  {
  }

private:
  bool _trajectory_received, _pose_received;
  int discretization;

  ros::Subscriber _trajectory_subscriber, _pose_subscriber, _reset_subscriber;
  ros::Publisher _control_publisher;
  ros::Timer _timer;
  ros::Duration _timer_duration;

  ml4kp_bridge::TrajectoryStamped _trajectory;
  prx_models::MushrObservation _pose;
  std::vector<ml4kp_bridge::SpacePoint> discretized_trajectory;

  ml4kp_bridge::SpacePoint _ctrl;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::mushr_stanley_t, nodelet::Nodelet);
