#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/Pose2D.h>
#include <ml4kp_bridge/TrajectoryStamped.h>

namespace task_planning
{
class feedback_nodelet_t : public nodelet::Nodelet
{
public:
  feedback_nodelet_t() : _trajectory_received(false), _trajectory()
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    std::string publisher_topic{};
    std::string subscriber_topic{};
    double frequency;

    private_nh.getParam("publisher_topic", publisher_topic);
    private_nh.getParam("subscriber_topic", subscriber_topic);
    private_nh.getParam("frequency", frequency);

    _timer_duration = ros::Duration(1.0 / frequency);
    _timer = private_nh.createTimer(_timer_duration, &feedback_nodelet_t::timer_callback, this);
    _publisher = private_nh.advertise<geometry_msgs::Pose2D>(publisher_topic, 1, true);
    _subscriber = private_nh.subscribe(subscriber_topic, 1, &feedback_nodelet_t::get_trajectory, this);
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStampedConstPtr trajectory_in)
  {
    _trajectory = *trajectory_in;
    _trajectory_received = true;
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    if (_trajectory_received)
    {
      if (_trajectory.trajectory.data.size() > 0)
      {
        // _current_goal.x = _trajectory.trajectory.points[0].x;
        // _current_goal.y = _trajectory.trajectory.points[0].y;
        // _current_goal.theta = _trajectory.trajectory.points[0].theta;
        // _publisher.publish(_current_goal);
      }
      _trajectory_received = false;
    }
  }

  geometry_msgs::Pose2D _current_goal;
  ml4kp_bridge::TrajectoryStamped _trajectory;
  bool _trajectory_received;

  ros::Publisher _publisher;
  ros::Subscriber _subscriber;
  ros::Timer _timer;
  ros::Duration _timer_duration;
};
}  // namespace task_planning
PLUGINLIB_EXPORT_CLASS(task_planning::feedback_nodelet_t, nodelet::Nodelet);