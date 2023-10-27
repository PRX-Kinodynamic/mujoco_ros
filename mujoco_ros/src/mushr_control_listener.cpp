#include "mujoco_ros/control_listener.hpp"
#include "mujoco_ros/MushrControl.h"

class MushrControlListener
{
private:
  ros::NodeHandle n;
  ros::Subscriber control_subscriber, reset_subscriber, save_subscriber;
  std::shared_ptr<mujoco_simulator_t> sim;
  std::string package_path;

public:
  MushrControlListener()
  {
    package_path = ros::package::getPath("mujoco_ros");

    std::string model_path;
    if (!n.getParam("/model_path", model_path))
    {
      ROS_ERROR("Failed to get param 'model_path'.");
      return;
    }

    bool save_trajectory;
    if (!n.getParam("/save_trajectory", save_trajectory))
    {
      ROS_ERROR("Failed to get param 'save_trajectory'.");
      return;
    }

    sim = std::make_shared<mujoco_simulator_t>(model_path, save_trajectory);
    control_subscriber = n.subscribe("control", 1000, &MushrControlListener::control_callback, this);
    reset_subscriber = n.subscribe("reset", 1000, &MushrControlListener::reset_callback, this);
    save_subscriber = n.subscribe("save_trajectory", 1000, &MushrControlListener::save_callback, this);
  }

  void control_callback(const mujoco_ros::MushrControl::ConstPtr& msg)
  {
    std::vector<double> control;
    control.push_back(msg->steering_angle.data);
    control.push_back(msg->velocity.data);
    sim->set_control(control);
    sim->propagate(msg->duration.data);
    ROS_INFO("Result: %f, %f", sim->d->qpos[0], sim->d->qpos[1]);
  }

  void reset_callback(const std_msgs::Empty::ConstPtr& msg)
  {
    sim->reset_simulation();
    ROS_INFO("Result: %f, %f", sim->d->qpos[0], sim->d->qpos[1]);
  }

  void save_callback(const std_msgs::Empty::ConstPtr& msg)
  {
    std::string trajectory_path = package_path + "/data/trajectory.txt";
    std::ofstream trajectory_file(trajectory_path);
    trajectory_file << sim->print_trajectory();
    trajectory_file.close();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mushr_control_listener");
  MushrControlListener listener;
  ros::spin();
  return 0;
}
