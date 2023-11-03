#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

#include <mujoco_ros/simulator.hpp>
#include <prx_models/MushrPlan.h>
#include <prx_models/mj_mushr.hpp>

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
    const std::string param_name_save_trajectory{ ros::this_node::getName() + "/save_trajectory" };
    if (!n.getParam(param_name_save_trajectory, save_trajectory))
    {
      ROS_ERROR("Failed to get param 'save_trajectory'.");
      return;
    }

    bool visualize;
    const std::string param_name_visualize{ ros::this_node::getName() + "/visualize" };
    if (!n.getParam(param_name_visualize, visualize))
    {
      ROS_ERROR("Failed to get param 'visualize'.");
      return;
    }

    sim = std::make_shared<mujoco_simulator_t>(model_path, save_trajectory, visualize);
    control_subscriber = n.subscribe("control", 1000, &MushrControlListener::control_callback, this);
    reset_subscriber = n.subscribe("reset", 1000, &MushrControlListener::reset_callback, this);
    save_subscriber = n.subscribe("save_trajectory", 1000, &MushrControlListener::save_callback, this);
  }

  void control_callback(const prx_models::MushrPlan::ConstPtr& msg)
  {
    for (int i = 0; i < msg->controls.size(); ++i)
    {
      prx_models::copy(sim->d->ctrl, msg->controls[i]);
      sim->propagate(msg->durations[i].data);
      ROS_INFO("Result: %f, %f", sim->d->qpos[0], sim->d->qpos[1]);
    }
  }

  void reset_callback(const std_msgs::Empty::ConstPtr& msg)
  {
    sim->reset_simulation();
    ROS_INFO("Result: %f, %f", sim->d->qpos[0], sim->d->qpos[1]);
  }

  void save_callback(const std_msgs::Empty::ConstPtr& msg)
  {
    std::string trajectory_path, trajectory_fname, full_param_name;
    full_param_name = ros::this_node::getName() + "/trajectory_file";
    n.getParam(full_param_name, trajectory_fname);
    trajectory_path = package_path + "/" + trajectory_fname;
    ROS_INFO_STREAM("Saving trajectory to " << trajectory_path << std::endl);
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
