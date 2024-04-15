#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>

namespace motion_planning
{
template <typename Base, typename Planner>
class sbmp_graph_publisher_t : public Base
{
public:
  nodelet_t() : graph_topic("/wTc/image"){};
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    ROS_PARAM_SETUP(private_nh, random_seed);
    ROS_PARAM_SETUP(private_nh, plant_config_file);
    ROS_PARAM_SETUP(private_nh, planner_config_file);
    ROS_PARAM_SETUP(private_nh, environment);
    PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);

    _trajectory_publisher = private_nh.advertise<ml4kp_bridge::TrajectoryStamped>(trajectory_topic, 1);
  }
};
}  // namespace motion_planning