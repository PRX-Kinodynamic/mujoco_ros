#pragma once

#include <ros/ros.h>

#include "mujoco_ros/defs.h"

namespace mj_ros
{
template <typename Observation>
class ground_truth_pose_t
{
public:
  ground_truth_pose_t(ros::NodeHandle& nh, SimulatorPtr sim, double frequency) : _sim(sim), _frequency(frequency)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string topic_name{ root + "/pose" };
    _publisher = nh.advertise<Observation>(topic_name, 1000);
  }

  void run()
  {
    ros::Rate rate(_frequency);
    while (ros::ok())
    {
      prx_models::get_observation(_message, _sim->d->sensordata);
      _publisher.publish(_message);
      rate.sleep();
    }
  }

private:
  const SimulatorPtr _sim;
  ros::Publisher _publisher;
  double _frequency;
  Observation _message;
};
}  // namespace mj_ros