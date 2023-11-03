#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

#include "mujoco_ros/defs.h"

namespace mj_ros
{
template <typename Service>
class feedback_client_t
{
public:
  feedback_client_t(ros::NodeHandle& nh, SimulatorPtr sim, double frequency) : _sim(sim), _frequency(frequency)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string feedback_service_name{ root + "/feedback_service" };
    _service_client = nh.serviceClient<Service>(feedback_service_name);
  }

  void run()
  {
    ros::Rate rate(_frequency);
    while (ros::ok())
    {
      prx_models::get_observation(_service.request.observation, _sim->d->sensordata);
      if (_service_client.call(_service))
      {
        prx_models::copy(_sim->d->ctrl, _service.response.control);
      }
      else
      {
        ROS_WARN_STREAM_ONCE("Failed to call service of " << typeid(*this).name());
      }
      rate.sleep();
    }
  }

private:
  const SimulatorPtr _sim;
  ros::ServiceClient _service_client;
  Service _service;
  double _frequency;
};
}  // namespace mj_ros
