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
  feedback_client_t(const std::string root, ros::NodeHandle& nh, mjData* mj_data, double frequency)
    : _mj_data(mj_data), _frequency(frequency)
  {
    const std::string feedback_service_name{ root + "/feedback_service" };
    _service_client = nh.serviceClient<Service>(feedback_service_name);
  }

  void run()
  {
    ros::Rate rate(_frequency);
    while (ros::ok())
    {
      mj_models::state_from_sensors(_service.request.state, _mj_data->sensordata);
      if (_service_client.call(_service))
      {
        mj_models::copy(_mj_data->ctrl, _service.response.control);
      }
      else
      {
        ROS_WARN_STREAM("Failed to call service of " << typeid(*this).name());
      }
      rate.sleep();
    }
  }

private:
  mjData* const _mj_data;
  ros::ServiceClient _service_client;
  Service _service;
  double _frequency;
};
}  // namespace mj_ros