#pragma once

#include <ros/ros.h>

#include "mujoco_ros/defs.h"

namespace mj_ros
{
class sensordata_publisher_t
{
public:
  sensordata_publisher_t(ros::NodeHandle& nh, SimulatorPtr sim, double frequency) : _sim(sim), _frequency(frequency)
  {
    const std::string root{ ros::this_node::getNamespace() };
    const std::string topic_name{ root + "/sensordata" };
    _publisher = nh.advertise<interface::SensorDataStamped>(topic_name, 1000, true);
  }

  void run()
  {
    ros::Rate rate(_frequency);
    while (ros::ok())
    {
      _message.raw_sensor_data.resize(_sim->m->nsensordata);
      for (int i = 0; i < _sim->m->nsensordata; ++i)
      {
        _message.raw_sensor_data[i].data = _sim->d->sensordata[i];
      }
      _message.header.seq++;
      _message.header.stamp = ros::Time::now();
      _publisher.publish(_message);
      rate.sleep();
    }
  }

private:
  SimulatorPtr _sim;
  double _frequency;
  ros::Publisher _publisher;
  interface::SensorDataStamped _message;
};
}  // namespace mj_ros