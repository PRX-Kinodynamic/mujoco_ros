#include <ros/node_handle.h>
#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

#include <memory>
#include <utils/rosparams_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <interface/node_status.hpp>
#include "interface/SensorDataStamped.h"
#include "prx_models/mushr_factors.hpp"

struct runner_t
{
  using State = prx_models::mushr_types::State::type;

  ros::Timer _timer;
  ros::Subscriber _sensor_subscriber, _collision_subscriber;

  State _state;

  std::shared_ptr<interface::node_status_t> mj_status, stela_status, rosbag_status;
  runner_t(ros::NodeHandle& nh)
  {
    std::string sensor_topic_name, collision_topic_name;
    std::string stela_node_id, mj_node_id, rosbag_node_id;

    PARAM_SETUP(nh, mj_node_id);
    PARAM_SETUP(nh, stela_node_id);
    PARAM_SETUP(nh, rosbag_node_id);
    PARAM_SETUP(nh, sensor_topic_name);
    PARAM_SETUP(nh, collision_topic_name);

    interface::node_status_t node_status(nh);

    mj_status = interface::node_status_t::create(nh, mj_node_id, true);
    stela_status = interface::node_status_t::create(nh, stela_node_id, true);
    rosbag_status = interface::node_status_t::create(nh, rosbag_node_id, true);

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &runner_t::sensor_callback, this);
    _collision_subscriber = nh.subscribe(collision_topic_name, 1, &runner_t::collision_callback, this);

    _timer = nh.createTimer(ros::Duration(1.0 / 10.0), &simulator_t::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
  }

  void collision_callback(const std_msgs::BoolConstPtr msg)
  {
    _collision = msg->data;
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<std_msgs::Float64>& zi{ msg->raw_sensor_data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3].data, zi[4].data, zi[5].data, zi[6].data) };
    _state[0] = zi[0].data;
    _state[1] = zi[1].data;
    _state[2] = prx::quaternion_to_euler(q)[2];
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrExperiments" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string stela_node_id, mj_node_id, rosbag_node_id;

  PARAM_SETUP(nh, mj_node_id);
  PARAM_SETUP(nh, stela_node_id);
  PARAM_SETUP(nh, rosbag_node_id);

  // const std::string root{ ros::this_node::getName() };

  ros::spin();
  return 0;
}