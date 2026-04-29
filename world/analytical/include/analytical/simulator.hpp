#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <stdio.h>

#include <ros/ros.h>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <prx/simulation/collision_checking/collision_group.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/spaces/space.hpp>
#include <utils/std_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/defs.h>
// #include <analytical/fg_ltv_sde.hpp>

// Keeping this file just in case..
#include <prx_models/defs.hpp>
#include "interface/SensorDataStamped.h"
#include "ml4kp_bridge/SpacePoint.h"
#include "ml4kp_bridge/SpacePointStamped.h"
#include <interface/node_status.hpp>

namespace analytical
{
class simulator_t
{
public:
  // Replicate MJ interface:
  // In: controls
  // Out: Sensors
  simulator_t(ros::NodeHandle& nh)
  {
    double& simulation_step{ prx::simulation_step };
    std::string set_state_topic, collision_topic, ctrl_topic, sensor_topic;
    DEBUG_PRINT

    PARAM_SETUP(nh, set_state_topic);
    PARAM_SETUP(nh, collision_topic);
    PARAM_SETUP(nh, simulation_step);
    PARAM_SETUP(nh, ctrl_topic);
    PARAM_SETUP(nh, sensor_topic);
    // GLOBAL_PARAM_SETUP(environment);

    // prx::param_loader plant_params;
    ml4kp_bridge::copy(_plant_params, nh);

    _node_status = interface::node_status_t::create(nh);

    _sensor_publisher = nh.advertise<interface::SensorDataStamped>(sensor_topic, 1, true);
    _collision_publisher = nh.advertise<std_msgs::Bool>(collision_topic, 1, true);

    _state_subscriber = nh.subscribe(set_state_topic, 1, &simulator_t::set_state_callback, this);
    _control_subscriber = nh.subscribe(ctrl_topic, 1, &simulator_t::control_callback, this);
    _control_stamped_subscriber =
        nh.subscribe(ctrl_topic + "_stamped", 1, &simulator_t::control_stamped_callback, this);

    _step_timer = nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::step_callback, this);

    _node_status->status(interface::NodeStatus::INITIALIZING);
  }

  virtual ~simulator_t()
  {
  }

protected:
  void init_ml4kp()
  {
    // const std::string plant_name{ _plant_params["name"].as<std::string>() };
    // const std::string plant_path{ _plant_params["path"].as<std::string>() };
    // _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    DEBUG_PRINT
    _world_model.reset();
    _system_group.reset();
    _collision_group.reset();

    DEBUG_PRINT
    _plant = prx::system_factory_t::create_system(_plant_params);
    prx_assert(_plant != nullptr, "Failed to create plant");
    // _plant->init(_plant_params);
    std::tie(_world_model, _system_group, _collision_group) = prx::world_model_t::create(_prx_params, _plant);

    DEBUG_PRINT
    // _state_space.reset(_system_group->get_state_space());
    // DEBUG_PRINT
    // _control_space.reset(_system_group->get_control_space());
    // DEBUG_PRINT
    // _sensor_space.reset(_system_group->get_sensor_space());
    // auto ps = context.first->get_parameter_space();

    DEBUG_PRINT
    _x0 = _system_group->get_state_space()->make_point();
    _u0 = _system_group->get_control_space()->make_point();

    DEBUG_PRINT
    _system_group->get_state_space()->copy_to(_x0);
    _system_group->get_control_space()->copy_to(_u0);

    // _system_group = prx::system_group(context);
    // _collision_group = prx::collision_group(context);
    DEBUG_PRINT

    _sensor_msg.raw_sensor_data.resize(_system_group->get_sensor_space()->size());
  }

  void set_state_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _system_group->get_state_space()->copy_from(msg->space_point.point);
  }

  void control_stamped_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _system_group->get_control_space()->copy_from(msg->space_point.point);
  }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    _system_group->get_control_space()->copy_from(msg->point);
  }

  void step_simulation()
  {
    if (not _system_group)
    {
      PRINT_MSG("[simulator_t] not initialize yet... call Reset on node status")
      return;
    }
    _collision_msg.data = false;

    _system_group->propagate_once();

    _system_group->sense();

    if (_collision_group->in_collision())
    {
      _collision_msg.data = true;
    }

    _system_group->get_sensor_space()->copy_to(_sensor_msg.raw_sensor_data);
    _sensor_msg.header.stamp = ros::Time::now();

    _sensor_publisher.publish(_sensor_msg);
    _collision_publisher.publish(_collision_msg);
  }

  bool reset_simulation()
  {
    PRINT_MSG("Reseting..")
    // ros::Duration(1.0).sleep();

    std::string environment;
    GLOBAL_PARAM_SETUP_DEFAULT(environment, _environment_file)
    // if (ros::param::has("/environment") and ros::param::get("/environment", environment))
    // DEBUG_VARS(environment)
    if (environment != _environment_file)
    {
      _prx_params.from_string(environment);
      _environment_file = environment;
      init_ml4kp();
    }
    if (_environment_file.size() > 0)
    {
      _system_group->get_state_space()->copy_from(_x0);
      _system_group->get_control_space()->copy_from(_u0);
      return true;
    }
    return false;
  }

  void step_callback(const ros::TimerEvent& event)
  {
    if (_node_status->new_request())
    {
      if (_node_status->requested_status() == interface::NodeStatus::RESET)
      {
        if (_node_status->status() != interface::NodeStatus::RESET)
        {
          PRINT_MSG("[mj_ros::simulator_t] Setting to reset")
          if (reset_simulation())
          {
            _node_status->status(_node_status->requested_status());
            _node_status->request_acknowledged();

            // _node_status->status(interface::NodeStatus::RUNNING);
          }
          else
          {
            PRINT_MSG("[mj_ros::simulator_t] reset failed!")
          }
        }
      }
      else if (_node_status->requested_status() == interface::NodeStatus::RUNNING)
      {
        PRINT_MSG("[mj_ros::simulator_t] Setting to running")

        _node_status->status(_node_status->requested_status());
        _node_status->request_acknowledged();
      }
      else if (_node_status->requested_status() == interface::NodeStatus::FINISH)
      {
        PRINT_MSG("[mj_ros::simulator_t] Finished, exiting...")
        ros::shutdown();
      }
      else if (_node_status->requested_status() == interface::NodeStatus::PAUSED)
      {
        _node_status->status(_node_status->requested_status());
        _node_status->request_acknowledged();
        return;
      }
      else
      {
        auto invalid_status = _node_status;
        DEBUG_VARS(invalid_status);
      }
    }
    if (_node_status->status() == interface::NodeStatus::RUNNING)
    {
      step_simulation();
    }
  }

  std::string _environment_file;

  prx::param_loader _prx_params;
  prx::param_loader _plant_params;

  prx::space_point_t _x0, _u0;

  std::shared_ptr<prx::world_model_t> _world_model;
  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  // std::shared_ptr<prx::space_t> _state_space, _control_space, _sensor_space;

  ros::Timer _step_timer;

  ros::Publisher _sensor_publisher;
  ros::Publisher _collision_publisher;

  ros::Subscriber _state_subscriber;
  ros::Subscriber _control_subscriber, _control_stamped_subscriber;

  std_msgs::Bool _collision_msg;
  interface::SensorDataStamped _sensor_msg;

  std::shared_ptr<interface::node_status_t> _node_status;
};

}  // namespace analytical
