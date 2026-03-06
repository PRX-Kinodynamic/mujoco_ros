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
    std::string set_state_topic, collision_topic, ctrl_topic, sensor_topic, environment;

    PARAM_SETUP(nh, set_state_topic);
    PARAM_SETUP(nh, collision_topic);
    PARAM_SETUP(nh, simulation_step);
    PARAM_SETUP(nh, ctrl_topic);
    PARAM_SETUP(nh, sensor_topic);
    GLOBAL_PARAM_SETUP(environment);

    // ml4kp_bridge::copy(_prx_params, env_nh);
    _prx_params.from_string(environment);

    prx::param_loader plant_params;
    ml4kp_bridge::copy(plant_params, nh);
    _prx_params["plant"] = plant_params;

    // DEBUG_VARS(_prx_params)

    init_ml4kp(_prx_params);

    _node_status = interface::node_status_t::create(nh);

    _sensor_publisher = nh.advertise<interface::SensorDataStamped>(sensor_topic, 1, true);
    _collision_publisher = nh.advertise<std_msgs::Bool>(collision_topic, 1, true);

    _state_subscriber = nh.subscribe(set_state_topic, 1, &simulator_t::set_state_callback, this);
    _control_subscriber = nh.subscribe(ctrl_topic, 1, &simulator_t::control_callback, this);
    _control_stamped_subscriber =
        nh.subscribe(ctrl_topic + "_stamped", 1, &simulator_t::control_stamped_callback, this);

    _step_timer = nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::step_callback, this);
    _node_status->status(interface::NodeStatus::RUNNING);
  }

  virtual ~simulator_t()
  {
  }

protected:
  void init_ml4kp(prx::param_loader& params)
  {
    const std::string plant_name{ params["/plant/name"].as<std::string>() };
    const std::string plant_path{ params["/plant/path"].as<std::string>() };
    _plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(_plant != nullptr, "Failed to create plant");
    _plant->init(params["plant"]);

    prx::obstacle_loader_t obstacles(params);
    // auto obstacles = prx::obstacle_loader_t(params);

    std::vector<std::string> obstacle_names{ obstacles.get_names() };
    std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.get_obstacles() };

    _world_model.reset(new prx::world_model_t({ _plant }, { obstacle_list }));
    _world_model->create_context("planner_context", { plant_name }, { obstacle_names });
    auto context = _world_model->get_context("planner_context");

    _state_space.reset(context.first->get_state_space());
    _control_space.reset(context.first->get_control_space());
    _sensor_space.reset(context.first->get_sensor_space());
    // auto ps = context.first->get_parameter_space();

    _x0 = _state_space->make_point();
    _u0 = _control_space->make_point();

    _state_space->copy_to(_x0);
    _control_space->copy_to(_u0);

    _system_group = prx::system_group(context);
    _collision_group = prx::collision_group(context);

    _sensor_msg.raw_sensor_data.resize(_sensor_space->size());
  }

  void set_state_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _state_space->copy_from(msg->space_point.point);
  }

  void control_stamped_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _control_space->copy_from(msg->space_point.point);
  }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    _control_space->copy_from(msg->point);
  }

  void step_simulation()
  {
    _collision_msg.data = false;
    // DEBUG_VARS(_system_group != nullptr)
    _system_group->propagate_once();
    _system_group->sense();

    if (_collision_group->in_collision())
    {
      _collision_msg.data = true;
    }

    _sensor_space->copy_to(_sensor_msg.raw_sensor_data);
    _sensor_msg.header.stamp = ros::Time::now();
  }

  void reset_simulation()
  {
    PRINT_MSG("Reseting..")
    _state_space->copy_from(_x0);
    _control_space->copy_from(_u0);
    // _state_space->init(_prx_params["state_space"]);
    // _control_space->init(_prx_params["control_space"]);

    // DEBUG_VARS(*_state_space)
  }

  void step_callback(const ros::TimerEvent& event)
  {
    if (_node_status->new_request())
    {
      _node_status->status(_node_status->requested_status());
      _node_status->request_acknowledged();
    }

    if (_node_status->status() == interface::NodeStatus::RUNNING)
    {
      step_simulation();
    }
    else if (_node_status->status() == interface::NodeStatus::RESET)
    {
      reset_simulation();
      // step_simulation();
      _node_status->status(interface::NodeStatus::RUNNING);
    }
    else if (_node_status->status() == interface::NodeStatus::FINISH)
    {
      PRINT_MSG("[mj_ros::simulator_t] Finished, exiting...")
      ros::shutdown();
    }
    else if (_node_status->status() == interface::NodeStatus::PAUSED)
    {
      return;
    }
    else
    {
      auto invalid_status = _node_status;
      DEBUG_VARS(invalid_status);
    }
    _sensor_publisher.publish(_sensor_msg);
    _collision_publisher.publish(_collision_msg);
  }

  prx::param_loader _prx_params;

  prx::space_point_t _x0, _u0;

  std::shared_ptr<prx::world_model_t> _world_model;
  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<prx::space_t> _state_space, _control_space, _sensor_space;

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
