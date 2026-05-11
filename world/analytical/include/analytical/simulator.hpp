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
#include <prx/simulation/collision_checking/pqp_collision_checker.hpp>
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

template <typename T, typename = void>
struct plant_stepper_t
{
};

template <>
struct plant_stepper_t<prx::system_t>
{
  prx::param_loader _prx_params;
  prx::param_loader _plant_params;

  std::shared_ptr<prx::world_model_t> _world_model;
  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;

  prx::space_point_t _x0, _u0;

  std::string _environment_file;

  plant_stepper_t(ros::NodeHandle& nh)
  {
    ml4kp_bridge::copy(_plant_params, nh);
  }

  void init(interface::SensorDataStamped& sensor_msg)
  {
    _world_model.reset();
    _system_group.reset();
    _collision_group.reset();

    _plant = prx::system_factory_t::create_system(_plant_params);
    prx_assert(_plant != nullptr, "Failed to create plant");

    std::tie(_world_model, _system_group, _collision_group) = prx::world_model_t::create(_prx_params, _plant);

    _x0 = _system_group->get_state_space()->make_point();
    _u0 = _system_group->get_control_space()->make_point();

    _system_group->get_state_space()->copy_to(_x0);
    _system_group->get_control_space()->copy_to(_u0);

    sensor_msg.raw_sensor_data.resize(_system_group->get_sensor_space()->size());
  }

  void step_simulation(std_msgs::Bool& collision_msg, interface::SensorDataStamped& sensor_msg)
  {
    if (not _system_group)
    {
      PRINT_MSG("[simulator_t] not initialize yet... call Reset on node status")
      return;
    }
    collision_msg.data = false;

    _system_group->propagate_once();

    _system_group->sense();

    if (_collision_group->in_collision())
    {
      collision_msg.data = true;
    }

    _system_group->get_sensor_space()->copy_to(sensor_msg.raw_sensor_data);
    sensor_msg.header.stamp = ros::Time::now();
  }

  bool reset_simulation(interface::SensorDataStamped& sensor_msg)
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
      init(sensor_msg);
    }
    if (_environment_file.size() > 0)
    {
      _system_group->get_state_space()->copy_from(_x0);
      _system_group->get_control_space()->copy_from(_u0);
      return true;
    }
    return false;
  }

  void state(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _system_group->get_state_space()->copy_from(msg->space_point.point);
  }

  void control(const ml4kp_bridge::SpacePoint& msg)
  {
    _system_group->get_control_space()->copy_from(msg.point);
  }
};

// template <typename PlantType>
// template <template <typename> class PlantType>
// template <template <typename> class DynamicalSystemType, typename PlantType>
// struct plant_stepper_t<DynamicalSystemType<PlantType>,
//                        std::enable_if_t<std::is_base_of<PlantType, prx::dynamical_system_t<PlantType>>::value>>
template <>
struct plant_stepper_t<prx::SO2_system_t>
{
  // using PlantType = prx::SO2_system_t;
  using DynamicalSystem = typename prx::SO2_system_t;
  using DynamicalSystemPtr = typename std::shared_ptr<DynamicalSystem>;
  using DynamicalSystemTraits = prx::dynamical_system_traits<DynamicalSystem>;

  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Parameters = typename DynamicalSystem::Parameters;
  using Observation = typename DynamicalSystem::Observation;

  using CollisionChecker = typename prx::collision_checking::pqp::system_checker_t<DynamicalSystem>;
  prx::param_loader _environment_params;
  prx::param_loader _plant_params;
  prx::param_loader _problem_params;

  DynamicalSystemPtr _plant;
  State _x;
  Control _u;
  Observation _z;

  std::string _environment_file, _problem_parameters;
  std::shared_ptr<CollisionChecker> _collision_checker;
  std::vector<std::shared_ptr<prx::collision_checking::pqp::rigid_body_t>> _obstacles_bodies;

  plant_stepper_t(ros::NodeHandle& nh)
  {
    // ml4kp_bridge::copy(_plant_params, nh);
  }

  void init(interface::SensorDataStamped& sensor_msg)
  {
    std::string plant_parameters;
    GLOBAL_PARAM_SETUP(plant_parameters);
    _plant_params.from_string(plant_parameters);

    _plant = std::make_shared<DynamicalSystem>(_plant_params);
    prx_assert(_plant != nullptr, "Failed to create plant");

    // _obstacles_bodies = prx::collision_checking::pqp::create_obstacles(_environment_params);

    sensor_msg.raw_sensor_data.resize(DynamicalSystemTraits::ObservationDimension);
  }

  void step_simulation(std_msgs::Bool& collision_msg, interface::SensorDataStamped& sensor_msg)
  {
    collision_msg.data = false;

    if (_collision_checker->collision(_x))
    {
      collision_msg.data = true;
    }

    _x = _plant->propagate(_x, _u, prx::simulation_step);

    // DEBUG_VARS(_x)
    sensor_msg.raw_sensor_data[0] = _x.first.theta();
    sensor_msg.raw_sensor_data[1] = _x.second;
    // DEBUG_VARS(sensor_msg.raw_sensor_data)
    // ml4kp_bridge::copy(sensor_msg, _z);
    // _system_group->get_sensor_space()->copy_to(sensor_msg.raw_sensor_data);

    sensor_msg.header.stamp = ros::Time::now();
  }

  bool reset_simulation(interface::SensorDataStamped& sensor_msg)
  {
    PRINT_MSG("Reseting..")
    // ros::Duration(1.0).sleep();

    // std::string environment, problem_parameters;
    std::string& environment{ _environment_file };
    std::string& problem_parameters{ _problem_parameters };

    GLOBAL_PARAM_SETUP(environment);
    GLOBAL_PARAM_SETUP(problem_parameters);
    // if (ros::param::has("/environment") and ros::param::get("/environment", environment))
    // DEBUG_VARS(environment)
    // if (environment != _environment_file)
    // {
    // }
    if (_environment_file.size() > 0)
    {
      _environment_params.from_string(environment);
      // _environment_file = environment;
      init(sensor_msg);
      DEBUG_VARS(_environment_params)
      _collision_checker = std::make_shared<CollisionChecker>(_plant, _environment_params);
      _problem_params.from_string(problem_parameters);
      _x = _problem_params["x0"].as<State>();
      _u = _problem_params["u0"].as<Control>();
      // DEBUG_VARS(_x, _u)
      // _system_group->get_state_space()->copy_from(_x0);
      // _system_group->get_control_space()->copy_from(_u0);
      return true;
    }
    return false;
  }

  void state(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    ml4kp_bridge::copy(_x, msg);
    // _system_group->get_state_space()->copy_from(msg->space_point.point);
  }

  void control(const ml4kp_bridge::SpacePoint& msg)
  {
    ml4kp_bridge::copy(_u, msg);
    // DEBUG_VARS(_u)
    // _system_group->get_control_space()->copy_from(msg.point);
  }
};

template <typename SystemType>
class simulator_t
{
public:
  // Replicate MJ interface:
  // In: controls
  // Out: Sensors
  simulator_t(ros::NodeHandle& nh) : _stepper(nh)
  {
    double& simulation_step{ prx::simulation_step };
    std::string set_state_topic, collision_topic, ctrl_topic, sensor_topic;

    PARAM_SETUP(nh, set_state_topic);
    PARAM_SETUP(nh, collision_topic);
    PARAM_SETUP(nh, simulation_step);
    PARAM_SETUP(nh, ctrl_topic);
    PARAM_SETUP(nh, sensor_topic);
    // GLOBAL_PARAM_SETUP(environment);

    _stepper.init(_sensor_msg);
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
  void set_state_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _stepper.state(msg);
  }

  void control_stamped_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _stepper.control(msg->space_point);
  }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    _stepper.control(*msg);
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
          if (_stepper.reset_simulation(_sensor_msg))
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
      _stepper.step_simulation(_collision_msg, _sensor_msg);
      _sensor_publisher.publish(_sensor_msg);
      _collision_publisher.publish(_collision_msg);
    }
  }

  // std::shared_ptr<prx::space_t> _state_space, _control_space, _sensor_space;

  ros::Timer _step_timer;

  ros::Publisher _sensor_publisher;
  ros::Publisher _collision_publisher;

  ros::Subscriber _state_subscriber;
  ros::Subscriber _control_subscriber, _control_stamped_subscriber;

  interface::SensorDataStamped _sensor_msg;

  plant_stepper_t<SystemType> _stepper;

  std_msgs::Bool _collision_msg;

  std::shared_ptr<interface::node_status_t> _node_status;
};

}  // namespace analytical
