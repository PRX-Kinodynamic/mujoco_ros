#include <stdio.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <analytical/fg_ltv_sde.hpp>

namespace analytical
{
template <typename Base>
class simulator_t : public Base
{
public:
  simulator_t() : _set_state_topic_name("/ml4kp/simulator/set_state"){};

  virtual ~simulator_t()
  {
  }

protected:
  void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    prx_assert(private_nh.ok(), "Simulator's node handle not initialized!");

    std::string plant_ml4kp_params{ "" };
    std::string state_topic{ "" };
    std::string control_topic{ "" };
    std::string environment{ "" };
    std::string robot_frame{ "robot" };
    double& simulation_step{ prx::simulation_step };
    std::vector<double> tf_noise_sigmas{ { 0, 0, 0 } };

    PARAM_SETUP(private_nh, plant_ml4kp_params);
    PARAM_SETUP(private_nh, state_topic);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, environment);
    PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    PARAM_SETUP_WITH_DEFAULT(private_nh, robot_frame, robot_frame);
    PARAM_SETUP_WITH_DEFAULT(private_nh, tf_noise_sigmas, tf_noise_sigmas);

    DEBUG_VARS(prx::simulation_step);

    _tf_sigmas = tf_noise_sigmas;
    // _params.set_input_path("/");
    // _params.add_file(plant_file);
    prx::param_loader plant_params(plant_ml4kp_params, "");
    ml4kp_bridge::check_for_ros_params(plant_params, private_nh);
    _params["plant"] = plant_params;

    auto obstacles = prx::load_obstacles(environment);
    _obstacle_list = obstacles.second;
    _obstacle_names = obstacles.first;
    _plant_name = plant_params["name"].as<>();
    _plant = prx::system_factory_t::create_system(_plant_name, _plant_name);
    prx_assert(_plant != nullptr, "Plant is nullptr!");

    // DEBUG_VARS(_plant_name, _obstacle_names.size())
    _world_model.reset(new prx::world_model_t({ _plant }, { _obstacle_list }));
    _world_model->create_context("sim_context", { _plant_name }, { _obstacle_names });
    auto context = _world_model->get_context("sim_context");
    _system_group = context.first;
    _collision_group = context.second;
    _state_space.reset(_system_group->get_state_space());
    _control_space.reset(_system_group->get_control_space());

    _plant->init(plant_params);
    std::cout << "Plant: " << (*_plant) << std::endl;

    _start_state = _state_space->make_point();
    _state_space->copy(_start_state, _params["/plant/start_state"].as<std::vector<double>>());

    DEBUG_VARS(*_start_state);

    _state_msg.header.seq = 0;
    _state_msg.header.stamp = ros::Time::now();
    _state_msg.header.frame_id = "world";

    _state_msg.space_point.point.resize(_state_space->size());

    _tf_gt.header.frame_id = "world";
    _tf_noise.header.frame_id = "world";

    _tf_gt.child_frame_id = robot_frame + "_gt";
    _tf_noise.child_frame_id = robot_frame;

    const std::string stamped_control_topic{ control_topic + "_stamped" };
    // TODO: handle non existence of params
    _stepper_timer = private_nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::step, this);
    _state_timer = private_nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::publish_state, this);
    _control_subscriber = private_nh.subscribe(control_topic, 1, &simulator_t::control_callback, this);
    _control_stamped_subscriber =
        private_nh.subscribe(stamped_control_topic, 1, &simulator_t::stamped_control_callback, this);
    _state_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(state_topic, 10, true);

    _set_state_subscriber = private_nh.subscribe(_set_state_topic_name, 1, &simulator_t::set_state_callback, this);

    _prev_time = ros::Time::now();
    _step_prev_time = ros::Time::now();
  }

  void set_state_callback(const ml4kp_bridge::SpacePointConstPtr message)
  {
    if (message->point.size() == _state_space->size())
    {
      _state_space->copy_from(message->point);
    }
    else if (message->point.size() == 0)
    {
      _state_space->copy_from(_start_state);
    }
    const std::string msg{ "New state" };
    DEBUG_VARS(msg, *_state_space);
  }

  inline void control_callback(const ml4kp_bridge::SpacePointConstPtr message)
  {
    _control_space->copy_from(message->point);
  }

  inline void stamped_control_callback(const ml4kp_bridge::SpacePointStampedConstPtr message)
  {
    _control_space->copy_from(message->space_point.point);
  }

  void add_tf_noise(geometry_msgs::Transform& tf) const
  {
    // Only adding translation noise for now
    tf.translation.x += prx::gaussian_random(0.0, _tf_sigmas[0]);
    tf.translation.y += prx::gaussian_random(0.0, _tf_sigmas[1]);
    tf.translation.z += prx::gaussian_random(0.0, _tf_sigmas[2]);
  }

  void publish_state(const ros::TimerEvent& event)
  {
    if (_prev_time == ros::Time::now())
      return;
    _state_msg.header.stamp = ros::Time::now();
    _state_space->copy_to(_state_msg.space_point.point);
    _state_publisher.publish(_state_msg);

    if (_plant_name == "fg_ltv_sde")
    {
      // _plant_name
      // DEBUG_VARS(ros::Time::now());
      _tf_gt.header.stamp = ros::Time::now();
      _tf_noise.header.stamp = ros::Time::now();
      _tf_gt.header.seq++;
      _tf_noise.header.seq++;
      prx::fg::ltv_sde_utils_t::copy(_tf_gt.transform, _state_msg.space_point.point);
      prx::fg::ltv_sde_utils_t::copy(_tf_noise.transform, _state_msg.space_point.point);
      add_tf_noise(_tf_noise.transform);

      _tf_broadcaster.sendTransform(_tf_gt);
      _tf_broadcaster.sendTransform(_tf_noise);
    }
    _prev_time = ros::Time::now();
  }

  void step(const ros::TimerEvent& event)
  {
    if (_step_prev_time == ros::Time::now())
      return;
    _system_group->propagate_once();
    _step_prev_time = ros::Time::now();

    // const double t(event.current_real.toSec());
    // DEBUG_VARS(t);
    // DEBUG_VARS(*_state_space);
    // DEBUG_VARS(*_control_space);
    // DEBUG_VARS(_state_space->print_memory(4));
  }

  ros::Time _prev_time;
  ros::Time _step_prev_time;
  // tf
  tf2_ros::TransformBroadcaster _tf_broadcaster;
  geometry_msgs::TransformStamped _tf_gt;
  geometry_msgs::TransformStamped _tf_noise;
  std::vector<double> _tf_sigmas;

  ros::Timer _state_timer;
  ros::Timer _stepper_timer;

  ros::Subscriber _control_subscriber;
  ros::Subscriber _control_stamped_subscriber;
  ros::Subscriber _set_state_subscriber;

  ros::Publisher _state_publisher;

  ml4kp_bridge::SpacePointStamped _state_msg;

  std::string _set_state_topic_name;
  // PRX vars
  std::string _plant_name;

  prx::param_loader _params;
  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<prx::space_t> _state_space, _control_space;
  std::shared_ptr<prx::world_model_t> _world_model;
  std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;
  std::vector<std::string> _obstacle_names;

  prx::space_point_t _start_state;
};
using SimulatorNodelet = simulator_t<nodelet::Nodelet>;

}  // namespace analytical
PLUGINLIB_EXPORT_CLASS(analytical::SimulatorNodelet, nodelet::Nodelet);
