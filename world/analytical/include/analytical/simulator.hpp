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
  simulator_t(){};

  virtual ~simulator_t()
  {
  }

protected:
  void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    prx_assert(private_nh.ok(), "Simulator's node handle not initialized!");

    std::string plant_file{ "" };
    std::string state_topic{ "" };
    std::string control_topic{ "" };
    std::string environment{ "" };
    std::string robot_frame{ "robot" };
    double& simulation_step{ prx::simulation_step };
    std::vector<double> tf_noise_sigmas{ { 0, 0, 0 } };

    PARAM_SETUP(private_nh, plant_file);
    PARAM_SETUP(private_nh, state_topic);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, environment);
    PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    PARAM_SETUP_WITH_DEFAULT(private_nh, robot_frame, robot_frame);
    PARAM_SETUP_WITH_DEFAULT(private_nh, tf_noise_sigmas, tf_noise_sigmas);

    _tf_sigmas = tf_noise_sigmas;
    _params.set_input_path("/");
    _params.add_file(plant_file);
    prx::param_loader plant_params{ _params["plant"] };

    auto obstacles = prx::load_obstacles(environment);
    _obstacle_list = obstacles.second;
    _obstacle_names = obstacles.first;
    _plant_name = plant_params["name"].as<>();
    _plant = prx::system_factory_t::create_system(_plant_name, _plant_name);
    prx_assert(_plant != nullptr, "Plant is nullptr!");

    DEBUG_VARS(_plant_name, _obstacle_names.size())
    _world_model.reset(new prx::world_model_t({ _plant }, { _obstacle_list }));
    _world_model->create_context("sim_context", { _plant_name }, { _obstacle_names });
    auto context = _world_model->get_context("sim_context");
    _system_group = context.first;
    _collision_group = context.second;
    _state_space.reset(_system_group->get_state_space());
    _control_space.reset(_system_group->get_control_space());

    _plant->init(plant_params);
    // using DoubleVector = std::vector<double>;
    // const DoubleVector min_space_limits{ plant_params["state_space/lower_bound"].as<DoubleVector>() };
    // const DoubleVector max_space_limits{ plant_params["state_space/upper_bound"].as<DoubleVector>() };
    // _state_space->set_bounds(min_space_limits, max_space_limits);

    // const DoubleVector min_control_limits{ plant_params["control_space/lower_bound"].as<DoubleVector>() };
    // const DoubleVector max_control_limits{ plant_params["control_space/upper_bound"].as<DoubleVector>() };
    // _control_space->set_bounds(min_control_limits, max_control_limits);

    _state_msg.header.seq = 0;
    _state_msg.header.stamp = ros::Time::now();
    _state_msg.header.frame_id = "world";

    _state_msg.space_point.point.resize(_state_space->size());

    _tf_gt.header.frame_id = "world";
    _tf_noise.header.frame_id = "world";

    _tf_gt.child_frame_id = robot_frame + "_gt";
    _tf_noise.child_frame_id = robot_frame;

    // TODO: handle non existence of params
    _stepper_timer = private_nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::step, this);
    _state_timer = private_nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::publish_state, this);
    _control_subscriber = private_nh.subscribe(control_topic, 1, &simulator_t::control_callback, this);
    _state_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(state_topic, 10, true);
  }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr message)
  {
    _control_space->copy_from(message->point);
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
    _state_msg.header.stamp = ros::Time::now();
    _state_space->copy_to(_state_msg.space_point.point);
    _state_publisher.publish(_state_msg);

    if (_plant_name == "fg_ltv_sde")
    {
      // _plant_name
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
  }

  void step(const ros::TimerEvent& event)
  {
    _system_group->propagate_once();
    DEBUG_VARS(_state_space->print_memory(4));
  }

  // tf
  tf2_ros::TransformBroadcaster _tf_broadcaster;
  geometry_msgs::TransformStamped _tf_gt;
  geometry_msgs::TransformStamped _tf_noise;
  std::vector<double> _tf_sigmas;

  ros::Timer _state_timer;
  ros::Timer _stepper_timer;

  ros::Subscriber _control_subscriber;

  ros::Publisher _state_publisher;

  ml4kp_bridge::SpacePointStamped _state_msg;

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
};
using SimulatorNodelet = simulator_t<nodelet::Nodelet>;

}  // namespace analytical
PLUGINLIB_EXPORT_CLASS(analytical::SimulatorNodelet, nodelet::Nodelet);
