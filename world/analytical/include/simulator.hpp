#include <stdio.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.h>
#include <interface/StampedMarkers.h>
#include <aruco/aruco_nano.h>
#include <utils/rosparams_utils.hpp>

namespace analytical
{
template <typename Base>
class simulator_t : public Base
{
public:
  simulator_t(){};

protected:
  void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };

    std::string param_file{ "" };
    double;
    PARAM_SETUP(private_nh, param_file);
    PARAM_SETUP(private_nh, param_file);

    _params.add_file(param_file);

    auto obstacles = load_obstacles(params["environment"].as<>());
    _obstacle_list = obstacles.second;
    _obstacle_names = obstacles.first;
    const std::string plant_name{ params["plant_name"].as<>() };
    _plant = prx::system_factory_t::create_system(plant_name);
    prx_assert(_plant != nullptr, "Plant is nullptr!");

    _world_model.reset(new prx::world_model_t({ _plant }, { _obstacle_list }));
    _world_model->create_context("sim_context", { _plant_name }, { _obstacle_names });
    auto context = _world_model->get_context("sim_context");
    _system_group = context.first;
    _collision_group = context.second;
    _state_space.reset(_system_group->get_state_space());
    _control_space.reset(_system_group->get_control_space());

    const std::vector<double> min_space_limits{ params["plant/state_space/lower_bound"].as<std::vector<double>>() };
    const std::vector<double> max_space_limits{ params["plant/state_space/upper_bound"].as<std::vector<double>>() };
    _state_space->set_bounds(min_space_limits, max_space_limits);

    const std::vector<double> min_control_limits{ params["plant/control_space/lower_bound"].as<std::vector<double>>() };
    const std::vector<double> max_control_limits{ params["plant/control_space/upper_bound"].as<std::vector<double>>() };
    _control_space->set_bounds(min_control_limits, max_control_limits);

    // TODO: handle non existance of params
    _stepper_timer = private_nh.createTimer(ros::Duration(simulation_step), &simulator_t::step, this);
    _state_timer = private_nh.createTimer(ros::Duration(simulation_step), &simulator_t::step, this);
    _control_subscriber = private_nh.subscribe(plan_topic, 1, &simulator_t::control_callback, this);
  }

  void get_plan(const ml4kp_bridge::SpacePointConstPtr message)
  {
    // _control_space->copy_from(*message);
  }

  void publish_state(const ros::TimerEvent& event)
  {
  }

  void step(const ros::TimerEvent& event)
  {
    _system_group->propagate_once();
  }

  ros::Timer _stepper_timer;
  ros::Timer _state_timer;

  // PRX vars
  prx::param_loader _params;
  std::shared_ptr<prx::system_t> _plant;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<prx::space_t> _state_space, _control_space;
  std::shared_ptr<prx::world_model_t> _world_model;
  std::vector<std::shared_ptr<movable_object_t>> _obstacle_list;
  std::vector<std::string> _obstacle_names;
};
}  // namespace analytical
