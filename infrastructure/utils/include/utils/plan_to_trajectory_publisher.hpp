#include <unordered_set>
#include <ml4kp_bridge/Trajectory.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
namespace utils
{

template <class Base>
class plan_to_trajectory_publisher_t : public Base
{
  using Derived = plan_to_trajectory_publisher_t<Base>;

public:
  plan_to_trajectory_publisher_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string plan_topic_name{};
    std::string traj_topic_name{};
    std::string plant_ml4kp_params{};
    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;

    PARAM_SETUP(private_nh, plan_topic_name);
    PARAM_SETUP(private_nh, traj_topic_name);
    PARAM_SETUP(private_nh, plant_ml4kp_params);

    _plant_params = prx::param_loader(plant_ml4kp_params, "");

    const std::string plant_name{ _plant_params["name"].as<>() };
    _plant = prx::system_factory_t::create_system(plant_name, plant_name);
    prx_assert(_plant != nullptr, "Plant is nullptr!");
    _plant->init(_plant_params);
    // DEBUG_VARS("Plan to traj", _plant)
    prx::simulation_step = 0.001;
    _world_model.reset(new prx::world_model_t({ _plant }, {}));
    _world_model->create_context("sim_context", { plant_name }, {});
    auto context = _world_model->get_context("sim_context");
    _system_group = context.first;

    prx::space_t* ss{ _system_group->get_state_space() };
    _start_state = ss->make_point();
    _trajectory = std::make_shared<prx::trajectory_t>(ss);
    _plan = std::make_shared<prx::plan_t>(_system_group->get_control_space());
    // subscribers
    _plan_subscriber = private_nh.subscribe(plan_topic_name, 1, &Derived::plan_callback, this);
    ss->copy(_start_state, _plant_params["start_state"].as<std::vector<double>>());

    // publishers
    _traj_publisher = private_nh.advertise<ml4kp_bridge::Trajectory>(traj_topic_name, 1, true);
  }

protected:
  // template <typename Graph>
  void plan_callback(const ml4kp_bridge::PlanStampedConstPtr msg)
  {
    _plan->clear();
    ml4kp_bridge::copy(_plan, msg);
    DEBUG_VARS(_plan);
    _system_group->propagate(_start_state, *_plan, *_trajectory);
    _traj_msg.data.clear();
    ml4kp_bridge::copy(_traj_msg, _trajectory);
    _traj_publisher.publish(_traj_msg);
  }

  // Topic names
  std::string _viz_traj_topic_name;

  // Subscribers
  ros::Subscriber _plan_subscriber;

  // Publishers
  ros::Publisher _traj_publisher;

  // ML4KP-ROS
  ml4kp_bridge::Trajectory _traj_msg;

  // ML4KP
  prx::param_loader _plant_params;
  prx::system_ptr_t _plant;
  prx::space_point_t _start_state;
  std::shared_ptr<prx::world_model_t> _world_model;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::plan_t> _plan;
  std::shared_ptr<prx::trajectory_t> _trajectory;
};
}  // namespace utils