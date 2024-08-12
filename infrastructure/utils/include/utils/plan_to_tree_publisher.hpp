#include <unordered_set>
#include <ml4kp_bridge/Trajectory.h>
#include <visualization_msgs/Marker.h>

#include <utils/rosparams_utils.hpp>
#include <prx_models/Tree.h>

namespace utils
{

template <class Base>
class plan_to_tree_publisher_t : public Base
{
  using Derived = plan_to_tree_publisher_t<Base>;

public:
  plan_to_tree_publisher_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string plan_topic_name{};
    std::string tree_topic_name{};
    std::string plant_ml4kp_params{};

    double& edge_duration{ _edge_duration };
    // _tree_topic_name = ros::this_node::getNamespace() + _tree_topic_name;

    PARAM_SETUP(private_nh, plan_topic_name);
    PARAM_SETUP(private_nh, tree_topic_name);
    PARAM_SETUP(private_nh, plant_ml4kp_params);
    PARAM_SETUP(private_nh, edge_duration);

    DEBUG_VARS(plan_topic_name);
    DEBUG_VARS(tree_topic_name);
    DEBUG_VARS(plant_ml4kp_params);
    DEBUG_VARS(edge_duration);

    _plant_params = prx::param_loader(plant_ml4kp_params, "");

    const std::string plant_name{ _plant_params["name"].as<>() };
    _plant = prx::system_factory_t::create_system(plant_name, plant_name);
    prx_assert(_plant != nullptr, "Plant is nullptr!");
    _plant->init(_plant_params);
    // DEBUG_VARS("Plan to traj", _plant)
    prx::simulation_step = _plant_params["simulation_step"].as<double>();
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
    _tree_publisher = private_nh.advertise<prx_models::Tree>(tree_topic_name, 1, true);
  }

protected:
  void add_node_edge(prx_models::Node& parent, const double edge_duration, const double ti)
  {
    // DEBUG_VARS(parent);

    prx_models::Node node;
    prx_models::Edge edge;

    edge.index = parent.index;

    node.index = parent.index + 1;
    node.parent = parent.index;
    node.parent_edge = edge.index;
    ml4kp_bridge::copy(node.point, _trajectory->at(ti, false));
    node.cost = parent.cost + edge_duration;
    // DEBUG_VARS(node);

    edge.source = parent.index;
    edge.target = node.index;

    parent.children.push_back(node.index);
    _tree.nodes.push_back(parent);
    parent = node;

    ml4kp_bridge::copy(edge.plan, _plan);
    edge.cost = edge_duration;
    // DEBUG_VARS(edge);
    _tree.edges.push_back(edge);
    DEBUG_PRINT;
  }

  // template <typename Graph>
  void plan_callback(const ml4kp_bridge::PlanStampedConstPtr msg)
  {
    _plan->clear();

    ml4kp_bridge::copy(_plan, msg);
    // DEBUG_VARS(_plan);
    _system_group->propagate(_start_state, *_plan, *_trajectory);
    // DEBUG_VARS(*_trajectory);

    _tree.root = 0;

    prx_models::Node parent;
    prx_models::Edge edge_parent;

    parent.index = 0;
    parent.parent = 0;
    parent.parent_edge = 0;
    ml4kp_bridge::copy(parent.point, _start_state);
    parent.cost = 0;

    const double traj_duration{ _plan->duration() };
    // DEBUG_VARS(traj_duration);

    double t_accum{ 0.0 };

    // for (double i = _edge_duration; i < traj_duration; i += _edge_duration)
    for (auto step : *_plan)
    {
      t_accum += step.duration;
      add_node_edge(parent, step.duration, t_accum);
    }
    _tree.nodes.push_back(parent);

    // DEBUG_VARS(_tree);
    // if (_plan->duration() > 0)
    // {
    //   add_node_edge(parent, _plan->duration(), traj_duration);
    // }

    _tree_publisher.publish(_tree);
  }

  double _edge_duration;

  // Subscribers
  ros::Subscriber _plan_subscriber;

  // Publishers
  ros::Publisher _tree_publisher;

  // ML4KP-ROS
  prx_models::Tree _tree;

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