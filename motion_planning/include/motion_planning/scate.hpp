#include <ml4kp_bridge/defs.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_planning/StelaGraphTraversalAction.h>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/factor_graphs/factors/obstacle_factor.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>

namespace motion_planning
{

// Assuming the system can be (roughly) divided into X, Xdot, Xddot...
template <typename SystemInterface, typename Base>
class scate_t : public Base
{
  using Derived = scate_t<SystemInterface, Base>;

  using Control = typename SystemInterface::Control;
  using State = typename SystemInterface::State;
  using StateDot = typename SystemInterface::StateDot;
  using Observation = typename SystemInterface::Observation;
  using ObstacleFactor = prx::fg::obstacle_factor_t<State, typename SystemInterface::ConfigFromState>;
  using SF = prx::fg::symbol_factory_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  scate_t()
    : _isam_params(gtsam::ISAM2GaussNewtonParams(), 0.1, 10, true, true, gtsam::ISAM2Params::CHOLESKY, true,
                   prx::fg::symbol_factory_t::formatter, true)
    , _tf_listener(_tf_buffer)
    , _isam(_isam_params)
    , _isam_initialized(false)
    , _current_node(0)
    , _fix_sigmas(1.0)
    , _obstacle_activation_distance(1.0)
    , _lm_params(prx::fg::default_levenberg_marquardt_parameters()) {};

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string tree_topic_name;
    std::string graph_topic_name{ "" };
    std::string control_topic;
    double control_frequency;
    double& obstacle_activation_distance{ _obstacle_activation_distance };
    double& fix_sigmas{ _fix_sigmas };
    double& obstacle_sigma{ _obstacle_sigma };
    std::string environment, solution_tree_topic;
    bool time_factor{ _time_factor };
    bool naive_guess{ false };
    int fg_iterations{ 100 };
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string& obstacle_mode{ _obstacle_mode };

    PARAM_SETUP(private_nh, solution_tree_topic);
    PARAM_SETUP(private_nh, control_topic);
    PARAM_SETUP(private_nh, control_frequency);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);
    PARAM_SETUP(private_nh, obstacle_sigma)
    PARAM_SETUP(private_nh, obstacle_activation_distance)
    PARAM_SETUP(private_nh, environment);
    PARAM_SETUP(private_nh, time_factor);
    PARAM_SETUP_WITH_DEFAULT(private_nh, fg_iterations, fg_iterations)
    PARAM_SETUP_WITH_DEFAULT(private_nh, naive_guess, naive_guess);
    PARAM_SETUP_WITH_DEFAULT(private_nh, fix_sigmas, fix_sigmas);

    _lm_params.setUseFixedLambdaFactor(true);
    _lm_params.setMaxIterations(fg_iterations);
    _obstacle_noise = gtsam::noiseModel::Isotropic::Sigma(1, obstacle_sigma);

    // PARAM_SETUP_WITH_DEFAULT(private_nh, simulation_step, 0.01);
    const std::string stamped_control_topic{ control_topic + "_stamped" };

    // _ol_timer = private_nh.createTimer(control_timer, &Derived::control_timer_callback, this);

    _control_publisher = private_nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = private_nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);

    // const std::string scate_tree_topic_name{ ros::this_node::getNamespace() + "/scate/tree" };
    _tree_publisher = private_nh.advertise<prx_models::Tree>(solution_tree_topic, 1, true);

    _control_stamped.header.seq = 0;
    _control_stamped.header.stamp = ros::Time::now();
    _control_stamped.header.frame_id = "ScateControl";

    _prev_header.stamp = ros::Time::now();

    auto obstacles = prx::load_obstacles(environment);
    _obstacle_list = obstacles.second;
    DEBUG_VARS(environment, _obstacle_list.size());

    const std::string plant_name{ SystemInterface::plant_name };
    _plant = prx::system_factory_t::create_system(plant_name, plant_name);
    _robot_collision_ptr = SystemInterface::collision_geometry();
    _obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

    if (naive_guess)
    {
      int& total_states{ _total_states };
      std::vector<double>& init_ctrl{ _init_ctrl };
      double& init_duration{ _init_duration };
      utils::get_param_and_check(private_nh, "/Plant/start_state", _start_state);
      utils::get_param_and_check(private_nh, "/Plant/goal/state", _goal_state);
      PARAM_SETUP(private_nh, total_states);
      PARAM_SETUP(private_nh, init_ctrl);
      PARAM_SETUP(private_nh, init_duration);
    }

    const ros::Duration control_timer(1.0 / control_frequency);
    _control_timer = private_nh.createTimer(control_timer, &Derived::action_function, this);
  }

  void action_function(const ros::TimerEvent& event)
  {
    if (_isam_initialized)
    {
      publish_tree();
    }
    else
    {
      init_from_naive_guess();
      _isam_initialized = true;
    }
  }

  bool query_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  void publish_control()
  {
  }

  void obstacle_factors(const ml4kp_bridge::SpacePoint& point, const int x_id)
  {
    const gtsam::Key keyX{ SystemInterface::keyX(1, x_id) };
    // SystemInterface::state(_state, point);
    DEBUG_PRINT;
    for (auto obstacle_info : _obstacle_collision_infos)
    {
      DEBUG_PRINT;
      _obstacle_graph.emplace_shared<ObstacleFactor>(obstacle_info, _robot_collision_ptr, keyX,
                                                     _obstacle_activation_distance, 0.1, _obstacle_noise);
    }
    DEBUG_PRINT;
  }

  void init_from_naive_guess()
  {
    prx::space_t* ss{ _plant->get_state_space() };
    ml4kp_bridge::SpacePoint state{};

    ml4kp_bridge::Plan plan;
    plan.steps.emplace_back();
    plan.steps[0].control.point = _init_ctrl;
    plan.steps[0].duration.data = ros::Duration(_init_duration);

    state.point = _start_state;
    GraphValues root_graph_values{ SystemInterface::fix_cost(0, state, _fix_sigmas) };
    obstacle_factors(state, 0);

    for (int i = 1; i <= _total_states; ++i)
    {
      const gtsam::Key xkey{ SystemInterface::keyX(1, i) };
      const gtsam::Key xdotkey{ SystemInterface::keyXdot(1, i) };

      const double t{ i / static_cast<double>(_total_states) };
      ss->interpolate(_start_state, _goal_state, t, state.point);
      GraphValues graph_values{ SystemInterface::scate_fg(i - 1, i, state, plan, _time_factor) };

      root_graph_values.first += graph_values.first;
      root_graph_values.second.insert(graph_values.second);
      obstacle_factors(state, i);
    }

    ss->copy(state.point, _goal_state);
    DEBUG_VARS(_total_states);
    GraphValues graph_values{ SystemInterface::fix_cost(_total_states, state, _fix_sigmas) };
    root_graph_values.first += graph_values.first;
    root_graph_values.first += _obstacle_graph;

    _factor_graph = root_graph_values.first;
    _current_estimate = root_graph_values.second;

    SF::symbols_to_file("/Users/Gary/pracsys/catkin_ws/symbols.txt");
    DEBUG_PRINT;
    gtsam::LevenbergMarquardtOptimizer optimizer(_factor_graph, _current_estimate, _lm_params);

    _current_estimate = optimizer.optimize();
  }

  void publish_tree()
  {
    _tree_manager.reset();
    _tree.nodes.clear();
    _tree.edges.clear();
    _tree.root = 0;

    for (int i = _current_node; i < _total_states; ++i)
    {
      const gtsam::Key xkey{ SystemInterface::keyX(1, i) };
      const gtsam::Key xdotkey{ SystemInterface::keyXdot(1, i) };
      const State xi{ _current_estimate.at<State>(xkey) };
      const State xdoti{ _current_estimate.at<StateDot>(xdotkey) };

      if (i > _current_node)
      {
        prx_models::Node parent_node{ _tree.nodes.back() };
        _tree.nodes.push_back(std::move(_tree_manager.create_node()));
        prx_models::Edge edge{ _tree_manager.create_edge(parent_node, _tree.nodes.back()) };
        _tree.edges.push_back(edge);
      }
      else
      {
        _tree.nodes.push_back(std::move(_tree_manager.create_node()));
      }

      SystemInterface::copy(_tree.nodes.back().point, xi, xdoti);
    }

    _tree_publisher.publish(_tree);
  }

private:
  Values _values;
  FactorGraph _factor_graph, _obstacle_graph;

  // gtsam
  gtsam::ISAM2Params _isam_params;
  gtsam::ISAM2 _isam;
  gtsam::ISAM2Result _isam2_result;

  ml4kp_bridge::SpacePointStamped _control_stamped;
  prx_models::Tree _tree;

  ros::Publisher _control_publisher;
  ros::Publisher _stamped_control_publisher;
  ros::Publisher _tree_publisher;

  ros::Timer _control_timer;

  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
  std_msgs::Header _prev_header;

  bool _isam_initialized;
  motion_planning::tree_manager_t _tree_manager;
  int _current_node;

  std::shared_ptr<prx::fg::collision_info_t> _robot_collision_ptr;
  std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;
  std::vector<std::shared_ptr<prx::fg::collision_info_t>> _obstacle_collision_infos;

  double _obstacle_sigma;
  double _obstacle_activation_distance;
  gtsam::Values _current_estimate;
  gtsam::LevenbergMarquardtParams _lm_params;

  int _total_states;
  std::vector<double> _start_state;
  std::vector<double> _goal_state;
  std::vector<double> _init_ctrl;
  double _init_duration;
  bool _time_factor;
  double _fix_sigmas;

  std::shared_ptr<prx::system_t> _plant;

  std::string _obstacle_mode;
  gtsam::noiseModel::Base::shared_ptr _obstacle_noise;
};
}  // namespace motion_planning