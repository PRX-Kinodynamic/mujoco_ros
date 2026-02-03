#pragma once
#include <ros/ros.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/lie_ode_observation.hpp>

template <typename DerivedRobotInterface, typename Types>
class stela_robot_interface_t
{
  using This = stela_robot_interface_t<DerivedRobotInterface, Types>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using State = typename Types::State;
  using StateDot = typename Types::StateDot;
  // using StateDotDot = typename Types::StateDotDot;

  using Control = typename Types::Control;
  using Observation = typename Types::Observation;

  using StateKeys = typename Types::StateKeys;
  using ControlKeys = typename Types::ControlKeys;
  using TimeKeys = typename Types::TimeKeys;

  using StateEstimates = typename Types::StateEstimates;
  using ControlEstimates = typename Types::ControlEstimates;

  // using PrxPlant = mushrFG_t;

  stela_robot_interface_t() : stela_robot_interface_t(State(), StateDot(), Control(), 0.1, Control(), Control())
  {
  }
  stela_robot_interface_t(const State idle_state,          // no-lint
                          const StateDot idle_state_dot,   // no-lint
                          const Control idle_control,      // no-lint
                          const double idle_dt,            // no-lint
                          const Control ctrl_lower_bound,  // no-lint
                          const Control ctrl_upper_bound)
    : _idle_state(idle_state)
    , _idle_state_dot(idle_state_dot)
    , _idle_control(idle_control)
    , _idle_dt(idle_dt)
    , _ctrl_lower_bound(ctrl_lower_bound)
    , _ctrl_upper_bound(ctrl_upper_bound)
  {
  }

  stela_robot_interface_t(ros::NodeHandle& nh, const State idle_state,  // no-lint
                          const StateDot idle_state_dot,                // no-lint
                          const Control idle_control,                   // no-lint
                          const double idle_dt,                         // no-lint
                          const Control ctrl_lower_bound,               // no-lint
                          const Control ctrl_upper_bound)
    : stela_robot_interface_t(idle_state, idle_state_dot, idle_control, idle_dt, ctrl_lower_bound, ctrl_upper_bound)
  {
    // std::string sensor_topic_name;

    // PARAM_SETUP(nh, sensor_topic_name)
    ros::NodeHandle nh_ctrl(nh, "control_space");

    std::vector<double> lower_bound;
    std::vector<double> upper_bound;

    PARAM_SETUP(nh_ctrl, lower_bound)
    PARAM_SETUP(nh_ctrl, upper_bound)

    _ctrl_lower_bound = Control(lower_bound.data());
    _ctrl_upper_bound = Control(upper_bound.data());
  }

  stela_robot_interface_t(ros::NodeHandle& nh)
    : stela_robot_interface_t(nh, State(), StateDot(), Control(), 0.1, Control(), Control())
  {
  }

  void bound(ml4kp_bridge::SpacePoint& ctrl) const
  {
    for (int i = 0; i < ctrl.point.size(); ++i)
    {
      ctrl.point[i] = std::max(ctrl.point[i], _ctrl_lower_bound[i]);
      ctrl.point[i] = std::min(ctrl.point[i], _ctrl_upper_bound[i]);
    }
  }

  void bound(ml4kp_bridge::SpacePointStamped& ctrl) const
  {
    bound(ctrl.space_point);
  }

  void bound(ml4kp_bridge::Plan& plan) const
  {
    for (auto& step : plan.steps)
    {
      bound(step.control);
    }
  }

  virtual GraphValues estimate_to_prior(const std::size_t idx, const StateEstimates& estimates,
                                        const std::vector<Eigen::MatrixXd>& covariances)
  {
    const gtsam::Key k_x{ keyX(1, idx) };
    const gtsam::Key k_xdot{ keyXdot(1, idx) };

    GraphValues graph_values;
    graph_values.first.addPrior(k_x, std::get<0>(estimates), covariances[0]);
    graph_values.first.addPrior(k_xdot, std::get<1>(estimates), covariances[1]);

    graph_values.second.insert(k_x, std::get<0>(estimates));
    graph_values.second.insert(k_xdot, std::get<1>(estimates));
    return graph_values;
  }

  // Factor graph for "Idle" state (i.e. before starting execution or after reaching the goal)
  virtual GraphValues idle_state_to_fg(const std::size_t parent, const std::size_t child,
                                       const bool time_as_variable = true) = 0;

  Control idle_control() const
  {
    return _idle_control;
  }

  double idle_dt() const
  {
    return _idle_dt;
  }

  virtual gtsam::Key keyU(const int& level, const int& step) const
  {
    return SF::create_hashed_symbol("U^{", level, "}_{", step, "}");
  }

  virtual gtsam::Key keyT(const int& level, const int& step) const
  {
    return SF::create_hashed_symbol("t^{", level, "}_{", step, "}");
  }

  virtual gtsam::Key keyParams(const int& level, const int& step) const
  {
    return SF::create_hashed_symbol("p^{", level, "}_{", step, "}");
  }

  // KeyX = X^{level}_{step}
  virtual gtsam::Key keyX(const int& level, const int& step) const
  {
    return SF::create_hashed_symbol("X^{", level, "}_{", step, "}");
  }

  // KeyXdot = X^{level}_{step}
  virtual gtsam::Key keyXdot(const int& level, const int& step) const
  {
    return SF::create_hashed_symbol("\\dot{X}^{", level, "}_{", step, "}");
  }

  virtual gtsam::Key keyXdotdot(const int& level, const int& step) const
  {
    return SF::create_hashed_symbol("\\ddot{X}^{", level, "}_{", step, "}");
  }

  virtual StateKeys keyState(const int& level, const int& step) const
  {
    return { keyX(level, step), keyXdot(level, step) };
  }

  virtual ControlKeys keyControl(const int& level, const int& step) const
  {
    return { keyU(level, step) };
  }
  virtual TimeKeys keyTime(const int& level, const int& step) const
  {
    return { keyT(level, step) };
  }

  // TODO: needed?
  // static void copy_observation(Observation& z, const geometry_msgs::TransformStamped& tf)
  // {
  //   z[0] = tf.transform.translation.x;
  //   z[1] = tf.transform.translation.y;
  //   const Eigen::Quaterniond q{ Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
  //                                                  tf.transform.rotation.y, tf.transform.rotation.z) };
  //   z[2] = prx::quaternion_to_euler(q)[2];
  //   // DEBUG_VARS(q)
  // }
  // TODO: TF needed?
  // template <typename StateIn>
  // static void copy(geometry_msgs::Transform& tf, const StateIn& x)
  // {
  //   const Eigen::Quaterniond q{ Eigen::AngleAxisd(x[2], Eigen::Vector3d::UnitZ()) };

  //   tf.translation.x = x[0];
  //   tf.translation.y = x[1];
  //   tf.translation.z = 0.0;
  //   tf.rotation.x = q.x();
  //   tf.rotation.y = q.y();
  //   tf.rotation.z = q.z();
  //   tf.rotation.w = q.w();
  // }

  virtual void copy_estimates(ml4kp_bridge::SpacePoint& pt, const StateEstimates& estimates) = 0;

  virtual void copy_control(ml4kp_bridge::SpacePoint& msg, const Control& u) = 0;

  virtual void copy_control(Control& u, const ml4kp_bridge::SpacePoint& msg) = 0;

  virtual void copy_control(Control& u, const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    copy_control(u, *msg);
  }

  virtual void plan_step(ml4kp_bridge::PlanStep& pt, const Control& u, const double duration)
  {
    pt.control.point.resize(u.size());
    copy_control(pt.control, u);
    pt.duration.data = ros::Duration(duration);
  }

  virtual void copy_state(State& x, const ml4kp_bridge::SpacePoint& pt) = 0;
  virtual void copy_stateDot(StateDot& xd, const ml4kp_bridge::SpacePoint& pt) = 0;

  virtual double distance(const State& x0, const State& x1) = 0;

  virtual double distance(const ml4kp_bridge::SpacePoint& pt0, const ml4kp_bridge::SpacePoint& pt1)
  {
    State x0, x1;
    copy_state(x0, pt0);
    copy_state(x1, pt1);
    return distance(x0, x1);
  }

  virtual std::shared_ptr<prx::fg::collision_info_t> collision_geometry() const
  {
    const std::string name{ Types::plant_name };

    std::shared_ptr<prx::plant_t> plant{ prx::system_factory_t::create_system_as<prx::plant_t>(name, name) };
    prx_assert(plant != nullptr, "Plant " << name << " couldn't be constructed!");

    prx::movable_object_t::Geometries geometries{ plant->get_geometries() };
    prx::movable_object_t::Configurations configurations{ plant->get_configurations() };

    const std::size_t total_geoms{ geometries.size() };
    std::shared_ptr<prx::geometry_t> g{ geometries[0].second };
    std::shared_ptr<prx::transform_t> tf{ configurations[0].second };

    const prx::geometry_type_t g_type{ g->get_geometry_type() };
    const std::vector<double> g_params{ g->get_geometry_params() };

    const Eigen::Matrix3d rot{ tf->rotation() };
    const Eigen::Vector3d t{ tf->translation() };

    return std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t);
  }

  virtual GraphValues add_observation_factor(const std::size_t prev_id, const std::size_t curr_id, const ros::Time& ti)
  {
    using ObservationFactor = prx::fg::lie_ode_observation_factor_t<State, StateDot>;

    GraphValues graph_values;
    if (not _new_observation)
    {
      return graph_values;
    }

    const gtsam::Key x0{ keyX(1, prev_id) };
    const gtsam::Key x1{ keyX(1, curr_id) };
    const gtsam::Key xdot0{ keyXdot(1, prev_id) };
    const gtsam::Key xdot1{ keyXdot(1, curr_id) };
    const gtsam::Key u01{ keyU(prev_id, curr_id) };

    NoiseModel z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1.0e-0) };
    // NoiseModel z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1.0e-2) };

    const Observation& zi{ _last_observation.first };
    const double dt{ (_last_observation.second - ti).toSec() };

    // LOG_MSG("Adding Observation Factor");
    // LOG_VARS(prev_id, curr_id, dt, zi);
    graph_values.first.emplace_shared<ObservationFactor>(x0, xdot0, z_noise, zi, dt, "Observation");

    _new_observation = false;
    return graph_values;
  }

  virtual GraphValues node_edge_to_fg(const prx_models::Node& node, const prx_models::Edge& edge) = 0;
  // {
  //   const ml4kp_bridge::SpacePoint& edge_control{ edge_plan.steps[0].control };
  //   const double dt{ edge_plan.steps[0].duration.data.toSec() };
  //   State x1;
  //   StateDot xdot1;
  //   Control u01;

  //   copy_state(x1, node_state);
  //   copy_stateDot(xdot1, node_state);
  //   u01[0] = edge_control.point[0];
  //   u01[1] = edge_control.point[1];

  //   return static_cast<DerivedRobotInterface*>(this)->node_edge_to_fg(parent, child, x1, xdot1, u01, dt,
  //                                                                     time_as_variable);
  //   // return node_edge_to_fg(parent, child, x1, xdot1, u01, dt, time_as_variable);
  // }
  // Create a FG that goes from N0 to N1 with plan P01
  // virtual GraphValues node_edge_to_fg(const std::size_t parent, const std::size_t child, const State& x1,
  //                                     const StateDot& xdot1, const Control& u01, const double& dt,
  //                                     const bool time_as_variable = true) = 0;

  virtual GraphValues idle_root(const std::size_t root)
  {
    GraphValues graph_values;

    const gtsam::Key k_x{ keyX(1, root) };
    const gtsam::Key k_xdot{ keyXdot(1, root) };

    NoiseModel x_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };

    graph_values.first.addPrior(k_x, _idle_state, x_prior_noise);
    graph_values.first.addPrior(k_xdot, _idle_state_dot, xdot_prior_noise);

    graph_values.second.insert(k_x, _idle_state);
    graph_values.second.insert(k_xdot, _idle_state_dot);

    return graph_values;
  }

  // template <typename Params>
  // static void set_params(const Params& params)
  // {
  //   default_params[mushr_types::Control::vel_desired] = params[mushr_types::Control::vel_desired];
  //   default_params[mushr_types::Control::steering] = params[mushr_types::Control::steering];
  //   default_params[mushr_types::Control::friction] = params[mushr_types::Control::friction];
  //   default_params[mushr_types::Control::delta_offset] = params[mushr_types::Control::delta_offset];
  //   default_params[mushr_types::Control::delta_gain] = params[mushr_types::Control::delta_gain];
  //   for (int i = 0; i < mushr_types::Control::PolyDeg; ++i)
  //   {
  //     default_poly[i] = params[i + 5];
  //   }
  //   // LOG_VARS(default_params.transpose());
  //   // LOG_VARS(default_poly.transpose());
  //   // prx_assert(params.size() == default_params.size(), "Wrong number of parameters!");
  //   // for (int i = 0; i < params.size(); ++i)
  //   // {
  //   //   default_params[i] = params[i];
  //   // }
  // }

  static void log_params()
  {
  }
  static void print_params()
  {
  }

protected:
  // static inline std::size_t first{ std::numeric_limits<std::size_t>::max() };

  // ros::Subscriber _sensor_subscriber;
  std::pair<Observation, ros::Time> _last_observation;
  const State _idle_state;
  const StateDot _idle_state_dot;
  const Control _idle_control;
  const double _idle_dt;
  bool _new_observation;

  Control _ctrl_lower_bound;
  Control _ctrl_upper_bound;
};