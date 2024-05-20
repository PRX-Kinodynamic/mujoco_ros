#pragma once

// Ros
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

namespace prx
{
namespace fg
{

// LTV from CS-BRM: A Probabilistic RoadMap for Consistent Belief Space Planning With Reachability Guarantees
// https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10404055
// Eq. 50
// Setting h = 0
class ltv_sde_utils_t
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;

public:
  using Values = gtsam::Values;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using GraphValues = std::pair<FactorGraph, Values>;

  using MatrixG = Eigen::Matrix<double, 2, 2>;  // Dims could be parametric

  using State = Eigen::Vector<double, 2>;
  using StateDot = Eigen::Vector<double, 2>;
  using Control = Eigen::Vector<double, 2>;
  using Noise = Eigen::Vector<double, 2>;
  using Observation = Eigen::Vector<double, 2>;

  using EulerFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
  using DynamicsFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
  using ObservationFactor = gtsam::PriorFactor<State>;

  // KeyU = U^{level}_{step}
  static gtsam::Key keyU(const int& level, const std::size_t& step)
  {
    return SF::create_hashed_symbol("U^{", level, "}_{", step, "}");
  }

  // KeyX = X^{level}_{step}
  static gtsam::Key keyX(const int& level, const std::size_t& step)
  {
    return SF::create_hashed_symbol("X^{", level, "}_{", step, "}");
  }

  // KeyXdot = X^{level}_{step}
  static gtsam::Key keyXdot(const int& level, const std::size_t& step)
  {
    return SF::create_hashed_symbol("\\dot{X}^{", level, "}_{", step, "}");
  }

  static void copy(Control& u, const ml4kp_bridge::SpacePointConstPtr& msg)
  {
    u[0] = msg->point[0];
    u[1] = msg->point[1];
  }

  static void copy(Observation& z, const geometry_msgs::TransformStamped& tf)
  {
    z[0] = tf.transform.translation.x;
    z[1] = tf.transform.translation.y;
  }

  template <typename StateIn>
  static void copy(geometry_msgs::Transform& tf, const StateIn& x)
  {
    tf.translation.x = x[0];
    tf.translation.y = x[1];
    tf.translation.z = 0.0;
    tf.rotation.x = 0.0;
    tf.rotation.y = 0.0;
    tf.rotation.z = 0.0;
    tf.rotation.w = 1.0;
  }
  static void copy(ml4kp_bridge::SpacePoint& pt, const State& x, const StateDot& xdot)
  {
    pt.point.resize(4);
    pt.point[0] = x[0];
    pt.point[1] = x[1];
    pt.point[2] = xdot[0];
    pt.point[3] = xdot[1];
  }

  /**
   * @brief compute the noise for the state from the matrix and a vector of sigmas
   * @details Compute the noise at state k given sigmas. The input sigmas is vector that is used as: w\sim N(0,\sigma_i)
   *
   * @param G Matrix that maps vector w\sim N(0,\sigma s) to the state. Likely a diagonal matrix
   * @param sigmas The variance per dimension.
   *
   * @return Computes $G * w$ where $w \sim N(0,\sigma)$
   */
  static Noise noise(const MatrixG& G, const Noise sigmas)
  {
    const Noise w{ prx::gaussian_random(0.0, sigmas[0]), prx::gaussian_random(0.0, sigmas[1]) };
    return G * w;
  }

  // class trajectory_estimation
  // {
  /**
    Factor graph at some time t:
                                 (Integration)
                           (x_t) ----- * ----- (x_{t+1})
                                       |
                     (Dynamic)         |
    (\dot{x_{t}}) ----- * ----- (\dot{x_{t+1}})
                        |
                        |
                (\ddot{x_t} = u_t)
   */

  /**
   * @brief Creates the small factor graph above: between t and t+1
   * @details Create a small factor graph x_{t+1} <- x_t + xdot * dt AND \dot{x}_{t+1} <- \dot{x}_t + \ddot{x} * dt  AND
   * \hat{x}_t <- x_t + noise ~= z intended to be an update to isam2, accounting for a new observation of a state.
   *
   * @param idx index of the observation. First observation is always 0
   * @param dt The time between x[idx] and x[idx+1]
   * @param keyX A function that produces a key for X taking as input idx
   * @param keyXdot A function that produces a key for Xdot taking as input idx
   * @param integration_noise Noise model to use for integration
   * @param observation_noise Noise model to use for observation
   * @tparam IntegrationFactor Integration class that does x_{t+1} <- x_t + xdot * dt in that order: (x(idx+1), x(idx) +
   * xdot(idx))
   * @return A factor graph
   */

  // The necessary info to create the FG from X_t to X_{t+1}
  struct local_update_t
  {
    local_update_t() : integration_noise(nullptr), dynamic_noise(nullptr), observation_noise(nullptr){};

    NoiseModel integration_noise;
    NoiseModel dynamic_noise;
    NoiseModel observation_noise;

    std::size_t step{};
    double dt{};
    // level:
    // If (-\inf,-1] ==> estimation (branch is in the past)
    // If [0,\inf) ==> plan (branch is in the future)
    int level{};

    Observation observation;
    Control control;
    StateDot xdot;
    State x;
  };

  static GraphValues initial_factor_graph(const local_update_t& update)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(update.level, update.step) };
    const gtsam::Key x1{ keyX(update.level, update.step + 1) };
    const gtsam::Key xdot0{ keyXdot(update.level, update.step) };
    const gtsam::Key xdot1{ keyXdot(update.level, update.step + 1) };
    const gtsam::Key u0{ keyU(update.level, update.step) };

    graph_values.first.addPrior(x0, update.x);
    graph_values.first.addPrior(xdot0, update.xdot);
    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, update.integration_noise, update.dt);
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u0, update.dynamic_noise, update.dt);

    graph_values.second.insert(x0, update.x);
    graph_values.second.insert(xdot0, update.xdot);
    graph_values.second.insert(u0, update.control);
    return graph_values;
  }

  static GraphValues local_factor_graph(const local_update_t& update)
  {
    using EulerStateStateDotFactor = prx::fg::euler_integration_factor_t<State, StateDot>;
    using EulerStateDotControlFactor = prx::fg::euler_integration_factor_t<StateDot, Control>;
    GraphValues graph_values;

    const gtsam::Key x0{ keyX(update.level, update.step) };
    const gtsam::Key x1{ keyX(update.level, update.step + 1) };
    const gtsam::Key xdot0{ keyXdot(update.level, update.step) };
    const gtsam::Key xdot1{ keyXdot(update.level, update.step + 1) };
    const gtsam::Key u1{ keyU(update.level, update.step + 1) };

    graph_values.first.emplace_shared<EulerStateStateDotFactor>(x1, x0, xdot0, update.integration_noise, update.dt);
    graph_values.first.emplace_shared<EulerStateDotControlFactor>(xdot1, xdot0, u1, update.dynamic_noise, update.dt);

    graph_values.second.insert(u1, update.control);
    graph_values.second.insert(xdot1, update.xdot);
    graph_values.second.insert(x1, update.x);
    return graph_values;
  }

  static GraphValues estimation_local_factor_graph(const local_update_t& update)
  {
    using ObservationFactor = gtsam::PriorFactor<State>;
    prx_assert(update.level < 0, "Positive branch on estimation FG");

    const gtsam::Key x0{ keyX(update.level, update.step) };
    GraphValues graph_values{ local_factor_graph(update) };
    graph_values.first.emplace_shared<ObservationFactor>(x0, update.observation, update.observation_noise);

    return graph_values;
  }

  // Take a SBMP edge-node and convert it to FG:
  // SBMP:  N0 ----E01---- N1 which is: N0=traj[0]; E01=plan
  // FG:    X0 ----F01---- X1
  static GraphValues sbmp_edge_node_to_factor_graph(const int level, const std::size_t step,
                                                    const Eigen::Vector4d& node_state, const Eigen::Vector2d& control,
                                                    const double& duration)
  {
    local_update_t update{};

    update.dt = duration;

    const double numerical_error{ update.dt };
    update.integration_noise = gtsam::noiseModel::Isotropic::Sigma(2, numerical_error);
    update.dynamic_noise = gtsam::noiseModel::Isotropic::Sigma(2, numerical_error);

    update.step = step;
    update.level = level;

    // const Eigen::Vector4d state{ Vec(traj.front()) };
    update.control = control;
    update.x = node_state.head(2);
    update.xdot = node_state.tail(2);

    return local_factor_graph(update);
  };
};

// class ltv_sde_observation_factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, 2>>
// {
// public:
//   using NoiseModelPtr = gtsam::noiseModel::Base::shared_ptr;

//   using State = fg::ltv_sde_utils_t::State;
//   using Observation = fg::ltv_sde_utils_t::Observation;
//   using Error = Eigen::VectorXd;

//   using Base = gtsam::NoiseModelFactor1<State>;

//   ltv_sde_observation_factor_t(const gtsam::Key x, const Observation& z, const NoiseModelPtr& cost_model)
//     : Base(cost_model, x), _z(z)
//   {
//   }

//   virtual Error evaluateError(const State& x, boost::optional<Eigen::MatrixXd&> H = boost::none) const override
//   {
//     // clang-format off
//     if(H) { *H = Eigen::Matrix<double,2,2>::Identity(); };
//     // clang-format on
//     return x.head(2) - _z;
//   }

// private:
//   const Observation _z;
// };

class fg_ltv_sde_t : public plant_t
{
  using State = fg::ltv_sde_utils_t::State;
  using StateDot = fg::ltv_sde_utils_t::StateDot;
  using Control = fg::ltv_sde_utils_t::Control;
  using Noise = fg::ltv_sde_utils_t::Noise;

  using EulerFactor = prx::fg::euler_integration_factor_t<State, StateDot>;

public:
  fg_ltv_sde_t(const std::string& path)
    : plant_t(path)
    , _x(State::Zero())
    , _u(Control::Zero())
    , _Gx(Eigen::DiagonalMatrix<double, 2>(5, 8) * 0.01)
    , _Gxdot(Eigen::DiagonalMatrix<double, 2>(5, 5) * 0.01)
    , _wx_sigmas(Noise::Ones())
    , _wxdot_sigmas(Noise::Ones())
  {
    state_memory = { &_x[0], &_x[1], &_xdot[0], &_xdot[1] };
    state_space = new space_t("EEEE", state_memory, "state_space");

    control_memory = { &_u[0], &_u[1] };
    input_control_space = new space_t("EE", control_memory, "control_space");

    derivative_memory = { &_xdot[0], &_xdot[1], &_u[0], &_u[1] };
    derivative_space = new space_t("EEEE", derivative_memory, "deriv_space");

    parameter_memory = { &_wx_sigmas[0], &_wx_sigmas[1], &_wxdot_sigmas[0], &_wxdot_sigmas[1],
                         &_Gx(0, 0),     &_Gx(1, 1),     &_Gxdot(0, 0),     &_Gxdot(1, 1) };
    parameter_space = new space_t("EEEEEEEE", parameter_memory, "params");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
    geometries["body"]->initialize_geometry({ 1 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();
  }

  virtual ~fg_ltv_sde_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    compute_derivative();
    _x = EulerFactor::integrate(_x, _xdot, prx::simulation_step) + fg::ltv_sde_utils_t::noise(_Gx, _wx_sigmas);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->setIdentity();
    body->translation() = Eigen::Vector3d(_x[0], _x[1], 0.5);
  }

protected:
  virtual void compute_derivative() override final
  {
    _xdot = EulerFactor::integrate(_xdot, _u, prx::simulation_step) + fg::ltv_sde_utils_t::noise(_Gxdot, _wxdot_sigmas);
  }

  State _x;
  StateDot _xdot;
  Control _u;
  Noise _wx_sigmas;
  Noise _wxdot_sigmas;
  fg::ltv_sde_utils_t::MatrixG _Gx;
  fg::ltv_sde_utils_t::MatrixG _Gxdot;
};

}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(fg::fg_ltv_sde_t, fg_ltv_sde)
