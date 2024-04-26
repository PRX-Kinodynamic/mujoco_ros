#pragma once

// Ros
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/dbg_utils.hpp>

// ML4KP
#include <prx/simulation/plant.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>

// Gtsam
#include <gtsam/nonlinear/NonlinearFactor.h>

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
public:
  using MatrixG = Eigen::Matrix<double, 2, 2>;  // Dims could be parametric

  using State = Eigen::Vector<double, 2>;
  using StateDot = Eigen::Vector<double, 2>;
  using Control = Eigen::Vector<double, 2>;
  using Noise = Eigen::Vector<double, 2>;
  using Observation = Eigen::Vector<double, 2>;

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

private:
};

class ltv_sde_observation_factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, 2>>
{
public:
  using NoiseModelPtr = gtsam::noiseModel::Base::shared_ptr;

  using State = fg::ltv_sde_utils_t::State;
  using Observation = fg::ltv_sde_utils_t::Observation;
  using Error = Eigen::VectorXd;

  using Base = gtsam::NoiseModelFactor1<State>;

  ltv_sde_observation_factor_t(const gtsam::Key x, const Observation& z, const NoiseModelPtr& cost_model)
    : Base(cost_model, x), _z(z)
  {
  }

  virtual Error evaluateError(const State& x, boost::optional<Eigen::MatrixXd&> H = boost::none) const override
  {
    // clang-format off
    if(H) { *H = Eigen::Matrix<double,2,2>::Identity(); };
    // clang-format on
    return x.head(2) - _z;
  }

private:
  const Observation _z;
};

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
