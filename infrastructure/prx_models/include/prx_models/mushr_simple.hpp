#pragma once
#include <array>
#include <numeric>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/simulation/plant.hpp"

namespace prx_models
{
inline double angle_diff(const double a, const double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

namespace mushrTypes
{

using State = Eigen::Vector<double, 3>;
using StateDot = Eigen::Vector<double, 3>;
using Control = Eigen::Vector<double, 2>;
using Ubar = Eigen::Vector<double, 3>;
using Parameters = Eigen::Vector<double, 6>;
using ParamsUbarU = Eigen::Vector<double, 3>;

static inline double positive_slope(const ParamsUbarU& param)
{
  return param[0];
}
static inline double steering_offset(const ParamsUbarU& params)
{
  return params[1];
}
static inline double velocity_gain(const ParamsUbarU& params)
{
  return params[2];
}
static inline double bound(const double value, const double min_bound, const double max_bound)
{
  return std::max(std::min(value, max_bound), min_bound);
}
static inline double& vel_delta_max(Parameters& params)
{
  return params[0];
}
static inline double& vel_delta_min(Parameters& params)
{
  return params[1];
}
static inline double& steering_gain(Parameters& params)
{
  return params[2];
}
static inline double& steering_offset(Parameters& params)
{
  return params[3];
}
static inline double& velocity_min(Parameters& params)
{
  return params[4];
}
static inline double& velocity_max(Parameters& params)
{
  return params[5];
}

static inline double vel_delta_max(const Parameters& params)
{
  return params[0];
}
static inline double vel_delta_min(const Parameters& params)
{
  return params[1];
}
static inline double steering_gain(const Parameters& params)
{
  return params[2];
}

static inline double steering_offset(const Parameters& params)
{
  return params[3];
}
static inline double velocity_min(const Parameters& params)
{
  return params[4];
}
static inline double velocity_max(const Parameters& params)
{
  return params[5];
}
static inline double steering(const Control& u)
{
  return u[0];
}
static inline double steering(const Control& u, const Parameters& params)
{
  const double us{ u[0] * steering_gain(params) + steering_offset(params) };
  // return bound(us, -1.0, 1.0);
  return us;
}
static inline double desired_velocity(const Control& u)
{
  return u[1];
}
static inline double desired_velocity(const Control& u, const Parameters& params)
{
  return bound(u[1], velocity_min(params), velocity_max(params));
}
template <typename State>
static inline double x(const State& state)
{
  return state[0];
}
template <typename State>
static inline double y(const State& state)
{
  return state[1];
}
template <typename State>
static inline double theta(const State& state)
{
  return state[2];
}
template <typename State>
static inline double current_velocity(const State& state)
{
  return state[3];
}
}  // namespace mushrTypes

// X_j = X_i + \dpt{x}_i * dt
using mushr_x_xdot_t = prx::fg::lie_integration_factor_t<mushr_types::State::type, mushr_types::StateDot::type, double>;
// class mushr_x_xdot_t : public gtsam::NoiseModelFactorN<mushr_types::StateDot::type, mushr_types::Ubar::type>
// {
// public:
//   using X = Eigen::Vector<double, 3>;
//   using Xdot = Eigen::Vector<double, 3>;

//   mushr_x_xdot_t() : _dt(prx::simulation_step)
//   {
//   }

//   static X predict(const X& x, const Xdot& xdot, const double dt)
//   {
//     // const gtsam::Pose2 res{ gtsam::Pose2(x[0], x[1], x[2]) * gtsam::Pose2::Expmap(xdot * dt) };
//     // return X{ res.x(), res.y(), res.theta() };
//     return x + xdot * dt;
//   }

//   virtual X compute_error(const X& xi, const Xdot& xdot, const X& xj) const
//   {
//     const X prediction{ predict(xi, xdot, _dt) };
//     X error{ prediction - xj };
//     error[2] = angle_diff(prediction[2], xj[2]);
//     return error;
//   }

// private:
//   const double _dt;
// };

class mushr_x_xdot_ub_t
{
public:
  using X = Eigen::Vector<double, 3>;
  using Xdot = Eigen::Vector<double, 3>;
  using Ubar = Eigen::Vector<double, 3>;

  static Xdot predict(const X& x, const Ubar& ubar)
  {
    const double beta{ ubar[2] };
    const double cTh{ std::cos(x[2] + beta) };  // cos(theta)
    const double sTh{ std::sin(x[2] + beta) };  // sin(theta)
    const double& vt{ ubar[0] };
    const double& wt{ ubar[1] };
    return Xdot{
      cTh * vt,  // no-indent
      sTh * vt,  // no-indent
      wt         // no-indent
    };
  }

  virtual Xdot compute_error(const X& xi, const Xdot& xdot, const Ubar& ub) const
  {
    return predict(xi, ub) - xdot;
  }

private:
};

class mushr_ub_u_xdot_t
{
public:
  using Xdot = Eigen::Vector<double, 3>;
  using Params = Eigen::Vector<double, 3>;
  using Ubar = Eigen::Vector<double, 2>;
  using U = Eigen::Vector<double, 2>;

  mushr_ub_u_xdot_t(double wheelbase) : _wheelbase(wheelbase)
  {
  }

  static Ubar predict(const U& u, const Xdot& xdot, const double& wheelbase)
  {
    const double vt{ xdot.head(2).norm() };  // \sqrt(\dot{x} + \dot{y})
    const double dv_cap{ mushrTypes::desired_velocity(u) };
    const double w{ vt * (std::tan(mushrTypes::steering(u)) / wheelbase) };
    return Ubar(dv_cap, w);
  }

  virtual Ubar compute_error(const Ubar& ub, const U& u, const Xdot& xdot) const
  {
    return predict(u, xdot, _wheelbase) - ub;
  }

private:
  const double _wheelbase;
};

class mushr_ub_u_xdot_param_t
{
public:
  using Xdot = Eigen::Vector<double, 3>;
  using Params = Eigen::Vector<double, 3>;
  using Ubar = mushrTypes::Ubar;
  using U = Eigen::Vector<double, 2>;

  mushr_ub_u_xdot_param_t(double length) : _wheelbase(length)
  {
  }

  static Ubar predict(const U& u, const Xdot& xdot, const Params& params, const double& length)
  {
    const double slope_pos{ mushrTypes::positive_slope(params) };
    const double steering_offset{ mushrTypes::steering_offset(params) };
    const double velocity_gain{ mushrTypes::velocity_gain(params) };
    // const double desired_vel_gain{ mushrTypes::desired_velocity_gain(params) };
    const double vel_desired{ mushrTypes::desired_velocity(u) };

    const double sign{ vel_desired > 0 ? +1.0 : -1.0 };
    const double vt{ sign * xdot.head(2).norm() };  // \sqrt(\dot{x} + \dot{y})
    const double dv{ vel_desired * velocity_gain - vt };
    const double dv_cap{ vt + dv * slope_pos };

    const double beta{ steering_offset * std::atan(0.5 * std::tan(mushrTypes::steering(u))) };
    const double w{ 2.0 * vt * std::sin(beta) / (length) };
    return Ubar(dv_cap, w, beta);
  }

  virtual Ubar compute_error(const Ubar& ub, const U& u, const Xdot& xdot, const Params& params) const
  {
    return predict(u, xdot, params, _wheelbase) - ub;
  }

private:
  const double _wheelbase;
};

class mushr_simple_t : public prx::plant_t
{
public:
  mushrFG_t(const std::string& path)
    : plant_t(path), _wheelbase(0.2965), _params_ubar_u(0.075, 0.259, 0.625), _ubar(mushrTypes::Ubar::Zero())
  {
    state_memory = { &_state[0], &_state[1], &_state[2], &_state_dot[0], &_state_dot[1] };
    state_space = new space_t("EEREE", state_memory, "mushr_state");
    state_space->set_bounds({ -100, -100, -prx::constants::pi, -100, -100 },
                            { 100, 100, prx::constants::pi, 100, 100 });

    control_memory = { &_ctrl[0], &_ctrl[1] };
    input_control_space = new space_t("EE", control_memory, "mushr_ctrl");
    input_control_space->set_bounds({ -prx::constants::pi / 2.0, -10 }, { prx::constants::pi / 2.0, 10 });

    derivative_memory = { &_state_dot[0], &_state_dot[1], &_state_dot[2], &_idle, &_idle };
    derivative_space = new space_t("EEEII", derivative_memory, "mushr_deriv");

    parameter_memory = { &_params_ubar_u[0], &_params_ubar_u[1], &_params_ubar_u[2] };
    parameter_space = new space_t("EEE", parameter_memory, "mushr_params");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.4, 0.28, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();
  }

  ~mushrFG_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    const mushrTypes::StateDot xdot{ _state_dot };
    _ubar = mushr_ub_u_xdot_param_t::predict(_ctrl, _state_dot, _params_ubar_u, _wheelbase);
    _state_dot = mushr_x_xdot_ub_t::predict(_state, _ubar);
    _state = mushr_x_xdot_t::predict(_state, xdot, simulation_step);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
    body->translation().head(2) = _state.head(2);
    body->translation()[2] = 0.0;
  }

  virtual void compute_derivative() override final
  {
  }

protected:
  mushrTypes::State _state;
  mushrTypes::StateDot _state_dot;
  mushrTypes::Control _ctrl;
  mushrTypes::Parameters _params;
  mushrTypes::Ubar _ubar;
  mushrTypes::ParamsUbarU _params_ubar_u;

  double _idle;

  double _wheelbase;
};  // namespace mushr

}  // namespace prx_models
PRX_REGISTER_SYSTEM(prx_models::mushr_simple_t, mushr_simple)
