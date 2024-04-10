#pragma once

#include <utils/dbg_utils.hpp>
#include <prx/simulation/plant.hpp>

namespace prx
{
// LTV from CS-BRM: A Probabilistic RoadMap for Consistent Belief Space Planning With Reachability Guarantees
// https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10404055
// Eq. 50
// Setting h = 0
class ltv_sde_t : public plant_t
{
  using State = Eigen::Vector<double, 4>;    // Maybe do this parametric
  using Control = Eigen::Vector<double, 2>;  // Maybe do this parametric
  using Noise = Eigen::Vector<double, 4>;    // Maybe do this parametric
public:
  ltv_sde_t(const std::string& path)
    : plant_t(path), _x(State::Zero()), _xdot(State::Zero()), _u(Control::Zero()), _w(Noise::Zero())
  {
    _A(0, 2) = prx::simulation_step;
    _A(1, 3) = prx::simulation_step;
    _B(0, 0) = prx::simulation_step * prx::simulation_step / 2;
    _B(1, 1) = prx::simulation_step * prx::simulation_step / 2;
    _B(2, 0) = prx::simulation_step;
    _B(2, 1) = prx::simulation_step;
    _G.diagonal() = Eigen::Vector4d(5, 8, 5, 5) * 0.01;

    state_memory = { &_x[0], &_x[1], &_x[2], &_x[3] };
    state_space = new space_t("EEEE", state_memory, "state_space");

    control_memory = { &_u[0], &_u[1] };
    input_control_space = new space_t("EE", control_memory, "control_space");

    derivative_memory = { &_xdot[0], &_xdot[1], &_xdot[2], &_xdot[3] };
    derivative_space = new space_t("EEEE", derivative_memory, "deriv_space");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
    geometries["body"]->initialize_geometry({ 1 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();

    set_integrator(integrator_t::kEULER);
  }

  virtual ~ltv_sde_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    integrator->integrate(simulation_step);
    // y = C(_x) *_x + D(_x) * _v; // observation
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->setIdentity();
    body->translation() = Eigen::Vector3d(_x[0], _x[1], 0.0);
  }

protected:
  virtual void compute_derivative() override final
  {
    _xdot = _A * _x + _B * _u + _G * _w;
  }
  Eigen::Matrix<double, 4, 4> _A;
  Eigen::Matrix<double, 4, 2> _B;
  Eigen::Matrix<double, 4, 4> _G;

  State _x, _xdot;
  Control _u;
  Noise _w;
};

}  // namespace prx
using LTV_CSBRM = prx::ltv_sde_t;

PRX_REGISTER_SYSTEM(LTV_CSBRM, LTV_CSBRM)
