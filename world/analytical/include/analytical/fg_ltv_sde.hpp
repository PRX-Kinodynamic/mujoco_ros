#pragma once

#include <utils/dbg_utils.hpp>
#include <prx/simulation/plant.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace prx
{
namespace fg
{
// LTV from CS-BRM: A Probabilistic RoadMap for Consistent Belief Space Planning With Reachability Guarantees
// https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10404055
// Eq. 50
// Setting h = 0
class ltv_sde_factor_t
  : public gtsam::NoiseModelFactor5<Eigen::Vector<double, 4>, Eigen::Vector<double, 4>, Eigen::Vector<double, 2>,
                                    Eigen::Vector<double, 4>, Eigen::Vector<double, 4>>
{
public:
  using NoiseModelPtr = gtsam::noiseModel::Base::shared_ptr;
  using MatrixA = Eigen::Matrix<double, 4, 4>;  // Dims could be parametric
  using MatrixB = Eigen::Matrix<double, 4, 2>;  // Dims could be parametric
  using MatrixG = Eigen::Matrix<double, 4, 4>;  // Dims could be parametric

  using State = Eigen::Vector<double, 4>;
  using Control = Eigen::Vector<double, 2>;
  using Noise = Eigen::Vector<double, 4>;
  using Error = Eigen::VectorXd;

  using Base = gtsam::NoiseModelFactor5<State, State, Control, Noise, Noise>;

  ltv_sde_factor_t(const gtsam::Key x1, const gtsam::Key x0, const gtsam::Key u, const gtsam::Key h, const gtsam::Key w,
                   const double dt, const NoiseModelPtr& cost_model, const MatrixA A, const MatrixB B, const MatrixG G)
    : Base(cost_model, x1, x0, u, h, w), _dt(dt), _A(A), _B(B), _G(G)
  {
  }

  static State predict(const State& x, const Control& u, const Noise& h, const Noise w,  // no-lint
                       const MatrixA& A, const MatrixB& B, const MatrixG& G,
                       boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                       boost::optional<Eigen::MatrixXd&> Hu = boost::none,
                       boost::optional<Eigen::MatrixXd&> Hh = boost::none,
                       boost::optional<Eigen::MatrixXd&> Hw = boost::none)
  {
    // clang-format off
    if(Hx) { *Hx =A; }; 
    if(Hu) { *Hu =B; }; 
    if(Hh) { *Hh =MatrixG::Identity(); }; 
    if(Hw) { *Hw =G; };
    // clang-format on
    return A * x + B * u + h + G * w;
  }

  virtual Error evaluateError(const State& x1, const State& x0, const Control& u, const Noise& h,
                              const Noise& w,  // no-lint
                              boost::optional<Eigen::MatrixXd&> Hx1 = boost::none,
                              boost::optional<Eigen::MatrixXd&> Hx0 = boost::none,
                              boost::optional<Eigen::MatrixXd&> Hu = boost::none,
                              boost::optional<Eigen::MatrixXd&> Hh = boost::none,
                              boost::optional<Eigen::MatrixXd&> Hw = boost::none) const override
  {
    // clang-format off
    if(Hx1) { *Hx1 = -1 * MatrixA::Identity(); };
    // clang-format on
    return predict(x0, u, h, w, _A, _B, _G, Hx0, Hu, Hh, Hw) - x1;
  }

private:
  const double _dt;

  const MatrixA _A;
  const MatrixB _B;
  const MatrixG _G;
};

}  // namespace fg
class fg_ltv_sde_t : public plant_t
{
  using State = Eigen::Vector<double, 4>;    // Maybe do this parametric
  using Control = Eigen::Vector<double, 2>;  // Maybe do this parametric
  using Noise = Eigen::Vector<double, 4>;    // Maybe do this parametric
public:
  fg_ltv_sde_t(const std::string& path)
    : plant_t(path)
    , _x(State::Zero())
    , _u(Control::Zero())
    , _h(Noise::Zero())
    , _w(Noise::Zero())
    , _A((fg::ltv_sde_factor_t::MatrixA() << 1, 0, simulation_step, 0, 0, 1, 0, simulation_step, 0, 0, 1, 0, 0, 0, 0, 1)
             .finished())
    , _B((fg::ltv_sde_factor_t::MatrixB() << simulation_step * simulation_step / 2.0, 0, 0,
          simulation_step * simulation_step / 2.0, simulation_step, 0, 0, simulation_step)
             .finished())
    , _G(Eigen::DiagonalMatrix<double, 4>(5, 8, 5, 5) * 0.01)
  {
    state_memory = { &_x[0], &_x[1], &_x[2], &_x[3] };
    state_space = new space_t("EEEE", state_memory, "state_space");

    control_memory = { &_u[0], &_u[1] };
    input_control_space = new space_t("EE", control_memory, "control_space");

    // Not using derivative memory since deriv is computed via ltv_sde_factor_t::predict
    // derivative_memory = { &_x[2], &_x[3], &_u[0], &_u[1] };
    // derivative_space = new space_t("EEEE", derivative_memory, "deriv_space");

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
    _x = fg::ltv_sde_factor_t::predict(_x, _u, _h, _w, _A, _B, _G);
    // y = C(_x) *_x + D(_x) * _v; // observation
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
  }
  State _x;
  Control _u;
  Noise _w, _h;
  fg::ltv_sde_factor_t::MatrixA _A;
  fg::ltv_sde_factor_t::MatrixB _B;
  fg::ltv_sde_factor_t::MatrixG _G;
};

}  // namespace prx

PRX_REGISTER_SYSTEM(fg_ltv_sde_t, fg_ltv_sde)
