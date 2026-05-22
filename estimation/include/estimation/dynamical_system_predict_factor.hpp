#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace estimation
{

// To use in EKF-type settings, where, we want an estimationg of {x1, x0} such that x1 = f(x0, u, dt), {u, dt} fixed
template <typename DynamicalSystem>
class model_predict_factor_t
  : public gtsam::NoiseModelFactorN<typename DynamicalSystem::State, typename DynamicalSystem::State>
{
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Base = gtsam::NoiseModelFactorN<State, State>;

  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  // using MushrCtrlAccel = mushr_CtrlAccel_t<>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  model_predict_factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const Control ui, const double dt,
                         const DynamicalSystemPtr plant, const NoiseModel& cost_model)
    : Base(cost_model, key_x0, key_x1), _ui(ui), _plant(plant), _dt(dt)

  {
  }

  ~model_predict_factor_t() override
  {
  }

  virtual Eigen::VectorXd evaluateError(const State& x0, const State& x1, OptDeriv Hx0 = boost::none,
                                        OptDeriv Hx1 = boost::none) const override
  {
    const bool derivs{ Hx0 or Hx1 };
    typename DynamicalSystem::JacX x1p_H_x0, xerr_H_x1p, xerr_H_x1, err_H_xerr;

    const State x1p{ _plant->propagate(x0, _ui, _dt, derivs ? &x1p_H_x0 : nullptr) };
    const State xerr{ gtsam::traits<State>::Between(x1p, x1,                         // no-lint
                                                    derivs ? &xerr_H_x1p : nullptr,  // no-lint
                                                    derivs ? &xerr_H_x1 : nullptr) };
    const Eigen::VectorXd err{ gtsam::traits<State>::Logmap(xerr, derivs ? &err_H_xerr : nullptr) };

    if (Hx0)
    {
      *Hx0 = err_H_xerr * xerr_H_x1p * x1p_H_x0;
    }
    if (Hx1)
    {
      *Hx1 = err_H_xerr * xerr_H_x1;
    }

    return err;
  }

private:
  const DynamicalSystemPtr _plant;
  const double _dt;
  const Control _ui;
};
}  // namespace estimation