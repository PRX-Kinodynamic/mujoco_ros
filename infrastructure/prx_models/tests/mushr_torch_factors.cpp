#include <ros/init.h>
#include <gtest/gtest.h>

#include <ml4kp_bridge/defs.h>
#include <memory>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx/utilities/general/random.hpp>
#include <prx_models/mushr_torch.hpp>
#include <gtsam/base/numericalDerivative.h>

#include <torch_bridge/sysid_runtime.hpp>

const double tolerance{ 1e-5 };

namespace mushr_torch_tests
{
std::string get_model_paths()
{
  const std::string path{ prx::lib_path_safe("ML4KP_ROS") };
  const std::string models_dir{ "/src/mujoco_ros/infrastructure/prx_models/models/learned_mushr/" };
  return path + models_dir;
}

std::string get_structured_file_path()
{
  const std::string nn_file{ "/S06_rollout_w1_structured_aux.ts.pt" };
  return mushr_torch_tests::get_model_paths() + nn_file;
}

std::string get_direct_file_path()
{
  const std::string nn_file{ "/D08_h10_w1_direct_model.ts.pt" };
  return mushr_torch_tests::get_model_paths() + nn_file;
}
}  // namespace mushr_torch_tests

struct StructuredParams
{
  static constexpr std::size_t friction{ prx_models::mushr_types::Control::friction };
  static constexpr std::size_t vel_desired{ prx_models::mushr_types::Control::vel_desired };
  static constexpr double L{ prx_models::mushr_types::Parameters::L };
};

TEST(TestMushrTorch, testJacobiansStructured)
{
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  using StateDot = torch_bridge::SysidRuntimeBase::StateDot;
  using Control = torch_bridge::SysidRuntimeBase::Control;
  using JacX = torch_bridge::SysidRuntimeBase::JacX;
  using JacU = torch_bridge::SysidRuntimeBase::JacU;

  using StructuredSysidRuntime = torch_bridge::StructuredSysidRuntime<MushrPlant, Params, Poly, StructuredParams>;

  using PartialXdot = std::function<StateDot(const StateDot&)>;
  using PartialCtrl = std::function<StateDot(const Control&)>;
  using DerivXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 2, 0>;
  using DerivCtrl = prx::math::first_order_derivative_t<PartialCtrl, Control, 2, 0>;

  const Params params{ Params(1.0, 1.0, 1.0, 0.0, 1.0) };
  const Poly poly{ Poly(0.0, 0.0, 1.0, 0.0) };

  const std::string structured_file{ mushr_torch_tests::get_structured_file_path() };
  std::shared_ptr<StructuredSysidRuntime> nn{ std::make_shared<StructuredSysidRuntime>(structured_file, params, poly,
                                                                                       false, "float32") };

  StateDot _xi;
  Control _ui;
  PartialXdot partial_xdot = [&](const StateDot& xd) { return nn->call(xd, _ui); };
  PartialCtrl partial_ctrl = [&](const Control& u) { return nn->call(_xi, u); };

  const double h{ 0.00001 };
  const DerivXdot derivative_xdot(partial_xdot, h);
  const DerivCtrl derivative_ctrl(partial_ctrl, h);

  std::vector<double> durations;
  double total_error_xdot{ 0.0 };
  double total_error_ctrl{ 0.0 };
  for (int i = 0; i < 1000; ++i)
  {
    _xi = StateDot::Random();
    _ui = Control::Random();

    const Eigen::Matrix<double, 3, 3> expectedHxd{ derivative_xdot(_xi) };
    const Eigen::Matrix<double, 3, 2> expectedHu{ derivative_ctrl(_ui) };

    auto [xd1p, actualHxd, actualHu] = nn->predict_with_jac(_xi, _ui);

    const double errXd{ (expectedHxd - actualHxd).norm() };
    const double errU{ (expectedHu - actualHu).norm() };
    total_error_xdot += errXd;
    total_error_ctrl += errU;

    if (errXd > 1.0)
    {
      DEBUG_VARS(expectedHxd);
      DEBUG_VARS(actualHxd);
    }
    ASSERT_TRUE(errXd < 1.0);
    ASSERT_TRUE(errU < 1.0);

    // durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
  }

  // print_stats("Numerical Jacobian Validation", true, durations);
  // std::cout << "Average Xdot error: " << total_error_xdot / total_calls_ << "\n";
  // std::cout << "Average Ctrl error: " << total_error_ctrl / total_calls_ << "\n";
}

TEST(TestMushrTorch, testJacobiansDirect)
{
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  using StateDot = torch_bridge::SysidRuntimeBase::StateDot;
  using Control = torch_bridge::SysidRuntimeBase::Control;
  using JacX = torch_bridge::SysidRuntimeBase::JacX;
  using JacU = torch_bridge::SysidRuntimeBase::JacU;

  using DirectSysidRuntime = torch_bridge::DirectSysidRuntime;

  using PartialXdot = std::function<StateDot(const StateDot&)>;
  using PartialCtrl = std::function<StateDot(const Control&)>;
  using DerivXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 2, 0>;
  using DerivCtrl = prx::math::first_order_derivative_t<PartialCtrl, Control, 2, 0>;

  const Params params{ Params(1.0, 1.0, 1.0, 0.0, 1.0) };
  const Poly poly{ Poly(0.0, 0.0, 1.0, 0.0) };

  const std::string direct_file{ mushr_torch_tests::get_direct_file_path() };
  std::shared_ptr<DirectSysidRuntime> nn{ std::make_shared<DirectSysidRuntime>(direct_file, false, "float32") };

  StateDot _xi;
  Control _ui;
  PartialXdot partial_xdot = [&](const StateDot& xd) { return nn->call(xd, _ui); };
  PartialCtrl partial_ctrl = [&](const Control& u) { return nn->call(_xi, u); };

  const double h{ 0.00001 };
  const DerivXdot derivative_xdot(partial_xdot, h);
  const DerivCtrl derivative_ctrl(partial_ctrl, h);

  std::vector<double> durations;
  double total_error_xdot{ 0.0 };
  double total_error_ctrl{ 0.0 };
  for (int i = 0; i < 1000; ++i)
  {
    _xi = StateDot::Random();
    _ui = Control::Random();

    const Eigen::Matrix<double, 3, 3> expectedHxd{ derivative_xdot(_xi) };
    const Eigen::Matrix<double, 3, 2> expectedHu{ derivative_ctrl(_ui) };

    auto [xd1p, actualHxd, actualHu] = nn->predict_with_jac(_xi, _ui);

    const double errXd{ (expectedHxd - actualHxd).norm() };
    const double errU{ (expectedHu - actualHu).norm() };
    total_error_xdot += errXd;
    total_error_ctrl += errU;

    if (errXd > 1.0)
    {
      DEBUG_VARS(expectedHxd);
      DEBUG_VARS(actualHxd);
    }
    ASSERT_TRUE(errXd < 1.0);
    ASSERT_TRUE(errU < 1.0);

    // durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
  }

  // print_stats("Numerical Jacobian Validation", true, durations);
  // std::cout << "Average Xdot error: " << total_error_xdot / total_calls_ << "\n";
  // std::cout << "Average Ctrl error: " << total_error_ctrl / total_calls_ << "\n";
}

TEST(TestMushrTorch, testTorchFactorDirect)
{
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  using StateDot = torch_bridge::SysidRuntimeBase::StateDot;
  using Control = torch_bridge::SysidRuntimeBase::Control;
  using JacX = torch_bridge::SysidRuntimeBase::JacX;
  using JacU = torch_bridge::SysidRuntimeBase::JacU;

  // using DirectSysidRuntime = torch_bridge::DirectSysidRuntime;

  using PartialXdot = std::function<StateDot(const StateDot&)>;
  using PartialCtrl = std::function<StateDot(const Control&)>;
  using PartialDt = std::function<StateDot(const double&)>;
  using DerivXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 2, 0>;
  using DerivCtrl = prx::math::first_order_derivative_t<PartialCtrl, Control, 2, 0>;
  using DerivDt = prx::math::first_order_derivative_t<PartialDt, double, 2, 0>;

  using MushrTorchFactor = prx_models::mushr_torch_factor_t<double>;

  const bool directNN{ true };
  const std::string direct_file{ mushr_torch_tests::get_direct_file_path() };
  // std::shared_ptr<DirectSysidRuntime> nn{ std::make_shared<DirectSysidRuntime>(direct_file, false, "float32") };

  const gtsam::Key xd1{ gtsam::Symbol('X', 1) };
  const gtsam::Key xd0{ gtsam::Symbol('X', 0) };
  const gtsam::Key u{ gtsam::Symbol('U', 0) };
  const gtsam::Key dt{ gtsam::Symbol('T', 0) };

  MushrTorchFactor factor(xd1, xd0, u, dt, nullptr, direct_file, directNN);

  // std::function<gtsam::Vector(const StateDot& xd1, const StateDot& xd0, const Control& u, const double& dt)> proxy_fn
  // =
  //     [&](const StateDot& xd1, const StateDot& xd0, const Control& u, const double& dt) {
  //       return factor.evaluateError(xd1, xd0, u, dt);
  //     };

  StateDot _xd0, _xd1;
  Control _u;
  double _dt;

  PartialXdot partial_xdot1 = [&](const StateDot& xd1) {
    return factor.evaluateError(xd1, _xd0, _u, _dt, boost::none, boost::none, boost::none, boost::none);
  };
  PartialXdot partial_xdot0 = [&](const StateDot& xd0) {
    return factor.evaluateError(_xd1, xd0, _u, _dt, boost::none, boost::none, boost::none, boost::none);
  };
  PartialCtrl partial_ctrl = [&](const Control& u) {
    return factor.evaluateError(_xd1, _xd0, u, _dt, boost::none, boost::none, boost::none, boost::none);
  };
  PartialDt partial_dt = [&](const double& dt) {
    return factor.evaluateError(_xd1, _xd0, _u, dt, boost::none, boost::none, boost::none, boost::none);
  };

  const double h{ 0.00001 };
  const DerivXdot derivative_xdot1(partial_xdot1, h);
  const DerivXdot derivative_xdot0(partial_xdot0, h);
  const DerivCtrl derivative_ctrl(partial_ctrl, h);
  const DerivDt derivative_dt(partial_dt, h);

  std::vector<double> durations;
  for (int i = 0; i < 1000; ++i)
  {
    _xd1 = StateDot::Random();
    _xd0 = StateDot::Random();
    _u = Control::Random();
    _dt = prx::uniform_random(0.05, 0.15);

    const Eigen::Matrix<double, 3, 3> expectedHxd1{ derivative_xdot1(_xd1) };
    const Eigen::Matrix<double, 3, 3> expectedHxd0{ derivative_xdot0(_xd0) };
    const Eigen::Matrix<double, 3, 2> expectedHu{ derivative_ctrl(_u) };
    const Eigen::Matrix<double, 3, 1> expectedHdt{ derivative_dt(_dt) };

    Eigen::MatrixXd actualHxd1, actualHxd0, actualHu, actualHdt;
    // auto [xd1p, actualHxd, actualHu] = nn->predict_with_jac(_xi, _ui);
    factor.evaluateError(_xd1, _xd0, _u, _dt, actualHxd1, actualHxd0, actualHu, actualHdt);

    const double errXd1{ (expectedHxd1 - actualHxd1).norm() };
    const double errXd0{ (expectedHxd0 - actualHxd0).norm() };
    const double errU{ (expectedHu - actualHu).norm() };
    const double errdt{ (expectedHdt - actualHdt).norm() };
    // total_error_xdot += errXd;
    // total_error_ctrl += errU;

    if (errXd1 > 1.0)
    {
      DEBUG_VARS(errXd1)
      DEBUG_VARS(expectedHxd1);
      DEBUG_VARS(actualHxd1);
    }
    if (errXd0 > 1.0)
    {
      DEBUG_VARS(errXd0)
      DEBUG_VARS(expectedHxd0);
      DEBUG_VARS(actualHxd0);
    }
    if (errU > 1.0)
    {
      DEBUG_VARS(errU)
      DEBUG_VARS(expectedHu);
      DEBUG_VARS(actualHu);
    }
    if (errdt > 1.0)
    {
      DEBUG_VARS(errdt)
      DEBUG_VARS(expectedHdt);
      DEBUG_VARS(actualHdt);
    }
    ASSERT_TRUE(errXd1 < 1.0);
    ASSERT_TRUE(errXd0 < 1.0);
    ASSERT_TRUE(errU < 1.0);
    ASSERT_TRUE(errdt < 1.0);

    // durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
  }

  // print_stats("Numerical Jacobian Validation", true, durations);
  // std::cout << "Average Xdot error: " << total_error_xdot / total_calls_ << "\n";
  // std::cout << "Average Ctrl error: " << total_error_ctrl / total_calls_ << "\n";
}

TEST(TestMushrTorch, testTorchFactorStructured)
{
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  using StateDot = torch_bridge::SysidRuntimeBase::StateDot;
  using Control = torch_bridge::SysidRuntimeBase::Control;
  using JacX = torch_bridge::SysidRuntimeBase::JacX;
  using JacU = torch_bridge::SysidRuntimeBase::JacU;

  // using DirectSysidRuntime = torch_bridge::DirectSysidRuntime;

  using PartialXdot = std::function<StateDot(const StateDot&)>;
  using PartialCtrl = std::function<StateDot(const Control&)>;
  using PartialDt = std::function<StateDot(const double&)>;
  using DerivXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 2, 0>;
  using DerivCtrl = prx::math::first_order_derivative_t<PartialCtrl, Control, 2, 0>;
  using DerivDt = prx::math::first_order_derivative_t<PartialDt, double, 2, 0>;

  using MushrTorchFactor = prx_models::mushr_torch_factor_t<double>;

  const bool directNN{ false };
  const std::string structured_file{ mushr_torch_tests::get_structured_file_path() };
  // std::shared_ptr<DirectSysidRuntime> nn{ std::make_shared<DirectSysidRuntime>(direct_file, false, "float32") };

  const gtsam::Key xd1{ gtsam::Symbol('X', 1) };
  const gtsam::Key xd0{ gtsam::Symbol('X', 0) };
  const gtsam::Key u{ gtsam::Symbol('U', 0) };
  const gtsam::Key dt{ gtsam::Symbol('T', 0) };

  MushrTorchFactor factor(xd1, xd0, u, dt, nullptr, structured_file, directNN);

  // std::function<gtsam::Vector(const StateDot& xd1, const StateDot& xd0, const Control& u, const double& dt)> proxy_fn
  // =
  //     [&](const StateDot& xd1, const StateDot& xd0, const Control& u, const double& dt) {
  //       return factor.evaluateError(xd1, xd0, u, dt);
  //     };

  StateDot _xd0, _xd1;
  Control _u;
  double _dt;

  PartialXdot partial_xdot1 = [&](const StateDot& xd1) {
    return factor.evaluateError(xd1, _xd0, _u, _dt, boost::none, boost::none, boost::none, boost::none);
  };
  PartialXdot partial_xdot0 = [&](const StateDot& xd0) {
    return factor.evaluateError(_xd1, xd0, _u, _dt, boost::none, boost::none, boost::none, boost::none);
  };
  PartialCtrl partial_ctrl = [&](const Control& u) {
    return factor.evaluateError(_xd1, _xd0, u, _dt, boost::none, boost::none, boost::none, boost::none);
  };
  PartialDt partial_dt = [&](const double& dt) {
    return factor.evaluateError(_xd1, _xd0, _u, dt, boost::none, boost::none, boost::none, boost::none);
  };

  const double h{ 0.00001 };
  const DerivXdot derivative_xdot1(partial_xdot1, h);
  const DerivXdot derivative_xdot0(partial_xdot0, h);
  const DerivCtrl derivative_ctrl(partial_ctrl, h);
  const DerivDt derivative_dt(partial_dt, h);

  std::vector<double> durations;
  for (int i = 0; i < 1000; ++i)
  {
    _xd1 = StateDot::Random();
    _xd0 = StateDot::Random();
    _u = Control::Random();
    _dt = prx::uniform_random(0.05, 0.15);

    const Eigen::Matrix<double, 3, 3> expectedHxd1{ derivative_xdot1(_xd1) };
    const Eigen::Matrix<double, 3, 3> expectedHxd0{ derivative_xdot0(_xd0) };
    const Eigen::Matrix<double, 3, 2> expectedHu{ derivative_ctrl(_u) };
    const Eigen::Matrix<double, 3, 1> expectedHdt{ derivative_dt(_dt) };

    Eigen::MatrixXd actualHxd1, actualHxd0, actualHu, actualHdt;
    // auto [xd1p, actualHxd, actualHu] = nn->predict_with_jac(_xi, _ui);
    factor.evaluateError(_xd1, _xd0, _u, _dt, actualHxd1, actualHxd0, actualHu, actualHdt);

    const double errXd1{ (expectedHxd1 - actualHxd1).norm() };
    const double errXd0{ (expectedHxd0 - actualHxd0).norm() };
    const double errU{ (expectedHu - actualHu).norm() };
    const double errdt{ (expectedHdt - actualHdt).norm() };
    // total_error_xdot += errXd;
    // total_error_ctrl += errU;

    if (errXd1 > 1.0)
    {
      DEBUG_VARS(errXd1)
      DEBUG_VARS(expectedHxd1);
      DEBUG_VARS(actualHxd1);
    }
    if (errXd0 > 1.0)
    {
      DEBUG_VARS(errXd0)
      DEBUG_VARS(expectedHxd0);
      DEBUG_VARS(actualHxd0);
    }
    if (errU > 1.0)
    {
      DEBUG_VARS(errU)
      DEBUG_VARS(expectedHu);
      DEBUG_VARS(actualHu);
    }
    if (errdt > 1.0)
    {
      DEBUG_VARS(errdt)
      DEBUG_VARS(expectedHdt);
      DEBUG_VARS(actualHdt);
    }
    ASSERT_TRUE(errXd1 < 1.0);
    ASSERT_TRUE(errXd0 < 1.0);
    ASSERT_TRUE(errU < 1.0);
    ASSERT_TRUE(errdt < 1.0);

    // durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
  }

  // print_stats("Numerical Jacobian Validation", true, durations);
  // std::cout << "Average Xdot error: " << total_error_xdot / total_calls_ << "\n";
  // std::cout << "Average Ctrl error: " << total_error_ctrl / total_calls_ << "\n";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
