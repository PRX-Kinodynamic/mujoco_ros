#include <ros/init.h>
#include <gtest/gtest.h>

#include <prx_models/mushr_factors.hpp>
#include <gtsam/base/numericalDerivative.h>
const double tolerance{ 1e-5 };

TEST(TestMushrFactors, testMushrCtrlXddot01DesiredXdot)
{
  using Control = prx_models::mushr_types::Control::type;
  using MushrCtrl = prx_models::mushr_u_xddot01_t;

  const Control u{ 1.0, 0.5 };

  // Check jacobians
  Eigen::Matrix<double, 3, 2> actualHu, expectedHu;
  std::function<gtsam::Vector(const Control&)> desired_xdot_proxy = [&](const Control& u) {
    return MushrCtrl::desired_xdot(u);
  };

  MushrCtrl::desired_xdot(u, &actualHu);
  expectedHu = gtsam::numericalDerivative11(desired_xdot_proxy, u);
  ASSERT_TRUE(expectedHu.isApprox(actualHu, tolerance));
}

TEST(TestMushrFactors, testMushrCtrlXddot01VelocityDelta)
{
  using StateDot = prx_models::mushr_types::StateDot::type;
  using MushrCtrl = prx_models::mushr_u_xddot01_t;

  const StateDot xd0{ 0.2450, 0.0497, 0.1 };  // Using Beta=0.2;Vin=0.25; w=0.1
  const double dt{ 0.1 };
  const StateDot xdotDesired{ 0.4975, 0.0499, 0.15 };  // Using Beta=0.1;Vin=0.5; w=0.1
  const double K{ 0.8 };

  // Check jacobians
  Eigen::Matrix<double, 3, 3> actualHxd0, expectedHxd0;
  Eigen::Matrix<double, 3, 1> actualHdt, expectedHdt;
  Eigen::Matrix<double, 3, 3> actualHxdotd, expectedHxdotd;
  Eigen::Matrix<double, 3, 1> actualHK, expectedHK;

  std::function<gtsam::Vector(const StateDot& xd0, const double& dt, const StateDot& xdot_d, const double& K)>
      vd_proxy = [](const StateDot& xd0, const double& dt, const StateDot& xdot_d, const double& K) {
        return MushrCtrl::velocity_delta(xd0, dt, xdot_d, K);
      };

  MushrCtrl::velocity_delta(xd0, dt, xdotDesired, K, &actualHxd0, &actualHdt, &actualHxdotd, &actualHK);
  expectedHxd0 = gtsam::numericalDerivative41(vd_proxy, xd0, dt, xdotDesired, K);
  expectedHdt = gtsam::numericalDerivative42(vd_proxy, xd0, dt, xdotDesired, K);
  expectedHxdotd = gtsam::numericalDerivative43(vd_proxy, xd0, dt, xdotDesired, K);
  expectedHK = gtsam::numericalDerivative44(vd_proxy, xd0, dt, xdotDesired, K);

  // PRX_DBG_VARS(expectedHxd0);
  // PRX_DBG_VARS(actualHxd0);

  PRX_DBG_VARS(expectedHdt);
  PRX_DBG_VARS(actualHdt);

  // PRX_DBG_VARS(expectedHxdotd);
  // PRX_DBG_VARS(actualHxdotd);

  // PRX_DBG_VARS(expectedHK);
  // PRX_DBG_VARS(actualHK);

  ASSERT_TRUE(expectedHxd0.isApprox(actualHxd0, tolerance));
  ASSERT_TRUE(expectedHdt.isApprox(actualHdt, tolerance));
  ASSERT_TRUE(expectedHxdotd.isApprox(actualHxdotd, tolerance));
  ASSERT_TRUE(expectedHK.isApprox(actualHK, tolerance));
}

TEST(TestMushrFactors, testMushrCtrlXddot01Predict)
{
  using StateDot = prx_models::mushr_types::StateDot::type;
  using Control = prx_models::mushr_types::Control::type;
  using MushrCtrl = prx_models::mushr_u_xddot01_t;
  using Params = prx_models::mushr_types::Ubar::params;

  const StateDot xd0{ 0.2450, 0.0497, 0.1 };  // Using Beta=0.2;Vin=0.25; w=0.1
  const double dt{ 0.1 };
  const Control u{ .25, 0.5 };
  const Params params{ 0.8, 0.8, 0.8 };
  // const StateDot xdotDesired{ 0.4975, 0.0499, 0.15 };  // Using Beta=0.1;Vin=0.5; w=0.1

  // Check jacobians
  Eigen::Matrix<double, 3, 3> actualHxd0, expectedHxd0;
  Eigen::Matrix<double, 3, 1> actualHdt, expectedHdt;
  Eigen::Matrix<double, 3, 2> actualHu, expectedHu;
  Eigen::Matrix<double, 3, 3> actualHparams, expectedHparams;

  std::function<gtsam::Vector(const StateDot&, const double&, const Control&, const Params&)> pred_proxy =
      [](const StateDot& xd0, const double& dt, const Control& u, const Params& params) {
        return MushrCtrl::predict(xd0, dt, u, params);
      };

  MushrCtrl::predict(xd0, dt, u, params, &actualHxd0, &actualHdt, &actualHu, &actualHparams);
  expectedHxd0 = gtsam::numericalDerivative41(pred_proxy, xd0, dt, u, params);
  expectedHdt = gtsam::numericalDerivative42(pred_proxy, xd0, dt, u, params);
  expectedHu = gtsam::numericalDerivative43(pred_proxy, xd0, dt, u, params);
  expectedHparams = gtsam::numericalDerivative44(pred_proxy, xd0, dt, u, params);

  // PRX_DBG_VARS(expectedHxd0);
  // PRX_DBG_VARS(actualHxd0);

  // PRX_DBG_VARS(expectedHdt);
  // PRX_DBG_VARS(actualHdt);

  // PRX_DBG_VARS(expectedHu);
  // PRX_DBG_VARS(actualHu);

  // PRX_DBG_VARS(expectedHparams);
  // PRX_DBG_VARS(actualHparams);

  const bool expectedHxd0_isApprox_actualHxd0{ expectedHxd0.isApprox(actualHxd0, tolerance) };
  const bool expectedHdt_isApprox_actualHdt{ expectedHdt.isApprox(actualHdt, tolerance) };
  const bool expectedHu_isApprox_actualHu{ expectedHu.isApprox(actualHu, tolerance) };
  const bool expectedHparams_isApprox_actualHparams{ expectedHparams.isApprox(actualHparams, tolerance) };

  // PRX_DBG_VARS(expectedHxd0_isApprox_actualHxd0);
  // PRX_DBG_VARS(expectedHdt_isApprox_actualHdt);
  // PRX_DBG_VARS(expectedHu_isApprox_actualHu);
  // PRX_DBG_VARS(expectedHparams_isApprox_actualHparams);

  ASSERT_TRUE(expectedHxd0.isApprox(actualHxd0, tolerance));
  ASSERT_TRUE(prx::are_matrices_approx_equal(expectedHdt, actualHdt, tolerance));
  // ASSERT_TRUE(expectedHdt.isApprox(actualHdt, tolerance));
  ASSERT_TRUE(expectedHu.isApprox(actualHu, tolerance));
  ASSERT_TRUE(expectedHparams.isApprox(actualHparams, tolerance));
}

TEST(TestMushrFactors, testMushrCtrlXddot01EvaluateError)
{
  using StateDot = prx_models::mushr_types::StateDot::type;
  using Control = prx_models::mushr_types::Control::type;
  using MushrCtrl = prx_models::mushr_u_xddot01_t;
  using Params = prx_models::mushr_types::Ubar::params;

  const StateDot xd0{ 0.0, 0.0, 0.0 };  // Using Beta=0.2;Vin=0.25; w=0.1
  const double dt{ 0.5 };
  const Control u{ .25, 0.5 };
  const Params params{ 0.8, 0.8, 0.8 };
  const StateDot xd1{ MushrCtrl::predict(xd0, dt, u, params) };
  // const StateDot xdotDesired{ 0.4975, 0.0499, 0.15 };  // Using Beta=0.1;Vin=0.5; w=0.1

  // Check jacobians
  Eigen::MatrixXd actualHxd1, expectedHxd1;
  Eigen::MatrixXd actualHxd0, expectedHxd0;
  Eigen::MatrixXd actualHdt, expectedHdt;
  Eigen::MatrixXd actualHu, expectedHu;

  const MushrCtrl factor(1, 0, 2, 3, params, nullptr);

  std::function<gtsam::Vector(const StateDot&, const StateDot&, const double&, const Control&)> err_proxy =
      [&factor](const StateDot& xd1, const StateDot& xd0, const double& dt, const Control& u) {
        return factor.evaluateError(xd1, xd0, dt, u);
      };

  factor.evaluateError(xd1, xd0, dt, u, actualHxd1, actualHxd0, actualHdt, actualHu);
  expectedHxd1 = gtsam::numericalDerivative41(err_proxy, xd1, xd0, dt, u);
  expectedHxd0 = gtsam::numericalDerivative42(err_proxy, xd1, xd0, dt, u);
  expectedHdt = gtsam::numericalDerivative43(err_proxy, xd1, xd0, dt, u);
  expectedHu = gtsam::numericalDerivative44(err_proxy, xd1, xd0, dt, u);

  // PRX_DBG_VARS(expectedHxd0);
  // PRX_DBG_VARS(actualHxd0);

  PRX_DBG_VARS(expectedHdt);
  PRX_DBG_VARS(actualHdt);

  // PRX_DBG_VARS(expectedHu);
  // PRX_DBG_VARS(actualHu);

  // PRX_DBG_VARS(expectedHparams);
  // PRX_DBG_VARS(actualHparams);

  const bool expectedHxd1_isApprox_actualHxd1{ expectedHxd1.isApprox(actualHxd1, tolerance) };
  const bool expectedHxd0_isApprox_actualHxd0{ expectedHxd0.isApprox(actualHxd0, tolerance) };
  const bool expectedHdt_isApprox_actualHdt{ expectedHdt.isApprox(actualHdt, tolerance) };
  const bool expectedHu_isApprox_actualHu{ expectedHu.isApprox(actualHu, tolerance) };

  PRX_DBG_VARS(expectedHxd1_isApprox_actualHxd1);
  PRX_DBG_VARS(expectedHxd0_isApprox_actualHxd0);
  PRX_DBG_VARS(expectedHdt_isApprox_actualHdt);
  PRX_DBG_VARS(expectedHu_isApprox_actualHu);

  ASSERT_TRUE(expectedHxd1.isApprox(actualHxd1, tolerance));
  ASSERT_TRUE(expectedHxd0.isApprox(actualHxd0, tolerance));
  ASSERT_TRUE(prx::are_matrices_approx_equal(expectedHdt, actualHdt, tolerance));
  // ASSERT_TRUE(expectedHdt.isApprox(actualHdt, tolerance));
  ASSERT_TRUE(expectedHu.isApprox(actualHu, tolerance));
}

TEST(TestMushrFactors, testMushrCtrlAccel)
{
  using StateDot = prx_models::mushr_types::StateDot::type;
  using Control = prx_models::mushr_types::Control::type;
  using MushrCtrl = prx_models::mushr_CtrlAccel_t;
  using Params = prx_models::mushr_types::Control::params;

  const StateDot xd0{ 0.0, 0.0, 0.0 };  // Using Beta=0.2;Vin=0.25; w=0.1
  const double dt{ 0.5 };
  const Control u{ .25, 0.5 };
  const Params params{ 0.2, 0.5 };
  const StateDot xd1{ MushrCtrl::predict(xd0, u, dt, params) };
  // const StateDot xdotDesired{ 0.4975, 0.0499, 0.15 };  // Using Beta=0.1;Vin=0.5; w=0.1

  // Check jacobians
  Eigen::MatrixXd actualHxd1, expectedHxd1;
  Eigen::MatrixXd actualHxd0, expectedHxd0;
  Eigen::MatrixXd actualHdt, expectedHdt;
  Eigen::MatrixXd actualHu, expectedHu;

  const MushrCtrl factor(1, 0, 2, 3, nullptr, params);

  std::function<gtsam::Vector(const StateDot&, const StateDot&, const Control&, const double&)> err_proxy =
      [&factor, &params](const StateDot& xd1, const StateDot& xd0, const Control& u, const double& dt) {
        return factor.evaluateError(xd1, xd0, u, dt);
      };

  factor.evaluateError(xd1, xd0, u, dt, actualHxd1, actualHxd0, actualHu, actualHdt);
  expectedHxd1 = gtsam::numericalDerivative41(err_proxy, xd1, xd0, u, dt);
  expectedHxd0 = gtsam::numericalDerivative42(err_proxy, xd1, xd0, u, dt);
  expectedHu = gtsam::numericalDerivative43(err_proxy, xd1, xd0, u, dt);
  expectedHdt = gtsam::numericalDerivative44(err_proxy, xd1, xd0, u, dt);

  // PRX_DBG_VARS(expectedHxd0);
  // PRX_DBG_VARS(actualHxd0);

  PRX_DBG_VARS(expectedHdt);
  PRX_DBG_VARS(actualHdt);

  PRX_DBG_VARS(expectedHu);
  PRX_DBG_VARS(actualHu);

  // PRX_DBG_VARS(expectedHparams);
  // PRX_DBG_VARS(actualHparams);

  const bool expectedHxd1_isApprox_actualHxd1{ expectedHxd1.isApprox(actualHxd1, tolerance) };
  const bool expectedHxd0_isApprox_actualHxd0{ expectedHxd0.isApprox(actualHxd0, tolerance) };
  const bool expectedHdt_isApprox_actualHdt{ expectedHdt.isApprox(actualHdt, tolerance) };
  const bool expectedHu_isApprox_actualHu{ expectedHu.isApprox(actualHu, tolerance) };

  PRX_DBG_VARS(expectedHxd1_isApprox_actualHxd1);
  PRX_DBG_VARS(expectedHxd0_isApprox_actualHxd0);
  PRX_DBG_VARS(expectedHdt_isApprox_actualHdt);
  PRX_DBG_VARS(expectedHu_isApprox_actualHu);

  ASSERT_TRUE(expectedHxd1.isApprox(actualHxd1, tolerance));
  ASSERT_TRUE(expectedHxd0.isApprox(actualHxd0, tolerance));
  ASSERT_TRUE(prx::are_matrices_approx_equal(expectedHdt, actualHdt, tolerance));
  // ASSERT_TRUE(expectedHdt.isApprox(actualHdt, tolerance));
  ASSERT_TRUE(expectedHu.isApprox(actualHu, tolerance));
}

TEST(TestMushrFactors, testMushrNHC)
{
  using StateDot = prx_models::mushr_types::StateDot::type;
  using Control = prx_models::mushr_types::Control::type;
  using MushrFactor = prx_models::mushr_NHC_t;
  using Params = prx_models::mushr_types::Control::params;

  const StateDot xd{ 0.049, -0.007, -0.042 };
  const Control u{ 0.1, -0.5 };
  const Params params{ 0.01, 0.52 };
  // const StateDot xdotDesired{ 0.4975, 0.0499, 0.15 };  // Using Beta=0.1;Vin=0.5; w=0.1

  // Check jacobians
  Eigen::MatrixXd actualHxd, expectedHxd;
  Eigen::MatrixXd actualHu, expectedHu;

  const MushrFactor factor(1, 0, nullptr, params);

  std::function<gtsam::Vector(const StateDot&, const Control&)> err_proxy =
      [&factor, &params](const StateDot& xd, const Control& u) { return factor.evaluateError(xd, u); };

  factor.evaluateError(xd, u, actualHxd, actualHu);
  expectedHxd = gtsam::numericalDerivative21(err_proxy, xd, u);
  expectedHu = gtsam::numericalDerivative22(err_proxy, xd, u);

  PRX_DBG_VARS(expectedHxd);
  PRX_DBG_VARS(actualHxd);

  PRX_DBG_VARS(expectedHu);
  PRX_DBG_VARS(actualHu);

  // PRX_DBG_VARS(expectedHparams);
  // PRX_DBG_VARS(actualHparams);

  const bool expectedHxd_isApprox_actualHxd{ expectedHxd.isApprox(actualHxd, tolerance) };
  const bool expectedHu_isApprox_actualHu{ expectedHu.isApprox(actualHu, tolerance) };

  PRX_DBG_VARS(expectedHxd_isApprox_actualHxd);
  PRX_DBG_VARS(expectedHu_isApprox_actualHu);

  ASSERT_TRUE(expectedHxd.isApprox(actualHxd, tolerance));
  ASSERT_TRUE(expectedHu.isApprox(actualHu, tolerance));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}