#include <gtest/gtest.h>
#include <ros/package.h>
#include <Eigen/Dense>

#ifndef TORCH_NOT_BUILT
#include <torch_bridge/sysid_runtime.hpp>

class SysidRuntimeTest : public ::testing::Test
{
protected:
  using StateDot = Eigen::Vector3d;
  using Control = Eigen::Vector2d;
  using JacX = Eigen::Matrix3d;
  using JacU = Eigen::Matrix<double, 3, 2>;

  void SetUp() override
  {
    model_path_ = std::getenv("SYSID_TEST_MODEL_PATH");
    if (model_path_.empty())
    {
      GTEST_SKIP() << "SYSID_TEST_MODEL_PATH not set, skipping test";
    }
  }

  std::string model_path_;
};

TEST_F(SysidRuntimeTest, DirectRuntimeLoads)
{
  ASSERT_NO_THROW({
    torch_bridge::DirectSysidRuntime runtime(model_path_);
  });
}

TEST_F(SysidRuntimeTest, DirectRuntimePredict)
{
  torch_bridge::DirectSysidRuntime runtime(model_path_);

  StateDot xd0;
  xd0 << 1.0, 0.1, 0.5;
  Control u;
  u << 0.5, 0.3;

  StateDot xd1 = runtime.predict(xd0, u);

  EXPECT_TRUE(xd1.allFinite()) << "Output contains NaN or Inf";
  EXPECT_LT(xd1.norm(), 100.0) << "Output seems unreasonably large";
}

TEST_F(SysidRuntimeTest, DirectRuntimePredictWithJacobian)
{
  torch_bridge::DirectSysidRuntime runtime(model_path_);

  StateDot xd0;
  xd0 << 1.0, 0.1, 0.5;
  Control u;
  u << 0.5, 0.3;

  auto [xd1, Jx, Ju] = runtime.predict_with_jac(xd0, u);

  EXPECT_TRUE(xd1.allFinite()) << "Output contains NaN or Inf";
  EXPECT_TRUE(Jx.allFinite()) << "Jx contains NaN or Inf";
  EXPECT_TRUE(Ju.allFinite()) << "Ju contains NaN or Inf";

  StateDot xd1_no_jac = runtime.predict(xd0, u);
  EXPECT_TRUE(xd1.isApprox(xd1_no_jac, 1e-10))
      << "predict() and predict_with_jac() outputs differ";
}

TEST_F(SysidRuntimeTest, DirectRuntimeJacobianNumericalCheck)
{
  torch_bridge::DirectSysidRuntime runtime(model_path_);

  StateDot xd0;
  xd0 << 0.8, -0.05, 0.3;
  Control u;
  u << 0.4, -0.2;

  auto [xd1, Jx_analytic, Ju_analytic] = runtime.predict_with_jac(xd0, u);

  const double eps = 1e-5;

  JacX Jx_numerical;
  for (int j = 0; j < 3; ++j)
  {
    StateDot xd0_plus = xd0;
    xd0_plus[j] += eps;
    StateDot xd0_minus = xd0;
    xd0_minus[j] -= eps;

    StateDot y_plus = runtime.predict(xd0_plus, u);
    StateDot y_minus = runtime.predict(xd0_minus, u);

    Jx_numerical.col(j) = (y_plus - y_minus) / (2 * eps);
  }

  JacU Ju_numerical;
  for (int j = 0; j < 2; ++j)
  {
    Control u_plus = u;
    u_plus[j] += eps;
    Control u_minus = u;
    u_minus[j] -= eps;

    StateDot y_plus = runtime.predict(xd0, u_plus);
    StateDot y_minus = runtime.predict(xd0, u_minus);

    Ju_numerical.col(j) = (y_plus - y_minus) / (2 * eps);
  }

  const double tol = 1e-4;
  double max_err_x = (Jx_analytic - Jx_numerical).cwiseAbs().maxCoeff();
  double max_err_u = (Ju_analytic - Ju_numerical).cwiseAbs().maxCoeff();

  EXPECT_LT(max_err_x, tol) << "Jx analytic vs numerical mismatch: " << max_err_x
                            << "\nAnalytic:\n" << Jx_analytic
                            << "\nNumerical:\n" << Jx_numerical;

  EXPECT_LT(max_err_u, tol) << "Ju analytic vs numerical mismatch: " << max_err_u
                            << "\nAnalytic:\n" << Ju_analytic
                            << "\nNumerical:\n" << Ju_numerical;
}

TEST_F(SysidRuntimeTest, DirectRuntimeDeterministic)
{
  torch_bridge::DirectSysidRuntime runtime(model_path_);

  StateDot xd0;
  xd0 << 0.5, 0.0, 1.0;
  Control u;
  u << 0.25, 0.1;

  StateDot xd1_a = runtime.predict(xd0, u);
  StateDot xd1_b = runtime.predict(xd0, u);
  StateDot xd1_c = runtime.predict(xd0, u);

  EXPECT_TRUE(xd1_a.isApprox(xd1_b, 1e-12)) << "Results not deterministic";
  EXPECT_TRUE(xd1_b.isApprox(xd1_c, 1e-12)) << "Results not deterministic";
}

#else

TEST(SysidRuntimeTest, TorchNotBuilt)
{
  GTEST_SKIP() << "Torch not built, skipping sysid runtime tests";
}

#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
