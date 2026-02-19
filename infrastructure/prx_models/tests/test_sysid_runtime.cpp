// ============================================================================
// SysidRuntime Usage Reference & Tests
// ============================================================================
//
// This file serves as both a test suite and a usage reference for
// torch_bridge::SysidRuntimeBase — the C++ interface for running
// TorchScript sysid models exported from Python.
//
// QUICK START
// -----------
//
// 1. Include headers:
//
//      #include <torch_bridge/sysid_runtime.hpp>
//      #include <prx_models/mushr_factors.hpp>
//
// 2. Define StructuredParams (maps plant parameter indices):
//
//      struct StructuredParams {
//        static constexpr std::size_t friction    = mushr_types::Control::friction;
//        static constexpr std::size_t vel_desired = mushr_types::Control::vel_desired;
//        static constexpr double L                = mushr_types::Parameters::L;
//      };
//
// 3. Create a runtime (handles both direct and structured .ts.pt files):
//
//      Params params;  // plant params [accel_gain, vel_desired_gain, friction, delta_offset, delta_gain]
//      params << 1.0, 1.0, 1.0, 0.0, 1.0;
//      Poly poly;      // steering polynomial [c0, c1, c2, c3]
//      poly << 0.0, 0.0, 1.0, 0.0;
//
//      auto runtime = torch_bridge::create_sysid_runtime<
//          MushrPlant, Params, Poly, StructuredParams>(
//          "model.ts.pt", params, poly, /*use_cuda=*/false, /*dtype=*/"float32");
//
// 4. Predict (raw xd0 in, raw xd1 out):
//
//      Eigen::Vector3d xd1 = runtime->predict(xd0, u);
//
// 5. Predict with Jacobians:
//
//      auto [xd1, Jx, Ju] = runtime->predict_with_jac(xd0, u);
//
// 6. Convenience call() — dispatches based on optional Jacobian refs:
//
//      Eigen::Vector3d xd1 = runtime->call(xd0, u);           // predict only
//      Eigen::Vector3d xd1 = runtime->call(xd0, u, Jx, Ju);   // with Jacobians
//
// 7. For direct-only callers that don't have plant headers:
//
//      auto runtime = torch_bridge::create_sysid_runtime(
//          "direct_model.ts.pt", /*use_cuda=*/false, /*dtype=*/"float32");
//
// ============================================================================

#include <gtest/gtest.h>
#include <ros/package.h>
#include <Eigen/Dense>

#ifndef TORCH_NOT_BUILT
#include <torch_bridge/sysid_runtime.hpp>
#include <prx_models/mushr_factors.hpp>

// ---------------------------------------------------------------------------
// Type aliases (same as any caller would define)
// ---------------------------------------------------------------------------
using StateDot = Eigen::Vector3d;
using Control = Eigen::Vector2d;
using JacX = Eigen::Matrix3d;
using JacU = Eigen::Matrix<double, 3, 2>;
using Params = prx_models::mushr_types::Control::params;
using Poly = prx_models::mushr_types::Control::Poly;
using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

// Maps plant parameter indices for StructuredSysidRuntime.
// Required by create_sysid_runtime when structured models are possible.
struct StructuredParams
{
  static constexpr std::size_t friction{ prx_models::mushr_types::Control::friction };
  static constexpr std::size_t vel_desired{ prx_models::mushr_types::Control::vel_desired };
  static constexpr double L{ prx_models::mushr_types::Parameters::L };
};

// Default plant parameters (identity — no physical calibration).
static Params default_params()
{
  Params p;
  p << 1.0, 1.0, 1.0, 0.0, 1.0;
  return p;
}

// Default steering polynomial (identity: delta = x).
static Poly default_poly()
{
  Poly p;
  p << 0.0, 0.0, 1.0, 0.0;
  return p;
}

// ---------------------------------------------------------------------------
// Resolve model paths relative to the prx_models package
// ---------------------------------------------------------------------------
static std::string models_dir()
{
  return ros::package::getPath("prx_models") + "/models/learned_mushr";
}

static std::string direct_model_path()
{
  return models_dir() + "/D08_h10_w1_direct_model.ts.pt";
}

static std::string structured_model_path()
{
  return models_dir() + "/S06_rollout_w1_structured_aux.ts.pt";
}

// ============================================================================
// Helper: numerical Jacobian via central differences
// ============================================================================
static void numerical_jacobians(torch_bridge::SysidRuntimeBase& runtime, const StateDot& xd0, const Control& u,
                                JacX& Jx_num, JacU& Ju_num, double eps = 5e-3)
{
  for (int j = 0; j < 3; ++j)
  {
    StateDot xd_plus = xd0, xd_minus = xd0;
    xd_plus[j] += eps;
    xd_minus[j] -= eps;
    Jx_num.col(j) = (runtime.predict(xd_plus, u) - runtime.predict(xd_minus, u)) / (2 * eps);
  }
  for (int j = 0; j < 2; ++j)
  {
    Control u_plus = u, u_minus = u;
    u_plus[j] += eps;
    u_minus[j] -= eps;
    Ju_num.col(j) = (runtime.predict(xd0, u_plus) - runtime.predict(xd0, u_minus)) / (2 * eps);
  }
}

// ============================================================================
// Test fixture: Direct model (end-to-end MLP)
// ============================================================================
class DirectModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // The recommended way: use the template factory.
    // It reads embedded metadata to detect model_type and dtype automatically.
    runtime_ = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
        direct_model_path(), default_params(), default_poly(), /*use_cuda=*/false);
  }

  std::unique_ptr<torch_bridge::SysidRuntimeBase> runtime_;
};

TEST_F(DirectModelTest, Predict)
{
  StateDot xd0(1.0, 0.1, 0.5);
  Control u(0.5, 0.3);

  StateDot xd1 = runtime_->predict(xd0, u);

  EXPECT_TRUE(xd1.allFinite()) << "Output contains NaN or Inf: " << xd1.transpose();
  EXPECT_LT(xd1.norm(), 100.0) << "Output unreasonably large: " << xd1.transpose();
}

TEST_F(DirectModelTest, PredictWithJacobian)
{
  StateDot xd0(0.8, -0.05, 0.3);
  Control u(0.4, -0.2);

  auto [xd1, Jx, Ju] = runtime_->predict_with_jac(xd0, u);

  EXPECT_TRUE(xd1.allFinite());
  EXPECT_TRUE(Jx.allFinite());
  EXPECT_TRUE(Ju.allFinite());

  // Forward value must match predict()
  StateDot xd1_predict = runtime_->predict(xd0, u);
  EXPECT_TRUE(xd1.isApprox(xd1_predict, 1e-10))
      << "predict() and predict_with_jac() forward values differ:\n"
      << "  predict:          " << xd1_predict.transpose() << "\n"
      << "  predict_with_jac: " << xd1.transpose();
}

TEST_F(DirectModelTest, JacobianNumericalCheck)
{
  StateDot xd0(0.8, -0.05, 0.3);
  Control u(0.4, -0.2);

  auto [xd1, Jx, Ju] = runtime_->predict_with_jac(xd0, u);

  JacX Jx_num;
  JacU Ju_num;
  numerical_jacobians(*runtime_, xd0, u, Jx_num, Ju_num);

  // Relative error: |a-b| / max(|a|, |b|, 1)
  auto rel_err = [](const auto& a, const auto& b) {
    auto diff = (a - b).cwiseAbs().array();
    auto scale = a.cwiseAbs().cwiseMax(b.cwiseAbs()).array().max(1.0);
    return (diff / scale).maxCoeff();
  };

  EXPECT_LT(rel_err(Jx, Jx_num), 0.02) << "Jx analytic:\n" << Jx << "\nJx numerical:\n" << Jx_num;
  EXPECT_LT(rel_err(Ju, Ju_num), 0.02) << "Ju analytic:\n" << Ju << "\nJu numerical:\n" << Ju_num;
}

TEST_F(DirectModelTest, Deterministic)
{
  StateDot xd0(0.5, 0.0, 1.0);
  Control u(0.25, 0.1);

  StateDot a = runtime_->predict(xd0, u);
  StateDot b = runtime_->predict(xd0, u);
  StateDot c = runtime_->predict(xd0, u);

  EXPECT_TRUE(a.isApprox(b, 1e-12));
  EXPECT_TRUE(b.isApprox(c, 1e-12));
}

TEST_F(DirectModelTest, CallConvenienceAPI)
{
  StateDot xd0(0.3, -0.1, 0.7);
  Control u(0.2, -0.3);

  // call() without Jacobians — equivalent to predict()
  StateDot xd1 = runtime_->call(xd0, u);
  EXPECT_TRUE(xd1.isApprox(runtime_->predict(xd0, u), 1e-12));

  // call() with Jacobians — equivalent to predict_with_jac()
  JacX Jx;
  JacU Ju;
  StateDot xd1_jac = runtime_->call(xd0, u, Jx, Ju);

  auto [xd1_ref, Jx_ref, Ju_ref] = runtime_->predict_with_jac(xd0, u);
  EXPECT_TRUE(xd1_jac.isApprox(xd1_ref, 1e-12));
  EXPECT_TRUE(Jx.isApprox(Jx_ref, 1e-12));
  EXPECT_TRUE(Ju.isApprox(Ju_ref, 1e-12));
}

// ============================================================================
// Test fixture: Structured model (physics-informed with learned corrections)
// ============================================================================
class StructuredModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Same factory call — automatically detects "structured" from embedded metadata
    runtime_ = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
        structured_model_path(), default_params(), default_poly(), /*use_cuda=*/false);
  }

  std::unique_ptr<torch_bridge::SysidRuntimeBase> runtime_;
};

TEST_F(StructuredModelTest, Predict)
{
  StateDot xd0(0.5, 0.05, -0.3);
  Control u(-0.1, 0.4);

  StateDot xd1 = runtime_->predict(xd0, u);

  EXPECT_TRUE(xd1.allFinite()) << "Output contains NaN or Inf: " << xd1.transpose();
  EXPECT_LT(xd1.norm(), 1000.0) << "Output unreasonably large: " << xd1.transpose();
}

TEST_F(StructuredModelTest, PredictWithJacobian)
{
  StateDot xd0(0.5, 0.05, -0.3);
  Control u(-0.1, 0.4);

  auto [xd1, Jx, Ju] = runtime_->predict_with_jac(xd0, u);

  EXPECT_TRUE(xd1.allFinite());
  EXPECT_TRUE(Jx.allFinite());
  EXPECT_TRUE(Ju.allFinite());

  // Forward value must match predict()
  StateDot xd1_predict = runtime_->predict(xd0, u);
  EXPECT_TRUE(xd1.isApprox(xd1_predict, 1e-10))
      << "predict() and predict_with_jac() forward values differ:\n"
      << "  predict:          " << xd1_predict.transpose() << "\n"
      << "  predict_with_jac: " << xd1.transpose();
}

TEST_F(StructuredModelTest, JacobianNumericalCheck)
{
  StateDot xd0(0.5, 0.05, -0.3);
  Control u(-0.1, 0.4);

  auto [xd1, Jx, Ju] = runtime_->predict_with_jac(xd0, u);

  JacX Jx_num;
  JacU Ju_num;
  numerical_jacobians(*runtime_, xd0, u, Jx_num, Ju_num);

  auto rel_err = [](const auto& a, const auto& b) {
    auto diff = (a - b).cwiseAbs().array();
    auto scale = a.cwiseAbs().cwiseMax(b.cwiseAbs()).array().max(1.0);
    return (diff / scale).maxCoeff();
  };

  EXPECT_LT(rel_err(Jx, Jx_num), 0.02) << "Jx analytic:\n" << Jx << "\nJx numerical:\n" << Jx_num;
  EXPECT_LT(rel_err(Ju, Ju_num), 0.02) << "Ju analytic:\n" << Ju << "\nJu numerical:\n" << Ju_num;
}

TEST_F(StructuredModelTest, Deterministic)
{
  StateDot xd0(0.5, 0.05, -0.3);
  Control u(-0.1, 0.4);

  StateDot a = runtime_->predict(xd0, u);
  StateDot b = runtime_->predict(xd0, u);
  StateDot c = runtime_->predict(xd0, u);

  EXPECT_TRUE(a.isApprox(b, 1e-12));
  EXPECT_TRUE(b.isApprox(c, 1e-12));
}

TEST_F(StructuredModelTest, CallConvenienceAPI)
{
  StateDot xd0(0.3, -0.1, 0.7);
  Control u(0.2, -0.3);

  StateDot xd1 = runtime_->call(xd0, u);
  EXPECT_TRUE(xd1.isApprox(runtime_->predict(xd0, u), 1e-12));

  JacX Jx;
  JacU Ju;
  StateDot xd1_jac = runtime_->call(xd0, u, Jx, Ju);

  auto [xd1_ref, Jx_ref, Ju_ref] = runtime_->predict_with_jac(xd0, u);
  EXPECT_TRUE(xd1_jac.isApprox(xd1_ref, 1e-12));
  EXPECT_TRUE(Jx.isApprox(Jx_ref, 1e-12));
  EXPECT_TRUE(Ju.isApprox(Ju_ref, 1e-12));
}

// ============================================================================
// Factory and metadata tests
// ============================================================================
class FactoryTest : public ::testing::Test
{
};

// The template factory auto-detects model type from embedded metadata.
// A single code path handles both direct and structured .ts.pt files.
TEST_F(FactoryTest, DirectViaTemplateFactory)
{
  auto runtime = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
      direct_model_path(), default_params(), default_poly(), /*use_cuda=*/false);

  StateDot xd1 = runtime->predict(StateDot(0.5, 0.1, 0.3), Control(0.2, -0.1));
  EXPECT_TRUE(xd1.allFinite());
}

TEST_F(FactoryTest, StructuredViaTemplateFactory)
{
  auto runtime = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
      structured_model_path(), default_params(), default_poly(), /*use_cuda=*/false);

  StateDot xd1 = runtime->predict(StateDot(0.5, 0.1, 0.3), Control(0.2, -0.1));
  EXPECT_TRUE(xd1.allFinite());
}

// The non-template factory works for direct models only (no plant headers needed).
// Throws if given a structured model.
TEST_F(FactoryTest, DirectViaNonTemplateFactory)
{
  auto runtime = torch_bridge::create_sysid_runtime(direct_model_path(), /*use_cuda=*/false);

  StateDot xd1 = runtime->predict(StateDot(0.5, 0.1, 0.3), Control(0.2, -0.1));
  EXPECT_TRUE(xd1.allFinite());
}

TEST_F(FactoryTest, NonTemplateFactoryRejectsStructured)
{
  EXPECT_THROW(torch_bridge::create_sysid_runtime(structured_model_path(), /*use_cuda=*/false), std::runtime_error);
}

TEST_F(FactoryTest, ReadModelMetadata)
{
  auto direct_meta = torch_bridge::read_model_meta(direct_model_path());
  EXPECT_EQ(direct_meta.model_type, "direct");
  EXPECT_FALSE(direct_meta.dtype.empty());

  auto structured_meta = torch_bridge::read_model_meta(structured_model_path());
  EXPECT_EQ(structured_meta.model_type, "structured");
  EXPECT_FALSE(structured_meta.dtype.empty());
}

TEST_F(FactoryTest, HasJacobianMethod)
{
  auto direct = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
      direct_model_path(), default_params(), default_poly(), /*use_cuda=*/false);
  EXPECT_TRUE(direct->has_jacobian_method());

  auto structured = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
      structured_model_path(), default_params(), default_poly(), /*use_cuda=*/false);
  EXPECT_TRUE(structured->has_jacobian_method());
}

// ============================================================================
// Polymorphism: same code path for both model types
// ============================================================================
class PolymorphismTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(PolymorphismTest, PredictThroughBasePointer)
{
  // This is the key pattern: a single std::unique_ptr<SysidRuntimeBase>
  // works identically regardless of whether the .ts.pt file is direct or structured.
  auto runtime = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
      GetParam(), default_params(), default_poly(), /*use_cuda=*/false);

  StateDot xd0(0.3, -0.2, 0.8);
  Control u(0.1, -0.4);

  StateDot xd1 = runtime->predict(xd0, u);
  EXPECT_TRUE(xd1.allFinite());

  auto [xd1_jac, Jx, Ju] = runtime->predict_with_jac(xd0, u);
  EXPECT_TRUE(xd1.isApprox(xd1_jac, 1e-10));
  EXPECT_TRUE(Jx.allFinite());
  EXPECT_TRUE(Ju.allFinite());
}

INSTANTIATE_TEST_SUITE_P(BothModels, PolymorphismTest,
                         ::testing::Values(direct_model_path(), structured_model_path()));

#else

TEST(SysidRuntimeTest, TorchNotBuilt)
{
  GTEST_SKIP() << "Torch not built, skipping sysid runtime tests";
}

#endif

int main(int argc, char** argv)
{
#ifndef TORCH_NOT_BUILT
  torch::set_num_threads(1);
  torch::set_num_interop_threads(1);
#endif
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
