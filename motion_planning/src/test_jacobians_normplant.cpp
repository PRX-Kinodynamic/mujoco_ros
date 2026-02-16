#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>
#include <torch_bridge/sysid_runtime.hpp>
#include <prx_models/mushr_factors.hpp>

struct MushrStructuredParams
{
  static constexpr std::size_t friction = prx_models::mushr_types::Control::friction;
  static constexpr std::size_t vel_desired = prx_models::mushr_types::Control::vel_desired;
  static constexpr double L = prx_models::mushr_types::Parameters::L;
};

using Params = prx_models::mushr_types::Control::params;
using Poly = prx_models::mushr_types::Control::Poly;
using MushrPlant = prx_models::mushr_CtrlAccel_t<>;
using StructuredParams = MushrStructuredParams;
using StructuredSysidRuntime = torch_bridge::StructuredSysidRuntime<MushrPlant, Params, Poly, StructuredParams>;

using StateDot = Eigen::Vector3d;
using Control = Eigen::Vector2d;
using JacX = Eigen::Matrix3d;
using JacU = Eigen::Matrix<double, 3, 2>;

// Finite difference approximation of Jacobian
JacX compute_numerical_jac_x(StructuredSysidRuntime& runtime, const StateDot& xd0, const Control& u, double eps = 1e-6)
{
  JacX jac_numerical;
  StateDot f0 = runtime.predict(xd0, u);

  for (int i = 0; i < 3; ++i)
  {
    StateDot xd_plus = xd0;
    xd_plus(i) += eps;
    StateDot f_plus = runtime.predict(xd_plus, u);
    jac_numerical.col(i) = (f_plus - f0) / eps;
  }

  return jac_numerical;
}

JacU compute_numerical_jac_u(StructuredSysidRuntime& runtime, const StateDot& xd0, const Control& u, double eps = 1e-6)
{
  JacU jac_numerical;
  StateDot f0 = runtime.predict(xd0, u);

  for (int i = 0; i < 2; ++i)
  {
    Control u_plus = u;
    u_plus(i) += eps;
    StateDot f_plus = runtime.predict(xd0, u_plus);
    jac_numerical.col(i) = (f_plus - f0) / eps;
  }

  return jac_numerical;
}

void print_comparison(const std::string& name, const Eigen::MatrixXd& analytical, const Eigen::MatrixXd& numerical)
{
  std::cout << "\n" << name << ":" << std::endl;
  std::cout << "Analytical:\n" << analytical << std::endl;
  std::cout << "Numerical:\n" << numerical << std::endl;

  Eigen::MatrixXd diff = analytical - numerical;
  double max_abs_error = diff.cwiseAbs().maxCoeff();
  double rel_error = max_abs_error / (std::max(analytical.cwiseAbs().maxCoeff(), 1e-10));

  std::cout << "Max absolute error: " << max_abs_error << std::endl;
  std::cout << "Relative error: " << rel_error << std::endl;

  if (max_abs_error < 1e-4)
  {
    std::cout << "✓ PASS (error < 1e-4)" << std::endl;
  }
  else if (max_abs_error < 1e-3)
  {
    std::cout << "⚠ WARNING (error < 1e-3)" << std::endl;
  }
  else
  {
    std::cout << "✗ FAIL (error >= 1e-3)" << std::endl;
  }
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <path_to_structured_aux.ts.pt>" << std::endl;
    return 1;
  }

  std::string model_path = argv[1];

  std::cout << "=== Jacobian Validation for Normalized-Plant Mode ===" << std::endl;
  std::cout << "Model: " << model_path << std::endl;

  // Initialize runtime with normalized-plant mode
  Params params;
  params << 1.0, 1.0, 1.0, 0.0, 1.0;

  Poly poly;
  poly << 0.0, 0.0, 1.0, 0.0;

  bool use_cuda = false;
  bool use_normalized_plant = true;

  std::cout << "\nInitializing runtime..." << std::endl;
  StructuredSysidRuntime runtime(model_path, params, poly, use_cuda, "float64");

  // Test cases: various state and control values
  std::vector<std::pair<StateDot, Control>> test_cases = { { StateDot(0.5, 0.0, 0.0), Control(0.5, 0.0) },
                                                           { StateDot(1.0, 0.1, 0.2), Control(0.8, 0.3) },
                                                           { StateDot(0.2, -0.1, -0.3), Control(-0.5, -0.4) },
                                                           { StateDot(1.5, 0.3, 0.5), Control(1.0, 0.5) },
                                                           { StateDot(0.0, 0.0, 0.0), Control(0.0, 0.0) } };

  int passed = 0;
  int total = test_cases.size();

  for (size_t i = 0; i < test_cases.size(); ++i)
  {
    const auto& [xd0, u] = test_cases[i];

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "Test case " << (i + 1) << "/" << total << std::endl;
    std::cout << "xd0 = [" << xd0.transpose() << "]" << std::endl;
    std::cout << "u   = [" << u.transpose() << "]" << std::endl;

    // Get analytical Jacobians
    StateDot xd1_analytical;
    JacX jac_x_analytical;
    JacU jac_u_analytical;
    std::tie(xd1_analytical, jac_x_analytical, jac_u_analytical) = runtime.predict_with_jac(xd0, u);

    // Compute numerical Jacobians
    JacX jac_x_numerical = compute_numerical_jac_x(runtime, xd0, u);
    JacU jac_u_numerical = compute_numerical_jac_u(runtime, xd0, u);

    // Compare
    print_comparison("Jacobian w.r.t. xd0", jac_x_analytical, jac_x_numerical);
    print_comparison("Jacobian w.r.t. u", jac_u_analytical, jac_u_numerical);

    // Check if both passed
    double max_error_x = (jac_x_analytical - jac_x_numerical).cwiseAbs().maxCoeff();
    double max_error_u = (jac_u_analytical - jac_u_numerical).cwiseAbs().maxCoeff();

    if (max_error_x < 1e-3 && max_error_u < 1e-3)
    {
      passed++;
      std::cout << "\n✓ Test case " << (i + 1) << " PASSED" << std::endl;
    }
    else
    {
      std::cout << "\n✗ Test case " << (i + 1) << " FAILED" << std::endl;
    }
  }

  std::cout << "\n" << std::string(60, '=') << std::endl;
  std::cout << "Summary: " << passed << "/" << total << " test cases passed" << std::endl;

  if (passed == total)
  {
    std::cout << "✓ All Jacobian tests PASSED!" << std::endl;
    return 0;
  }
  else
  {
    std::cout << "✗ Some Jacobian tests FAILED" << std::endl;
    return 1;
  }
}
