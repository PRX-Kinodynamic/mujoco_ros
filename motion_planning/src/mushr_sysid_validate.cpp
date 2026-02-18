#ifndef TORCH_NOT_BUILT
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <random>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <torch/script.h>

#include <torch_bridge/sysid_runtime.hpp>
#include <prx_models/mushr_factors.hpp>

/**
 * Dumps C++ SysidRuntime predict() and predict_with_jac() outputs for
 * fixed test inputs so that a Python script can compare against the
 * original (non-TorchScript) model.
 *
 * Writes a text file with one sample per line:
 *   xi0 xi1 xi2 ui0 ui1 y0 y1 y2 [Jx(9 values row-major) Ju(6 values row-major)]
 */

namespace fs = std::filesystem;

struct StructuredParams
{
  static constexpr std::size_t friction{ prx_models::mushr_types::Control::friction };
  static constexpr std::size_t vel_desired{ prx_models::mushr_types::Control::vel_desired };
  static constexpr double L{ prx_models::mushr_types::Parameters::L };
};

using StateDot = Eigen::Vector3d;
using Control = Eigen::Vector2d;
using JacX = Eigen::Matrix3d;
using JacU = Eigen::Matrix<double, 3, 2>;
using Params = prx_models::mushr_types::Control::params;
using Poly = prx_models::mushr_types::Control::Poly;
using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SysidValidate");
  ros::NodeHandle nh("~");

  torch::set_num_threads(1);
  torch::set_num_interop_threads(1);

  std::string model_path;
  if (!nh.getParam("model_path", model_path) || model_path.empty())
  {
    ROS_ERROR("model_path parameter is required");
    return 1;
  }

  std::string output_file;
  if (!nh.getParam("output_file", output_file) || output_file.empty())
  {
    ROS_ERROR("output_file parameter is required");
    return 1;
  }

  int n_samples = 20;
  nh.getParam("n_samples", n_samples);

  std::string dtype;
  nh.getParam("dtype", dtype);

  // Default plant params
  std::vector<double> params_vec = { 1.0, 1.0, 1.0, 0.0, 1.0 };
  nh.getParam("plant_params", params_vec);
  Params params;
  for (int i = 0; i < 5; ++i)
    params[i] = params_vec[i];

  std::vector<double> poly_vec = { 0.0, 0.0, 1.0, 0.0 };
  nh.getParam("steering_poly", poly_vec);
  Poly poly;
  for (int i = 0; i < 4; ++i)
    poly[i] = poly_vec[i];

  auto meta = torch_bridge::read_model_meta(model_path);
  if (dtype.empty())
    dtype = meta.dtype;

  ROS_INFO("Model: %s", model_path.c_str());
  ROS_INFO("Type: %s, dtype: %s, dt: %f", meta.model_type.c_str(), dtype.c_str(), meta.dt);
  ROS_INFO("Output: %s, N=%d", output_file.c_str(), n_samples);

  auto runtime = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(
      model_path, params, poly, false, dtype);

  // Generate fixed test inputs
  std::mt19937 rng(42);
  std::normal_distribution<double> dist_x(0.0, 1.0);
  std::normal_distribution<double> dist_u(0.0, 0.5);

  std::vector<StateDot> xi_samples(n_samples);
  std::vector<Control> ui_samples(n_samples);
  for (int i = 0; i < n_samples; ++i)
  {
    xi_samples[i] = StateDot(dist_x(rng), dist_x(rng), dist_x(rng));
    ui_samples[i] = Control(dist_u(rng), dist_u(rng));
  }

  // Warmup
  for (int i = 0; i < 5; ++i)
  {
    runtime->predict(xi_samples[0], ui_samples[0]);
  }

  // Open output file
  std::ofstream ofs(output_file);
  if (!ofs.is_open())
  {
    ROS_ERROR("Cannot open output file: %s", output_file.c_str());
    return 1;
  }

  ofs << std::setprecision(15) << std::scientific;

  bool has_jac = runtime->has_jacobian_method();

  // Header
  ofs << "# C++ SysidRuntime validation dump\n";
  ofs << "# model_path: " << model_path << "\n";
  ofs << "# model_type: " << meta.model_type << "\n";
  ofs << "# dtype: " << dtype << "\n";
  ofs << "# dt: " << meta.dt << "\n";
  ofs << "# n_samples: " << n_samples << "\n";
  ofs << "# has_jacobians: " << (has_jac ? "true" : "false") << "\n";
  ofs << "# PREDICT section: xi0 xi1 xi2 ui0 ui1 y0 y1 y2\n";
  ofs << "PREDICT\n";

  for (int i = 0; i < n_samples; ++i)
  {
    StateDot y = runtime->predict(xi_samples[i], ui_samples[i]);
    ofs << xi_samples[i](0) << " " << xi_samples[i](1) << " " << xi_samples[i](2) << " "
        << ui_samples[i](0) << " " << ui_samples[i](1) << " "
        << y(0) << " " << y(1) << " " << y(2) << "\n";
  }

  if (has_jac)
  {
    // Warmup jacobians
    for (int i = 0; i < 5; ++i)
    {
      runtime->predict_with_jac(xi_samples[0], ui_samples[0]);
    }

    ofs << "# PREDICT_WITH_JAC section: xi0 xi1 xi2 ui0 ui1 y0 y1 y2 Jx(9 row-major) Ju(6 row-major)\n";
    ofs << "PREDICT_WITH_JAC\n";

    for (int i = 0; i < n_samples; ++i)
    {
      auto [y, Jx, Ju] = runtime->predict_with_jac(xi_samples[i], ui_samples[i]);
      ofs << xi_samples[i](0) << " " << xi_samples[i](1) << " " << xi_samples[i](2) << " "
          << ui_samples[i](0) << " " << ui_samples[i](1) << " "
          << y(0) << " " << y(1) << " " << y(2);

      // Jx row-major 3x3
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          ofs << " " << Jx(r, c);

      // Ju row-major 3x2
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 2; ++c)
          ofs << " " << Ju(r, c);

      ofs << "\n";
    }
  }

  ofs.close();
  ROS_INFO("Wrote %d samples to %s", n_samples, output_file.c_str());

  std::cout << std::flush;
  std::cerr << std::flush;

#ifndef __APPLE__
  std::quick_exit(EXIT_SUCCESS);
#else
  exit(0);
#endif
}

#else
int main()
{
  return 0;
}
#endif
