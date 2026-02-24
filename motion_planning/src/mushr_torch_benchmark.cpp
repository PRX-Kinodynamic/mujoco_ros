#ifndef TORCH_NOT_BUILT
#include <chrono>
#include <cstdlib>
#include <numeric>
#include <iomanip>
#include <filesystem>

#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <random>

#include <torch_bridge/query_utils.hpp>
#include <torch_bridge/sysid_runtime.hpp>
#include <prx_models/mushr_factors.hpp>

// namespace fs = std::filesystem;

class SysidBenchmark
{
  struct StructuredParams
  {
    static constexpr std::size_t friction{ prx_models::mushr_types::Control::friction };
    static constexpr std::size_t vel_desired{ prx_models::mushr_types::Control::vel_desired };
    static constexpr double L{ prx_models::mushr_types::Parameters::L };
  };

public:
  using StateDot = Eigen::Vector3d;
  using Control = Eigen::Vector2d;
  using JacX = Eigen::Matrix3d;
  using JacU = Eigen::Matrix<double, 3, 2>;
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  SysidBenchmark(ros::NodeHandle& nh)
    : verbose_(false)
    , total_calls_(100)
    , warmup_calls_(10)
    , use_ros_service_(false)
    , dtype_("float32")
    , use_cuda_(false)
  {
    nh.getParam("verbose", verbose_);
    nh.getParam("total_calls", total_calls_);
    nh.getParam("warmup_calls", warmup_calls_);
    nh.getParam("use_ros_service", use_ros_service_);
    nh.getParam("use_cuda", use_cuda_);

    // model_path (.ts.pt file) is required
    std::string model_path;
    if (!nh.getParam("model_path", model_path) || model_path.empty())
    {
      throw std::runtime_error("model_path parameter is required (path to .ts.pt file)");
    }

    // Read metadata embedded in the model file
    auto meta = torch_bridge::read_model_meta(model_path);
    dtype_ = meta.dtype;

    // Allow param overrides
    nh.getParam("dtype", dtype_);
    if (dtype_ != "float32" && dtype_ != "float64")
    {
      ROS_WARN("Invalid dtype '%s', must be 'float32' or 'float64'. Defaulting to 'float32'", dtype_.c_str());
      dtype_ = "float32";
    }

    // Default plant params: [accel_gain, vel_desired_gain, friction, delta_offset, delta_gain]
    std::vector<double> params_vec = { 1.0, 1.0, 1.0, 0.0, 1.0 };
    nh.getParam("plant_params", params_vec);
    if (params_vec.size() != 5)
    {
      ROS_WARN("plant_params must have 5 elements, using defaults");
      params_vec = { 1.0, 1.0, 1.0, 0.0, 1.0 };
    }
    Params params;
    for (int i = 0; i < 5; ++i)
      params[i] = params_vec[i];

    // Default steering polynomial coefficients [c0, c1, c2, c3] for poly(x) = c0*x^3 + c1*x^2 + c2*x + c3
    std::vector<double> poly_vec = { 0.0, 0.0, 1.0, 0.0 };
    nh.getParam("steering_poly", poly_vec);
    if (poly_vec.size() != 4)
    {
      ROS_WARN("steering_poly must have 4 elements, using defaults");
      poly_vec = { 0.0, 0.0, 1.0, 0.0 };
    }
    Poly poly;
    for (int i = 0; i < 4; ++i)
      poly[i] = poly_vec[i];

    ROS_INFO("Creating runtime from: %s", model_path.c_str());
    ROS_INFO("Model type: %s, dtype: %s, CUDA: %s", meta.model_type.c_str(), dtype_.c_str(), use_cuda_ ? "yes" : "no");

    // Canonical usage: create_sysid_runtime handles both direct and structured models
    runtime_ = torch_bridge::create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>(model_path, params, poly,
                                                                                              use_cuda_, dtype_);

    ROS_INFO("Successfully created SysidRuntime");

    if (use_ros_service_)
    {
      torch_service_client_ = nh.serviceClient<torch_bridge::TorchQuery>("/torch/service", true);
      torch_service_call_.request.inputs = 2;
      torch_service_call_.request.input_dimensions = { 3, 2 };
    }

    // Pre-generate inputs so the benchmark measures inference, not RNG overhead.
    generate_samples();
  }

  void run_ros_service_benchmark(bool with_jacobians)
  {
    if (!torch_service_client_.exists())
    {
      ROS_WARN("ROS Torch service not available, skipping ROS benchmark");
      return;
    }

    for (int i = 0; i < warmup_calls_; ++i)
    {
      set_sample(i);
      call_ros_service(with_jacobians);
    }

    std::vector<double> durations;
    for (int i = 0; i < total_calls_; ++i)
    {
      set_sample(warmup_calls_ + i);
      const auto start = std::chrono::high_resolution_clock::now();
      call_ros_service(with_jacobians);
      const auto end = std::chrono::high_resolution_clock::now();
      durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    print_stats("ROS Service", with_jacobians, durations);
  }

  void run_benchmark(bool with_jacobians)
  {
    for (int i = 0; i < warmup_calls_; ++i)
    {
      set_sample(i);
      call_runtime(with_jacobians);
    }

    std::vector<double> durations;
    for (int i = 0; i < total_calls_; ++i)
    {
      set_sample(warmup_calls_ + i);
      const auto start = std::chrono::high_resolution_clock::now();
      call_runtime(with_jacobians);
      const auto end = std::chrono::high_resolution_clock::now();
      durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    print_stats("In-Process", with_jacobians, durations);
  }

  void run_all()
  {
    std::cout << "\n========================================\n";
    std::cout << "Sysid Runtime Benchmark\n";
    std::cout << "Dtype: " << dtype_ << "\n";
    std::cout << "CUDA: " << (use_cuda_ ? "enabled" : "disabled") << "\n";
    std::cout << "Torch threads: " << torch::get_num_threads() << "\n";
    std::cout << "Total calls: " << total_calls_ << "\n";
    std::cout << "Warmup calls: " << warmup_calls_ << "\n";
    std::cout << "========================================\n\n";

    if (use_ros_service_ && torch_service_client_.exists())
    {
      run_ros_service_benchmark(false);
      run_ros_service_benchmark(true);
    }

    run_benchmark(false);
    if (runtime_->has_jacobian_method())
    {
      run_benchmark(true);
      check_jacobians();
    }
    else
    {
      ROS_INFO("Model does not have 'forward_with_jacobian' method, skipping Jacobian benchmark");
    }

    std::cout << "\n========================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "========================================\n";
  }

  void check_jacobians()
  {
    for (int i = 0; i < warmup_calls_; ++i)
    {
      set_sample(i);
      call_runtime(true);
    }

    using PartialXdot = std::function<StateDot(const StateDot&)>;
    using PartialCtrl = std::function<StateDot(const Control&)>;
    using DerivXdot = prx::math::first_order_derivative_t<PartialXdot, StateDot, 2, 0>;
    using DerivCtrl = prx::math::first_order_derivative_t<PartialCtrl, Control, 2, 0>;

    PartialXdot partial_xdot = [&](const StateDot& xd) { return runtime_->call(xd, ui_); };
    PartialCtrl partial_ctrl = [&](const Control& u) { return runtime_->call(xi_, u); };

    const double h{ 0.0001 };
    const DerivXdot derivative_xdot(partial_xdot, h);
    const DerivCtrl derivative_ctrl(partial_ctrl, h);

    std::vector<double> durations;
    double total_error_xdot{ 0.0 };
    double total_error_ctrl{ 0.0 };
    for (int i = 0; i < total_calls_; ++i)
    {
      set_sample(warmup_calls_ + i);

      const auto start = std::chrono::high_resolution_clock::now();
      const Eigen::Matrix<double, 3, 3> expectedHxd{ derivative_xdot(xi_) };
      const Eigen::Matrix<double, 3, 2> expectedHu{ derivative_ctrl(ui_) };
      const auto end = std::chrono::high_resolution_clock::now();

      auto [xd1p, actualHxd, actualHu] = runtime_->predict_with_jac(xi_, ui_);

      const double errXd{ (expectedHxd - actualHxd).norm() };
      const double errU{ (expectedHu - actualHu).norm() };
      total_error_xdot += errXd;
      total_error_ctrl += errU;

      if (errXd > 1.0)
      {
        DEBUG_VARS(expectedHxd);
        DEBUG_VARS(actualHxd);
      }
      durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    print_stats("Numerical Jacobian Validation", true, durations);
    std::cout << "Average Xdot error: " << total_error_xdot / total_calls_ << "\n";
    std::cout << "Average Ctrl error: " << total_error_ctrl / total_calls_ << "\n";
  }

private:
  void generate_samples()
  {
    const int n = warmup_calls_ + total_calls_;
    xi_samples_.resize(n);
    ui_samples_.resize(n);

    std::mt19937 rng(42);
    std::normal_distribution<double> dist_x(0.0, 1.0);
    std::normal_distribution<double> dist_u(0.0, 0.5);

    for (int i = 0; i < n; ++i)
    {
      xi_samples_[i] = StateDot(dist_x(rng), dist_x(rng), dist_x(rng));
      ui_samples_[i] = Control(dist_u(rng), dist_u(rng));
    }
  }

  void set_sample(int idx)
  {
    xi_ = xi_samples_[static_cast<size_t>(idx)];
    ui_ = ui_samples_[static_cast<size_t>(idx)];
  }

  void call_ros_service(bool with_jacobians)
  {
    torch_service_call_.request.compute_jacobians = with_jacobians;
    torch_service_call_.request.data.clear();
    torch_bridge::update_request(torch_service_call_, xi_, ui_);

    if (torch_service_client_.call(torch_service_call_))
    {
      torch_bridge::get_result(torch_service_call_, x_res_);
      if (with_jacobians)
      {
        torch_bridge::get_jacobian(torch_service_call_, dres_dx_, dres_du_);
      }
    }
  }

  void call_runtime(bool with_jacobians)
  {
    if (with_jacobians)
    {
      auto [xd1, Jx, Ju] = runtime_->predict_with_jac(xi_, ui_);
      x_res_ = xd1;
      dres_dx_ = Jx;
      dres_du_ = Ju;
    }
    else
    {
      x_res_ = runtime_->predict(xi_, ui_);
    }
  }

  void print_stats(const std::string& method, bool with_jacobians, const std::vector<double>& durations)
  {
    const auto [min_it, max_it] = std::minmax_element(durations.begin(), durations.end());
    const size_t min_idx = static_cast<size_t>(std::distance(durations.begin(), min_it));
    const size_t max_idx = static_cast<size_t>(std::distance(durations.begin(), max_it));
    const double sum = std::accumulate(durations.begin(), durations.end(), 0.0);
    const double mean = sum / durations.size();

    std::vector<double> sorted_durations = durations;
    std::sort(sorted_durations.begin(), sorted_durations.end());
    const double p50 = sorted_durations[sorted_durations.size() / 2];
    const double p95 = sorted_durations[static_cast<size_t>(sorted_durations.size() * 0.95)];
    const double p99 = sorted_durations[static_cast<size_t>(sorted_durations.size() * 0.99)];

    std::cout << method << (with_jacobians ? " [With Jacobians]" : " [Predict Only]") << ":\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Min:  " << std::setw(8) << *min_it << " ms (iter " << min_idx << ")\n";
    std::cout << "  Max:  " << std::setw(8) << *max_it << " ms (iter " << max_idx
              << (max_idx == 0 ? " - FIRST CALL" : "") << ")\n";
    std::cout << "  Mean: " << std::setw(8) << mean << " ms\n";
    std::cout << "  P50:  " << std::setw(8) << p50 << " ms\n";
    std::cout << "  P95:  " << std::setw(8) << p95 << " ms\n";
    std::cout << "  P99:  " << std::setw(8) << p99 << " ms\n";
    std::cout << "\n";
  }

  ros::ServiceClient torch_service_client_;
  torch_bridge::TorchQuery torch_service_call_;

  std::unique_ptr<torch_bridge::SysidRuntimeBase> runtime_;

  StateDot xi_;
  Control ui_;
  StateDot x_res_;
  JacX dres_dx_;
  JacU dres_du_;

  std::vector<StateDot> xi_samples_;
  std::vector<Control> ui_samples_;

  bool verbose_;
  int total_calls_;
  int warmup_calls_;
  bool use_ros_service_;
  std::string dtype_;
  bool use_cuda_;
};

int main(int argc, char** argv)
{
  const std::string node_name{ "SysidRuntimeBenchmark" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // Single-thread mode: reduces LibTorch threadpool overhead for tiny models.
  // For single-sample latency, thread management overhead can exceed computation time.
  bool single_thread_mode = true;
  nh.getParam("single_thread_mode", single_thread_mode);
  if (single_thread_mode)
  {
    torch::set_num_threads(1);
    torch::set_num_interop_threads(1);
    ROS_INFO("Single-thread mode enabled (torch threads=1, interop_threads=1)");
  }

  SysidBenchmark benchmark(nh);
  benchmark.run_all();

  // Flush output before exit
  std::cout << std::flush;
  std::cerr << std::flush;

// Use quick_exit to avoid PyTorch/LibTorch cleanup issues that cause segfault
// The benchmark results are valid, the crash only happens during library cleanup
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
