#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <c10/cuda/CUDAStream.h>

#include <torch_bridge/gpu_plant.hpp>
#include <prx_models/mushr_factors.hpp>

/**
 * @brief Benchmark comparing CPU analytical plant vs GPU plant dynamics.
 *
 * Usage:
 *   rosrun torch_bridge gpu_plant_benchmark _total_calls:=5000 _use_cuda:=true
 */
class GpuPlantBenchmark
{
public:
  using StateDot = Eigen::Vector3d;
  using Control = Eigen::Vector2d;
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;
  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  GpuPlantBenchmark(ros::NodeHandle& nh) : total_calls_(5000), warmup_calls_(100), use_cuda_(true), dtype_("float32")
  {
    nh.getParam("total_calls", total_calls_);
    nh.getParam("warmup_calls", warmup_calls_);
    nh.getParam("use_cuda", use_cuda_);
    nh.getParam("dtype", dtype_);

    // Default plant parameters
    params_[0] = 1.0;  // accel_gain
    params_[1] = 1.0;  // vel_desired_gain
    params_[2] = 1.0;  // friction
    params_[3] = 0.0;  // delta_offset
    params_[4] = 1.0;  // delta_gain

    // Default steering polynomial (identity: delta = deltaIn)
    poly_[0] = 0.0;
    poly_[1] = 0.0;
    poly_[2] = 1.0;
    poly_[3] = 0.0;

    dt_ = 0.1;
    L_ = prx_models::mushr_types::Parameters::L;

    // Initialize device
    if (use_cuda_ && torch::cuda::is_available())
    {
      device_ = torch::Device(torch::kCUDA);
      ROS_INFO("Using CUDA device");
    }
    else
    {
      device_ = torch::Device(torch::kCPU);
      use_cuda_ = false;
      ROS_INFO("Using CPU device");
    }

    // Set dtype
    if (dtype_ == "float32")
    {
      dtype_torch_ = torch::kFloat32;
    }
    else
    {
      dtype_torch_ = torch::kFloat64;
    }

    // Create GPU plant
    std::vector<double> poly_vec(poly_.begin(), poly_.end());
    gpu_plant_ = std::make_unique<torch_bridge::GpuPlant>(L_, poly_vec, dtype_torch_, device_);

    // Pre-allocate tensors
    xd0_gpu_ = torch::empty({3}, torch::TensorOptions().dtype(dtype_torch_).device(device_));
    u_eff_gpu_ = torch::empty({2}, torch::TensorOptions().dtype(dtype_torch_).device(device_));
    residual_gpu_ = torch::zeros({3}, torch::TensorOptions().dtype(dtype_torch_).device(device_));
    xd1_gpu_ = torch::empty({3}, torch::TensorOptions().dtype(dtype_torch_).device(device_));
    friction_gpu_ = torch::tensor(params_[2], torch::TensorOptions().dtype(dtype_torch_).device(device_));
    accel_gain_gpu_ = torch::tensor(params_[0], torch::TensorOptions().dtype(dtype_torch_).device(device_));
    dt_gpu_ = torch::tensor(dt_, torch::TensorOptions().dtype(dtype_torch_).device(device_));

    if (use_cuda_)
    {
      // Pinned memory for fast transfers
      xd0_cpu_ = torch::empty({3}, torch::TensorOptions().dtype(dtype_torch_).pinned_memory(true));
      u_eff_cpu_ = torch::empty({2}, torch::TensorOptions().dtype(dtype_torch_).pinned_memory(true));
      xd1_cpu_ = torch::empty({3}, torch::TensorOptions().dtype(dtype_torch_).pinned_memory(true));
    }
  }

  void run_cpu_benchmark()
  {
    ROS_INFO("Running CPU analytical plant benchmark...");

    // Warmup
    for (int i = 0; i < warmup_calls_; ++i)
    {
      xd0_eigen_ = StateDot::Random();
      u_eff_eigen_ = Control::Random();
      xd1_eigen_ = MushrPlant::predict(xd0_eigen_, u_eff_eigen_, dt_, params_, poly_);
    }

    // Timed calls
    std::vector<double> durations;
    for (int i = 0; i < total_calls_; ++i)
    {
      xd0_eigen_ = StateDot::Random();
      u_eff_eigen_ = Control::Random();

      auto start = std::chrono::high_resolution_clock::now();
      xd1_eigen_ = MushrPlant::predict(xd0_eigen_, u_eff_eigen_, dt_, params_, poly_);
      auto end = std::chrono::high_resolution_clock::now();

      durations.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }

    print_stats("CPU Analytical Plant", durations);
  }

  void run_gpu_benchmark()
  {
    if (!use_cuda_)
    {
      ROS_WARN("CUDA not available, skipping GPU benchmark");
      return;
    }

    ROS_INFO("Running GPU plant benchmark...");

    // Warmup
    for (int i = 0; i < warmup_calls_; ++i)
    {
      xd0_eigen_ = StateDot::Random();
      u_eff_eigen_ = Control::Random();
      copy_to_gpu();
      xd1_gpu_ = gpu_plant_->forward(xd0_gpu_, u_eff_gpu_, friction_gpu_, accel_gain_gpu_, dt_gpu_, residual_gpu_);
    }
    if (use_cuda_)
    {
      c10::cuda::getCurrentCUDAStream().synchronize();
    }

    // Timed calls (including CPU-GPU transfer)
    std::vector<double> durations;
    for (int i = 0; i < total_calls_; ++i)
    {
      xd0_eigen_ = StateDot::Random();
      u_eff_eigen_ = Control::Random();

      auto start = std::chrono::high_resolution_clock::now();

      copy_to_gpu();
      xd1_gpu_ = gpu_plant_->forward(xd0_gpu_, u_eff_gpu_, friction_gpu_, accel_gain_gpu_, dt_gpu_, residual_gpu_);
      copy_from_gpu();

      auto end = std::chrono::high_resolution_clock::now();
      durations.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }

    print_stats("GPU Plant (with transfers)", durations);
  }

  void run_gpu_no_transfer_benchmark()
  {
    if (!use_cuda_)
    {
      ROS_WARN("CUDA not available, skipping GPU benchmark");
      return;
    }

    ROS_INFO("Running GPU plant benchmark (no CPU transfers, GPU-only)...");

    // Pre-fill with random data on GPU
    xd0_gpu_ = torch::randn({3}, torch::TensorOptions().dtype(dtype_torch_).device(device_));
    u_eff_gpu_ = torch::randn({2}, torch::TensorOptions().dtype(dtype_torch_).device(device_));

    // Warmup
    for (int i = 0; i < warmup_calls_; ++i)
    {
      xd1_gpu_ = gpu_plant_->forward(xd0_gpu_, u_eff_gpu_, friction_gpu_, accel_gain_gpu_, dt_gpu_, residual_gpu_);
    }
    c10::cuda::getCurrentCUDAStream().synchronize();

    // Timed calls (GPU only, no transfers)
    std::vector<double> durations;
    for (int i = 0; i < total_calls_; ++i)
    {
      auto start = std::chrono::high_resolution_clock::now();

      xd1_gpu_ = gpu_plant_->forward(xd0_gpu_, u_eff_gpu_, friction_gpu_, accel_gain_gpu_, dt_gpu_, residual_gpu_);
      c10::cuda::getCurrentCUDAStream().synchronize();

      auto end = std::chrono::high_resolution_clock::now();
      durations.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }

    print_stats("GPU Plant (GPU-only, no transfers)", durations);
  }

  void run_validation()
  {
    ROS_INFO("Validating GPU plant against CPU analytical plant...");

    // Test with several random inputs
    double max_error = 0.0;
    int num_tests = 100;

    for (int i = 0; i < num_tests; ++i)
    {
      xd0_eigen_ = StateDot::Random();
      u_eff_eigen_ = Control::Random();

      // CPU result
      StateDot xd1_cpu = MushrPlant::predict(xd0_eigen_, u_eff_eigen_, dt_, params_, poly_);

      // GPU result
      copy_to_gpu();
      xd1_gpu_ = gpu_plant_->forward(xd0_gpu_, u_eff_gpu_, friction_gpu_, accel_gain_gpu_, dt_gpu_, residual_gpu_);
      copy_from_gpu();

      // Compare
      double error = (xd1_cpu - xd1_eigen_).norm();
      max_error = std::max(max_error, error);
    }

    ROS_INFO("Validation complete. Max error: %.6e", max_error);
    if (max_error < 1e-4)
    {
      ROS_INFO("PASSED: GPU plant matches CPU analytical plant");
    }
    else
    {
      ROS_WARN("FAILED: GPU plant differs from CPU analytical plant");
    }
  }

  void run_all()
  {
    std::cout << "\n========================================\n";
    std::cout << "GPU Plant Benchmark\n";
    std::cout << "Dtype: " << dtype_ << "\n";
    std::cout << "Total calls: " << total_calls_ << "\n";
    std::cout << "Warmup calls: " << warmup_calls_ << "\n";
    std::cout << "========================================\n\n";

    run_validation();
    std::cout << "\n";

    run_cpu_benchmark();
    run_gpu_benchmark();
    run_gpu_no_transfer_benchmark();

    std::cout << "\n========================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "========================================\n";
  }

private:
  void copy_to_gpu()
  {
    if (dtype_torch_ == torch::kFloat32)
    {
      float* data = xd0_cpu_.data_ptr<float>();
      data[0] = static_cast<float>(xd0_eigen_[0]);
      data[1] = static_cast<float>(xd0_eigen_[1]);
      data[2] = static_cast<float>(xd0_eigen_[2]);

      data = u_eff_cpu_.data_ptr<float>();
      data[0] = static_cast<float>(u_eff_eigen_[0]);
      data[1] = static_cast<float>(u_eff_eigen_[1]);
    }
    else
    {
      double* data = xd0_cpu_.data_ptr<double>();
      data[0] = xd0_eigen_[0];
      data[1] = xd0_eigen_[1];
      data[2] = xd0_eigen_[2];

      data = u_eff_cpu_.data_ptr<double>();
      data[0] = u_eff_eigen_[0];
      data[1] = u_eff_eigen_[1];
    }

    xd0_gpu_.copy_(xd0_cpu_, /*non_blocking=*/true);
    u_eff_gpu_.copy_(u_eff_cpu_, /*non_blocking=*/true);
  }

  void copy_from_gpu()
  {
    xd1_cpu_.copy_(xd1_gpu_, /*non_blocking=*/false);  // Blocking to ensure sync

    if (dtype_torch_ == torch::kFloat32)
    {
      const float* data = xd1_cpu_.data_ptr<float>();
      xd1_eigen_[0] = static_cast<double>(data[0]);
      xd1_eigen_[1] = static_cast<double>(data[1]);
      xd1_eigen_[2] = static_cast<double>(data[2]);
    }
    else
    {
      const double* data = xd1_cpu_.data_ptr<double>();
      xd1_eigen_[0] = data[0];
      xd1_eigen_[1] = data[1];
      xd1_eigen_[2] = data[2];
    }
  }

  void print_stats(const std::string& name, const std::vector<double>& durations)
  {
    auto [min_it, max_it] = std::minmax_element(durations.begin(), durations.end());
    double sum = std::accumulate(durations.begin(), durations.end(), 0.0);
    double mean = sum / durations.size();

    std::vector<double> sorted = durations;
    std::sort(sorted.begin(), sorted.end());
    double p50 = sorted[sorted.size() / 2];
    double p95 = sorted[static_cast<size_t>(sorted.size() * 0.95)];
    double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];

    std::cout << name << ":\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Min:  " << std::setw(8) << *min_it << " us\n";
    std::cout << "  Max:  " << std::setw(8) << *max_it << " us\n";
    std::cout << "  Mean: " << std::setw(8) << mean << " us\n";
    std::cout << "  P50:  " << std::setw(8) << p50 << " us\n";
    std::cout << "  P95:  " << std::setw(8) << p95 << " us\n";
    std::cout << "  P99:  " << std::setw(8) << p99 << " us\n";
    std::cout << "\n";
  }

  int total_calls_;
  int warmup_calls_;
  bool use_cuda_;
  std::string dtype_;
  double dt_;
  double L_;

  Params params_;
  Poly poly_;

  torch::Device device_{torch::kCPU};
  torch::Dtype dtype_torch_;

  std::unique_ptr<torch_bridge::GpuPlant> gpu_plant_;

  // GPU tensors
  torch::Tensor xd0_gpu_;
  torch::Tensor u_eff_gpu_;
  torch::Tensor residual_gpu_;
  torch::Tensor xd1_gpu_;
  torch::Tensor friction_gpu_;
  torch::Tensor accel_gain_gpu_;
  torch::Tensor dt_gpu_;

  // Pinned CPU tensors
  torch::Tensor xd0_cpu_;
  torch::Tensor u_eff_cpu_;
  torch::Tensor xd1_cpu_;

  // Eigen vectors
  StateDot xd0_eigen_;
  Control u_eff_eigen_;
  StateDot xd1_eigen_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpu_plant_benchmark");
  ros::NodeHandle nh("~");

  GpuPlantBenchmark benchmark(nh);
  benchmark.run_all();

  return 0;
}
