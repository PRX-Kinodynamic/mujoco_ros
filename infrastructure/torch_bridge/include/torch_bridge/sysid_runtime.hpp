#include <memory>
#ifndef TORCH_NOT_BUILT
#pragma once

#include <optional>
#include <Eigen/Core>
#include <torch/script.h>
// #include <c10/cuda/CUDAStream.h>
// #include <c10/cuda/CUDAGuard.h>
// #include <ATen/cuda/CUDAGraph.h>
#include <boost/optional.hpp>
#include "torch_eigen_bridge.hpp"
#include "gpu_plant.hpp"
#include <utils/dbg_utils.hpp>

#include <ros/console.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace torch_bridge
{

// ---------------------------------------------------------------------------
// Abstract base class for all sysid runtimes
// ---------------------------------------------------------------------------
class SysidRuntimeBase
{
public:
  using StateDot = Eigen::Vector3d;
  using Control = Eigen::Vector2d;
  using JacX = Eigen::Matrix3d;
  using JacU = Eigen::Matrix<double, 3, 2>;
  using OptionalJacX = boost::optional<JacX&>;
  using OptionalJacU = boost::optional<JacU&>;

  virtual ~SysidRuntimeBase() = default;

  virtual StateDot predict(const StateDot& xd0, const Control& u) = 0;
  virtual std::tuple<StateDot, JacX, JacU> predict_with_jac(const StateDot& xd0, const Control& u) = 0;
  virtual bool has_jacobian_method() const = 0;

  // Convenience: dispatch to predict() or predict_with_jac() based on whether
  // jacobian references are supplied.
  StateDot call(const StateDot& xd, const Control& u, OptionalJacX jacX = boost::none, OptionalJacU jacU = boost::none)
  {
    const bool get_derivs{ jacX || jacU };
    if (get_derivs)
    {
      auto [xd1, Jx, Ju] = predict_with_jac(xd, u);
      if (jacX)
        *jacX = Jx;
      if (jacU)
        *jacU = Ju;
      return xd1;
    }
    return predict(xd, u);
  }

  StateDot operator()(const StateDot& xd, const Control& u, OptionalJacX jacX = boost::none,
                      OptionalJacU jacU = boost::none)
  {
    return call(xd, u, jacX, jacU);
  }
};

// ---------------------------------------------------------------------------
// Direct MLP runtime
// ---------------------------------------------------------------------------
class DirectSysidRuntime : public SysidRuntimeBase
{
  static void print_torch(const torch::Device& device_)
  {
    static bool printed{ false };
    if (not printed)
    {
      if (device_.is_cuda())
      {
        PRINT_MSG("[DirectSysidRuntime] Using CUDA device.");
      }
      else
      {
        PRINT_MSG("[DirectSysidRuntime] Using CPU device");
      }
    }
    printed = true;
  }

public:
  DirectSysidRuntime(const std::string& model_path, bool use_cuda = true, const std::string& dtype = "float64")
    : device_(use_cuda && torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)
  {
    try
    {
      module_ = torch::jit::load(model_path, device_);
      module_.eval();

      print_torch(device_);
      // Log which device is being used
      // PRINT_MSG_ONCE("");
      // if (device_.is_cuda())
      // {
      //   std::cout << "[DirectSysidRuntime] Using CUDA device" << std::endl;
      // }
      // else
      // {
      //   std::cout << "[DirectSysidRuntime] Using CPU device" << std::endl;
      // }
    }
    catch (const c10::Error& e)
    {
      throw std::runtime_error("Failed to load TorchScript model: " + std::string(e.what()));
    }

    if (dtype == "float32")
    {
      dtype_ = torch::kFloat32;
      use_float32_ = true;
      xd_in_ = make_vector_f32(3, device_);
      u_in_ = make_vector_f32(2, device_);
    }
    else if (dtype == "float64")
    {
      dtype_ = torch::kFloat64;
      use_float32_ = false;
      xd_in_ = make_vector_f64(3, device_);
      u_in_ = make_vector_f64(2, device_);
    }
    else
    {
      throw std::runtime_error("Invalid dtype: " + dtype + ". Must be 'float32' or 'float64'");
    }

    // Pre-allocate IValue input vector to avoid allocation per call
    inputs_.reserve(2);
    inputs_.push_back(xd_in_);
    inputs_.push_back(u_in_);

    // Cache data pointers for CPU path
    if (!device_.is_cuda())
    {
      if (use_float32_)
      {
        xd_in_ptr_f32_ = xd_in_.data_ptr<float>();
        u_in_ptr_f32_ = u_in_.data_ptr<float>();
      }
      else
      {
        xd_in_ptr_f64_ = xd_in_.data_ptr<double>();
        u_in_ptr_f64_ = u_in_.data_ptr<double>();
      }
    }

    // Cache forward_with_jacobian method if available
    if (module_.find_method("forward_with_jacobian").has_value())
    {
      forward_with_jacobian_method_.emplace(module_.get_method("forward_with_jacobian"));
    }
  }

  StateDot predict(const StateDot& xd0, const Control& u) override
  {
    c10::InferenceMode guard;

    if (!device_.is_cuda())
    {
      // Optimized CPU path: direct write to cached pointers
      if (use_float32_)
      {
        xd_in_ptr_f32_[0] = static_cast<float>(xd0(0));
        xd_in_ptr_f32_[1] = static_cast<float>(xd0(1));
        xd_in_ptr_f32_[2] = static_cast<float>(xd0(2));
        u_in_ptr_f32_[0] = static_cast<float>(u(0));
        u_in_ptr_f32_[1] = static_cast<float>(u(1));
      }
      else
      {
        xd_in_ptr_f64_[0] = xd0(0);
        xd_in_ptr_f64_[1] = xd0(1);
        xd_in_ptr_f64_[2] = xd0(2);
        u_in_ptr_f64_[0] = u(0);
        u_in_ptr_f64_[1] = u(1);
      }
    }
    else
    {
      // CUDA path (not latency-optimized)
      if (use_float32_)
      {
        copy_f32(xd_in_, xd0);
        copy_f32(u_in_, u);
      }
      else
      {
        copy_f64(xd_in_, xd0);
        copy_f64(u_in_, u);
      }
    }

    auto result = module_.forward(inputs_);

    if (!result.isTensor())
    {
      throw std::runtime_error(
          "DirectSysidRuntime: model forward() must return a Tensor, not a Tuple. "
          "Use StructuredSysidRuntime for structured models.");
    }
    torch::Tensor output = result.toTensor();

    StateDot xd1;
    if (use_float32_)
    {
      copy_f32(xd1, output);
    }
    else
    {
      copy_f64(xd1, output);
    }
    return xd1;
  }

  virtual std::tuple<StateDot, JacX, JacU> predict_with_jac(const StateDot& xd0, const Control& u) override
  {
    c10::InferenceMode guard;

    if (!device_.is_cuda())
    {
      if (use_float32_)
      {
        xd_in_ptr_f32_[0] = static_cast<float>(xd0(0));
        xd_in_ptr_f32_[1] = static_cast<float>(xd0(1));
        xd_in_ptr_f32_[2] = static_cast<float>(xd0(2));
        u_in_ptr_f32_[0] = static_cast<float>(u(0));
        u_in_ptr_f32_[1] = static_cast<float>(u(1));
      }
      else
      {
        xd_in_ptr_f64_[0] = xd0(0);
        xd_in_ptr_f64_[1] = xd0(1);
        xd_in_ptr_f64_[2] = xd0(2);
        u_in_ptr_f64_[0] = u(0);
        u_in_ptr_f64_[1] = u(1);
      }
    }
    else
    {
      if (use_float32_)
      {
        copy_f32(xd_in_, xd0);
        copy_f32(u_in_, u);
      }
      else
      {
        copy_f64(xd_in_, xd0);
        copy_f64(u_in_, u);
      }
    }

    if (!forward_with_jacobian_method_.has_value())
    {
      throw std::runtime_error("Model does not have 'forward_with_jacobian' method");
    }
    auto result = (*forward_with_jacobian_method_)(inputs_);
    auto tuple = result.toTuple();

    StateDot xd1;
    JacX Jx;
    JacU Ju;

    if (use_float32_)
    {
      copy_f32(xd1, tuple->elements()[0].toTensor());

      torch::Tensor Jx_t = tuple->elements()[1].toTensor();
      torch::Tensor Ju_t = tuple->elements()[2].toTensor();
      // Move tensors to CPU if on CUDA before accessing data pointer
      if (Jx_t.is_cuda())
        Jx_t = Jx_t.to(torch::kCPU);
      if (Ju_t.is_cuda())
        Ju_t = Ju_t.to(torch::kCPU);

      float* Jx_data = Jx_t.data_ptr<float>();
      float* Ju_data = Ju_t.data_ptr<float>();

      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          Jx(i, j) = static_cast<double>(Jx_data[i * 3 + j]);
        }
        for (int j = 0; j < 2; ++j)
        {
          Ju(i, j) = static_cast<double>(Ju_data[i * 2 + j]);
        }
      }
    }
    else
    {
      copy_f64(xd1, tuple->elements()[0].toTensor());

      torch::Tensor Jx_t = tuple->elements()[1].toTensor();
      torch::Tensor Ju_t = tuple->elements()[2].toTensor();
      // Move tensors to CPU if on CUDA before accessing data pointer
      if (Jx_t.is_cuda())
        Jx_t = Jx_t.to(torch::kCPU);
      if (Ju_t.is_cuda())
        Ju_t = Ju_t.to(torch::kCPU);

      double* Jx_data = Jx_t.data_ptr<double>();
      double* Ju_data = Ju_t.data_ptr<double>();

      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          Jx(i, j) = Jx_data[i * 3 + j];
        }
        for (int j = 0; j < 2; ++j)
        {
          Ju(i, j) = Ju_data[i * 2 + j];
        }
      }
    }

    return std::make_tuple(xd1, Jx, Ju);
  }

  bool has_jacobian_method() const override
  {
    return module_.find_method("forward_with_jacobian").has_value();
  }

private:
  torch::jit::script::Module module_;
  torch::Device device_;
  torch::Dtype dtype_;
  bool use_float32_;
  torch::Tensor xd_in_;
  torch::Tensor u_in_;

  // Pre-allocated input vector (avoids allocation per call)
  std::vector<torch::jit::IValue> inputs_;

  // Cached data pointers for CPU path
  float* xd_in_ptr_f32_ = nullptr;
  float* u_in_ptr_f32_ = nullptr;
  double* xd_in_ptr_f64_ = nullptr;
  double* u_in_ptr_f64_ = nullptr;

  // Cached method handle
  std::optional<torch::jit::Method> forward_with_jacobian_method_;
};

// struct StructuredParams
// {
//   const std::size_t params = ??;
//   const std::size_t friction = ??;
//   const std::size_t vel_desired = ??;
//   const std::size_t L = ??;
// }

// ---------------------------------------------------------------------------
// Structured dynamics runtime (template: depends on plant types from caller)
// ---------------------------------------------------------------------------
template <typename MushrPlant, typename Params, typename Poly, typename StructuredParams>
class StructuredSysidRuntime : public SysidRuntimeBase
{
  static void print_torch(const torch::Device& device_)
  {
    static bool printed{ false };
    if (not printed)
    {
      if (device_.is_cuda())
      {
        PRINT_MSG("[StructuredSysidRuntime] Using CUDA device.");
      }
      else
      {
        PRINT_MSG("[StructuredSysidRuntime] Using CPU device");
      }
    }
    printed = true;
  }

public:
  // using Params = prx_models::mushr_types::Control::params;
  // using Poly = prx_models::mushr_types::Control::Poly;
  // using MushrPlant = prx_models::mushr_CtrlAccel_t<>;

  StructuredSysidRuntime(const std::string& model_path, const Params& params, const Poly& poly, bool use_cuda = true,
                         const std::string& dtype = "float64")
    : params_(params)
    , poly_(poly)
    , model_path_(model_path)
    , device_(use_cuda && torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)
    , use_cuda_(use_cuda && torch::cuda::is_available())
    , dt_(0.1)
  {
    try
    {
      module_ = torch::jit::load(model_path, device_);
      module_.eval();

      print_torch(device_);
      // Log which device is being used
      // if (device_.is_cuda())
      // {
      //   std::cout << "[StructuredSysidRuntime] Using CUDA device" << std::endl;
      // }
      // else
      // {
      //   std::cout << "[StructuredSysidRuntime] Using CPU device" << std::endl;
      // }
    }
    catch (const c10::Error& e)
    {
      throw std::runtime_error("Failed to load TorchScript model: " + std::string(e.what()));
    }

    if (dtype == "float32")
    {
      dtype_ = torch::kFloat32;
      use_float32_ = true;
      // PRINT_MSG("Using float32");
    }
    else if (dtype == "float64")
    {
      dtype_ = torch::kFloat64;
      use_float32_ = false;
    }
    else
    {
      throw std::runtime_error("Invalid dtype: " + dtype + ". Must be 'float32' or 'float64'");
    }

    // Allocate tensors for model I/O
    xd_in_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).device(device_));
    u_in_ = torch::empty({ 2 }, torch::TensorOptions().dtype(dtype_).device(device_));

    // Pre-allocate IValue input vector to avoid allocation per call
    inputs_.reserve(2);
    inputs_.push_back(xd_in_);
    inputs_.push_back(u_in_);

    // Cache data pointers for CPU path (avoid repeated data_ptr() calls)
    if (!use_cuda_)
    {
      if (use_float32_)
      {
        xd_in_ptr_f32_ = xd_in_.data_ptr<float>();
        u_in_ptr_f32_ = u_in_.data_ptr<float>();
      }
      else
      {
        xd_in_ptr_f64_ = xd_in_.data_ptr<double>();
        u_in_ptr_f64_ = u_in_.data_ptr<double>();
      }
    }

    if (use_cuda_)
    {
      // Allocate pinned CPU tensors for fast async transfers
      xd_in_cpu_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
      u_in_cpu_ = torch::empty({ 2 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));

      // Pre-allocate output tensors on GPU
      u_eff_out_ = torch::empty({ 2 }, torch::TensorOptions().dtype(dtype_).device(device_));
      k_out_ = torch::empty({ 1 }, torch::TensorOptions().dtype(dtype_).device(device_));
      residual_out_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).device(device_));

      // Pinned output staging tensors
      u_eff_out_cpu_ = torch::empty({ 2 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
      k_out_cpu_ = torch::empty({ 1 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
      residual_out_cpu_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
    }

    // Cache forward_with_jacobian method if available
    if (module_.find_method("forward_with_jacobian").has_value())
    {
      forward_with_jacobian_method_.emplace(module_.get_method("forward_with_jacobian"));
    }

    // Extract standardizer buffers from TorchScript module
    extract_standardizers_from_module();
    // std::cout << "[StructuredSysidRuntime] Using normalized-plant semantics" << std::endl;
  }

  // Capture CUDA graph for predict() - call once after construction
  // NOTE: CUDA graph capture is currently disabled, but standard CUDA inference works
  void warmup_cuda_graph(int warmup_iters = 3)
  {
    if (!use_cuda_)
      return;
    ROS_WARN_ONCE("CUDA graph capture disabled, using standard CUDA inference");
    // c10::InferenceMode guard;

    // // Warmup iterations to stabilize CUDA state
    // std::vector<torch::jit::IValue> inputs;
    // inputs.push_back(xd_in_);
    // inputs.push_back(u_in_);

    // for (int i = 0; i < warmup_iters; ++i)
    // {
    //   auto result = module_.forward(inputs);
    //   auto tuple = result.toTuple();
    //   u_eff_out_.copy_(tuple->elements()[0].toTensor());
    //   k_out_.copy_(tuple->elements()[1].toTensor());
    //   residual_out_.copy_(tuple->elements()[2].toTensor());
    // }

    // Create a non-default stream for graph capture (required by PyTorch)
    // at::cuda::CUDAStream capture_stream = at::cuda::getStreamFromPool(/*isHighPriority=*/false, device_.index());
    // capture_stream_.emplace(capture_stream);

    // Capture the graph on the non-default stream
    // {
    // c10::cuda::CUDAStreamGuard stream_guard(capture_stream);
    // capture_stream.synchronize();

    // // cuda_graph_.capture_begin();

    // auto result = module_.forward(inputs);
    // auto tuple = result.toTuple();
    // u_eff_out_.copy_(tuple->elements()[0].toTensor());
    // k_out_.copy_(tuple->elements()[1].toTensor());
    // residual_out_.copy_(tuple->elements()[2].toTensor());

    // cuda_graph_.capture_end();
    // }

    // Synchronize streams to ensure clean state
    // capture_stream.synchronize();
    // at::cuda::getCurrentCUDAStream().synchronize();
    // graph_captured_ = true;
  }

  // Enable/disable GPU-based plant dynamics (keeps everything on GPU, avoids CPU round-trip)
  void set_use_gpu_plant(bool use_gpu_plant)
  {
    if (use_gpu_plant && !use_cuda_)
      return;

    // Lazy allocation: only allocate GPU plant resources when first enabled
    if (use_gpu_plant && !gpu_plant_initialized_)
    {
      // Pre-allocate final output tensor for GPU plant path
      xd1_out_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).device(device_));
      xd1_out_cpu_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));

      // Store plant parameters as GPU tensors
      plant_friction_ =
          torch::tensor(params_[StructuredParams::friction], torch::TensorOptions().dtype(dtype_).device(device_));
      plant_accel_gain_ =
          torch::tensor(params_[StructuredParams::vel_desired], torch::TensorOptions().dtype(dtype_).device(device_));
      plant_dt_ = torch::tensor(dt_, torch::TensorOptions().dtype(dtype_).device(device_));

      // Create GPU plant instance
      std::vector<double> poly_vec(poly_.begin(), poly_.end());
      gpu_plant_ = std::make_unique<GpuPlant>(StructuredParams::L, poly_vec, dtype_, device_);

      gpu_plant_initialized_ = true;
    }

    use_gpu_plant_ = use_gpu_plant;
  }

  bool is_gpu_plant_enabled() const
  {
    return use_gpu_plant_;
  }

  // Enable hybrid mode: CUDA+Graph for predict(), CPU for predict_with_jac()
  // This provides optimal latency for both paths based on benchmark results.
  // Requires CUDA to be available; no-op if already in CPU-only mode.
  void enable_hybrid_mode(int graph_warmup_iters = 3)
  {
    if (!use_cuda_)
    {
      std::cout << "[StructuredSysidRuntime] Hybrid mode not available (CUDA not enabled)" << std::endl;
      return;
    }

    if (hybrid_mode_)
    {
      std::cout << "[StructuredSysidRuntime] Hybrid mode already enabled" << std::endl;
      return;
    }

    std::cout << "[StructuredSysidRuntime] Enabling hybrid mode (CUDA+Graph for predict, CPU for jacobians)"
              << std::endl;

    // Load a separate copy of the model on CPU for jacobians
    try
    {
      cpu_module_ = torch::jit::load(model_path_, torch::kCPU);
      cpu_module_.eval();
    }
    catch (const c10::Error& e)
    {
      throw std::runtime_error("Failed to load CPU model for hybrid mode: " + std::string(e.what()));
    }

    // Allocate CPU tensors for jacobian path
    cpu_xd_in_ = torch::empty({ 3 }, torch::TensorOptions().dtype(dtype_).device(torch::kCPU));
    cpu_u_in_ = torch::empty({ 2 }, torch::TensorOptions().dtype(dtype_).device(torch::kCPU));

    // Pre-allocate CPU input vector
    cpu_inputs_.reserve(2);
    cpu_inputs_.push_back(cpu_xd_in_);
    cpu_inputs_.push_back(cpu_u_in_);

    // Cache CPU data pointers for fast access
    if (use_float32_)
    {
      cpu_xd_in_ptr_f32_ = cpu_xd_in_.data_ptr<float>();
      cpu_u_in_ptr_f32_ = cpu_u_in_.data_ptr<float>();
    }
    else
    {
      cpu_xd_in_ptr_f64_ = cpu_xd_in_.data_ptr<double>();
      cpu_u_in_ptr_f64_ = cpu_u_in_.data_ptr<double>();
    }

    // Cache CPU forward_with_jacobian method
    if (cpu_module_.find_method("forward_with_jacobian").has_value())
    {
      cpu_forward_with_jacobian_method_.emplace(cpu_module_.get_method("forward_with_jacobian"));
    }

    // Warm up CUDA graph for predict() if not already done
    if (!graph_captured_)
    {
      warmup_cuda_graph(graph_warmup_iters);
    }

    hybrid_mode_ = true;
    std::cout << "[StructuredSysidRuntime] Hybrid mode enabled successfully" << std::endl;
  }

  bool is_hybrid_mode() const
  {
    return hybrid_mode_;
  }

  StateDot predict(const StateDot& xd0, const Control& u) override
  {
    c10::InferenceMode guard;

    // Standard paths with CPU plant dynamics
    Control u_eff;
    double friction_k;
    StateDot residual;

    if (use_cuda_ && graph_captured_)
    {
      // CUDA graph path with CPU plant
      // c10::cuda::CUDAStreamGuard stream_guard(*capture_stream_);

      if (use_float32_)
      {
        copy_to_pinned_f32(xd_in_cpu_, xd0);
        copy_to_pinned_f32(u_in_cpu_, u);
      }
      else
      {
        copy_to_pinned_f64(xd_in_cpu_, xd0);
        copy_to_pinned_f64(u_in_cpu_, u);
      }

      xd_in_.copy_(xd_in_cpu_, /*non_blocking=*/true);
      u_in_.copy_(u_in_cpu_, /*non_blocking=*/true);

      // cuda_graph_.replay();

      u_eff_out_cpu_.copy_(u_eff_out_, /*non_blocking=*/true);
      k_out_cpu_.copy_(k_out_, /*non_blocking=*/true);
      residual_out_cpu_.copy_(residual_out_, /*non_blocking=*/true);

      // capture_stream_->synchronize();

      if (use_float32_)
      {
        copy_from_pinned_f32(u_eff, u_eff_out_cpu_);
        friction_k = k_out_cpu_.data_ptr<float>()[0];
        copy_from_pinned_f32(residual, residual_out_cpu_);
      }
      else
      {
        copy_from_pinned_f64(u_eff, u_eff_out_cpu_);
        friction_k = k_out_cpu_.data_ptr<double>()[0];
        copy_from_pinned_f64(residual, residual_out_cpu_);
      }
    }
    else if (use_cuda_)
    {
      // CUDA path without graph capture - use pinned memory transfers
      if (use_float32_)
      {
        copy_to_pinned_f32(xd_in_cpu_, xd0);
        copy_to_pinned_f32(u_in_cpu_, u);
      }
      else
      {
        copy_to_pinned_f64(xd_in_cpu_, xd0);
        copy_to_pinned_f64(u_in_cpu_, u);
      }

      xd_in_.copy_(xd_in_cpu_, /*non_blocking=*/true);
      u_in_.copy_(u_in_cpu_, /*non_blocking=*/true);

      auto result = module_.forward(inputs_);
      const auto& elements = result.toTuple()->elements();

      // Copy outputs to pinned CPU memory
      u_eff_out_.copy_(elements[0].toTensor());
      k_out_.copy_(elements[1].toTensor());
      residual_out_.copy_(elements[2].toTensor());

      u_eff_out_cpu_.copy_(u_eff_out_, /*non_blocking=*/true);
      k_out_cpu_.copy_(k_out_, /*non_blocking=*/true);
      residual_out_cpu_.copy_(residual_out_, /*non_blocking=*/true);

      // at::cuda::getCurrentCUDAStream().synchronize();

      if (use_float32_)
      {
        copy_from_pinned_f32(u_eff, u_eff_out_cpu_);
        friction_k = k_out_cpu_.data_ptr<float>()[0];
        copy_from_pinned_f32(residual, residual_out_cpu_);
      }
      else
      {
        copy_from_pinned_f64(u_eff, u_eff_out_cpu_);
        friction_k = k_out_cpu_.data_ptr<double>()[0];
        copy_from_pinned_f64(residual, residual_out_cpu_);
      }
    }
    else
    {
      // Optimized CPU path - use cached pointers and pre-allocated inputs
      if (use_float32_)
      {
        // Direct write to cached pointers (faster than copy_f32 with Eigen::Map)
        xd_in_ptr_f32_[0] = static_cast<float>(xd0(0));
        xd_in_ptr_f32_[1] = static_cast<float>(xd0(1));
        xd_in_ptr_f32_[2] = static_cast<float>(xd0(2));
        u_in_ptr_f32_[0] = static_cast<float>(u(0));
        u_in_ptr_f32_[1] = static_cast<float>(u(1));
      }
      else
      {
        // Direct write for float64
        xd_in_ptr_f64_[0] = xd0(0);
        xd_in_ptr_f64_[1] = xd0(1);
        xd_in_ptr_f64_[2] = xd0(2);
        u_in_ptr_f64_[0] = u(0);
        u_in_ptr_f64_[1] = u(1);
      }

      // Use pre-allocated inputs vector
      auto result = module_.forward(inputs_);
      const auto& elements = result.toTuple()->elements();

      if (use_float32_)
      {
        const float* u_eff_data = elements[0].toTensor().template data_ptr<float>();
        u_eff(0) = u_eff_data[0];
        u_eff(1) = u_eff_data[1];
        friction_k = elements[1].toTensor().template data_ptr<float>()[0];
        const float* res_data = elements[2].toTensor().template data_ptr<float>();
        residual(0) = res_data[0];
        residual(1) = res_data[1];
        residual(2) = res_data[2];
      }
      else
      {
        const double* u_eff_data = elements[0].toTensor().template data_ptr<double>();
        u_eff(0) = u_eff_data[0];
        u_eff(1) = u_eff_data[1];
        friction_k = elements[1].toTensor().template data_ptr<double>()[0];
        const double* res_data = elements[2].toTensor().template data_ptr<double>();
        residual(0) = res_data[0];
        residual(1) = res_data[1];
        residual(2) = res_data[2];
      }
    }

    // CPU plant dynamics (normalized-plant semantics)
    // TorchScript now returns u_eff and residual already in normalized space.
    // Only xd0 (from caller) needs normalizing.
    StateDot xd0_norm = (xd0 - input_mean_x_).cwiseProduct(inv_input_std_x_);
    const Control& u_eff_norm = u_eff;
    const StateDot& residual_norm = residual;

    // Run plant on normalized inputs with identity params/poly and friction_k applied
    Params params_identity;
    params_identity.setConstant(1.0);
    params_identity[StructuredParams::friction] *= friction_k;

    Poly poly_identity;
    poly_identity.setZero();
    poly_identity[2] = 1.0;  // Identity polynomial: delta = 1.0 * x

    StateDot xd1_plant_norm = MushrPlant::predict(xd0_norm, u_eff_norm, dt_, params_identity, poly_identity);

    // Add residual in normalized space
    StateDot xd1_norm = xd1_plant_norm + residual_norm;

    // 4. Unstandardize to raw space
    StateDot xd1_raw = xd1_norm.cwiseProduct(target_std_) + target_mean_;
    return xd1_raw;
  }

  // call() and operator() are inherited from SysidRuntimeBase

  virtual std::tuple<StateDot, JacX, JacU> predict_with_jac(const StateDot& xd0, const Control& u) override
  {
    c10::InferenceMode guard;

    Control u_eff;
    double friction_k;
    StateDot residual;
    Eigen::Matrix<double, 2, 3> J_ueff_x;
    Eigen::Matrix<double, 2, 2> J_ueff_u;
    Eigen::Matrix<double, 1, 3> J_k_x;
    Eigen::Matrix<double, 1, 2> J_k_u;
    JacX J_r_x;
    JacU J_r_u;

    if (use_cuda_ && !hybrid_mode_)
    {
      // CUDA path: use async pinned memory transfers for performance
      ensure_jacobian_tensors_initialized();

      // Copy inputs to pinned memory, then async to GPU
      if (use_float32_)
      {
        copy_to_pinned_f32(xd_in_cpu_, xd0);
        copy_to_pinned_f32(u_in_cpu_, u);
      }
      else
      {
        copy_to_pinned_f64(xd_in_cpu_, xd0);
        copy_to_pinned_f64(u_in_cpu_, u);
      }
      xd_in_.copy_(xd_in_cpu_, /*non_blocking=*/true);
      u_in_.copy_(u_in_cpu_, /*non_blocking=*/true);

      // Run model
      // Use pre-allocated input vector to avoid per-call allocation.
      if (!forward_with_jacobian_method_.has_value())
      {
        throw std::runtime_error("Model does not have 'forward_with_jacobian' method");
      }
      auto result = (*forward_with_jacobian_method_)(inputs_);
      auto tuple = result.toTuple();

      // Copy all outputs to pre-allocated GPU tensors, then async to pinned CPU
      u_eff_out_.copy_(tuple->elements()[0].toTensor());
      k_out_.copy_(tuple->elements()[1].toTensor());
      residual_out_.copy_(tuple->elements()[2].toTensor());
      J_ueff_x_gpu_.copy_(tuple->elements()[3].toTensor());
      J_ueff_u_gpu_.copy_(tuple->elements()[4].toTensor());
      J_k_x_gpu_.copy_(tuple->elements()[5].toTensor());
      J_k_u_gpu_.copy_(tuple->elements()[6].toTensor());
      J_r_x_gpu_.copy_(tuple->elements()[7].toTensor());
      J_r_u_gpu_.copy_(tuple->elements()[8].toTensor());

      // Async copy all to pinned CPU memory
      u_eff_out_cpu_.copy_(u_eff_out_, /*non_blocking=*/true);
      k_out_cpu_.copy_(k_out_, /*non_blocking=*/true);
      residual_out_cpu_.copy_(residual_out_, /*non_blocking=*/true);
      J_ueff_x_cpu_.copy_(J_ueff_x_gpu_, /*non_blocking=*/true);
      J_ueff_u_cpu_.copy_(J_ueff_u_gpu_, /*non_blocking=*/true);
      J_k_x_cpu_.copy_(J_k_x_gpu_, /*non_blocking=*/true);
      J_k_u_cpu_.copy_(J_k_u_gpu_, /*non_blocking=*/true);
      J_r_x_cpu_.copy_(J_r_x_gpu_, /*non_blocking=*/true);
      J_r_u_cpu_.copy_(J_r_u_gpu_, /*non_blocking=*/true);

      // Single sync for all transfers
      // at::cuda::getCurrentCUDAStream().synchronize();

      // Extract from pinned memory (no GPU sync needed, already done)
      if (use_float32_)
      {
        copy_from_pinned_f32(u_eff, u_eff_out_cpu_);
        friction_k = k_out_cpu_.template data_ptr<float>()[0];
        copy_from_pinned_f32(residual, residual_out_cpu_);
        copy_from_pinned_matrix_f32(J_ueff_x, J_ueff_x_cpu_);
        copy_from_pinned_matrix_f32(J_ueff_u, J_ueff_u_cpu_);
        copy_from_pinned_matrix_f32(J_k_x, J_k_x_cpu_);
        copy_from_pinned_matrix_f32(J_k_u, J_k_u_cpu_);
        copy_from_pinned_matrix_f32(J_r_x, J_r_x_cpu_);
        copy_from_pinned_matrix_f32(J_r_u, J_r_u_cpu_);
      }
      else
      {
        copy_from_pinned_f64(u_eff, u_eff_out_cpu_);
        friction_k = k_out_cpu_.template data_ptr<double>()[0];
        copy_from_pinned_f64(residual, residual_out_cpu_);
        copy_from_pinned_matrix_f64(J_ueff_x, J_ueff_x_cpu_);
        copy_from_pinned_matrix_f64(J_ueff_u, J_ueff_u_cpu_);
        copy_from_pinned_matrix_f64(J_k_x, J_k_x_cpu_);
        copy_from_pinned_matrix_f64(J_k_u, J_k_u_cpu_);
        copy_from_pinned_matrix_f64(J_r_x, J_r_x_cpu_);
        copy_from_pinned_matrix_f64(J_r_u, J_r_u_cpu_);
      }
    }
    else
    {
      // CPU path: used for regular CPU mode or hybrid mode (jacobians on CPU)
      // Select pointers based on mode
      float* xd_ptr_f32 = hybrid_mode_ ? cpu_xd_in_ptr_f32_ : xd_in_ptr_f32_;
      float* u_ptr_f32 = hybrid_mode_ ? cpu_u_in_ptr_f32_ : u_in_ptr_f32_;
      double* xd_ptr_f64 = hybrid_mode_ ? cpu_xd_in_ptr_f64_ : xd_in_ptr_f64_;
      double* u_ptr_f64 = hybrid_mode_ ? cpu_u_in_ptr_f64_ : u_in_ptr_f64_;
      auto& method = hybrid_mode_ ? cpu_forward_with_jacobian_method_ : forward_with_jacobian_method_;
      auto& inputs = hybrid_mode_ ? cpu_inputs_ : inputs_;

      if (use_float32_)
      {
        xd_ptr_f32[0] = static_cast<float>(xd0(0));
        xd_ptr_f32[1] = static_cast<float>(xd0(1));
        xd_ptr_f32[2] = static_cast<float>(xd0(2));
        u_ptr_f32[0] = static_cast<float>(u(0));
        u_ptr_f32[1] = static_cast<float>(u(1));
      }
      else
      {
        xd_ptr_f64[0] = xd0(0);
        xd_ptr_f64[1] = xd0(1);
        xd_ptr_f64[2] = xd0(2);
        u_ptr_f64[0] = u(0);
        u_ptr_f64[1] = u(1);
      }

      if (!method.has_value())
      {
        throw std::runtime_error("Model does not have 'forward_with_jacobian' method");
      }

      auto result = (*method)(inputs);
      const auto& elements = result.toTuple()->elements();

      if (use_float32_)
      {
        // Extract outputs with direct pointer access
        const float* u_eff_data = elements[0].toTensor().template data_ptr<float>();
        u_eff(0) = u_eff_data[0];
        u_eff(1) = u_eff_data[1];
        friction_k = elements[1].toTensor().template data_ptr<float>()[0];
        const float* res_data = elements[2].toTensor().template data_ptr<float>();
        residual(0) = res_data[0];
        residual(1) = res_data[1];
        residual(2) = res_data[2];

        // Extract jacobians with direct pointer access (unrolled for small matrices)
        const float* J_ueff_x_data = elements[3].toTensor().template data_ptr<float>();
        J_ueff_x(0, 0) = J_ueff_x_data[0];
        J_ueff_x(0, 1) = J_ueff_x_data[1];
        J_ueff_x(0, 2) = J_ueff_x_data[2];
        J_ueff_x(1, 0) = J_ueff_x_data[3];
        J_ueff_x(1, 1) = J_ueff_x_data[4];
        J_ueff_x(1, 2) = J_ueff_x_data[5];

        const float* J_ueff_u_data = elements[4].toTensor().template data_ptr<float>();
        J_ueff_u(0, 0) = J_ueff_u_data[0];
        J_ueff_u(0, 1) = J_ueff_u_data[1];
        J_ueff_u(1, 0) = J_ueff_u_data[2];
        J_ueff_u(1, 1) = J_ueff_u_data[3];

        const float* J_k_x_data = elements[5].toTensor().template data_ptr<float>();
        J_k_x(0, 0) = J_k_x_data[0];
        J_k_x(0, 1) = J_k_x_data[1];
        J_k_x(0, 2) = J_k_x_data[2];

        const float* J_k_u_data = elements[6].toTensor().template data_ptr<float>();
        J_k_u(0, 0) = J_k_u_data[0];
        J_k_u(0, 1) = J_k_u_data[1];

        const float* J_r_x_data = elements[7].toTensor().template data_ptr<float>();
        J_r_x(0, 0) = J_r_x_data[0];
        J_r_x(0, 1) = J_r_x_data[1];
        J_r_x(0, 2) = J_r_x_data[2];
        J_r_x(1, 0) = J_r_x_data[3];
        J_r_x(1, 1) = J_r_x_data[4];
        J_r_x(1, 2) = J_r_x_data[5];
        J_r_x(2, 0) = J_r_x_data[6];
        J_r_x(2, 1) = J_r_x_data[7];
        J_r_x(2, 2) = J_r_x_data[8];

        const float* J_r_u_data = elements[8].toTensor().template data_ptr<float>();
        J_r_u(0, 0) = J_r_u_data[0];
        J_r_u(0, 1) = J_r_u_data[1];
        J_r_u(1, 0) = J_r_u_data[2];
        J_r_u(1, 1) = J_r_u_data[3];
        J_r_u(2, 0) = J_r_u_data[4];
        J_r_u(2, 1) = J_r_u_data[5];
      }
      else
      {
        // Extract outputs with direct pointer access (float64)
        const double* u_eff_data = elements[0].toTensor().template data_ptr<double>();
        u_eff(0) = u_eff_data[0];
        u_eff(1) = u_eff_data[1];
        friction_k = elements[1].toTensor().template data_ptr<double>()[0];
        const double* res_data = elements[2].toTensor().template data_ptr<double>();
        residual(0) = res_data[0];
        residual(1) = res_data[1];
        residual(2) = res_data[2];

        // Extract jacobians with direct pointer access (unrolled for small matrices)
        const double* J_ueff_x_data = elements[3].toTensor().template data_ptr<double>();
        J_ueff_x(0, 0) = J_ueff_x_data[0];
        J_ueff_x(0, 1) = J_ueff_x_data[1];
        J_ueff_x(0, 2) = J_ueff_x_data[2];
        J_ueff_x(1, 0) = J_ueff_x_data[3];
        J_ueff_x(1, 1) = J_ueff_x_data[4];
        J_ueff_x(1, 2) = J_ueff_x_data[5];

        const double* J_ueff_u_data = elements[4].toTensor().template data_ptr<double>();
        J_ueff_u(0, 0) = J_ueff_u_data[0];
        J_ueff_u(0, 1) = J_ueff_u_data[1];
        J_ueff_u(1, 0) = J_ueff_u_data[2];
        J_ueff_u(1, 1) = J_ueff_u_data[3];

        const double* J_k_x_data = elements[5].toTensor().template data_ptr<double>();
        J_k_x(0, 0) = J_k_x_data[0];
        J_k_x(0, 1) = J_k_x_data[1];
        J_k_x(0, 2) = J_k_x_data[2];

        const double* J_k_u_data = elements[6].toTensor().template data_ptr<double>();
        J_k_u(0, 0) = J_k_u_data[0];
        J_k_u(0, 1) = J_k_u_data[1];

        const double* J_r_x_data = elements[7].toTensor().template data_ptr<double>();
        J_r_x(0, 0) = J_r_x_data[0];
        J_r_x(0, 1) = J_r_x_data[1];
        J_r_x(0, 2) = J_r_x_data[2];
        J_r_x(1, 0) = J_r_x_data[3];
        J_r_x(1, 1) = J_r_x_data[4];
        J_r_x(1, 2) = J_r_x_data[5];
        J_r_x(2, 0) = J_r_x_data[6];
        J_r_x(2, 1) = J_r_x_data[7];
        J_r_x(2, 2) = J_r_x_data[8];

        const double* J_r_u_data = elements[8].toTensor().template data_ptr<double>();
        J_r_u(0, 0) = J_r_u_data[0];
        J_r_u(0, 1) = J_r_u_data[1];
        J_r_u(1, 0) = J_r_u_data[2];
        J_r_u(1, 1) = J_r_u_data[3];
        J_r_u(2, 0) = J_r_u_data[4];
        J_r_u(2, 1) = J_r_u_data[5];
      }
    }

    // Plant dynamics and Jacobians (normalized-plant semantics)
    // TorchScript now returns u_eff, residual, and their Jacobians already in normalized space.
    // Jacobians from TorchScript are d(output_norm)/d(input_raw), so no re-scaling needed.
    // Only xd0 (from caller) needs normalizing.
    StateDot xd0_norm = (xd0 - input_mean_x_).cwiseProduct(inv_input_std_x_);
    const Control& u_eff_norm = u_eff;
    const StateDot& residual_norm = residual;

    // Run plant on normalized inputs with identity params/poly and friction_k applied
    Params params_identity;
    params_identity.setConstant(1.0);
    const double base_friction = 1.0;
    params_identity[StructuredParams::friction] *= friction_k;

    Poly poly_identity;
    poly_identity.setZero();
    poly_identity[2] = 1.0;  // Identity polynomial: delta = 1.0 * x

    JacX plant_Jx_norm;
    JacU plant_Ju_norm;
    Eigen::Matrix<double, 3, 5> plant_Hparams;

    StateDot xd1_plant_norm = MushrPlant::predict(xd0_norm, u_eff_norm, dt_, params_identity, poly_identity,
                                                  plant_Jx_norm, plant_Ju_norm, boost::none, plant_Hparams);

    // Chain rule in normalized space (including friction sensitivity)
    // dx_norm/dx_raw = diag(1/input_std_x)
    const Eigen::Vector3d& plant_H_friction = plant_Hparams.col(StructuredParams::friction);

    // Jx_norm_raw = plant_Jx * diag(1/input_std_x) + plant_Ju * J_ueff_x + H_friction * base_friction * J_k_x + J_r_x
    JacX Jx_norm_raw = plant_Jx_norm * inv_input_std_x_.asDiagonal() + plant_Ju_norm * J_ueff_x +
                       plant_H_friction * base_friction * J_k_x + J_r_x;

    // Ju_norm_raw = plant_Ju * J_ueff_u + H_friction * base_friction * J_k_u + J_r_u
    JacU Ju_norm_raw = plant_Ju_norm * J_ueff_u + plant_H_friction * base_friction * J_k_u + J_r_u;

    // Unstandardize output Jacobians
    JacX Jx_raw = target_std_.asDiagonal() * Jx_norm_raw;
    JacU Ju_raw = target_std_.asDiagonal() * Ju_norm_raw;

    // Compute forward value
    StateDot xd1_norm = xd1_plant_norm + residual_norm;
    StateDot xd1_raw = xd1_norm.cwiseProduct(target_std_) + target_mean_;

    return std::make_tuple(xd1_raw, Jx_raw, Ju_raw);
  }

  // void set_dt(double dt)
  // {
  //   dt_ = dt;
  // }
  // double get_dt() const
  // {
  //   return dt_;
  // }

  bool has_jacobian_method() const override
  {
    return module_.find_method("forward_with_jacobian").has_value();
  }

  bool is_cuda_graph_enabled() const
  {
    return graph_captured_;
  }

private:
  // Lazy initialization of jacobian staging tensors (only when first needed)
  void ensure_jacobian_tensors_initialized()
  {
    if (jacobian_tensors_initialized_ || !use_cuda_)
      return;

    // GPU tensors for receiving jacobian outputs from model
    J_ueff_x_gpu_ = torch::empty({ 2, 3 }, torch::TensorOptions().dtype(dtype_).device(device_));
    J_ueff_u_gpu_ = torch::empty({ 2, 2 }, torch::TensorOptions().dtype(dtype_).device(device_));
    J_k_x_gpu_ = torch::empty({ 1, 3 }, torch::TensorOptions().dtype(dtype_).device(device_));
    J_k_u_gpu_ = torch::empty({ 1, 2 }, torch::TensorOptions().dtype(dtype_).device(device_));
    J_r_x_gpu_ = torch::empty({ 3, 3 }, torch::TensorOptions().dtype(dtype_).device(device_));
    J_r_u_gpu_ = torch::empty({ 3, 2 }, torch::TensorOptions().dtype(dtype_).device(device_));

    // Pinned CPU tensors for fast async transfer
    J_ueff_x_cpu_ = torch::empty({ 2, 3 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
    J_ueff_u_cpu_ = torch::empty({ 2, 2 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
    J_k_x_cpu_ = torch::empty({ 1, 3 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
    J_k_u_cpu_ = torch::empty({ 1, 2 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
    J_r_x_cpu_ = torch::empty({ 3, 3 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));
    J_r_u_cpu_ = torch::empty({ 3, 2 }, torch::TensorOptions().dtype(dtype_).pinned_memory(true));

    jacobian_tensors_initialized_ = true;
  }

  // Helper functions for pinned memory copies (no GPU transfer, just CPU pinned memory access)
  template <typename Derived>
  void copy_to_pinned_f32(torch::Tensor& tensor, const Eigen::MatrixBase<Derived>& vec)
  {
    float* data = tensor.data_ptr<float>();
    for (int i = 0; i < vec.size(); ++i)
    {
      data[i] = static_cast<float>(vec(i));
    }
  }

  template <typename Derived>
  void copy_to_pinned_f64(torch::Tensor& tensor, const Eigen::MatrixBase<Derived>& vec)
  {
    double* data = tensor.data_ptr<double>();
    for (int i = 0; i < vec.size(); ++i)
    {
      data[i] = vec(i);
    }
  }

  template <typename Derived>
  void copy_from_pinned_f32(Eigen::MatrixBase<Derived>& vec, const torch::Tensor& tensor)
  {
    const float* data = tensor.data_ptr<float>();
    for (int i = 0; i < vec.size(); ++i)
    {
      const_cast<typename Derived::Scalar&>(vec(i)) = static_cast<double>(data[i]);
    }
  }

  template <typename Derived>
  void copy_from_pinned_f64(Eigen::MatrixBase<Derived>& vec, const torch::Tensor& tensor)
  {
    const double* data = tensor.data_ptr<double>();
    for (int i = 0; i < vec.size(); ++i)
    {
      const_cast<typename Derived::Scalar&>(vec(i)) = data[i];
    }
  }

  // Matrix copy helpers for jacobians (row-major layout)
  template <typename Derived>
  void copy_from_pinned_matrix_f32(Eigen::MatrixBase<Derived>& mat, const torch::Tensor& tensor)
  {
    const float* data = tensor.data_ptr<float>();
    for (int i = 0; i < mat.rows(); ++i)
    {
      for (int j = 0; j < mat.cols(); ++j)
      {
        const_cast<typename Derived::Scalar&>(mat(i, j)) = static_cast<double>(data[i * mat.cols() + j]);
      }
    }
  }

  template <typename Derived>
  void copy_from_pinned_matrix_f64(Eigen::MatrixBase<Derived>& mat, const torch::Tensor& tensor)
  {
    const double* data = tensor.data_ptr<double>();
    for (int i = 0; i < mat.rows(); ++i)
    {
      for (int j = 0; j < mat.cols(); ++j)
      {
        const_cast<typename Derived::Scalar&>(mat(i, j)) = data[i * mat.cols() + j];
      }
    }
  }

  torch::jit::script::Module module_;
  torch::Device device_;
  torch::Dtype dtype_;
  bool use_float32_;
  bool use_cuda_;
  torch::Tensor xd_in_;
  torch::Tensor u_in_;

  // Cached method handle
  std::optional<torch::jit::Method> forward_with_jacobian_method_;

  // Pre-allocated input vector (avoids allocation per call)
  std::vector<torch::jit::IValue> inputs_;

  // Cached data pointers for CPU path (avoids data_ptr() call overhead)
  float* xd_in_ptr_f32_ = nullptr;
  float* u_in_ptr_f32_ = nullptr;
  double* xd_in_ptr_f64_ = nullptr;
  double* u_in_ptr_f64_ = nullptr;

  // CUDA Graph members
  bool graph_captured_ = false;
  // at::cuda::CUDAGraph cuda_graph_;
  // std::optional<at::cuda::CUDAStream> capture_stream_;

  // Pinned CPU staging tensors (for fast async GPU transfers)
  torch::Tensor xd_in_cpu_;
  torch::Tensor u_in_cpu_;
  torch::Tensor u_eff_out_cpu_;
  torch::Tensor k_out_cpu_;
  torch::Tensor residual_out_cpu_;

  // Pre-allocated GPU output tensors (used in graph capture)
  torch::Tensor u_eff_out_;
  torch::Tensor k_out_;
  torch::Tensor residual_out_;

  // Jacobian staging tensors (lazy-initialized for zero overhead when not used)
  bool jacobian_tensors_initialized_ = false;
  // GPU tensors for jacobian outputs
  torch::Tensor J_ueff_x_gpu_;  // 2x3
  torch::Tensor J_ueff_u_gpu_;  // 2x2
  torch::Tensor J_k_x_gpu_;     // 1x3
  torch::Tensor J_k_u_gpu_;     // 1x2
  torch::Tensor J_r_x_gpu_;     // 3x3
  torch::Tensor J_r_u_gpu_;     // 3x2
  // Pinned CPU tensors for async transfer
  torch::Tensor J_ueff_x_cpu_;
  torch::Tensor J_ueff_u_cpu_;
  torch::Tensor J_k_x_cpu_;
  torch::Tensor J_k_u_cpu_;
  torch::Tensor J_r_x_cpu_;
  torch::Tensor J_r_u_cpu_;

  // GPU plant computation (lazy-initialized, zero overhead when disabled)
  bool use_gpu_plant_ = false;
  bool gpu_plant_initialized_ = false;
  std::unique_ptr<GpuPlant> gpu_plant_;
  torch::Tensor xd1_out_;
  torch::Tensor xd1_out_cpu_;
  torch::Tensor plant_friction_;
  torch::Tensor plant_accel_gain_;
  torch::Tensor plant_dt_;

  // Hybrid mode: CUDA+Graph for predict(), CPU for jacobians
  bool hybrid_mode_ = false;
  std::string model_path_;  // Stored for loading CPU module in hybrid mode
  torch::jit::script::Module cpu_module_;
  torch::Tensor cpu_xd_in_;
  torch::Tensor cpu_u_in_;
  std::vector<torch::jit::IValue> cpu_inputs_;
  std::optional<torch::jit::Method> cpu_forward_with_jacobian_method_;
  float* cpu_xd_in_ptr_f32_ = nullptr;
  float* cpu_u_in_ptr_f32_ = nullptr;
  double* cpu_xd_in_ptr_f64_ = nullptr;
  double* cpu_u_in_ptr_f64_ = nullptr;

  Params params_;
  Poly poly_;
  const double dt_;

  // Standardizers for normalized-plant semantics
  Eigen::Vector3d input_mean_x_;
  Eigen::Vector2d input_mean_u_;
  Eigen::Vector3d input_std_x_;
  Eigen::Vector2d input_std_u_;
  Eigen::Vector3d target_mean_;
  Eigen::Vector3d target_std_;
  Eigen::Vector3d inv_input_std_x_;
  Eigen::Vector2d inv_input_std_u_;
  Eigen::Vector3d inv_target_std_;

  void extract_standardizers_from_module()
  {
    try
    {
      auto input_mean = module_.attr("input_mean").toTensor().to(torch::kCPU).to(torch::kFloat64);
      auto input_std = module_.attr("input_std").toTensor().to(torch::kCPU).to(torch::kFloat64);
      auto target_mean = module_.attr("target_mean").toTensor().to(torch::kCPU).to(torch::kFloat64);
      auto target_std = module_.attr("target_std").toTensor().to(torch::kCPU).to(torch::kFloat64);

      auto input_mean_acc = input_mean.template accessor<double, 1>();
      auto input_std_acc = input_std.template accessor<double, 1>();
      auto target_mean_acc = target_mean.template accessor<double, 1>();
      auto target_std_acc = target_std.template accessor<double, 1>();

      for (int i = 0; i < 3; ++i)
      {
        input_mean_x_(i) = input_mean_acc[i];
        input_std_x_(i) = input_std_acc[i];
        inv_input_std_x_(i) = 1.0 / input_std_acc[i];
        target_mean_(i) = target_mean_acc[i];
        target_std_(i) = target_std_acc[i];
        inv_target_std_(i) = 1.0 / target_std_acc[i];
      }

      for (int i = 0; i < 2; ++i)
      {
        input_mean_u_(i) = input_mean_acc[3 + i];
        input_std_u_(i) = input_std_acc[3 + i];
        inv_input_std_u_(i) = 1.0 / input_std_acc[3 + i];
      }

      // std::cout << "[StructuredSysidRuntime] Extracted standardizers from TorchScript module" << std::endl;
    }
    catch (const c10::Error& e)
    {
      throw std::runtime_error(
          "Failed to extract standardizer buffers from TorchScript module: " + std::string(e.what()) +
          ". Ensure the model was exported with StructuredAuxDeployModule.");
    }
  }
};

// ---------------------------------------------------------------------------
// JSON helpers (inline, header-only)
// ---------------------------------------------------------------------------
namespace detail
{

inline std::string parse_json_string_field(const std::string& file_path, const std::string& field_name)
{
  std::ifstream file(file_path);
  if (!file.is_open())
    return "";

  std::string line;
  std::string search_key = "\"" + field_name + "\"";
  while (std::getline(file, line))
  {
    auto key_pos = line.find(search_key);
    if (key_pos != std::string::npos)
    {
      auto colon_pos = line.find(':', key_pos);
      if (colon_pos != std::string::npos)
      {
        auto first_quote = line.find('"', colon_pos);
        auto second_quote = line.find('"', first_quote + 1);
        if (first_quote != std::string::npos && second_quote != std::string::npos)
          return line.substr(first_quote + 1, second_quote - first_quote - 1);
      }
    }
  }
  return "";
}

inline std::string parse_model_type_from_config(const std::string& config_path)
{
  std::string type = parse_json_string_field(config_path, "type");
  if (type == "structured" || type == "direct")
    return type;
  return "";
}

// ---------------------------------------------------------------------------
// Parse JSON string from raw JSON text (for embedded metadata)
// ---------------------------------------------------------------------------
inline std::string parse_json_string_from_text(const std::string& json_text, const std::string& field_name)
{
  std::string search_key = "\"" + field_name + "\"";
  auto key_pos = json_text.find(search_key);
  if (key_pos == std::string::npos)
    return "";
  auto colon_pos = json_text.find(':', key_pos);
  if (colon_pos == std::string::npos)
    return "";
  auto first_quote = json_text.find('"', colon_pos);
  auto second_quote = json_text.find('"', first_quote + 1);
  if (first_quote == std::string::npos || second_quote == std::string::npos)
    return "";
  return json_text.substr(first_quote + 1, second_quote - first_quote - 1);
}

inline double parse_json_double_from_text(const std::string& json_text, const std::string& field_name,
                                          double default_value)
{
  std::string search_key = "\"" + field_name + "\"";
  auto key_pos = json_text.find(search_key);
  if (key_pos == std::string::npos)
    return default_value;
  auto colon_pos = json_text.find(':', key_pos);
  if (colon_pos == std::string::npos)
    return default_value;
  auto start = json_text.find_first_not_of(" \t\n", colon_pos + 1);
  if (start == std::string::npos)
    return default_value;
  auto end = json_text.find_first_of(",}\n", start);
  try
  {
    return std::stod(json_text.substr(start, end - start));
  }
  catch (...)
  {
    return default_value;
  }
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Model metadata: embedded in the .ts.pt file or from external JSON
// ---------------------------------------------------------------------------
struct ModelMeta
{
  std::string model_type;
  std::string dtype;
  double dt = 0.1;
};

/// Read metadata embedded in a TorchScript .ts.pt file (via _extra_files).
/// Falls back to export_metadata.json / config.json in the same directory.
inline ModelMeta read_model_meta(const std::string& model_path)
{
  namespace fs = std::filesystem;

  // Try embedded metadata first
  std::unordered_map<std::string, std::string> extra_files = { { "metadata.json", "" } };
  try
  {
    torch::jit::load(model_path, torch::kCPU, extra_files);
  }
  catch (const c10::Error& e)
  {
    throw std::runtime_error("Failed to load TorchScript model for metadata: " + std::string(e.what()));
  }

  const std::string& meta_json = extra_files["metadata.json"];
  if (!meta_json.empty())
  {
    ModelMeta meta;
    meta.model_type = detail::parse_json_string_from_text(meta_json, "model_type");
    meta.dtype = detail::parse_json_string_from_text(meta_json, "dtype");
    meta.dt = detail::parse_json_double_from_text(meta_json, "dt", 0.1);
    if (meta.dtype.empty())
      meta.dtype = "float32";
    return meta;
  }

  // Fallback: look for export_metadata.json next to the model file
  fs::path dir = fs::path(model_path).parent_path();
  fs::path metadata_path = dir / "export_metadata.json";
  if (fs::exists(metadata_path))
  {
    ModelMeta meta;
    meta.model_type = detail::parse_json_string_field(metadata_path.string(), "model_type");
    meta.dtype = detail::parse_json_string_field(metadata_path.string(), "dtype");
    if (meta.dtype.empty())
      meta.dtype = "float32";
    return meta;
  }

  // Fallback: look for config.json
  fs::path config_path = dir / "config.json";
  if (fs::exists(config_path))
  {
    ModelMeta meta;
    meta.model_type = detail::parse_model_type_from_config(config_path.string());
    meta.dtype = "float32";
    return meta;
  }

  throw std::runtime_error("read_model_meta: no metadata found in " + model_path +
                           " (no embedded metadata.json, no export_metadata.json, no config.json)");
}

// ---------------------------------------------------------------------------
// Factory: create the right runtime from a .ts.pt model path
// ---------------------------------------------------------------------------

/// Creates a SysidRuntimeBase from a .ts.pt model path.
/// Reads model_type and dtype from metadata embedded in the model file.
/// Supports both direct and structured models.
template <typename MushrPlant, typename Params, typename Poly, typename StructuredParams>
std::unique_ptr<SysidRuntimeBase> create_sysid_runtime(const std::string& model_path, const Params& params,
                                                       const Poly& poly, bool use_cuda = false,
                                                       const std::string& dtype_override = "")
{
  ModelMeta meta = read_model_meta(model_path);
  const std::string& dtype = dtype_override.empty() ? meta.dtype : dtype_override;

  if (meta.model_type == "structured")
  {
    return std::make_unique<StructuredSysidRuntime<MushrPlant, Params, Poly, StructuredParams>>(model_path, params,
                                                                                                poly, use_cuda, dtype);
  }

  return std::make_unique<DirectSysidRuntime>(model_path, use_cuda, dtype);
}

/// Creates a SysidRuntimeBase for direct models only. Throws if the model
/// is a structured model.
inline std::unique_ptr<SysidRuntimeBase> create_sysid_runtime(const std::string& model_path, bool use_cuda = false,
                                                              const std::string& dtype_override = "")
{
  ModelMeta meta = read_model_meta(model_path);

  if (meta.model_type == "structured")
  {
    throw std::runtime_error(
        "create_sysid_runtime: structured models require plant template parameters. "
        "Use the template overload: create_sysid_runtime<MushrPlant, Params, Poly, StructuredParams>"
        "(model_path, params, poly, use_cuda, dtype).");
  }

  const std::string& dtype = dtype_override.empty() ? meta.dtype : dtype_override;
  return std::make_unique<DirectSysidRuntime>(model_path, use_cuda, dtype);
}

}  // namespace torch_bridge
#endif
