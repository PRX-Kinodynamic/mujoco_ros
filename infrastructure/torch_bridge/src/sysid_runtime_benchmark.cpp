#include <chrono>
#include <cstdlib>
#include <numeric>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <random>

#include <torch_bridge/query_utils.hpp>
#include <torch_bridge/sysid_runtime.hpp>

namespace fs = std::filesystem;

namespace
{

std::string parse_json_string_field(const std::string& file_path, const std::string& field_name)
{
  std::ifstream file(file_path);
  if (!file.is_open())
  {
    return "";
  }

  std::string line;
  std::string search_key = "\"" + field_name + "\"";
  while (std::getline(file, line))
  {
    size_t key_pos = line.find(search_key);
    if (key_pos != std::string::npos)
    {
      // Find the value after the colon
      size_t colon_pos = line.find(':', key_pos);
      if (colon_pos != std::string::npos)
      {
        size_t first_quote = line.find('"', colon_pos);
        size_t second_quote = line.find('"', first_quote + 1);
        if (first_quote != std::string::npos && second_quote != std::string::npos)
        {
          return line.substr(first_quote + 1, second_quote - first_quote - 1);
        }
      }
    }
  }
  return "";
}

double parse_json_double_field(const std::string& file_path, const std::string& field_name, double default_value)
{
  std::ifstream file(file_path);
  if (!file.is_open())
  {
    return default_value;
  }

  std::string line;
  std::string search_key = "\"" + field_name + "\"";
  while (std::getline(file, line))
  {
    size_t key_pos = line.find(search_key);
    if (key_pos != std::string::npos)
    {
      size_t colon_pos = line.find(':', key_pos);
      if (colon_pos != std::string::npos)
      {
        std::string value_part = line.substr(colon_pos + 1);
        // Remove whitespace and trailing comma
        size_t start = value_part.find_first_not_of(" \t");
        size_t end = value_part.find_first_of(",}");
        if (start != std::string::npos)
        {
          std::string value_str = value_part.substr(start, end - start);
          try
          {
            return std::stod(value_str);
          }
          catch (...)
          {
            return default_value;
          }
        }
      }
    }
  }
  return default_value;
}

std::string parse_model_type_from_config(const std::string& config_path)
{
  std::string type = parse_json_string_field(config_path, "type");
  if (type == "structured" || type == "direct")
  {
    return type;
  }
  return "";
}

}  // namespace

// class SysidBenchmark
// {
// public:
//   using StateDot = Eigen::Vector3d;
//   using Control = Eigen::Vector2d;
//   using JacX = Eigen::Matrix3d;
//   using JacU = Eigen::Matrix<double, 3, 2>;
//   using StructuredSysidRuntime = torch_bridge::StructuredSysidRuntime<>;
//   using Params = torch_bridge::StructuredSysidRuntime::Params;
//   using Poly = torch_bridge::StructuredSysidRuntime::Poly;

//   SysidBenchmark(ros::NodeHandle& nh)
//       : verbose_(false), total_calls_(100), warmup_calls_(10), use_ros_service_(true), dtype_("float32"),
//       use_cuda_(true),
//         use_gpu_plant_(false), use_cuda_graph_(true), use_hybrid_mode_(false)
//   {
//     nh.getParam("verbose", verbose_);
//     nh.getParam("total_calls", total_calls_);
//     nh.getParam("warmup_calls", warmup_calls_);
//     nh.getParam("use_ros_service", use_ros_service_);
//     nh.getParam("use_cuda", use_cuda_);
//     nh.getParam("use_gpu_plant", use_gpu_plant_);
//     nh.getParam("use_cuda_graph", use_cuda_graph_);
//     nh.getParam("use_hybrid_mode", use_hybrid_mode_);

//     // exp_dir is required - infer model_type, dtype, dt from it
//     std::string exp_dir;
//     if (!nh.getParam("exp_dir", exp_dir) || exp_dir.empty())
//     {
//       throw std::runtime_error("exp_dir parameter is required");
//     }

//     fs::path exp_path(exp_dir);
//     fs::path config_path = exp_path / "config.json";
//     fs::path metadata_path = exp_path / "export_metadata.json";

//     if (!fs::exists(config_path))
//     {
//       throw std::runtime_error("config.json not found at " + config_path.string());
//     }

//     // Try to read from export_metadata.json first (created by export_torchscript.py)
//     std::string model_type;
//     double dt = 0.05;

//     if (fs::exists(metadata_path))
//     {
//       model_type = parse_json_string_field(metadata_path.string(), "model_type");
//       std::string metadata_dtype = parse_json_string_field(metadata_path.string(), "dtype");
//       double metadata_dt = parse_json_double_field(metadata_path.string(), "dt", -1.0);

//       if (!metadata_dtype.empty())
//       {
//         dtype_ = metadata_dtype;
//         ROS_INFO("Using dtype='%s' from export_metadata.json", dtype_.c_str());
//       }
//       if (metadata_dt > 0)
//       {
//         dt = metadata_dt;
//         ROS_INFO("Using dt=%.4f from export_metadata.json", dt);
//       }
//       if (!model_type.empty())
//       {
//         ROS_INFO("Using model_type='%s' from export_metadata.json", model_type.c_str());
//       }
//     }

//     // Fall back to config.json for model_type if not in metadata
//     if (model_type.empty())
//     {
//       model_type = parse_model_type_from_config(config_path.string());
//       if (!model_type.empty())
//       {
//         ROS_INFO("Inferred model_type='%s' from config.json", model_type.c_str());
//       }
//     }

//     if (model_type.empty())
//     {
//       throw std::runtime_error("Could not infer model type. Expected 'model_type' in export_metadata.json or 'type'
//       in config.json");
//     }

//     // Allow param overrides for dtype and dt
//     nh.getParam("dtype", dtype_);
//     nh.getParam("dt", dt);

//     if (dtype_ != "float32" && dtype_ != "float64")
//     {
//       ROS_WARN("Invalid dtype '%s', must be 'float32' or 'float64'. Defaulting to 'float32'", dtype_.c_str());
//       dtype_ = "float32";
//     }

//     // Determine model file path based on type
//     std::string model_path;
//     if (model_type == "structured")
//     {
//       model_path = (exp_path / "structured_aux.ts.pt").string();
//     }
//     else
//     {
//       model_path = (exp_path / "direct_model.ts.pt").string();
//     }

//     if (!fs::exists(model_path))
//     {
//       throw std::runtime_error("Model file not found at " + model_path);
//     }

//     // Default plant params: [accel_gain, vel_desired_gain, friction, delta_offset, delta_gain]
//     std::vector<double> params_vec = { 1.0, 1.0, 1.0, 0.0, 1.0 };
//     nh.getParam("plant_params", params_vec);
//     if (params_vec.size() != 5)
//     {
//       ROS_WARN("plant_params must have 5 elements, using defaults");
//       params_vec = { 1.0, 1.0, 1.0, 0.0, 1.0 };
//     }
//     Params params;
//     for (int i = 0; i < 5; ++i)
//       params[i] = params_vec[i];

//     // Default steering polynomial coefficients [c0, c1, c2, c3] for poly(x) = c0*x^3 + c1*x^2 + c2*x + c3
//     std::vector<double> poly_vec = { 0.0, 0.0, 1.0, 0.0 };
//     nh.getParam("steering_poly", poly_vec);
//     if (poly_vec.size() != 4)
//     {
//       ROS_WARN("steering_poly must have 4 elements, using defaults");
//       poly_vec = { 0.0, 0.0, 1.0, 0.0 };
//     }
//     Poly poly;
//     for (int i = 0; i < 4; ++i)
//       poly[i] = poly_vec[i];

//     ROS_INFO("Loading TorchScript model from: %s", model_path.c_str());
//     ROS_INFO("Using dtype: %s", dtype_.c_str());
//     ROS_INFO("Model type: %s", model_type.c_str());

//     if (model_type == "structured")
//     {
//       structured_runtime_ = std::make_unique<torch_bridge::StructuredSysidRuntime>(model_path, params, poly,
//       use_cuda_, dtype_); structured_runtime_->set_dt(dt); has_structured_runtime_ = true; ROS_INFO("Successfully
//       loaded StructuredSysidRuntime with dt=%.4f", dt); ROS_INFO("Plant params: [%.3f, %.3f, %.3f, %.3f, %.3f]",
//       params[0], params[1], params[2], params[3], params[4]); ROS_INFO("Steering poly: [%.3f, %.3f, %.3f, %.3f]",
//       poly[0], poly[1], poly[2], poly[3]);

//       // Enable GPU plant if requested (keeps all computation on GPU)
//       if (use_gpu_plant_ && use_cuda_)
//       {
//         structured_runtime_->set_use_gpu_plant(true);
//         ROS_INFO("GPU plant dynamics enabled");
//       }

//       // Enable hybrid mode: CUDA+Graph for predict(), CPU for jacobians
//       if (use_hybrid_mode_ && use_cuda_)
//       {
//         structured_runtime_->enable_hybrid_mode();
//         ROS_INFO("Hybrid mode enabled (CUDA+Graph for predict, CPU for jacobians)");
//       }
//       // Capture CUDA graph for optimized inference (if not using hybrid mode, which does this automatically)
//       else if (use_cuda_ && use_cuda_graph_)
//       {
//         structured_runtime_->warmup_cuda_graph();
//         ROS_INFO("CUDA graph captured successfully");
//       }
//     }
//     else
//     {
//       direct_runtime_ = std::make_unique<torch_bridge::DirectSysidRuntime>(model_path, use_cuda_, dtype_);
//       has_direct_runtime_ = true;
//       ROS_INFO("Successfully loaded DirectSysidRuntime");
//     }

//     if (use_ros_service_)
//     {
//       torch_service_client_ = nh.serviceClient<torch_bridge::TorchQuery>("/torch/service", true);
//       torch_service_call_.request.inputs = 2;
//       torch_service_call_.request.input_dimensions = { 3, 2 };
//     }

//     // Pre-generate inputs so the benchmark measures inference, not RNG overhead.
//     generate_samples();
//   }

//   void run_ros_service_benchmark(bool with_jacobians)
//   {
//     if (!torch_service_client_.exists())
//     {
//       ROS_WARN("ROS Torch service not available, skipping ROS benchmark");
//       return;
//     }

//     for (int i = 0; i < warmup_calls_; ++i)
//     {
//       set_sample(i);
//       call_ros_service(with_jacobians);
//     }

//     std::vector<double> durations;
//     for (int i = 0; i < total_calls_; ++i)
//     {
//       set_sample(warmup_calls_ + i);
//       const auto start = std::chrono::high_resolution_clock::now();
//       call_ros_service(with_jacobians);
//       const auto end = std::chrono::high_resolution_clock::now();
//       durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
//     }

//     print_stats("ROS Service", with_jacobians, durations);
//   }

//   void run_inprocess_benchmark(bool with_jacobians)
//   {
//     if (!has_direct_runtime_)
//     {
//       ROS_WARN("In-process runtime not available, skipping in-process benchmark");
//       return;
//     }

//     for (int i = 0; i < warmup_calls_; ++i)
//     {
//       set_sample(i);
//       call_inprocess(with_jacobians);
//     }

//     std::vector<double> durations;
//     for (int i = 0; i < total_calls_; ++i)
//     {
//       set_sample(warmup_calls_ + i);
//       const auto start = std::chrono::high_resolution_clock::now();
//       call_inprocess(with_jacobians);
//       const auto end = std::chrono::high_resolution_clock::now();
//       durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
//     }

//     print_stats("In-Process Direct", with_jacobians, durations);
//   }

//   void run_structured_benchmark(bool with_jacobians)
//   {
//     if (!has_structured_runtime_)
//     {
//       ROS_WARN("Structured runtime not available, skipping structured benchmark");
//       return;
//     }

//     for (int i = 0; i < warmup_calls_; ++i)
//     {
//       set_sample(i);
//       call_structured(with_jacobians);
//     }

//     std::vector<double> durations;
//     for (int i = 0; i < total_calls_; ++i)
//     {
//       set_sample(warmup_calls_ + i);
//       const auto start = std::chrono::high_resolution_clock::now();
//       call_structured(with_jacobians);
//       const auto end = std::chrono::high_resolution_clock::now();
//       durations.push_back(std::chrono::duration<double, std::milli>(end - start).count());
//     }

//     print_stats("In-Process Structured", with_jacobians, durations);
//   }

//   void run_all()
//   {
//     std::cout << "\n========================================\n";
//     std::cout << "Sysid Runtime Benchmark\n";
//     std::cout << "Dtype: " << dtype_ << "\n";
//     std::cout << "CUDA: " << (use_cuda_ ? "enabled" : "disabled") << "\n";
//     std::cout << "CUDA Graph: " << ((use_cuda_graph_ || use_hybrid_mode_) && use_cuda_ ? "enabled" : "disabled") <<
//     "\n"; std::cout << "GPU Plant: " << (use_gpu_plant_ && use_cuda_ ? "enabled" : "disabled") << "\n"; std::cout <<
//     "Hybrid Mode: " << (use_hybrid_mode_ && use_cuda_ ? "enabled" : "disabled") << "\n"; std::cout << "Torch threads:
//     " << torch::get_num_threads() << "\n"; std::cout << "Total calls: " << total_calls_ << "\n"; std::cout << "Warmup
//     calls: " << warmup_calls_ << "\n"; std::cout << "========================================\n\n";

//     if (use_ros_service_ && torch_service_client_.exists())
//     {
//       run_ros_service_benchmark(false);
//       run_ros_service_benchmark(true);
//     }

//     if (has_direct_runtime_)
//     {
//       run_inprocess_benchmark(false);
//       if (direct_runtime_->has_jacobian_method())
//       {
//         run_inprocess_benchmark(true);
//       }
//       else
//       {
//         ROS_INFO("Model does not have 'forward_with_jacobian' method, skipping Jacobian benchmark");
//       }
//     }

//     if (has_structured_runtime_)
//     {
//       run_structured_benchmark(false);
//       if (structured_runtime_->has_jacobian_method())
//       {
//         run_structured_benchmark(true);
//       }
//       else
//       {
//         ROS_INFO("Structured model does not have 'forward_with_jacobian' method, skipping Jacobian benchmark");
//       }
//     }

//     std::cout << "\n========================================\n";
//     std::cout << "Benchmark Complete\n";
//     std::cout << "========================================\n";
//   }

// private:
//   void generate_samples()
//   {
//     const int n = warmup_calls_ + total_calls_;
//     xi_samples_.resize(n);
//     ui_samples_.resize(n);

//     std::mt19937 rng(42);
//     std::normal_distribution<double> dist_x(0.0, 1.0);
//     std::normal_distribution<double> dist_u(0.0, 0.5);

//     for (int i = 0; i < n; ++i)
//     {
//       xi_samples_[i] = StateDot(dist_x(rng), dist_x(rng), dist_x(rng));
//       ui_samples_[i] = Control(dist_u(rng), dist_u(rng));
//     }
//   }

//   void set_sample(int idx)
//   {
//     xi_ = xi_samples_[static_cast<size_t>(idx)];
//     ui_ = ui_samples_[static_cast<size_t>(idx)];
//   }

//   void call_ros_service(bool with_jacobians)
//   {
//     torch_service_call_.request.compute_jacobians = with_jacobians;
//     torch_service_call_.request.data.clear();
//     torch_bridge::update_request(torch_service_call_, xi_, ui_);

//     if (torch_service_client_.call(torch_service_call_))
//     {
//       torch_bridge::get_result(torch_service_call_, x_res_);
//       if (with_jacobians)
//       {
//         torch_bridge::get_jacobian(torch_service_call_, dres_dx_, dres_du_);
//       }
//     }
//   }

//   void call_inprocess(bool with_jacobians)
//   {
//     if (with_jacobians)
//     {
//       auto [xd1, Jx, Ju] = direct_runtime_->predict_with_jac(xi_, ui_);
//       x_res_ = xd1;
//       dres_dx_ = Jx;
//       dres_du_ = Ju;
//     }
//     else
//     {
//       x_res_ = direct_runtime_->predict(xi_, ui_);
//     }
//   }

//   void call_structured(bool with_jacobians)
//   {
//     if (with_jacobians)
//     {
//       auto [xd1, Jx, Ju] = structured_runtime_->predict_with_jac(xi_, ui_);
//       x_res_ = xd1;
//       dres_dx_ = Jx;
//       dres_du_ = Ju;
//     }
//     else
//     {
//       x_res_ = structured_runtime_->predict(xi_, ui_);
//     }
//   }

//   void print_stats(const std::string& method, bool with_jacobians, const std::vector<double>& durations)
//   {
//     const auto [min_it, max_it] = std::minmax_element(durations.begin(), durations.end());
//     const size_t min_idx = static_cast<size_t>(std::distance(durations.begin(), min_it));
//     const size_t max_idx = static_cast<size_t>(std::distance(durations.begin(), max_it));
//     const double sum = std::accumulate(durations.begin(), durations.end(), 0.0);
//     const double mean = sum / durations.size();

//     std::vector<double> sorted_durations = durations;
//     std::sort(sorted_durations.begin(), sorted_durations.end());
//     const double p50 = sorted_durations[sorted_durations.size() / 2];
//     const double p95 = sorted_durations[static_cast<size_t>(sorted_durations.size() * 0.95)];
//     const double p99 = sorted_durations[static_cast<size_t>(sorted_durations.size() * 0.99)];

//     std::cout << method << (with_jacobians ? " [With Jacobians]" : " [Predict Only]") << ":\n";
//     std::cout << std::fixed << std::setprecision(3);
//     std::cout << "  Min:  " << std::setw(8) << *min_it << " ms (iter " << min_idx << ")\n";
//     std::cout << "  Max:  " << std::setw(8) << *max_it << " ms (iter " << max_idx << (max_idx == 0 ? " - FIRST CALL"
//     : "") << ")\n"; std::cout << "  Mean: " << std::setw(8) << mean << " ms\n"; std::cout << "  P50:  " <<
//     std::setw(8) << p50 << " ms\n"; std::cout << "  P95:  " << std::setw(8) << p95 << " ms\n"; std::cout << "  P99:
//     " << std::setw(8) << p99 << " ms\n"; std::cout << "\n";
//   }

//   ros::ServiceClient torch_service_client_;
//   torch_bridge::TorchQuery torch_service_call_;

//   std::unique_ptr<torch_bridge::DirectSysidRuntime> direct_runtime_;
//   std::unique_ptr<torch_bridge::StructuredSysidRuntime> structured_runtime_;
//   bool has_direct_runtime_ = false;
//   bool has_structured_runtime_ = false;

//   StateDot xi_;
//   Control ui_;
//   StateDot x_res_;
//   JacX dres_dx_;
//   JacU dres_du_;

//   std::vector<StateDot> xi_samples_;
//   std::vector<Control> ui_samples_;

//   bool verbose_;
//   int total_calls_;
//   int warmup_calls_;
//   bool use_ros_service_;
//   std::string dtype_;
//   bool use_cuda_;
//   bool use_gpu_plant_;
//   bool use_cuda_graph_;
//   bool use_hybrid_mode_;
// };

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

  // SysidBenchmark benchmark(nh);
  // benchmark.run_all();

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
