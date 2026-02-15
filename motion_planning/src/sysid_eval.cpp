/**
 * @file sysid_eval.cpp
 * @brief Comprehensive evaluation of sysid models on data_eval trajectories (C++ version)
 *
 * This is a C++ port of eval_on_data_eval_comprehensive.py that evaluates:
 * 1. One-step predictions (from each state, predict next state)
 * 2. N-step rollouts (from each state, forward propagate N steps)
 * 3. Full trajectory rollouts (from initial state, propagate entire trajectory)
 *
 * For each horizon type, it computes:
 * - Final state error (error at end of prediction horizon)
 * - Error along horizon (aggregated with mean, variance, median)
 * - Per-dimension errors and aggregated metrics
 */

#ifndef TORCH_NOT_BUILT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <torch_bridge/sysid_runtime.hpp>
#include <prx_models/mushr_factors.hpp>

namespace fs = std::filesystem;

// ============================================================================
// Statistics utilities
// ============================================================================

struct AggregatedStats
{
  double mean;
  double median;
  double variance;
  double p90;
  double p99;

  static AggregatedStats compute(std::vector<double> values)
  {
    AggregatedStats stats{};
    if (values.empty())
    {
      stats.mean = stats.median = stats.variance = stats.p90 = stats.p99 = std::nan("");
      return stats;
    }

    // Sort for percentiles
    std::sort(values.begin(), values.end());
    size_t n = values.size();

    // Mean
    stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / n;

    // Median
    if (n % 2 == 0)
    {
      stats.median = (values[n / 2 - 1] + values[n / 2]) / 2.0;
    }
    else
    {
      stats.median = values[n / 2];
    }

    // Variance
    double sum_sq = 0.0;
    for (double v : values)
    {
      sum_sq += (v - stats.mean) * (v - stats.mean);
    }
    stats.variance = sum_sq / n;

    // Percentiles
    stats.p90 = values[static_cast<size_t>(n * 0.90)];
    stats.p99 = values[static_cast<size_t>(n * 0.99)];

    return stats;
  }

  void print(const std::string& prefix, std::ostream& os = std::cout) const
  {
    os << std::fixed << std::setprecision(6);
    os << prefix << "mean=" << mean << ", median=" << median << ", var=" << variance << ", p90=" << p90
       << ", p99=" << p99 << "\n";
  }
};

// ============================================================================
// Angle utilities
// ============================================================================

inline double angle_diff(double a, double b)
{
  double d = a - b;
  while (d > M_PI)
    d -= 2.0 * M_PI;
  while (d < -M_PI)
    d += 2.0 * M_PI;
  return d;
}

// ============================================================================
// File parsing utilities
// ============================================================================

struct PlanSegment
{
  double steering;
  double velocity;
  double duration;
  double start_time;
};

std::vector<PlanSegment> read_plan(const std::string& path)
{
  std::vector<PlanSegment> segments;
  std::ifstream file(path);
  if (!file.is_open())
  {
    std::cerr << "Failed to open plan file: " << path << "\n";
    return segments;
  }

  std::string line;
  while (std::getline(file, line))
  {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#')
      continue;

    std::istringstream iss(line);
    PlanSegment seg;
    iss >> seg.steering >> seg.velocity >> seg.duration;
    if (iss >> seg.start_time)
    {
      // start_time was provided
    }
    else
    {
      seg.start_time = 0.0;
    }
    segments.push_back(seg);
  }

  return segments;
}

void build_control_sequence(const std::vector<PlanSegment>& segments, double total_time, double dt,
                            std::vector<Eigen::Vector2d>& controls, std::vector<double>& times)
{
  int N = static_cast<int>(std::round(total_time / dt));
  controls.resize(N);
  times.resize(N + 1);

  for (int i = 0; i <= N; ++i)
  {
    times[i] = i * dt;
  }

  for (int i = 0; i < N; ++i)
  {
    double t = i * dt;
    double ctrl_steer = 0.0;
    double ctrl_vel = 0.0;

    for (const auto& seg : segments)
    {
      double seg_start = seg.start_time;
      double seg_end = seg_start + seg.duration;
      if (seg_start <= t && t < seg_end)
      {
        ctrl_steer = seg.steering;
        ctrl_vel = seg.velocity;
        break;
      }
    }

    // Control is [velocity, steering]
    controls[i] = Eigen::Vector2d(ctrl_vel, ctrl_steer);
  }
}

struct MJTrajectory
{
  std::vector<double> times;
  std::vector<Eigen::Vector2d> xy;
  std::vector<double> theta;
};

MJTrajectory read_mj_traj(const std::string& path)
{
  MJTrajectory traj;
  std::ifstream file(path);
  if (!file.is_open())
  {
    std::cerr << "Failed to open MJ trajectory file: " << path << "\n";
    return traj;
  }

  double cumulative_time = 0.0;
  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    std::istringstream iss(line);
    double dt_val, x, y, col3, cos_th, col5, col6, sin_th;
    if (!(iss >> dt_val >> x >> y >> col3 >> cos_th >> col5 >> col6 >> sin_th))
    {
      continue;
    }

    cumulative_time += dt_val;
    traj.times.push_back(cumulative_time);
    traj.xy.push_back(Eigen::Vector2d(x, y));
    traj.theta.push_back(std::atan2(sin_th, cos_th));
  }

  return traj;
}

// Linear interpolation for trajectories
double interp(const std::vector<double>& x_data, const std::vector<double>& y_data, double x)
{
  if (x_data.empty())
    return std::nan("");
  if (x <= x_data.front())
    return y_data.front();
  if (x >= x_data.back())
    return y_data.back();

  // Find interval
  auto it = std::lower_bound(x_data.begin(), x_data.end(), x);
  size_t i = static_cast<size_t>(it - x_data.begin());
  if (i == 0)
    i = 1;

  double t = (x - x_data[i - 1]) / (x_data[i] - x_data[i - 1]);
  return y_data[i - 1] + t * (y_data[i] - y_data[i - 1]);
}

// ============================================================================
// SE2 Integration (matching Python MushrPlant)
// ============================================================================

class SE2Integrator
{
public:
  static Eigen::Matrix3d SE2(double x, double y, double theta)
  {
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T(0, 0) = std::cos(theta);
    T(0, 1) = -std::sin(theta);
    T(0, 2) = x;
    T(1, 0) = std::sin(theta);
    T(1, 1) = std::cos(theta);
    T(1, 2) = y;
    return T;
  }

  static Eigen::Matrix3d rotation_mat_2d(double theta)
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0, 0) = std::cos(theta);
    R(0, 1) = -std::sin(theta);
    R(1, 0) = std::sin(theta);
    R(1, 1) = std::cos(theta);
    return R;
  }

  static Eigen::Matrix3d SE2_expmap(const Eigen::Vector3d& xd)
  {
    double vx = xd[0];
    double vy = xd[1];
    double w = xd[2];

    if (std::abs(w) < 1e-10)
    {
      return SE2(vx, vy, w);
    }

    // R = rotation matrix for angle w
    double c = std::cos(w);
    double s = std::sin(w);

    // v_ortho = [-vy, vx]
    double vox = -vy;
    double voy = vx;

    // R * v_ortho
    double Rvox = c * vox - s * voy;
    double Rvoy = s * vox + c * voy;

    // t = (v_ortho - R*v_ortho) / w
    double tx = (vox - Rvox) / w;
    double ty = (voy - Rvoy) / w;

    return SE2(tx, ty, w);
  }

  static Eigen::Matrix3d integrate_SE2(const Eigen::Matrix3d& pose, const Eigen::Vector3d& xdot, double dt)
  {
    Eigen::Vector3d xdot_dt = xdot * dt;
    Eigen::Matrix3d exp_map = SE2_expmap(xdot_dt);
    return pose * exp_map;
  }

  static Eigen::Vector3d pose_from_matrix(const Eigen::Matrix3d& T)
  {
    double x = T(0, 2);
    double y = T(1, 2);
    double theta = std::atan2(T(1, 0), T(0, 0));
    return Eigen::Vector3d(x, y, theta);
  }
};

// ============================================================================
// Evaluation results structures
// ============================================================================

struct TrajectoryErrors
{
  double avg_pos_err;
  double max_pos_err;
  double final_pos_err;
  double t1s_pos_err;  // Error at t=1s
  double avg_angle_err;
  double max_angle_err;
  double final_angle_err;
  double t1s_angle_err;
};

TrajectoryErrors compute_trajectory_errors(const MJTrajectory& gt, const std::vector<double>& pred_times,
                                           const std::vector<Eigen::Vector2d>& pred_xy,
                                           const std::vector<double>& pred_theta)
{
  TrajectoryErrors errs{};

  if (gt.times.empty() || pred_times.empty())
  {
    errs.avg_pos_err = errs.max_pos_err = errs.final_pos_err = errs.t1s_pos_err = std::nan("");
    errs.avg_angle_err = errs.max_angle_err = errs.final_angle_err = errs.t1s_angle_err = std::nan("");
    return errs;
  }

  // Extract x, y coordinates for interpolation
  std::vector<double> pred_x(pred_xy.size()), pred_y(pred_xy.size());
  for (size_t i = 0; i < pred_xy.size(); ++i)
  {
    pred_x[i] = pred_xy[i][0];
    pred_y[i] = pred_xy[i][1];
  }

  std::vector<double> pos_errors;
  std::vector<double> angle_errors;

  size_t t1s_idx = 0;
  double min_t1s_diff = std::abs(gt.times[0] - 1.0);

  for (size_t i = 0; i < gt.times.size(); ++i)
  {
    double t = gt.times[i];

    // Interpolate prediction at GT time
    double px = interp(pred_times, pred_x, t);
    double py = interp(pred_times, pred_y, t);
    double pth = interp(pred_times, pred_theta, t);

    double err_x = gt.xy[i][0] - px;
    double err_y = gt.xy[i][1] - py;
    double err_th = std::abs(angle_diff(gt.theta[i], pth));

    double pos_err = std::sqrt(err_x * err_x + err_y * err_y);
    pos_errors.push_back(pos_err);
    angle_errors.push_back(err_th);

    // Track t=1s index
    if (std::abs(t - 1.0) < min_t1s_diff)
    {
      min_t1s_diff = std::abs(t - 1.0);
      t1s_idx = i;
    }
  }

  errs.avg_pos_err = std::accumulate(pos_errors.begin(), pos_errors.end(), 0.0) / pos_errors.size();
  errs.max_pos_err = *std::max_element(pos_errors.begin(), pos_errors.end());
  errs.final_pos_err = pos_errors.back();
  errs.t1s_pos_err = pos_errors[t1s_idx];

  errs.avg_angle_err = std::accumulate(angle_errors.begin(), angle_errors.end(), 0.0) / angle_errors.size();
  errs.max_angle_err = *std::max_element(angle_errors.begin(), angle_errors.end());
  errs.final_angle_err = angle_errors.back();
  errs.t1s_angle_err = angle_errors[t1s_idx];

  return errs;
}

// ============================================================================
// Evaluation class using StructuredSysidRuntime
// ============================================================================

template <typename MushrPlant, typename Params, typename Poly, typename StructuredParams>
class SysidEvaluator
{
public:
  using StateDot = Eigen::Vector3d;
  using Control = Eigen::Vector2d;
  using Runtime = torch_bridge::StructuredSysidRuntime<MushrPlant, Params, Poly, StructuredParams>;

  SysidEvaluator(const std::string& model_path, const Params& params, const Poly& poly, double dt,
                 bool use_cuda = false, const std::string& dtype = "float32", bool use_normalized_plant = true)
    : runtime_(model_path, params, poly, use_cuda, dtype, use_normalized_plant), dt_(dt)
  {
  }

  // Full trajectory rollout from initial state (0,0,0) with zero velocity
  void full_trajectory_rollout(const std::vector<Control>& controls, std::vector<Eigen::Vector3d>& poses,
                               std::vector<StateDot>& velocities)
  {
    int N = static_cast<int>(controls.size());
    poses.resize(N + 1);
    velocities.resize(N + 1);

    // Initial state
    StateDot xd_curr = StateDot::Zero();
    Eigen::Matrix3d pose_mat = SE2Integrator::SE2(0, 0, 0);

    poses[0] = SE2Integrator::pose_from_matrix(pose_mat);
    velocities[0] = xd_curr;

    for (int step = 0; step < N; ++step)
    {
      StateDot xd_next = runtime_.predict(xd_curr, controls[step]);
      pose_mat = SE2Integrator::integrate_SE2(pose_mat, xd_next, dt_);

      poses[step + 1] = SE2Integrator::pose_from_matrix(pose_mat);
      velocities[step + 1] = xd_next;
      xd_curr = xd_next;
    }
  }

  // One-step prediction evaluation
  void evaluate_one_step(const std::vector<StateDot>& gt_xd, const std::vector<Eigen::Vector3d>& gt_pose,
                         const std::vector<Control>& controls, std::vector<double>& pos_errors,
                         std::vector<double>& vel_errors, std::vector<double>& angle_errors)
  {
    int N = static_cast<int>(gt_xd.size()) - 1;
    if (N <= 0)
      return;

    pos_errors.clear();
    vel_errors.clear();
    angle_errors.clear();

    for (int t = 0; t < N; ++t)
    {
      // Predict next velocity from GT state
      StateDot xd_pred = runtime_.predict(gt_xd[t], controls[t]);

      // Integrate pose
      Eigen::Matrix3d pose_mat = SE2Integrator::SE2(gt_pose[t][0], gt_pose[t][1], gt_pose[t][2]);
      pose_mat = SE2Integrator::integrate_SE2(pose_mat, xd_pred, dt_);
      Eigen::Vector3d pred_pose = SE2Integrator::pose_from_matrix(pose_mat);

      // Compute position errors
      double dx = gt_pose[t + 1][0] - pred_pose[0];
      double dy = gt_pose[t + 1][1] - pred_pose[1];
      pos_errors.push_back(std::sqrt(dx * dx + dy * dy));

      // Compute angle error
      double angle_err = std::abs(angle_diff(gt_pose[t + 1][2], pred_pose[2]));
      angle_errors.push_back(angle_err);

      Eigen::Vector3d vel_err = gt_xd[t + 1] - xd_pred;
      vel_errors.push_back(vel_err.norm());
    }
  }

  // N-step rollout evaluation
  void evaluate_n_step_rollout(const std::vector<StateDot>& gt_xd, const std::vector<Eigen::Vector3d>& gt_pose,
                               const std::vector<Control>& controls, int horizon,
                               std::vector<double>& final_pos_errors, std::vector<double>& final_vel_errors,
                               std::vector<double>& final_angle_errors)
  {
    int T = static_cast<int>(gt_xd.size());
    int num_rollouts = T - horizon;
    if (num_rollouts <= 0)
      return;

    final_pos_errors.clear();
    final_vel_errors.clear();
    final_angle_errors.clear();

    for (int start = 0; start < num_rollouts; ++start)
    {
      // Rollout from gt_xd[start] for horizon steps
      StateDot xd_curr = gt_xd[start];
      Eigen::Matrix3d pose_mat = SE2Integrator::SE2(gt_pose[start][0], gt_pose[start][1], gt_pose[start][2]);

      for (int step = 0; step < horizon; ++step)
      {
        StateDot xd_next = runtime_.predict(xd_curr, controls[start + step]);
        pose_mat = SE2Integrator::integrate_SE2(pose_mat, xd_next, dt_);
        xd_curr = xd_next;
      }

      Eigen::Vector3d final_pred_pose = SE2Integrator::pose_from_matrix(pose_mat);

      // Final position errors
      double dx = gt_pose[start + horizon][0] - final_pred_pose[0];
      double dy = gt_pose[start + horizon][1] - final_pred_pose[1];
      final_pos_errors.push_back(std::sqrt(dx * dx + dy * dy));

      // Final angle error
      double angle_err = std::abs(angle_diff(gt_pose[start + horizon][2], final_pred_pose[2]));
      final_angle_errors.push_back(angle_err);

      Eigen::Vector3d vel_err = gt_xd[start + horizon] - xd_curr;
      final_vel_errors.push_back(vel_err.norm());
    }
  }

private:
  Runtime runtime_;
  double dt_;
};

// ============================================================================
// MuSHR Structured Parameters (needed for StructuredSysidRuntime)
// ============================================================================

struct MushrStructuredParams
{
  static constexpr std::size_t friction = prx_models::mushr_types::Control::friction;
  static constexpr std::size_t vel_desired = prx_models::mushr_types::Control::vel_desired;
  static constexpr double L = prx_models::mushr_types::Parameters::L;
};

// ============================================================================
// Main evaluation function
// ============================================================================

void run_evaluation(ros::NodeHandle& nh)
{
  std::string exp_dir, data_eval_dir, output_dir;
  double dt = 0.1;
  double total_time = 10.0;
  int rollout_horizon = 10;
  bool use_cuda = torch::cuda::is_available();  // Auto-detect CUDA
  std::string dtype = "float32";
  bool use_normalized_plant = true;  // Default: match python-model semantics

  nh.getParam("exp_dir", exp_dir);
  nh.getParam("data_eval_dir", data_eval_dir);
  nh.getParam("output_dir", output_dir);
  nh.getParam("dt", dt);
  nh.getParam("total_time", total_time);
  nh.getParam("rollout_horizon", rollout_horizon);
  nh.getParam("use_cuda", use_cuda);  // Can override auto-detect
  nh.getParam("dtype", dtype);
  nh.getParam("use_normalized_plant", use_normalized_plant);

  if (exp_dir.empty())
  {
    ROS_ERROR("exp_dir parameter is required");
    return;
  }

  if (data_eval_dir.empty())
  {
    data_eval_dir = "/common/home/st1122/Projects/mushr_mujoco_sysid/data_eval";
  }

  fs::path exp_path(exp_dir);
  fs::path model_path = exp_path / "structured_aux.ts.pt";

  if (!fs::exists(model_path))
  {
    ROS_ERROR_STREAM("Model not found at: " << model_path);
    return;
  }

  // Default parameters (matching mushr_stela_t::default_params)
  using Params = prx_models::mushr_types::Control::params;
  using Poly = prx_models::mushr_types::Control::Poly;

  // Try to load params from config or use defaults
  Params params;
  params << 1.0, 1.0, 1.0, 0.0, 1.0;

  Poly poly;
  poly << -0.4397, 3.773e-5, 0.8677, 5.8e-6;

  using MushrPlant = prx_models::mushr_CtrlAccel_t<>;
  using StructuredParams = MushrStructuredParams;

  ROS_INFO_STREAM("Loading model from: " << model_path);
  ROS_INFO_STREAM("Data eval dir: " << data_eval_dir);
  ROS_INFO_STREAM("Device: " << (use_cuda ? "CUDA" : "CPU"));
  ROS_INFO_STREAM("dt=" << dt << ", total_time=" << total_time << ", rollout_horizon=" << rollout_horizon);
  ROS_INFO_STREAM("use_normalized_plant=" << (use_normalized_plant ? "true (matches python-model)" : "false (raw-plant)"));

  SysidEvaluator<MushrPlant, Params, Poly, StructuredParams> evaluator(model_path.string(), params, poly, dt, use_cuda,
                                                                       dtype, use_normalized_plant);

  // Find trajectory indices
  fs::path mj_dir = fs::path(data_eval_dir) / "mj_trajs";
  std::vector<int> indices;

  for (const auto& entry : fs::directory_iterator(mj_dir))
  {
    std::string fname = entry.path().filename().string();
    if (fname.rfind("plan_", 0) == 0 && fname.size() > 8)
    {
      std::string idx_str = fname.substr(5, 3);
      try
      {
        indices.push_back(std::stoi(idx_str));
      }
      catch (...)
      {
      }
    }
  }
  std::sort(indices.begin(), indices.end());

  ROS_INFO_STREAM("Found " << indices.size() << " trajectories to evaluate");

  // Output directory - default to ROS workspace data/sysid_eval/
  if (output_dir.empty())
  {
    std::string ros_ws = "/common/home/st1122/Projects/ros_workspace";
    output_dir = (fs::path(ros_ws) / "data" / "sysid_eval" / exp_path.filename()).string();
  }
  fs::create_directories(output_dir);

  // Collectors for statistics - POSITION
  std::vector<double> all_1step_pos_errors;
  std::vector<double> all_1step_vel_errors;
  std::vector<double> all_nstep_final_pos_errors;
  std::vector<double> all_nstep_final_vel_errors;
  std::vector<double> all_full_avg_pos_errors;
  std::vector<double> all_full_final_pos_errors;
  std::vector<double> all_full_max_pos_errors;

  // Collectors for statistics - ANGLE
  std::vector<double> all_full_avg_angle_errors;
  std::vector<double> all_full_final_angle_errors;
  std::vector<double> all_full_max_angle_errors;
  std::vector<double> all_1step_angle_errors;
  std::vector<double> all_nstep_final_angle_errors;

  // For plotting: per-trajectory data
  struct TrajPlotData
  {
    int idx;
    std::vector<double> times;
    std::vector<double> gt_x, gt_y, gt_theta;
    std::vector<double> pred_x, pred_y, pred_theta;
    std::vector<double> pos_errors, angle_errors;
    double final_pos_err, final_angle_err;
  };
  std::vector<TrajPlotData> all_traj_data;

  int valid_trajs = 0;

  for (int idx : indices)
  {
    std::ostringstream plan_ss, mj_ss;
    plan_ss << "plan_" << std::setw(3) << std::setfill('0') << idx << ".txt";
    mj_ss << "mj_traj_e" << std::setw(3) << std::setfill('0') << idx << ".txt";

    fs::path plan_path = mj_dir / plan_ss.str();
    fs::path mj_path = mj_dir / mj_ss.str();

    if (!fs::exists(mj_path))
    {
      continue;
    }

    // Read plan and build controls
    auto segments = read_plan(plan_path.string());
    std::vector<Eigen::Vector2d> controls;
    std::vector<double> times;
    build_control_sequence(segments, total_time, dt, controls, times);

    // Read GT trajectory
    MJTrajectory mj_traj = read_mj_traj(mj_path.string());
    if (mj_traj.times.empty())
    {
      continue;
    }

    // Full trajectory rollout
    std::vector<Eigen::Vector3d> pred_poses;
    std::vector<Eigen::Vector3d> pred_vels;
    evaluator.full_trajectory_rollout(controls, pred_poses, pred_vels);

    // Extract for error computation
    std::vector<Eigen::Vector2d> pred_xy(pred_poses.size());
    std::vector<double> pred_theta(pred_poses.size());
    for (size_t i = 0; i < pred_poses.size(); ++i)
    {
      pred_xy[i] = pred_poses[i].head<2>();
      pred_theta[i] = pred_poses[i][2];
    }

    // Compute full trajectory errors
    TrajectoryErrors full_errs = compute_trajectory_errors(mj_traj, times, pred_xy, pred_theta);

    // Position errors
    all_full_avg_pos_errors.push_back(full_errs.avg_pos_err);
    all_full_final_pos_errors.push_back(full_errs.final_pos_err);
    all_full_max_pos_errors.push_back(full_errs.max_pos_err);

    // Angle errors
    all_full_avg_angle_errors.push_back(full_errs.avg_angle_err);
    all_full_final_angle_errors.push_back(full_errs.final_angle_err);
    all_full_max_angle_errors.push_back(full_errs.max_angle_err);

    // Store data for plotting
    TrajPlotData plot_data;
    plot_data.idx = idx;
    plot_data.times = times;
    plot_data.final_pos_err = full_errs.final_pos_err;
    plot_data.final_angle_err = full_errs.final_angle_err;

    // Store GT data
    std::vector<double> gt_x_vec(mj_traj.xy.size()), gt_y_vec(mj_traj.xy.size());
    for (size_t i = 0; i < mj_traj.xy.size(); ++i)
    {
      gt_x_vec[i] = mj_traj.xy[i][0];
      gt_y_vec[i] = mj_traj.xy[i][1];
    }

    // Interpolate GT to pred times for plotting
    for (size_t i = 0; i < times.size(); ++i)
    {
      plot_data.gt_x.push_back(interp(mj_traj.times, gt_x_vec, times[i]));
      plot_data.gt_y.push_back(interp(mj_traj.times, gt_y_vec, times[i]));
      plot_data.gt_theta.push_back(interp(mj_traj.times, mj_traj.theta, times[i]));
      plot_data.pred_x.push_back(pred_xy[i][0]);
      plot_data.pred_y.push_back(pred_xy[i][1]);
      plot_data.pred_theta.push_back(pred_theta[i]);

      double dx = plot_data.gt_x[i] - pred_xy[i][0];
      double dy = plot_data.gt_y[i] - pred_xy[i][1];
      plot_data.pos_errors.push_back(std::sqrt(dx * dx + dy * dy));
      plot_data.angle_errors.push_back(std::abs(angle_diff(plot_data.gt_theta[i], pred_theta[i])));
    }
    all_traj_data.push_back(plot_data);

    // Interpolate GT to get pose and velocity at evaluation times
    std::vector<double> gt_x(mj_traj.xy.size()), gt_y(mj_traj.xy.size());
    for (size_t i = 0; i < mj_traj.xy.size(); ++i)
    {
      gt_x[i] = mj_traj.xy[i][0];
      gt_y[i] = mj_traj.xy[i][1];
    }

    std::vector<Eigen::Vector3d> gt_pose_interp(times.size());
    std::vector<Eigen::Vector3d> gt_xd_interp(times.size());

    for (size_t i = 0; i < times.size(); ++i)
    {
      gt_pose_interp[i][0] = interp(mj_traj.times, gt_x, times[i]);
      gt_pose_interp[i][1] = interp(mj_traj.times, gt_y, times[i]);
      gt_pose_interp[i][2] = interp(mj_traj.times, mj_traj.theta, times[i]);

      // Estimate velocity via finite differences (if not first step)
      if (i > 0)
      {
        double dx = gt_pose_interp[i][0] - gt_pose_interp[i - 1][0];
        double dy = gt_pose_interp[i][1] - gt_pose_interp[i - 1][1];
        double dth = angle_diff(gt_pose_interp[i][2], gt_pose_interp[i - 1][2]);
        double theta_avg = gt_pose_interp[i - 1][2];
        double c = std::cos(theta_avg);
        double s = std::sin(theta_avg);
        gt_xd_interp[i][0] = (c * dx + s * dy) / dt;
        gt_xd_interp[i][1] = (-s * dx + c * dy) / dt;
        gt_xd_interp[i][2] = dth / dt;
      }
    }

    // One-step evaluation
    std::vector<double> one_step_pos, one_step_vel, one_step_angle;
    evaluator.evaluate_one_step(gt_xd_interp, gt_pose_interp, controls, one_step_pos, one_step_vel, one_step_angle);

    all_1step_pos_errors.insert(all_1step_pos_errors.end(), one_step_pos.begin(), one_step_pos.end());
    all_1step_vel_errors.insert(all_1step_vel_errors.end(), one_step_vel.begin(), one_step_vel.end());
    all_1step_angle_errors.insert(all_1step_angle_errors.end(), one_step_angle.begin(), one_step_angle.end());

    // N-step rollout evaluation
    std::vector<double> nstep_final_pos, nstep_final_vel, nstep_final_angle;
    evaluator.evaluate_n_step_rollout(gt_xd_interp, gt_pose_interp, controls, rollout_horizon, nstep_final_pos,
                                      nstep_final_vel, nstep_final_angle);

    all_nstep_final_pos_errors.insert(all_nstep_final_pos_errors.end(), nstep_final_pos.begin(), nstep_final_pos.end());
    all_nstep_final_vel_errors.insert(all_nstep_final_vel_errors.end(), nstep_final_vel.begin(), nstep_final_vel.end());
    all_nstep_final_angle_errors.insert(all_nstep_final_angle_errors.end(), nstep_final_angle.begin(), nstep_final_angle.end());

    valid_trajs++;

    // Write trajectory file
    std::ostringstream traj_out_ss;
    traj_out_ss << "traj_" << std::setw(3) << std::setfill('0') << idx << ".txt";
    std::ofstream traj_out(fs::path(output_dir) / traj_out_ss.str());
    for (size_t i = 0; i < pred_poses.size(); ++i)
    {
      traj_out << std::fixed << std::setprecision(5);
      traj_out << times[i] << " " << pred_poses[i][0] << " " << pred_poses[i][1] << " " << pred_poses[i][2] << " "
               << pred_vels[i][0] << " " << pred_vels[i][1] << " " << pred_vels[i][2] << "\n";
    }
  }

  // Compute and print summary statistics
  std::cout << "\n========================================\n";
  std::cout << "COMPREHENSIVE EVALUATION RESULTS\n";
  std::cout << "========================================\n";
  std::cout << "Trajectories evaluated: " << valid_trajs << "\n";
  std::cout << "Rollout horizon: " << rollout_horizon << "\n\n";

  std::cout << "--- ONE-STEP PREDICTION ERRORS ---\n";
  auto stats_1step_pos = AggregatedStats::compute(all_1step_pos_errors);
  auto stats_1step_vel = AggregatedStats::compute(all_1step_vel_errors);
  auto stats_1step_angle = AggregatedStats::compute(all_1step_angle_errors);
  std::cout << "  Total samples: " << all_1step_pos_errors.size() << "\n";
  stats_1step_pos.print("  Position error (m): ");
  stats_1step_angle.print("  Angle error (rad): ");
  stats_1step_vel.print("  Velocity error: ");

  std::cout << "\n--- " << rollout_horizon << "-STEP ROLLOUT ERRORS ---\n";
  auto stats_nstep_pos = AggregatedStats::compute(all_nstep_final_pos_errors);
  auto stats_nstep_vel = AggregatedStats::compute(all_nstep_final_vel_errors);
  auto stats_nstep_angle = AggregatedStats::compute(all_nstep_final_angle_errors);
  std::cout << "  Total rollouts: " << all_nstep_final_pos_errors.size() << "\n";
  stats_nstep_pos.print("  Final pos error (m): ");
  stats_nstep_angle.print("  Final angle error (rad): ");
  stats_nstep_vel.print("  Final vel error: ");

  std::cout << "\n--- FULL TRAJECTORY ERRORS ---\n";
  auto stats_full_avg_pos = AggregatedStats::compute(all_full_avg_pos_errors);
  auto stats_full_final_pos = AggregatedStats::compute(all_full_final_pos_errors);
  auto stats_full_max_pos = AggregatedStats::compute(all_full_max_pos_errors);
  auto stats_full_avg_angle = AggregatedStats::compute(all_full_avg_angle_errors);
  auto stats_full_final_angle = AggregatedStats::compute(all_full_final_angle_errors);
  auto stats_full_max_angle = AggregatedStats::compute(all_full_max_angle_errors);
  std::cout << "  Total trajectories: " << valid_trajs << "\n";
  std::cout << "  Position Errors (m):\n";
  stats_full_avg_pos.print("    Avg: ");
  stats_full_final_pos.print("    Final: ");
  stats_full_max_pos.print("    Max: ");
  std::cout << "  Angle Errors (rad):\n";
  stats_full_avg_angle.print("    Avg: ");
  stats_full_final_angle.print("    Final: ");
  stats_full_max_angle.print("    Max: ");

  // Write summary JSON
  std::ofstream summary_out(fs::path(output_dir) / "summary.json");
  summary_out << std::fixed << std::setprecision(6);
  summary_out << "{\n";
  summary_out << "  \"n_trajectories\": " << valid_trajs << ",\n";
  summary_out << "  \"exp_dir\": \"" << exp_dir << "\",\n";
  summary_out << "  \"rollout_horizon\": " << rollout_horizon << ",\n";
  summary_out << "  \"dt\": " << dt << ",\n";
  // One-step errors
  summary_out << "  \"one_step_pos_mean\": " << stats_1step_pos.mean << ",\n";
  summary_out << "  \"one_step_pos_median\": " << stats_1step_pos.median << ",\n";
  summary_out << "  \"one_step_pos_p90\": " << stats_1step_pos.p90 << ",\n";
  summary_out << "  \"one_step_pos_p99\": " << stats_1step_pos.p99 << ",\n";
  summary_out << "  \"one_step_angle_mean\": " << stats_1step_angle.mean << ",\n";
  summary_out << "  \"one_step_angle_median\": " << stats_1step_angle.median << ",\n";
  summary_out << "  \"one_step_angle_p90\": " << stats_1step_angle.p90 << ",\n";
  summary_out << "  \"one_step_angle_p99\": " << stats_1step_angle.p99 << ",\n";
  // N-step errors
  summary_out << "  \"nstep_final_pos_mean\": " << stats_nstep_pos.mean << ",\n";
  summary_out << "  \"nstep_final_pos_median\": " << stats_nstep_pos.median << ",\n";
  summary_out << "  \"nstep_final_pos_p90\": " << stats_nstep_pos.p90 << ",\n";
  summary_out << "  \"nstep_final_pos_p99\": " << stats_nstep_pos.p99 << ",\n";
  summary_out << "  \"nstep_final_angle_mean\": " << stats_nstep_angle.mean << ",\n";
  summary_out << "  \"nstep_final_angle_median\": " << stats_nstep_angle.median << ",\n";
  summary_out << "  \"nstep_final_angle_p90\": " << stats_nstep_angle.p90 << ",\n";
  summary_out << "  \"nstep_final_angle_p99\": " << stats_nstep_angle.p99 << ",\n";
  // Full trajectory position errors
  summary_out << "  \"traj_avg_pos_mean\": " << stats_full_avg_pos.mean << ",\n";
  summary_out << "  \"traj_avg_pos_median\": " << stats_full_avg_pos.median << ",\n";
  summary_out << "  \"traj_final_pos_mean\": " << stats_full_final_pos.mean << ",\n";
  summary_out << "  \"traj_final_pos_median\": " << stats_full_final_pos.median << ",\n";
  summary_out << "  \"traj_max_pos_mean\": " << stats_full_max_pos.mean << ",\n";
  summary_out << "  \"traj_max_pos_median\": " << stats_full_max_pos.median << ",\n";
  // Full trajectory angle errors
  summary_out << "  \"traj_avg_angle_mean\": " << stats_full_avg_angle.mean << ",\n";
  summary_out << "  \"traj_avg_angle_median\": " << stats_full_avg_angle.median << ",\n";
  summary_out << "  \"traj_final_angle_mean\": " << stats_full_final_angle.mean << ",\n";
  summary_out << "  \"traj_final_angle_median\": " << stats_full_final_angle.median << ",\n";
  summary_out << "  \"traj_max_angle_mean\": " << stats_full_max_angle.mean << ",\n";
  summary_out << "  \"traj_max_angle_median\": " << stats_full_max_angle.median << "\n";
  summary_out << "}\n";
  summary_out.close();

  // Write plot data CSV files
  fs::path plots_dir = fs::path(output_dir) / "plots";
  fs::create_directories(plots_dir);

  // Write error distributions for histograms
  auto write_errors_csv = [&](const std::string& fname, const std::vector<double>& pos_errors,
                              const std::vector<double>& angle_errors) {
    std::ofstream f(plots_dir / fname);
    f << "pos_error,angle_error\n";
    size_t n = std::min(pos_errors.size(), angle_errors.size());
    for (size_t i = 0; i < n; ++i)
    {
      f << std::fixed << std::setprecision(8) << pos_errors[i] << "," << angle_errors[i] << "\n";
    }
  };

  write_errors_csv("1step_errors.csv", all_1step_pos_errors, all_1step_angle_errors);
  write_errors_csv("nstep_errors.csv", all_nstep_final_pos_errors, all_nstep_final_angle_errors);
  write_errors_csv("full_avg_errors.csv", all_full_avg_pos_errors, all_full_avg_angle_errors);
  write_errors_csv("full_final_errors.csv", all_full_final_pos_errors, all_full_final_angle_errors);
  write_errors_csv("full_max_errors.csv", all_full_max_pos_errors, all_full_max_angle_errors);

  // Write trajectory comparison data for 2D plots
  for (const auto& traj : all_traj_data)
  {
    std::ostringstream fname;
    fname << "traj_" << std::setw(3) << std::setfill('0') << traj.idx << "_comparison.csv";
    std::ofstream f(plots_dir / fname.str());
    f << "time,gt_x,gt_y,gt_theta,pred_x,pred_y,pred_theta,pos_error,angle_error\n";
    for (size_t i = 0; i < traj.times.size(); ++i)
    {
      f << std::fixed << std::setprecision(6);
      f << traj.times[i] << "," << traj.gt_x[i] << "," << traj.gt_y[i] << "," << traj.gt_theta[i] << ","
        << traj.pred_x[i] << "," << traj.pred_y[i] << "," << traj.pred_theta[i] << "," << traj.pos_errors[i] << ","
        << traj.angle_errors[i] << "\n";
    }
  }

  // Write error over time aggregated across all trajectories
  {
    size_t max_len = 0;
    for (const auto& traj : all_traj_data)
    {
      max_len = std::max(max_len, traj.times.size());
    }

    std::ofstream f(plots_dir / "error_over_time.csv");
    f << "time,mean_pos_error,std_pos_error,mean_angle_error,std_angle_error\n";

    for (size_t t = 0; t < max_len; ++t)
    {
      std::vector<double> pos_errs, angle_errs;
      double time_val = 0.0;
      for (const auto& traj : all_traj_data)
      {
        if (t < traj.pos_errors.size())
        {
          pos_errs.push_back(traj.pos_errors[t]);
          angle_errs.push_back(traj.angle_errors[t]);
          time_val = traj.times[t];
        }
      }
      if (pos_errs.empty())
        continue;

      double mean_pos = std::accumulate(pos_errs.begin(), pos_errs.end(), 0.0) / pos_errs.size();
      double mean_angle = std::accumulate(angle_errs.begin(), angle_errs.end(), 0.0) / angle_errs.size();

      double var_pos = 0.0, var_angle = 0.0;
      for (size_t i = 0; i < pos_errs.size(); ++i)
      {
        var_pos += (pos_errs[i] - mean_pos) * (pos_errs[i] - mean_pos);
        var_angle += (angle_errs[i] - mean_angle) * (angle_errs[i] - mean_angle);
      }
      var_pos /= pos_errs.size();
      var_angle /= angle_errs.size();

      f << std::fixed << std::setprecision(6);
      f << time_val << "," << mean_pos << "," << std::sqrt(var_pos) << "," << mean_angle << "," << std::sqrt(var_angle)
        << "\n";
    }
  }

  // Generate a simple plotting script
  {
    std::ofstream script(plots_dir / "plot_results.py");
    script << R"(#!/common/home/st1122/miniconda3/envs/mushr_sysid/bin/python
"""Auto-generated plotting script for sysid evaluation results."""
import os
import sys

# Check numpy version compatibility
try:
    import numpy as np
    if int(np.__version__.split('.')[0]) >= 2:
        # NumPy 2.x - should work with recent matplotlib
        pass
except Exception as e:
    print(f"NumPy import error: {e}")
    print("Try running with: conda activate mushr_sysid && python plot_results.py")
    sys.exit(1)

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from pathlib import Path

try:
    plt.style.use('seaborn-v0_8-whitegrid')
except:
    plt.style.use('ggplot')  # Fallback style

output_dir = Path(__file__).parent

# 1. Error distribution histograms
def plot_error_histograms():
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))

    datasets = [
        ('1step_errors.csv', '1-Step'),
        ('nstep_errors.csv', 'N-Step'),
        ('full_final_errors.csv', 'Full Traj Final'),
    ]

    for col, (fname, title) in enumerate(datasets):
        fpath = output_dir / fname
        if not fpath.exists():
            continue
        df = pd.read_csv(fpath)

        # Position error
        axes[0, col].hist(df['pos_error'], bins=50, alpha=0.7, color='blue', edgecolor='black')
        axes[0, col].set_xlabel('Position Error (m)')
        axes[0, col].set_ylabel('Count')
        axes[0, col].set_title(f'{title} Position Error')
        axes[0, col].axvline(df['pos_error'].mean(), color='red', linestyle='--', label=f'Mean: {df["pos_error"].mean():.4f}')
        axes[0, col].legend()

        # Angle error
        axes[1, col].hist(df['angle_error'], bins=50, alpha=0.7, color='orange', edgecolor='black')
        axes[1, col].set_xlabel('Angle Error (rad)')
        axes[1, col].set_ylabel('Count')
        axes[1, col].set_title(f'{title} Angle Error')
        axes[1, col].axvline(df['angle_error'].mean(), color='red', linestyle='--', label=f'Mean: {df["angle_error"].mean():.4f}')
        axes[1, col].legend()

    plt.tight_layout()
    plt.savefig(output_dir / 'error_histograms.png', dpi=150)
    plt.close()

# 2. Error over time
def plot_error_over_time():
    fpath = output_dir / 'error_over_time.csv'
    if not fpath.exists():
        return
    df = pd.read_csv(fpath)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Position error
    axes[0].plot(df['time'], df['mean_pos_error'], 'b-', label='Mean', linewidth=2)
    axes[0].fill_between(df['time'],
                         df['mean_pos_error'] - df['std_pos_error'],
                         df['mean_pos_error'] + df['std_pos_error'],
                         alpha=0.3, color='blue', label='±1 Std')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Position Error (m)')
    axes[0].set_title('Position Error Over Time')
    axes[0].legend()
    axes[0].grid(True)

    # Angle error
    axes[1].plot(df['time'], df['mean_angle_error'], 'orange', label='Mean', linewidth=2)
    axes[1].fill_between(df['time'],
                         df['mean_angle_error'] - df['std_angle_error'],
                         df['mean_angle_error'] + df['std_angle_error'],
                         alpha=0.3, color='orange', label='±1 Std')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Angle Error (rad)')
    axes[1].set_title('Angle Error Over Time')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig(output_dir / 'error_over_time.png', dpi=150)
    plt.close()

# 3. Sample trajectory comparisons
def plot_trajectory_comparisons(n_samples=5):
    traj_files = sorted(output_dir.glob('traj_*_comparison.csv'))[:n_samples]
    if not traj_files:
        return

    fig, axes = plt.subplots(2, n_samples, figsize=(4*n_samples, 8))
    if n_samples == 1:
        axes = axes.reshape(2, 1)

    for col, fpath in enumerate(traj_files):
        df = pd.read_csv(fpath)
        idx = fpath.stem.split('_')[1]

        # XY trajectory
        axes[0, col].plot(df['gt_x'], df['gt_y'], 'b-', label='GT', linewidth=2)
        axes[0, col].plot(df['pred_x'], df['pred_y'], 'r--', label='Pred', linewidth=2)
        axes[0, col].set_xlabel('X (m)')
        axes[0, col].set_ylabel('Y (m)')
        axes[0, col].set_title(f'Trajectory {idx}')
        axes[0, col].legend()
        axes[0, col].axis('equal')
        axes[0, col].grid(True)

        # Error over time
        ax2 = axes[1, col]
        ax2.plot(df['time'], df['pos_error'], 'b-', label='Pos', linewidth=1.5)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Pos Error (m)', color='blue')
        ax2.tick_params(axis='y', labelcolor='blue')

        ax2_twin = ax2.twinx()
        ax2_twin.plot(df['time'], df['angle_error'], 'orange', label='Angle', linewidth=1.5)
        ax2_twin.set_ylabel('Angle Error (rad)', color='orange')
        ax2_twin.tick_params(axis='y', labelcolor='orange')
        ax2.set_title(f'Errors for Traj {idx}')
        ax2.grid(True)

    plt.tight_layout()
    plt.savefig(output_dir / 'trajectory_comparisons.png', dpi=150)
    plt.close()

if __name__ == '__main__':
    print('Generating plots...')
    plot_error_histograms()
    print('  - error_histograms.png')
    plot_error_over_time()
    print('  - error_over_time.png')
    plot_trajectory_comparisons()
    print('  - trajectory_comparisons.png')
    print('Done!')
)";
  }

  std::cout << "\n========================================\n";
  std::cout << "Results saved to: " << output_dir << "\n";
  std::cout << "Plot data saved to: " << (fs::path(output_dir) / "plots").string() << "\n";
  std::cout << "Run: python3 " << (plots_dir / "plot_results.py").string() << " to generate plots\n";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sysid_eval");
  ros::NodeHandle nh("~");

  // Parse command line arguments (override ROS params)
  // Usage: sysid_eval <model_path> [data_eval_dir] [output_dir]
  if (argc >= 2)
  {
    // First arg is model path - extract exp_dir from it
    fs::path model_path(argv[1]);
    if (model_path.filename() == "structured_aux.ts.pt")
    {
      nh.setParam("exp_dir", model_path.parent_path().string());
    }
    else
    {
      nh.setParam("exp_dir", model_path.string());
    }
  }
  if (argc >= 3)
  {
    nh.setParam("data_eval_dir", std::string(argv[2]));
  }
  if (argc >= 4)
  {
    nh.setParam("output_dir", std::string(argv[3]));
  }

  // Single-thread mode for consistent benchmarking
  bool single_thread_mode = true;
  nh.getParam("single_thread_mode", single_thread_mode);
  if (single_thread_mode)
  {
    torch::set_num_threads(1);
    torch::set_num_interop_threads(1);
  }

  run_evaluation(nh);

  return 0;
}

#else

int main()
{
  std::cerr << "Torch not built. Cannot run sysid_eval." << std::endl;
  return 1;
}

#endif
