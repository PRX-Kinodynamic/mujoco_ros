#include <thread>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <prx_models/mushr_factors.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include "utils/dbg_utils.hpp"

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

using Parameters = prx_models::mushr_types::Control::params;
using Poly = prx_models::mushr_types::Control::Poly;
using XdotIntegrationTimeFactor = prx_models::mushr_CtrlAccel_t<double>;
using StateStateDotTimeFactor = prx_models::mushr_x_xdot_t;
using DtLimitFactor = prx::fg::constraint_factor_t<double, std::less<double>>;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

using State = prx_models::mushr_types::State::type;
using StateDot = prx_models::mushr_types::StateDot::type;
using Control = prx_models::mushr_types::Control::type;

int main(int argc, char** argv)
{
  const std::string node_name{ "FgTbbCheck" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string output_file;
  PARAM_SETUP(nh, output_file);

  std::ofstream ofs(output_file);

#ifdef GTSAM_USE_TBB
  PRINT_MSG("USING TBB");

  int max_threads;
  PARAM_SETUP(nh, max_threads);
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, max_threads);
#endif

  Parameters params{ .75, 0.2, 0.99, 0.90, 1.05 };
  Poly poly{ -0.4397, 3.773e-5, 0.8677, 5.8e-6 };

  gtsam::LevenbergMarquardtParams lm_params(prx::fg::default_levenberg_marquardt_parameters());
  lm_params.setMaxIterations(100);
  lm_params.setVerbosityLM("SILENT");

  NoiseModel dt_limit_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-1) };
  NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };
  NoiseModel xd_integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };

  const int traj_length{ 100 };
  const int total_fg_runs{ 100 };

  std::vector<double> dts;
  std::vector<double> iters;

  for (int i = 0; i < total_fg_runs; ++i)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const ros::WallTime start_stamp{ ros::WallTime::now() };
    for (int i = 0; i < traj_length; ++i)
    {
      const gtsam::Key k_x0{ gtsam::Symbol('X', i) };
      const gtsam::Key k_x1{ gtsam::Symbol('X', i + 1) };

      const gtsam::Key k_xdot0{ gtsam::Symbol('D', i) };
      const gtsam::Key k_xdot1{ gtsam::Symbol('D', i + 1) };

      const gtsam::Key k_u01{ gtsam::Symbol('U', i + 1) };
      const gtsam::Key k_t01{ gtsam::Symbol('t', i + 1) };

      const State x{ State::random() };
      const StateDot xd{ StateDot::Random() };
      const Control u{ Control::Random() };

      values.insert(k_x0, x);
      values.insert(k_xdot0, xd);
      values.insert(k_u01, u);
      values.insert(k_t01, 0.1);

      graph.emplace_shared<XdotIntegrationTimeFactor>(k_xdot1, k_xdot0, k_u01, k_t01, xd_integration_noise, params,
                                                      poly);
      graph.emplace_shared<StateStateDotTimeFactor>(k_x1, k_x0, k_xdot0, k_t01, integration_noise);
      graph.emplace_shared<DtLimitFactor>(k_t01, 0.0, dt_limit_noise);
      graph.addPrior(k_t01, 0.1, dt_limit_noise);
    }
    const State x{ State::random() };
    const StateDot xd{ StateDot::Random() };
    values.insert(gtsam::Symbol('X', traj_length), x);
    values.insert(gtsam::Symbol('D', traj_length), xd);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    const gtsam::Values result{ optimizer.optimize() };
    const std::size_t total_iterations{ optimizer.iterations() };

    const ros::WallTime end_stamp{ ros::WallTime::now() };
    const double fg_dt{ (end_stamp - start_stamp).toSec() };

    dts.push_back(fg_dt);
    iters.push_back(total_iterations);
  }

  double total_time{ 0 };
  double total_iters{ 0 };
  for (int i = 0; i < dts.size(); ++i)
  {
    total_time += dts[i];
    total_iters += iters[i];
    ofs << iters[i] << " ";
    ofs << dts[i] << " ";
    ofs << "\n";
  }
  ofs.close();

  const double avg_time_per_iter{ total_time / total_iters };
  DEBUG_VARS(total_time, total_iters, avg_time_per_iter);

#ifndef __APPLE__
  std::quick_exit(EXIT_SUCCESS);
#else
  exit(0);
#endif
  // return 0;
}