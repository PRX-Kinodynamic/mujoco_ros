#include <fstream>
#include <prx/utilities/general/random.hpp>
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
#include "utils/rosparams_utils.hpp"

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

  Parameters params{ .75, 0.2, 0.99, 0.90, 1.05 };
  Poly poly{ -0.4397, 3.773e-5, 0.8677, 5.8e-6 };

  bool use_jacobians;
  std::string out_file;

  PARAM_SETUP(nh, use_jacobians);
  PARAM_SETUP(nh, out_file);

  std::ofstream ofs(out_file.c_str());

  int total_runs{ 1'000'000 };
  std::vector<double> dts;

  Eigen::Matrix<double, 3, 3> Hxd0;
  Eigen::Matrix<double, 3, 2> Hu;
  Eigen::Matrix<double, 3, 1> Hdt;
  Eigen::Matrix<double, 3, prx_models::mushr_types::Control::ParamsDim> Hparams;

  for (int i = 0; i < total_runs; ++i)
  {
    const StateDot xd0{ StateDot::Random() * 2.0 };
    const Control u{ Control::Random() };
    const double dt{ prx::uniform_random(0.01, 0.15) };

    const ros::WallTime start_stamp{ ros::WallTime::now() };
    if (use_jacobians)
    {
      XdotIntegrationTimeFactor::predict(xd0, u, dt, params, poly, Hxd0, Hu, Hdt, Hparams);
    }
    else
    {
      XdotIntegrationTimeFactor::predict(xd0, u, dt, params, poly);
    }

    const ros::WallTime end_stamp{ ros::WallTime::now() };
    const double fg_dt_ms{ (end_stamp - start_stamp).toSec() * 1000 };

    ofs << std::fixed << std::setprecision(10) << fg_dt_ms << " ";
    ofs << "\n";
  }
  ofs.close();

#ifndef __APPLE__
  std::quick_exit(EXIT_SUCCESS);
#else
  exit(0);
#endif
  // return 0;
}