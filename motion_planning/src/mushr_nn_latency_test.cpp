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
#include <prx_models/mushr_torch.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <prx_models/mushr_factors.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include "utils/dbg_utils.hpp"
#include "utils/rosparams_utils.hpp"

#include <torch_bridge/sysid_runtime.hpp>

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
using MushrTorchFactor = prx_models::mushr_torch_factor_t<double>;

std::string get_model_paths()
{
  const std::string path{ prx::lib_path_safe("ML4KP_ROS") };
  const std::string models_dir{ "/src/mujoco_ros/infrastructure/prx_models/models/learned_mushr/" };
  return path + models_dir;
}

int main(int argc, char** argv)
{
  const std::string node_name{ "FgTbbCheck" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  bool use_jacobians;
  std::string out_file;

  std::string model_type;

  PARAM_SETUP(nh, use_jacobians);
  PARAM_SETUP(nh, out_file);
  PARAM_SETUP(nh, model_type);

  std::ofstream ofs(out_file.c_str());

  int total_runs{ 1'000'000 };
  std::vector<double> dts;

  Eigen::MatrixXd Hxd1, Hxd0;
  Eigen::MatrixXd Hu;
  Eigen::MatrixXd Hdt;

  std::string model_file;
  bool directNN;
  if (model_type == "structured")
  {
    directNN = false;
    model_file = get_model_paths() + "/S06_rollout_w1_structured_aux.ts.pt";
    // nn = std::make_shared<StructuredSysidRuntime>(structured_file, params, poly, false, "float32");
  }
  else if (model_type == "direct")
  {
    directNN = true;
    model_file = get_model_paths() + "/D08_h10_w1_direct_model.ts.pt";
    // nn = std::make_shared<DirectSysidRuntime>(direct_file, false, "float32");
  }

  const gtsam::Key kxd1{ gtsam::Symbol('X', 1) };
  const gtsam::Key kxd0{ gtsam::Symbol('X', 0) };
  const gtsam::Key ku{ gtsam::Symbol('U', 0) };
  const gtsam::Key kdt{ gtsam::Symbol('T', 0) };
  MushrTorchFactor factor(kxd1, kxd0, ku, kdt, nullptr, model_file, directNN);

  for (int i = 0; i < total_runs; ++i)
  {
    const StateDot xd1{ StateDot::Random() * 2.0 };
    const StateDot xd0{ StateDot::Random() * 2.0 };
    const Control u{ Control::Random() };
    const double dt{ prx::uniform_random(0.01, 0.15) };

    const ros::WallTime start_stamp{ ros::WallTime::now() };
    if (use_jacobians)
    {
      factor.evaluateError(xd1, xd0, u, dt, Hxd1, Hxd0, Hu, Hdt);
    }
    else
    {
      factor.evaluateError(xd1, xd0, u, dt, boost::none, boost::none, boost::none, boost::none);
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