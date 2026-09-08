#include <memory>
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/defs.hpp>
#include <prx_models/unicycle_fg_tracking.hpp>

#include <motion_planning/stela_sliding_window.hpp>
#include <motion_planning/tree_validation.hpp>

#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>
#include <interface/levenberg_marquardt_interface.hpp>
#include <interface/StelaStatus.h>
#include <std_msgs/Int32.h>
#include <control/mushr_contingency_controllers.hpp>
#include <motion_planning/goal_checker.hpp>
#include <motion_planning/safety_checker.hpp>
#include <motion_planning/randup.hpp>
#include <prx_models/mushr.hpp>

// #include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <prx_models/StelaKraft.h>
#include <motion_planning/morse_graph_reachability.hpp>
#include <motion_planning/reachability_gt.hpp>
#include <prx_models/unicycle_model.hpp>
#include <motion_planning/gotube.hpp>
#include <motion_planning/nonlinear_clustering.hpp>
#include <prx_models/linear_mixture_model.hpp>

using StatePendulumAnalyticalFull = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
using ControlPendulum = double;

template <typename State, typename Control>
struct tester_t
{
  int _total_samples;
  std::ofstream _ofs;
  prx_models::linear_mixture_model_t<State, Control> _lmm;

  tester_t(ros::NodeHandle& nh)
  {
    std::string test;
    std::string lmm_filename;
    std::string output_filename;
    int& total_samples{ _total_samples };

    PARAM_SETUP(nh, test)
    PARAM_SETUP(nh, lmm_filename)
    PARAM_SETUP(nh, total_samples)
    PARAM_SETUP(nh, output_filename)

    _lmm.from_file(lmm_filename);
    _ofs.open(output_filename);

    get_data(test);
  }

  void get_data(const std::string test)
  {
    if (test == "PendulumAnalyticalLQR")
    {
      pendulum_lqr_test();
    }
  }

  void pendulum_lqr_test()
  {
    using StateSampler = prx::sampler_t<StatePendulumAnalyticalFull>;
    const double th_max{ prx::constants::pi };
    const double th_min{ -prx::constants::pi };

    const double thdot_max{ 2. * prx::constants::pi };
    const double thdot_min{ -2. * prx::constants::pi };

    const double u_bound{ 0.6371781908344007 };
    StateSampler state_sampler(th_min, th_max, thdot_min, thdot_max);
    const Eigen::RowVector2d K{ Eigen::RowVector2d(7.39050619, 2.60611851) };

    prx::simulation_step = 0.01;
    for (int i = 0; i < _total_samples; ++i)
    {
      State x0{ state_sampler() };
      std::cout << "\n";

      DEBUG_VARS(i, x0)

      for (double ti = 0.; ti < 5.; ti += prx::simulation_step)
      {
        const double uk{ -K * gtsam::traits<State>::Logmap(x0) };
        const double ui{ std::min(u_bound, std::max(-u_bound, uk)) };
        auto [valid, x1] = _lmm.predict_safe(x0, ui);

        prx::to_stream(_ofs, x0);
        prx::to_stream(_ofs, ui);
        prx::to_stream(_ofs, x1);
        prx::to_stream(_ofs, valid);
        _ofs << "\n";

        std::cout << ".";
        if (not valid)
        {
          // DEBUG_VARS(valid)
          break;
        }
        x0 = x1;
      }
      _ofs << "\n\n";
    }

    _ofs.close();
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "GLMtesting" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  using PendulumTester = tester_t<StatePendulumAnalyticalFull, ControlPendulum>;

  std::shared_ptr<PendulumTester> pendulum_analytical2d_tester;

  std::string plant;
  PARAM_SETUP(nh, plant)

  DEBUG_VARS(plant)

  if (plant == "pendulum_analytical")
  {
    pendulum_analytical2d_tester = std::make_shared<PendulumTester>(nh);
  }

  ros::spinOnce();
  // ros::sleep(ros:);

  return 0;
}