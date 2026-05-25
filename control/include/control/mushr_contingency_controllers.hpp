#pragma once
#include <cstddef>
#include <fstream>
#include <iterator>
#include <memory>
#include <thread>

#include <prx/factor_graphs/lie_groups/se2.hpp>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx/utilities/general/random.hpp>
#include <prx/utilities/spaces/space.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <prx/simulation/controllers/pid.hpp>
#include <prx/simulation/controllers/lqr.hpp>
#include <ml4kp_bridge/defs.h>
#include <prx_models/MushrPlanner.h>
// #include <prx_models/mj_mushr.hpp>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.hpp>
#include <utils/rosparams_utils.hpp>

#include <prx_models/defs.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace control
{

struct contingency_controller_t
{
  using Velocity = Eigen::Vector<double, 1>;
  using Gain = Eigen::Array<double, 1, 1>;
  using PID = prx::simulation::pid_t<1>;
  using LQR = prx::simulation::lqr_t<1, 1>;
  using This = prx::controller_t;

  std::shared_ptr<PID> _pid;
  std::shared_ptr<LQR> _lqr;

  ros::Subscriber _sensor_subscriber;
  ros::Publisher _stamped_control_publisher;
  contingency_controller_t(ros::NodeHandle nh) : _pid(nullptr)
  {
    std::string control;
    PARAM_SETUP(nh, control);
    init(nh, control);
  }

  contingency_controller_t(ros::NodeHandle nh, std::string control) : _pid(nullptr), _lqr(nullptr)
  {
    init(nh, control);
  }

  void init(ros::NodeHandle& nh, const std::string control)
  {
    // PARAM_SETUP(nh, sensor_topic_name);
    // PARAM_SETUP(nh, stamped_control_topic);
    if (control == "PID")
    {
      double kp{ 1.0 };
      double ki{ 0.0 };

      PARAM_SETUP_WITH_DEFAULT(nh, kp, kp);
      PARAM_SETUP_WITH_DEFAULT(nh, ki, ki);
      Gain Kp{ Gain(kp) };
      Gain Ki{ Gain(ki) };
      Gain Kd{ Gain::Zero() };
      _pid = std::make_shared<PID>(Kp, Ki, Kd, Velocity::Zero());
    }
    else if (control == "LQR")
    {
      const Eigen::Vector3d statedot{ Eigen::Vector3d::Zero() };
      const Eigen::Vector2d ctrl{ Eigen::Vector2d::Zero() };
      Eigen::VectorXd params_u(5), delta_poly(4);
      params_u << 0.09, 0.2, 1.0, 0.9, 1.05;
      delta_poly << -0.4397, 3.773e-5, 0.8677, 5.8e-6;

      LQR::MatrixA Ap;
      LQR::MatrixB Bp;
      Eigen::Matrix<double, 3, 3> A;
      Eigen::Matrix<double, 3, 2> B;
      LQR::MatrixQ Q;
      LQR::MatrixR R{ LQR::MatrixR::Identity() * 10 };
      Q.diagonal() << 0.01;
      prx_models::mushr_CtrlAccel_t<>::predict(statedot, ctrl, 0.1, params_u, delta_poly, A, B);

      Ap = A.block<1, 1>(0, 0);
      Bp = 2. * B.block<1, 1>(0, 0);  // assuming bounds on xdot is 0.5 -> 0.5^-1 = 2 (normalizing B as in pendulum)
      _lqr = std::make_shared<LQR>(Ap, Bp, Q, R);
      const LQR::MatrixK K{ _lqr->K() };

      DEBUG_VARS(A);
      DEBUG_VARS(B);
      DEBUG_VARS(Bp);
      DEBUG_VARS(Q);
      DEBUG_VARS(R);
      DEBUG_VARS(K);
    }
    else
    {
      prx_throw("[mushr_contingency_controllers] Unknown controller: " << control);
    }

    // _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
    // _stamped_control_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);
  }

  // u = Ctrl(xi)
  Eigen::Vector<double, 2> control(const Eigen::Vector3d xdot)
  {
    Eigen::Vector<double, 2> ui;
    if (_pid)
    {
      const Velocity v{ xdot[0] };
      const double upid{ (*_pid)(v)[0] };
      ui[prx_models::mushr_t::control::velocity_idx] = std::max(-1., std::min(1., upid));
      ui[prx_models::mushr_t::control::steering_idx] = 0.0;
    }
    else if (_lqr)
    {
      const Velocity v{ xdot[0] };
      const double ulqr{ (*_lqr)(v)[0] };
      ui[prx_models::mushr_t::control::velocity_idx] = std::max(-1., std::min(1., ulqr));
      ui[prx_models::mushr_t::control::steering_idx] = 0.0;
    }
    else  // No controller
    {
      PRINT_MSG("[MushrContingency] No contingency controller");
      ui = Eigen::Vector<double, 2>::Zero();
    }
    return ui;
  }
};
}  // namespace control
