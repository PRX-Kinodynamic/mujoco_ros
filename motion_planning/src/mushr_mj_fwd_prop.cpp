#include <thread>
#include "mujoco/mujoco.h"

#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include <utils/std_utils.cpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MjFactorPlayground" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string model_path;
  // std::string file_out;
  // std::string plan_file;

  PARAM_SETUP(nh, model_path);
  // PARAM_SETUP(nh, file_out);
  // PARAM_SETUP(nh, plan_file);

  // using MjFactor = mushr_mj_factor_t<>;
  // template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>

  // mjModel* mj_model{ MjFactor::init_mj_model(model_path) };
  // mjData* mj_data{ MjFactor::init_mj_data(mj_model) };

  // // for (int i = 0; i < mj_model->nbody; ++i)
  // // {
  // //   const std::string body_name{ std::string(mj_model->names + mj_model->name_bodyadr[i]) };
  // //   DEBUG_VARS(i, body_name);
  // // }

  // const gtsam::Key xd1{ gtsam::Symbol('V', 1) };
  // const gtsam::Key xd0{ gtsam::Symbol('V', 0) };
  // const gtsam::Key u{ gtsam::Symbol('U', 0) };
  // // const double dt{ 0.01 };
  // const double dt{ 0.01 };

  // DEBUG_VARS(mj_model->opt.timestep);
  // MjFactor mj_factor(xd1, xd0, u, dt, nullptr, mj_model, mj_data);

  // prx_models::mushr_types::State::type x(1.0, 0.0, 0.0);
  // prx_models::mushr_types::StateDot::type xdot;
  // prx_models::mushr_types::Control::type ctrl;
  // ctrl[0] = 0.91120905;
  // ctrl[1] = -0.73472644;

  // // DEBUG_VARS(x, xdot.transpose())
  // for (double ti = 0; ti < 10.0; ti += dt)
  // {
  //   xdot = mj_factor.predict(xdot, ctrl, dt);
  //   x = prx_models::mushr_x_xdot_t::predict(x, xdot, dt);

  //   LOG_VARS(ti, x, xdot.transpose());
  //   // DEBUG_VARS(ti, x, xdot.transpose())
  // }

  // mj_factor.predict(xdot, ctrl, dt);
  return 0;
}