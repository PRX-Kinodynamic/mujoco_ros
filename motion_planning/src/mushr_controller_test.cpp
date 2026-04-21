#include <cstddef>
#include <fstream>
#include <iterator>
#include <prx/factor_graphs/lie_groups/se2.hpp>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx/utilities/general/random.hpp>
#include <prx/utilities/spaces/space.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <thread>
// #include "mujoco_ros/control_listener.hpp"
// #include "mujoco_ros/sensordata_publisher.hpp"
// #include "mujoco_ros/Collision.h"
#include <Eigen/src/Core/Matrix.h>
#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "utils/dbg_utils.hpp"
// #include "control/MushrControlPropagation.h"
// #include "motion_planning/replanner_service.hpp"
// #include "motion_planning/planner_client.hpp"
// #include "motion_planning/PlanningResult.h"
// #include "mujoco_ros/Collision.h"
// #include "std_msgs/Empty.h"
#include <utils/std_utils.hpp>

#include <prx_models/defs.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

struct ctrlr_t
{
  enum type_t
  {
    RANDOM = 0,
    POLY_JACOBIAN = 1
  };
  using MushrAnalytical = prx_models::mushr_CtrlAccel_t<double>;
  prx_models::mushr_types::Control::params params;
  prx_models::mushr_types::Control::Poly poly;
  prx::space_t* _ctrl_space;

  type_t _ctrl_type;
  ctrlr_t(const std::string ctrl_type, prx::space_t* ctrl_space)
    : _ctrl_space(ctrl_space), poly(-0.4397, 3.773e-5, 0.8677, 5.8e-6), params(0.09, 0.2, 1.0, 0.9, 1.05)
  {
    if (ctrl_type == "RANDOM")
    {
      _ctrl_type = type_t::RANDOM;
    }
    else if (ctrl_type == "POLY_JACOBIAN")
    {
      _ctrl_type = type_t::POLY_JACOBIAN;
    }
  }

  void operator()(prx::space_point_t ui, prx::space_point_t xi, prx::space_point_t xgoal)
  {
    if (_ctrl_type == type_t::RANDOM)
    {
      random_ctrl(ui);
    }
    else if (_ctrl_type == type_t::POLY_JACOBIAN)
    {
      jacobian_ctrl(xi, ui, xgoal);
    }
  }

  void random_ctrl(prx::space_point_t ui)
  {
    _ctrl_space->sample(ui);
    DEBUG_VARS(ui)
  }

  void jacobian_ctrl(prx::space_point_t xi, prx::space_point_t ui, prx::space_point_t xgoal)
  {
    using LieIntegrator = prx::fg::lie_integration_factor_t<prx::fg::SE2_t, Eigen::Vector3d, double>;
    Eigen::Matrix<double, 3, 3> A;
    Eigen::Matrix<double, 3, 2> B;
    const prx::fg::SE2_t x0(Vec(xi).head(3));
    const prx::fg::SE2_t xG(Vec(xgoal).head(3));
    const Eigen::Vector3d xd0(Vec(xi).tail(3));
    const Eigen::Vector3d xdG(Vec(xgoal).tail(3));
    Eigen::MatrixXd Hxdot;
    const Eigen::Vector3d xerr{ LieIntegrator::error(xG, x0, xd0, prx::simulation_step, boost::none, boost::none,
                                                     Hxdot) };
    const Eigen::Vector3d xpt1{ MushrAnalytical::predict(xd0, Vec(ui), prx::simulation_step, params, poly, A, B) };

    //                         (2x3)     * ((3x1)-(3x1)-(3x3)(3x1) )
    // Eigen::Vector3d dx{ xd0 - Vec(xgoal).tail(3) };
    // // Eigen::Matrix<double, 2, 3> Binv{ B.inverse() };
    const Eigen::Vector3d xdiff{ xdG - A * xerr };
    // // Eigen::Vector2d u_next{ B.bdcSvd().solve(xdiff) };
    auto Binv{ B.completeOrthogonalDecomposition().pseudoInverse() };
    Eigen::Vector2d u_next{ B.completeOrthogonalDecomposition().pseudoInverse() * xdiff };
    // // Eigen::Vector2d u_next{  * xpt1 };
    // // Eigen::Vector2d u_next{ B.inverse() * (xpt1 - xd0 - A * dx) };
    // Vec(ui) = u_next.normalized();
    // DEBUG_VARS(xd0.transpose())
    // DEBUG_VARS(ui)
    // DEBUG_VARS(xdG.transpose())
    // DEBUG_VARS(xerr.transpose())
    // DEBUG_VARS(A)
    DEBUG_VARS(xerr.transpose())
    DEBUG_VARS(xdiff.transpose())
    DEBUG_VARS(B)
    DEBUG_VARS(Binv)
    DEBUG_VARS(u_next.transpose())
    // DEBUG_VARS(xdiff.transpose(), u_next.transpose(), ui)
  }
};

void sample_x0_and_goal(prx::space_point_t x0, prx::space_point_t xgoal)
{
  // x0->at(0) = 0.0;
  // x0->at(1) = 0.0;
  // x0->at(2) = 0.0;
  Vec(x0).head(3) = Eigen::Vector3d::Zero();
  Vec(x0).tail(3) = Eigen::Vector3d::Random() * 0.5;
  Vec(xgoal).head(2) = Eigen::Vector2d::Random() * 0.5;
  xgoal->at(2) = prx::uniform_random(-prx::constants::pi, prx::constants::pi);
  Vec(xgoal).tail(3) = Eigen::Vector3d::Random() * 0.5;
}

Eigen::Vector<double, 6> error(prx::space_point_t x, prx::space_point_t xgoal)
{
  Eigen::Vector<double, 6> error;
  prx::fg::SE2_t xT(Vec(x).head(3));
  prx::fg::SE2_t xG(Vec(xgoal).head(3));

  const prx::fg::SE2_t btw{ gtsam::traits<prx::fg::SE2_t>::Between(xT, xG) };
  error.head(3) = gtsam::traits<prx::fg::SE2_t>::Logmap(btw);
  error.tail(3) = Vec(xgoal).tail(3) - Vec(x).tail(3);
  return error;
}
// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MuSHRControllerTest" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // std::string params_file;
  // std::string file_out;
  // std::string plan_file;
  // std::string traj_file;
  std::string plant_params_filename, controller_type, output_file, trajs_file;
  prx::simulation_step = 0.1;
  int total_goals;
  int total_steps{ 5 };  // mean [2,8]
  int traj_step;

  PARAM_SETUP(nh, total_goals);
  PARAM_SETUP(nh, output_file);
  PARAM_SETUP(nh, trajs_file);
  PARAM_SETUP(nh, total_steps);
  PARAM_SETUP(nh, controller_type);
  PARAM_SETUP(nh, plant_params_filename);

  prx::param_loader plant_params(plant_params_filename);
  auto plant = prx::system_factory_t::create_system(plant_params);
  prx_assert(plant != nullptr, "Error loading plant");
  auto [planning_model, system_group, collision_group] = prx::world_model_t::create(plant);

  auto ss = system_group->get_state_space();
  auto cs = system_group->get_control_space();

  prx::space_point_t x0(ss->make_point());
  prx::space_point_t xgoal(ss->make_point());

  prx::space_point_t ui{ cs->make_point() };
  system_group->propagate_once(ui);

  ctrlr_t controller(controller_type, cs);

  std::ofstream ofs(output_file);
  std::ofstream ofs_trajs(trajs_file);

  ofs << "# [1-6] [7-12] [13-18] [19-24]  [25]    [26]          [27-29]\n";
  ofs << "# x0(6) xGoal(6) xT(6) xErr(6) XY-err(1) Theta-err(1) Dots-err(3)\n";

  for (int i = 0; i < total_goals; ++i)
  {
    sample_x0_and_goal(x0, xgoal);
    ss->copy_from(x0);
    ofs << Vec(x0).transpose() << " ";
    ofs << Vec(xgoal).transpose() << " ";
    ofs_trajs << Vec(x0).transpose() << "\n";
    for (int j = 0; j < total_steps; ++j)
    {
      controller(ui, x0, xgoal);
      system_group->propagate_once(ui);
      ss->copy_to(x0);
      ofs_trajs << Vec(x0).transpose() << "\n";
    }
    ofs_trajs << "\n";
    ofs_trajs << "\n";
    auto error_vec = error(x0, xgoal);
    ofs << Vec(x0).transpose() << " ";
    ofs << error_vec.transpose() << " ";
    ofs << error_vec.head(2).norm() << " ";
    ofs << std::fabs(error_vec[2]) << " ";
    ofs << error_vec.tail(3).norm() << " ";
    ofs << "\n";
  }
  ofs.close();
  ofs_trajs.close();
  return 0;
}