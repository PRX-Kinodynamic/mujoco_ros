#include <cstddef>
#include <fstream>
#include <iterator>
#include <memory>
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
#include <thread>
#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "utils/dbg_utils.hpp"
#include <utils/std_utils.hpp>

#include <prx_models/defs.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

struct controller_t
{
  using Velocity = Eigen::Vector<double, 1>;
  using Gain = Eigen::Array<double, 1, 1>;
  using PID = prx::simulation::pid_t<1>;
  using LQR = prx::simulation::lqr_t<1, 1>;
  using This = controller_t;

  std::shared_ptr<PID> _pid;
  std::shared_ptr<LQR> _lqr;

  ros::Subscriber _sensor_subscriber;
  ros::Publisher _stamped_control_publisher;
  controller_t(ros::NodeHandle& nh) : _pid(nullptr)
  {
    std::string control, stamped_control_topic, sensor_topic_name;
    PARAM_SETUP(nh, control);
    PARAM_SETUP(nh, sensor_topic_name);
    PARAM_SETUP(nh, stamped_control_topic);
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
      prx_models::mushr_CtrlAccel_t<>::predict(statedot, ctrl, prx::simulation_step, params_u, delta_poly, A, B);

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

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
    _stamped_control_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    // const std::vector<double>& zi{ msg->raw_sensor_data };
    // _last_observation.first[0] = zi[0];
    // _last_observation.first[1] = zi[1];
    // const Eigen::Quaterniond q{ Eigen::Quaterniond(zi[3], zi[4], zi[5], zi[6]) };
    // _last_observation.first[2] = prx::quaternion_to_euler(q)[2];
    // _last_observation.second = msg->header.stamp;
    // _new_observation = true;
    // DEBUG_VARS(_last_observation.first);
    // DEBUG_VARS(_new_observation, _last_observation.first[0], _last_observation.first[1], _last_observation.first[2])
  }

  // u = Ctrl(xi)
  void operator()(prx::space_point_t ui, prx::space_point_t xi)
  {
    if (_pid)
    {
      const Velocity v{ Vec(xi)[3] };
      const double upid{ (*_pid)(v)[0] };
      Vec(ui)[prx_models::mushr_t::control::velocity_idx] = std::max(-1., std::min(1., upid));
      Vec(ui)[prx_models::mushr_t::control::steering_idx] = 0.0;
    }
    else if (_lqr)
    {
      const Velocity v{ Vec(xi)[3] };
      const double upid{ (*_lqr)(v)[0] };
      Vec(ui)[prx_models::mushr_t::control::velocity_idx] = std::max(-1., std::min(1., upid));
      Vec(ui)[prx_models::mushr_t::control::steering_idx] = 0.0;
    }
    else  // No controller
    {
      Vec(ui) = Eigen::Vector<double, 2>::Zero();
    }
  }
};

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MuSHRContingencies" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string stamped_control_topic;
  std::string plant_params_filename, controller_type, output_file, trajs_file;
  prx::simulation_step = 0.1;
  int total_goals;
  int total_steps{ 5 };  // mean [2,8]
  int traj_step;

  // PARAM_SETUP(nh, total_goals);
  // PARAM_SETUP(nh, output_file);
  // PARAM_SETUP(nh, total_steps);
  // PARAM_SETUP(nh, controller_type);
  PARAM_SETUP(nh, plant_params_filename);

  std::ofstream ofs(output_file);

  prx::param_loader plant_params(plant_params_filename);
  auto plant = prx::system_factory_t::create_system(plant_params);
  prx_assert(plant != nullptr, "Error loading plant");
  auto [planning_model, system_group, collision_group] = prx::world_model_t::create(plant);

  auto ss = system_group->get_state_space();
  auto cs = system_group->get_control_space();

  prx::space_point_t x0(ss->make_point());
  prx::space_point_t ui{ cs->make_point() };

  return 0;
}