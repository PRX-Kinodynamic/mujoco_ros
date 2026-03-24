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
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

std::vector<Eigen::Vector3d> test_velocities()
{
  std::vector<Eigen::Vector3d> vels;
  vels.emplace_back(0.309614, 0.0575576, 0.363487);
  vels.emplace_back(0.168474, 0.0300776, 0.206303);
  vels.emplace_back(-0.0462574, 0.0109655, 0.0649229);
  vels.emplace_back(0.115311, -0.0246432, -0.152009);
  vels.emplace_back(-0.193634, 0.0452812, 0.26007);
  vels.emplace_back(0.0735324, 0.0145944, 0.0907716);
  vels.emplace_back(0.147297, -0.0030388, -0.0159019);
  vels.emplace_back(0.301186, 0.0618815, 0.403463);
  vels.emplace_back(0.133843, -0.0216358, -0.130788);
  vels.emplace_back(0.0813488, -0.0187105, -0.118569);
  vels.emplace_back(-0.0900315, 0.000848902, 0.00793001);
  vels.emplace_back(-0.211729, -0.0404844, -0.253254);
  vels.emplace_back(-0.100563, 0.023152, 0.130796);
  vels.emplace_back(0.128233, -0.0306726, -0.185207);
  vels.emplace_back(0.136561, 0.0296614, 0.187185);
  vels.emplace_back(0.0708626, 0.0153945, 0.0909631);
  vels.emplace_back(0.136822, -0.0229954, -0.134334);
  vels.emplace_back(0.0432794, -0.0116801, -0.0628926);
  vels.emplace_back(0.135205, 0.0204844, 0.127911);
  vels.emplace_back(0.0523703, -0.00777113, -0.0306409);
  vels.emplace_back(0.150152, 0.036673, 0.211737);
  vels.emplace_back(0.0047957, -0.000147129, -0.0158577);
  vels.emplace_back(-0.0399, -0.00937074, -0.0603108);
  vels.emplace_back(0.146915, 0.0204298, 0.0986701);
  vels.emplace_back(-0.114361, 0.0210155, 0.122858);
  vels.emplace_back(0.0604427, 0.0142252, 0.086799);
  vels.emplace_back(0.0563333, 0.00611499, 0.0228023);
  vels.emplace_back(0.150616, 0.0177135, 0.141108);
  vels.emplace_back(-0.111765, 0.0236586, 0.137984);
  vels.emplace_back(-0.249049, -0.0122902, -0.123379);
  vels.emplace_back(-0.0588006, 0.013706, 0.0792558);
  vels.emplace_back(0.102759, 0.0166054, 0.105116);
  vels.emplace_back(-0.0313583, 0.00671058, 0.0397477);
  vels.emplace_back(0.181319, 0.0377637, 0.230902);
  vels.emplace_back(0.145533, 0.036996, 0.213537);
  vels.emplace_back(-0.0170305, -0.0031142, -0.0190627);
  vels.emplace_back(0.0427847, 0.0109934, 0.0610171);
  vels.emplace_back(0.0806065, -0.012039, -0.068686);
  vels.emplace_back(-0.121773, 0.0177104, 0.111541);
  vels.emplace_back(-0.138414, 0.00291697, 0.0191327);
  vels.emplace_back(0.215972, 0.0472107, 0.297947);
  vels.emplace_back(-0.137332, 0.0306593, 0.169042);
  vels.emplace_back(0.0556566, 0.0120718, 0.07649);
  vels.emplace_back(-0.149146, 0.0135556, 0.0944336);
  vels.emplace_back(0.123137, -0.00443833, -0.0274191);
  vels.emplace_back(-0.098437, 0.00976278, 0.0443222);
  vels.emplace_back(0.190643, -0.0422041, -0.288442);
  vels.emplace_back(0.123956, 0.0326071, 0.221094);
  vels.emplace_back(0.0519961, 0.011359, 0.0740343);
  vels.emplace_back(0.167695, -0.0434601, -0.255783);
  vels.emplace_back(0.194537, -0.0393022, -0.229316);
  vels.emplace_back(-0.126784, 0.0307704, 0.174124);
  vels.emplace_back(-0.0718624, 0.0153876, 0.0898785);
  vels.emplace_back(0.0349444, 0.00843019, 0.051343);
  vels.emplace_back(0.18631, -0.0399191, -0.24509);
  vels.emplace_back(0.374913, 0.0144725, 0.0946194);
  vels.emplace_back(-0.0418057, -0.00188696, -0.00564604);
  vels.emplace_back(-0.0249365, -0.0029328, -0.020099);
  vels.emplace_back(0.219793, -0.0542378, -0.328828);
  vels.emplace_back(-0.253507, -0.0376119, -0.217703);
  vels.emplace_back(0.030952, -0.00377157, -0.0250258);
  vels.emplace_back(-0.0259435, -0.00102103, -0.0134906);
  vels.emplace_back(0.0248078, -0.00240902, -0.0234882);
  vels.emplace_back(-0.195612, 0.00116141, -0.0119918);
  vels.emplace_back(-0.0774088, -0.00119378, -0.01507);
  vels.emplace_back(0.0646039, 0.012687, 0.0726539);
  vels.emplace_back(-0.0727771, -0.0166312, -0.0947663);
  vels.emplace_back(0.0224601, -0.000397108, 0.00167344);
  vels.emplace_back(0.343226, 0.0389819, 0.249324);
  vels.emplace_back(0.0126613, -0.00337117, -0.0215287);
  vels.emplace_back(0.153245, 0.0176883, 0.103631);
  vels.emplace_back(0.10457, -0.0246147, -0.153341);
  vels.emplace_back(0.169405, -0.0364053, -0.228914);
  vels.emplace_back(0.0107589, -0.00157317, -0.0108414);
  vels.emplace_back(-0.0354005, 0.00309069, 0.0498002);
  vels.emplace_back(0.0559077, 0.0109473, 0.0884385);
  vels.emplace_back(-0.0314862, -0.00675553, -0.0341731);
  vels.emplace_back(0.255974, 0.0545211, 0.335273);
  vels.emplace_back(0.0942168, -0.00957573, -0.0588224);
  vels.emplace_back(-0.0211013, 0.00923815, 0.0387616);
  vels.emplace_back(0.00684852, 0.00135717, 0.00724945);
  vels.emplace_back(-0.107561, 0.0254836, 0.150013);
  vels.emplace_back(-0.0291057, -0.00285091, -0.0219901);
  vels.emplace_back(-0.0169085, 0.00256921, 0.021376);
  vels.emplace_back(-0.107604, 0.014639, 0.0880384);
  vels.emplace_back(0.0570507, -0.0136486, -0.0852225);
  vels.emplace_back(0.243026, 0.0475009, 0.293775);
  vels.emplace_back(-0.00707769, 0.00118219, -0.00752993);
  vels.emplace_back(0.0826597, -0.0125645, -0.0746218);
  vels.emplace_back(-0.10665, 0.0150015, 0.0515147);
  vels.emplace_back(-0.179017, -7.13059e-06, -0.00828643);
  vels.emplace_back(0.159551, -0.00423688, -0.0520253);
  vels.emplace_back(-0.184086, -0.0265693, -0.165833);
  vels.emplace_back(-0.220386, 0.00141235, 0.0309061);
  vels.emplace_back(-0.370477, -0.0408502, -0.237983);
  vels.emplace_back(-0.0368731, -0.00199664, -0.00680089);
  vels.emplace_back(-0.211202, 0.0532066, 0.309386);
  vels.emplace_back(-0.306185, -0.0437212, -0.255871);
  vels.emplace_back(0.219057, 0.047005, 0.291399);
  vels.emplace_back(-0.0885866, -0.0158344, -0.0998639);
  return vels;
}

struct controller_t
{
  using Velocity = Eigen::Vector<double, 1>;
  using Gain = Eigen::Array<double, 1, 1>;
  using PID = prx::simulation::pid_t<1>;
  using LQR = prx::simulation::lqr_t<1, 1>;

  std::shared_ptr<PID> _pid;
  std::shared_ptr<LQR> _lqr;

  controller_t(ros::NodeHandle& nh) : _pid(nullptr)
  {
    std::string control;
    PARAM_SETUP(nh, control);
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

  std::string plant_params_filename, controller_type, output_file, trajs_file;
  prx::simulation_step = 0.1;
  int total_goals;
  int total_steps{ 5 };  // mean [2,8]
  int traj_step;

  // PARAM_SETUP(nh, total_goals);
  PARAM_SETUP(nh, output_file);
  // PARAM_SETUP(nh, trajs_file);
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

  controller_t controller(nh);
  std::vector<Eigen::Vector<double, 3>> velocities{ test_velocities() };
  for (auto vel : velocities)
  {
    Vec(x0).head(3) = Eigen::Vector<double, 3>::Zero();
    Vec(x0).tail(3) = vel;
    ss->copy_from(x0);

    for (double i = 0; i < 20; i += prx::simulation_step)
    {
      controller(ui, x0);
      system_group->propagate_once(ui);
      ofs << i << " ";
      ofs << Vec(x0).transpose() << " ";
      ofs << Vec(ui).transpose() << " ";
      ofs << "\n";

      ss->copy_to(x0);
    }
    ofs << "\n\n";
    // break;
  }
  ofs.close();

  return 0;
}