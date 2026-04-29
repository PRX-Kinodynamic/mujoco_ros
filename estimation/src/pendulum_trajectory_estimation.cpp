#include <filesystem>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <ros/subscriber.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <fstream>
#include <queue>
#include <thread>
#include <utility>
#include <utils/dbg_utils.hpp>
#include <utils/nodelet_as_node.hpp>
#include <prx/utilities/general/csv_reader.hpp>

#include <prx/factor_graphs/lie_groups/se3.hpp>
#include <prx/factor_graphs/lie_groups/screw_axis.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/plants/pusher_slider.hpp>
#include <prx/factor_graphs/utilities/values_utilities.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/utilities/general/type_conversions.hpp>

#include <gtsam/base/VectorSpace.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>

#include <interface/StampedMarkers.h>

#include <utils/rosparams_utils.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <estimation/camera_calibration_factors.hpp>
#include <interface/levenberg_marquardt_interface.hpp>
#include <vector>
#include "ml4kp_bridge/Trajectory.h"
#include <ml4kp_bridge/PlanTrajectory.h>
#include <ml4kp_bridge/lie_ode_observation.hpp>
// #include "nodelets/plant_estimator.cpp"
// using SE3 = prx::fg::se3_t;

// template <typename State, typename Observation>
struct pendulum_observation_function_t
{
  using State = gtsam::Rot2;
  using Observation = Eigen::Vector2d;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimZ{ gtsam::traits<Observation>::dimension };

  static inline Eigen::Vector2d pendulum_length{ Eigen::Vector2d(0.0, 1.0) };

  static Observation predict(const State& xi, gtsam::OptionalJacobian<DimZ, DimX> H = boost::none)
  {
    // Zi = R * pl;
    const Observation zi{ xi.rotate(pendulum_length, H) };
    // if (Hx)
    // {
    //   // TODO:
    //   *Hx = Eigen::Matrix<double, DimX, DimZ>::Identity();
    // }
    return zi;
  }
};

struct estimator_t
{
  using This = estimator_t;

  using State = gtsam::Rot2;
  using Velocity = Eigen::Vector<double, 1>;
  using Control = double;
  using Observation = std::pair<Eigen::Vector2d, double>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Plan = std::vector<ml4kp_bridge::PlanStepStamped>;

  ros::Subscriber _trajs_subscriber;
  gtsam::LevenbergMarquardtParams _lm_params;

  std::queue<std::pair<std::vector<Observation>, Plan>> _received_observations, _failure_observations;
  std::ofstream _ofs, _ofs_gt;

  std::mutex _q_mutex, _ofs_mutex;
  std::atomic<std::size_t> _trajs_estimated;
  std::atomic<std::size_t> _trajs_remaining;
  std::atomic<std::size_t> _rejected;
  std::atomic<std::size_t> _rejected_remaining;

  estimator_t(ros::NodeHandle& nh)
    : _lm_params(prx::fg::default_levenberg_marquardt_parameters())
    , _trajs_remaining(0)
    , _trajs_estimated(0)
    , _rejected(0)
    , _rejected_remaining(0)
  {
    interface::initialize(_lm_params, ros::NodeHandle(nh, "lm"));

    std::string trajectory_topic, estimations_file;
    PARAM_SETUP(nh, trajectory_topic);
    PARAM_SETUP(nh, estimations_file);
    // PARAM_SETUP(nh, failures_file);

    _ofs.open(estimations_file);
    _ofs_gt.open("/tmp/gt.txt");
    _trajs_subscriber = nh.subscribe(trajectory_topic, 10000, &This::trajectory_callback, this);
    // _timer = nh.createTimer(ros::Duration(prx::simulation_step), &simulator_t::step_callback, this);
    // for (auto& topic_name : aruco_topics)
  }

  ~estimator_t()
  {
    _ofs.close();
  }

  void trajectory_callback(const ml4kp_bridge::PlanTrajectoryConstPtr msg)
  {
    bool reject{ false };
    std::vector<Observation> observations;

    const double max_q_value{ 0.35 };  // ~0.9 euler rads on x, y
    const double t0{ msg->trajectory.front().header.stamp.toSec() };
    for (auto state : msg->trajectory)
    {
      // Pendulum moves in plane YZ
      // const double x{ state.space_point.point[0] };
      const double y{ state.space_point.point[1] };
      const double z{ state.space_point.point[2] };

      const double ti{ state.header.stamp.toSec() };
      const Eigen::Vector2d zi(y, z);
      observations.push_back(std::make_pair(zi, ti));
      _ofs_gt << ti - t0 << " ";
      // _ofs_gt << x << " ";
      _ofs_gt << y << " ";
      _ofs_gt << z << " ";
      _ofs_gt << "\n";
    }

    std::scoped_lock lock{ _q_mutex };
    _received_observations.push(std::make_pair(observations, msg->plan));
    _trajs_remaining++;
  }

  void run(const int thread_id)
  {
    std::size_t iters{ 0 };
    while (ros::ok())
    {
      std::pair<std::vector<Observation>, Plan> obs_plan;
      if (_trajs_remaining > 0)
      {
        // DEBUG_VARS(thread_id, _trajs_remaining)
        std::scoped_lock lock{ _q_mutex };
        if (_received_observations.size() > 0)
        {
          obs_plan = _received_observations.front();
          _received_observations.pop();
          _trajs_remaining--;
        }
      }
      else
      {
        continue;
      }

      if (thread_id == 0 and iters % 10 == 0)
      {
        auto total_estimated = _trajs_estimated.load();
        auto remaining = _trajs_remaining.load();
        auto rejected = _rejected.load();
        auto rejected_remaining = _rejected_remaining.load();

        DEBUG_VARS(total_estimated, remaining, rejected, rejected_remaining)
      }
      if (obs_plan.first.size() > 0)
      {
        const std::string estimation{ estimate_trajectory(obs_plan.first, obs_plan.second, _lm_params) };

        _trajs_estimated++;
        std::scoped_lock lock{ _ofs_mutex };
        _ofs << estimation;
        _ofs << std::flush;
      }
      iters++;
    }
  }

  static Control find_ctrl(const Plan plan, const double ti)
  {
    const ros::Time start{ plan[0].header.stamp };
    ml4kp_bridge::PlanStep current;
    for (auto step : plan)
    {
      const double t_plan{ (step.header.stamp - start).toSec() };
      if (t_plan <= ti)
      {
        current = step.plan_step;
        continue;
      }
      break;
    }
    return current.control.point[0];
  }

  static std::string estimate_trajectory(const std::vector<Observation>& trajectory, const Plan& plan,
                                         const gtsam::LevenbergMarquardtParams lm_params)
  {
    using VelocityExpresion = gtsam::Expression<Velocity>;
    using LieIntegrator =
        prx::fg::lie_ode_observation_factor_t<State, Velocity, Eigen::Vector2d, pendulum_observation_function_t>;
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const NoiseModel velocity_nm{ gtsam::noiseModel::Isotropic::Sigma(1, 1e0) };
    const NoiseModel integrator_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };

    const Velocity zero{ Velocity::Zero() };
    double start{ trajectory[0].second };
    const double estimation_resolution{ 0.1 };
    double dt{ 0.0 };
    int i{ 0 };
    // ti, idx,
    // std::vector<std::tuple<double, int>> data;
    for (auto& state_pair : trajectory)
    {
      const Eigen::Vector2d zi{ state_pair.first };
      const double ti{ state_pair.second };

      const gtsam::Key key_xt0{ gtsam::Symbol('X', i) };
      const gtsam::Key key_xdot{ gtsam::Symbol('V', i) };

      const double zdt{ ti - start };
      graph.emplace_shared<LieIntegrator>(key_xt0, key_xdot, integrator_nm, zi, zdt);

      // dt += zdt;
      if (zdt > estimation_resolution)
      {
        i++;
        start += estimation_resolution;
        values.insert(key_xt0, gtsam::Rot2::relativeBearing(zi));
      }
    }
    values.insert(gtsam::Symbol('X', i), gtsam::Rot2::relativeBearing(trajectory.back().first));

    const int total{ i };
    for (; 0 < i; --i)
    {
      VelocityExpresion vel_btw{ gtsam::between(VelocityExpresion('V', i - 1), VelocityExpresion('V', i)) };
      graph.addExpressionFactor(velocity_nm, zero, vel_btw);
      values.insert(gtsam::Symbol('V', i), zero);
    }
    values.insert(gtsam::Symbol('V', i), zero);
    // DEBUG_VARS(i)

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    const gtsam::Values result{ optimizer.optimize() };

    std::stringstream strstr;

    // Files with lines:
    strstr << "# ti zX0 zY0 th0 th1 thdot0 thdot1 u0  \n";
    strstr << "# error: " << optimizer.error() << "\n";
    // file: "State0 State1 Velocity01"
    double ti{ 0.0 };
    for (int i = 0; i < total - 1; ++i)
    {
      ti += estimation_resolution;
      const gtsam::Key key_xt0{ gtsam::Symbol('X', i) };
      const gtsam::Key key_xt1{ gtsam::Symbol('X', i + 1) };
      const gtsam::Key key_xd0{ gtsam::Symbol('V', i) };
      const gtsam::Key key_xd1{ gtsam::Symbol('V', i + 1) };

      const State x0{ result.at<State>(key_xt0) };
      const State x1{ result.at<State>(key_xt1) };
      const Velocity v0{ result.at<Velocity>(key_xd0) };
      const Velocity v1{ result.at<Velocity>(key_xd1) };

      const Control ctrl{ find_ctrl(plan, ti) };

      to_file(strstr, ti, x0, x1, v0, v1, ctrl);
    }
    strstr << "\n\n";

    return strstr.str();
    // _ofs.close();
  }

  static void to_file(std::stringstream& strstr, const double ti, const State x0, const State x1, const Velocity v0,
                      const Velocity v1, const Control ctrl)
  {
    // # ti zX0 zY0 th0 th1 thdot0 thdot1 u0 ;
    const Eigen::Vector2d z0{ pendulum_observation_function_t::predict(x0) };
    strstr << ti << " ";
    strstr << z0[0] << " " << z0[1] << " ";
    strstr << x0.theta() << " ";
    strstr << x1.theta() << " ";
    strstr << v0[0] << " ";
    strstr << v1[0] << " ";
    strstr << ctrl << " ";
    strstr << "\n";
  }
};

// std::messages

// Calibrate the world T=[R|t] of the world.
// This is, find the T that transforms C0 to C1, which can be used by ros::Tf to transform between frames
int main(int argc, char** argv)
{
  ros::init(argc, argv, "world_calibration");
  ros::NodeHandle nh("~");

  estimator_t estimator(nh);

  // ros::AsyncSpinner spinner(2);  // 1 thread for the controller
  // spinner.start();

  unsigned int total_threads{ std::thread::hardware_concurrency() };
  DEBUG_VARS(total_threads)

  int i = 0;
  std::vector<std::thread> threads{};
  for (; i < total_threads; ++i)
  {
    threads.emplace_back(&estimator_t::run, &estimator, i);
  }

  // estimator.run();
  ros::spin();
  // spinner.stop();

  return 0;
}