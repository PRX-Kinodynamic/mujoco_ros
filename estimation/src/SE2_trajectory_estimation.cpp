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

struct estimator_t
{
  using This = estimator_t;

  // Zi, ti
  using Observation = std::pair<gtsam::Pose2, double>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Plan = std::vector<ml4kp_bridge::PlanStepStamped>;

  ros::Subscriber _trajs_subscriber;
  gtsam::LevenbergMarquardtParams _lm_params;

  std::queue<std::pair<std::vector<Observation>, Plan>> _received_observations, _failure_observations;
  std::ofstream _ofs, _ofs_failure;

  std::mutex _q_mutex, _ofs_mutex, _failure_mutex, _ofs_failure_mutex;
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

    std::string trajectory_topic, estimations_file, failures_file;
    PARAM_SETUP(nh, trajectory_topic);
    PARAM_SETUP(nh, estimations_file);
    PARAM_SETUP(nh, failures_file);

    _ofs.open(estimations_file);
    _ofs_failure.open(failures_file);
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
    for (auto state : msg->trajectory)
    {
      const double x{ state.space_point.point[0] };
      const double y{ state.space_point.point[1] };

      const double qw{ state.space_point.point[3] };
      const double qx{ state.space_point.point[4] };
      const double qy{ state.space_point.point[5] };
      const double qz{ state.space_point.point[6] };

      const double theta{ prx::quaternion_to_angle(qx, qy, qz, qw, 'z') };
      const double ti{ state.header.stamp.toSec() };
      const gtsam::Pose2 zi(x, y, theta);

      observations.push_back(std::make_pair(zi, ti));

      if (std::fabs(qx) > max_q_value or std::fabs(qy) > max_q_value)
      {
        // const std::string msg{ "Error with trajectory" };
        // DEBUG_VARS(msg, qw, qx, qy, qz);
        _rejected++;
        reject = true;
        break;
      }
    }

    if (reject)
    {
      std::scoped_lock lock{ _failure_mutex };
      _failure_observations.push(std::make_pair(observations, msg->plan));
      _rejected_remaining++;
    }
    else
    {
      std::scoped_lock lock{ _q_mutex };
      _received_observations.push(std::make_pair(observations, msg->plan));
      _trajs_remaining++;
    }
  }

  void run(const int thread_id, const bool failure_processor)
  {
    std::size_t iters{ 0 };
    while (ros::ok())
    {
      std::pair<std::vector<Observation>, Plan> obs_plan;
      if (failure_processor)
      {
        // DEBUG_VARS(failure_processor)
        std::scoped_lock lock{ _failure_mutex };
        if (_failure_observations.size() > 0)
        {
          obs_plan = _failure_observations.front();
          _failure_observations.pop();
          _rejected_remaining--;
        }
      }
      else if (_trajs_remaining > 0)
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

        if (failure_processor)
        {
          _trajs_estimated++;
          std::scoped_lock lock{ _ofs_failure_mutex };
          _ofs_failure << estimation;
        }
        else
        {
          _trajs_estimated++;
          std::scoped_lock lock{ _ofs_mutex };
          _ofs << estimation;
        }
      }
      iters++;
    }
  }

  static Eigen::Vector2d find_ctrl(const Plan plan, const double ti)
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
    return Eigen::Vector2d(current.control.point[0], current.control.point[1]);
  }

  static std::string estimate_trajectory(const std::vector<Observation>& trajectory, const Plan& plan,
                                         const gtsam::LevenbergMarquardtParams lm_params)
  {
    using VelocityExpresion = gtsam::Expression<Eigen::Vector3d>;
    using LieIntegrator = prx::fg::lie_ode_observation_factor_t<gtsam::Pose2, Eigen::Vector3d>;
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const NoiseModel velocity_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
    const NoiseModel integrator_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };

    const Eigen::Vector3d zero{ Eigen::Vector3d::Zero() };
    double start{ trajectory[0].second };
    const double estimation_resolution{ 0.1 };
    double dt{ 0.0 };
    int i{ 0 };
    // ti, idx,
    // std::vector<std::tuple<double, int>> data;
    for (auto& state_pair : trajectory)
    {
      const gtsam::Pose2 zi{ state_pair.first };
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
        values.insert(key_xt0, zi);
      }
    }
    values.insert(gtsam::Symbol('X', i), trajectory.back().first);
    // DEBUG_VARS(i)
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
    strstr << "# ti x0 y0 th0 x1 y1 th1 xdot0 ydot0 thdot0 xdot1 ydot1 thdot1 u0 u1  \n";
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

      const gtsam::Pose2 x0{ result.at<gtsam::Pose2>(key_xt0) };
      const gtsam::Pose2 x1{ result.at<gtsam::Pose2>(key_xt1) };
      const Eigen::Vector3d v0{ result.at<Eigen::Vector3d>(key_xd0) };
      const Eigen::Vector3d v1{ result.at<Eigen::Vector3d>(key_xd1) };

      const Eigen::Vector2d ctrl{ find_ctrl(plan, ti) };

      to_file(strstr, ti, x0, x1, v0, v1, ctrl);
    }
    strstr << "\n\n";

    return strstr.str();
    // _ofs.close();
  }

  static void to_file(std::stringstream& strstr, const double ti, const gtsam::Pose2 x0, const gtsam::Pose2 x1,
                      const Eigen::Vector3d v0, const Eigen::Vector3d v1, const Eigen::Vector2d ctrl)
  {
    strstr << ti << " ";
    strstr << x0.x() << " " << x0.y() << " " << x0.theta() << " ";
    strstr << x1.x() << " " << x1.y() << " " << x1.theta() << " ";
    strstr << v0[0] << " " << v0[1] << " " << v0[2] << " ";
    strstr << v1[0] << " " << v1[1] << " " << v1[2] << " ";
    strstr << ctrl[0] << " " << ctrl[1] << " ";
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
  for (; i < total_threads - 1; ++i)
  {
    threads.emplace_back(&estimator_t::run, &estimator, i, false);
  }
  threads.emplace_back(&estimator_t::run, &estimator, i, true);

  // estimator.run();
  ros::spin();
  // spinner.stop();

  return 0;
}