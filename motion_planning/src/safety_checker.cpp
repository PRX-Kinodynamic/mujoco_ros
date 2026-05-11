#include <memory>
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/SO2_system.hpp>

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

// #include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <prx_models/StelaKraft.h>
template <typename DynamicalSystem, typename Controller>
struct safety_helper_t
{
  using Randup = motion_planning::randup_t<DynamicalSystem, Controller>;
  using RandupCovariance = typename Randup::Covariance;

  ros::Timer timer;
  ros::Subscriber _total_time_subscriber;
  ros::Subscriber _state_subscriber, _plan_subscriber, _cov_x0_subscriber, _total_trajs_subscriber;
  std::shared_ptr<motion_planning::safety_checker_t> safety_checker;

  ml4kp_bridge::SpacePointStamped _state_estimate;
  ml4kp_bridge::PlanStepStampedArray _plan;

  bool _randup_time;
  Randup _randup;
  RandupCovariance _cov_x0, _cov_w;
  int _total_trajs;
  ros::Duration _check_duration;

  bool valid_state, valid_plan;
  safety_helper_t(ros::NodeHandle& nh)
    : valid_state(false)
    , valid_plan(false)
    , _randup(nh)
    , _total_trajs(100)
    , _cov_x0(RandupCovariance::Identity() * 0.1)
    , _cov_w(RandupCovariance::Identity() * 0.1)
  {
    bool& randup_time{ _randup_time };
    std::string state_topic;
    PARAM_SETUP(nh, state_topic)
    PARAM_SETUP(nh, randup_time)
    // safety_checker = std::make_shared<motion_planning::safety_checker_t>(ros::NodeHandle(nh, "safety"));
    timer = nh.createTimer(ros::Duration(1.0), &safety_helper_t::timer_callback, this);

    _state_subscriber = nh.subscribe(state_topic, 1, &safety_helper_t::state_callback, this);
    _plan_subscriber = nh.subscribe("/safety/input/plan", 1, &safety_helper_t::plan_callback, this);
    _cov_x0_subscriber = nh.subscribe<ml4kp_bridge::SpacePoint>(
        "/safety/input/cov_x0", 1,
        boost::bind(&safety_helper_t::covariance_callback, this, boost::placeholders::_1, _cov_x0));
    _cov_x0_subscriber = nh.subscribe<ml4kp_bridge::SpacePoint>(
        "/safety/input/cov_w", 1,
        boost::bind(&safety_helper_t::covariance_callback, this, boost::placeholders::_1, boost::ref(_cov_w)));

    _total_trajs_subscriber =
        nh.subscribe("/safety/input/randup/total_trajs", 1, &safety_helper_t::total_randup_trajs_callback, this);

    _total_time_subscriber =
        nh.subscribe("/safety/input/randup/total_time", 1, &safety_helper_t::total_time_callback, this);
  }

  void total_time_callback(const std_msgs::DurationConstPtr msg)
  {
    _check_duration = msg->data;
  }

  void total_randup_trajs_callback(const std_msgs::Int32ConstPtr msg)
  {
    _total_trajs = msg->data;
  }

  void covariance_callback(const ml4kp_bridge::SpacePointConstPtr msg, RandupCovariance& cov)
  {
    for (int i = 0; i < cov.rows(); ++i)
    {
      cov(i, i) = msg->point[i];
    }
    DEBUG_VARS(cov)
    DEBUG_VARS(_cov_w)
    DEBUG_VARS(_cov_x0)
  }

  void state_callback(const ml4kp_bridge::SpacePointStampedConstPtr msg)
  {
    _state_estimate = *msg;
    valid_state = true;
    DEBUG_VARS(valid_state)
  }

  void plan_callback(const ml4kp_bridge::PlanStepStampedArrayConstPtr msg)
  {
    _plan = *msg;
    valid_plan = true;
    DEBUG_VARS(valid_plan)
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    if (valid_state and valid_plan)
    {
      PRINT_MSG("Calling safety checker")
      DEBUG_VARS(_state_estimate, _plan)
      DEBUG_VARS(_cov_x0, _cov_w, _total_trajs)

      if (_randup_time)
      {
        auto start = ros::Time::now();
        const std::chrono::time_point randup_limit{ std::chrono::steady_clock::now() +
                                                    std::chrono::seconds(_check_duration.sec) +
                                                    std::chrono::nanoseconds(_check_duration.nsec) };

        _randup.is_safe(_state_estimate, _plan, _cov_x0, _cov_w, randup_limit);
        auto end = ros::Time::now();
        auto randup_real_dt = (end - start).toSec();
        DEBUG_VARS(randup_real_dt)
      }
      else
      {
        _randup.is_safe(_state_estimate, _plan, _cov_x0, _cov_w, _total_trajs);
      }
      PRINT_MSG("Safety checker finished")
      valid_state = false;
      valid_plan = false;
    }
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrSbmpOpenLoop" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  using PieceWiseStep = prx::piecewise_step_t<prx::SO2_system_t::Control, double>;
  using Controller = std::vector<PieceWiseStep>;
  safety_helper_t<prx::SO2_system_t, Controller> helper(nh);

  ros::spin();
  // ros::AsyncSpinner spinner(4);
  // spinner.start();

  // while (ros::ok())
  // {
  // }
  // spinner.stop();

  return 0;
}