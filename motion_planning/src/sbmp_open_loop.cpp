#include <memory>
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <prx_models/StelaKraft.h>

#include <motion_planning/stela_sliding_window.hpp>
#include <motion_planning/tree_validation.hpp>

#include <utils/dbg_utils.hpp>

#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>
#include <interface/levenberg_marquardt_interface.hpp>
#include <interface/StelaStatus.h>
#include <control/mushr_contingency_controllers.hpp>
#include <motion_planning/goal_checker.hpp>
#include <motion_planning/safety_checker.hpp>

void handle_node_state(std::shared_ptr<interface::node_status_t> node_status)
{
  if (node_status->new_request())
  {
    node_status->status(node_status->requested_status());
  }
}

struct mushr_sbmp_open_loop_t
{
  using RePlanner = motion_planning::sbmp_caller_t;
  using RePlannerResult = RePlanner::Result;

  using RePlannerPlan = RePlanner::Plan;
  using RePlannerTrajectory = RePlanner::Trajectory;

  using ContingencyController = control::contingency_controller_t;
  ml4kp_bridge::SpacePointStamped _current_control;

  ml4kp_bridge::SpacePointStamped _state_estimate;
  ml4kp_bridge::SpacePointStamped _end_window_estimate;

  RePlannerPlan _current_plan, _next_plan;
  RePlannerTrajectory _current_trajectory, _next_trajectory;

  ros::Timer _timer;
  ros::Publisher _contingency_publisher;
  ros::Publisher _control_publisher, _control_stamped_publisher;
  ros::Subscriber _ekf_subscriber;

  std::shared_ptr<ContingencyController> _contingency_controller;
  std::shared_ptr<motion_planning::goal_checker_t> _goal_checker;
  std::shared_ptr<motion_planning::safety_checker_t> _safety_checker;

  std_msgs::Bool _contingency_msg;

public:
  mushr_sbmp_open_loop_t(ros::NodeHandle& nh, std::shared_ptr<interface::node_status_t> node_status)
    : _node_status(node_status), _current_cycle(0)
  {
    // _node_status = interface::node_status_t::create(nh);
    _node_status->status(interface::NodeStatus::INITIALIZING);
    _clock = std::make_unique<motion_planning::planner_clock_t>(ros::NodeHandle(nh, "clock"));
    _replanner = std::make_shared<motion_planning::sbmp_caller_t>(ros::NodeHandle(nh, "replanner"));
    _contingency_controller = std::make_shared<ContingencyController>(ros::NodeHandle(nh, "contingency"));
    _goal_checker = std::make_shared<motion_planning::goal_checker_t>(ros::NodeHandle(nh, "goal_checker"));
    _safety_checker = std::make_shared<motion_planning::safety_checker_t>(ros::NodeHandle(nh, "safety"));

    // TOPICS
    std::string control_topic, estimation_topic, contingency_topic;

    PARAM_SETUP(nh, control_topic)
    PARAM_SETUP(nh, estimation_topic)
    PARAM_SETUP(nh, contingency_topic)

    _current_control.space_point.point.emplace_back();  // u0
    _current_control.space_point.point.emplace_back();  // u1

    // PUBLISHERS
    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1);
    _control_stamped_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(control_topic + "_stamped", 1);
    _contingency_publisher = nh.advertise<std_msgs::Bool>(contingency_topic, 1);

    // SUBSCRIBERS
    _ekf_subscriber = nh.subscribe(estimation_topic, 1, &mushr_sbmp_open_loop_t::ekf_callback, this);

    // TIMERS
    const ros::Duration timer_duration(0.01);
    _timer = nh.createTimer(timer_duration, &mushr_sbmp_open_loop_t::timer_callback, this);

    _node_status->status(interface::NodeStatus::RUNNING);
  }

  ~mushr_sbmp_open_loop_t()
  {
  }

  void ekf_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _state_estimate = *msg;
  }

  void merge_result()
  {
    for (auto&& step : _next_plan)
    {
      _current_plan.push_back(step);
    }
    for (auto&& state : _next_trajectory)
    {
      _current_trajectory.push_back(state);
    }
  }

  template <typename Control>
  void publish_control(const Control& control)
  {
    _current_control.header.stamp = ros::Time::now();
    _current_control.space_point.point[0] = control[0];
    _current_control.space_point.point[1] = control[1];
    _control_publisher.publish(_current_control.space_point);
    _control_stamped_publisher.publish(_current_control);
  }

  void apply_contingency(const std::vector<double>& state)
  {
    const Eigen::Vector3d xdot(state[3], state[4], state[5]);
    const Eigen::Vector2d ctrl{ _contingency_controller->control(xdot) };
    publish_control(ctrl);

    _contingency_msg.data = true;
    _contingency_publisher.publish(_contingency_msg);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    auto& state = _state_estimate.space_point.point;
    if (state.size() != 6)
    {
      // prx_warn("[sbmp_open_loop] EKF state is empty!");
      return;
    }
    if (_current_plan.size() == 0)
    {
      apply_contingency(state);
      return;
    }
    const gtsam::Pose2 x_hat(state[0], state[1], state[2]);
    if (_goal_checker->goal_reached(x_hat))
    {
      apply_contingency(state);
      return;
    }

    // if (collision)
    // {
    //   apply_contingency(_current_trajectory.back().space_point.point);
    // }

    if (_safety_checker->is_safe(_state_estimate, _current_plan))
    {
      apply_contingency(state);
    }
    else if (ros::Time::now() > _current_plan.front().header.stamp)
    {
      // _current_control.space_point = _current_plan.front().plan_step.control;
      auto& ctrl{ _current_plan.front().plan_step.control.point };
      publish_control(ctrl);

      _current_plan.erase(_current_plan.begin());
      _contingency_msg.data = false;
      _contingency_publisher.publish(_contingency_msg);
    }
  }

  ml4kp_bridge::SpacePointStamped next_replanner_root()
  {
    ml4kp_bridge::SpacePointStamped root;
    root.header.stamp = _clock->cycle_end();

    prx_warn_cond((ros::Time::now() - _state_estimate.header.stamp).toSec() < 1.0,
                  "[sbmp_open_loop] EKF estimate is older than 1 sec.");

    // No trajectory available -> use current EKF state
    if (_current_trajectory.size() == 0)
    {
      root.space_point = _state_estimate.space_point;
      prx_assert(root.space_point.point.size() > 0, "[sbmp_open_loop] Empty point ekf state");
    }
    else
    {
      // Find the state closer to the end cycle.
      for (auto state : _current_trajectory)
      {
        if (state.header.stamp >= root.header.stamp)
        {
          root.space_point = state.space_point;
          prx_assert(root.space_point.point.size() > 0, "[sbmp_open_loop] Empty point in current trajectory");
          break;
        }
      }
      // Trajectory is shorter than the window -> just use the last point
      root.space_point = _current_trajectory.back().space_point;
      prx_assert(root.space_point.point.size() > 0, "[sbmp_open_loop] Empty root -- last point of current trajectory");
    }
    return root;
  }

  RePlannerPlan get_retainment_plan()
  {
    return RePlannerPlan();
  }

  void replanning_loop()
  {
    if (not _replanner->valid())
    {
      return;
    }
    if (_state_estimate.space_point.point.size() == 0)
    {
      const ros::Time deadline{ _clock->cycle_end() };
      const ros::Duration next_cycle{ (deadline - ros::Time::now()) * 0.8 };
      next_cycle.sleep();
      return;
    }

    if (_current_cycle < _clock->cycle())
    {
      _current_cycle = _clock->cycle();
      const ros::Time deadline{ _clock->cycle_end() };
      const ros::Duration planning_duration{ deadline - ros::Time::now() };
      const std::chrono::time_point future_limit{ std::chrono::steady_clock::now() +
                                                  std::chrono::seconds(planning_duration.sec) +
                                                  std::chrono::nanoseconds(planning_duration.nsec) };

      const ml4kp_bridge::SpacePointStamped root{ next_replanner_root() };

      const RePlannerPlan plan{ get_retainment_plan() };

      const double planning_time{ planning_duration.toSec() };
      std::future<RePlannerResult> future_result{ std::async(&RePlanner::call, _replanner,  // no-lint
                                                             planning_time, root, plan) };

      std::future_status status{ future_result.wait_until(future_limit) };

      if (status == std::future_status::ready)
      {
        std::tie(_next_trajectory, _next_plan) = future_result.get();
        merge_result();
      }
      else
      {
        auto now = std::chrono::steady_clock::now();
        const ros::Time replanner_failed_time{ ros::Time::now() };
        auto ms_now = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        auto ms_limit = std::chrono::duration_cast<std::chrono::milliseconds>(future_limit.time_since_epoch()).count();
        DEBUG_VARS(ms_now, ms_limit, deadline, replanner_failed_time)
      }
    }
    // }
  }

private:
  int _current_cycle;
  std::unique_ptr<motion_planning::planner_clock_t> _clock;
  std::shared_ptr<motion_planning::sbmp_caller_t> _replanner;

  std::shared_ptr<interface::node_status_t> _node_status;

  ros::Duration _postprocessing_duration;
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrSbmpOpenLoop" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // std::string mushr_model;
  std::string experiments_node_id;
  PARAM_SETUP(nh, experiments_node_id)

  std::shared_ptr<interface::node_status_t> node_status;
  std::shared_ptr<interface::node_status_t> experiments_node_status;
  node_status = interface::node_status_t::create(nh);
  node_status->status(interface::NodeStatus::INITIALIZING);

  experiments_node_status = interface::node_status_t::create(nh, experiments_node_id, true);

  std::shared_ptr<mushr_sbmp_open_loop_t> sbmp_caller;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (experiments_node_status->status() != interface::NodeStatus::FINISH)
  {
    handle_node_state(node_status);
    if (node_status->status() == interface::NodeStatus::RESET)
    {
      sbmp_caller = nullptr;
    }
    if (node_status->status() == interface::NodeStatus::RUNNING)
    {
      if (sbmp_caller == nullptr)
        sbmp_caller = std::make_shared<mushr_sbmp_open_loop_t>(nh, node_status);
      sbmp_caller->replanning_loop();
    }
  }
  spinner.stop();

  return 0;
}