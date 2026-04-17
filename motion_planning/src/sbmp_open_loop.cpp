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

struct mushr_sbmp_open_loop_t
{
  using This = mushr_sbmp_open_loop_t;
  using RobotInterface = prx_models::mushr_stela_t;
  using State = prx_models::mushr_types::State::type;
  using StateDot = prx_models::mushr_types::StateDot::type;

  prx_models::StelaKraft _planner_service_call;
  ros::ServiceClient _planner_service_client;
  interface::PlannerClock _planner_clock_msg;

  ros::Timer _clock_timer, _control_publisher_timer;
  ros::Publisher _planner_clock_publisher;
  ros::Subscriber _ekf_subscriber;
  ros::Publisher _stamped_control_publisher, _control_publisher;

  bool _call_replanner, _new_tree_available;
  bool _validation_plan_feasibility, _validation_collision_only;

  std::shared_ptr<RobotInterface> _robot;

  ros::Time _next_control_dt;

  ml4kp_bridge::Plan _plan;

  State _q_hat;
  StateDot _qdot_hat;
  Eigen::Matrix3d _q_cov, _qdot_cov;
  motion_planning::tree_validation_params_t<RobotInterface> _validation_params;
  prx_models::tree_msg_wrapper_t _new_tree;
  ml4kp_bridge::SpacePointStamped _control_stamped;

  std::shared_ptr<control::contingency_controller_t> _contingency;

  mushr_sbmp_open_loop_t(ros::NodeHandle& nh)
    : _call_replanner(false)
    , _validation_plan_feasibility(false)
    , _validation_collision_only(false)
    , _new_tree_available(false)
    , _q_cov(Eigen::Matrix3d::Identity())
    , _qdot_cov(Eigen::Matrix3d::Identity())
  {
    double cycle_duration{ 1.0 };
    double replanner_solution_duration;
    std::string replanner_service, planner_clock_topic, plant_parameters, ekf_topic;
    std::string control_topic;

    PARAM_SETUP(nh, replanner_service)
    PARAM_SETUP(nh, planner_clock_topic)
    PARAM_SETUP(nh, control_topic)
    PARAM_SETUP(nh, ekf_topic)
    PARAM_SETUP(nh, replanner_solution_duration)
    PARAM_SETUP(nh, cycle_duration);
    GLOBAL_PARAM_SETUP_DEFAULT(plant_parameters, plant_parameters);

    _contingency = std::make_shared<control::contingency_controller_t>(nh, "LQR");
    _control_stamped.space_point.point.push_back(0.);
    _control_stamped.space_point.point.push_back(0.);

    const std::string stamped_control_topic{ control_topic + "_stamped" };

    interface::initialize(_validation_params.lm_params, ros::NodeHandle(nh, "lm"));

    prx::param_loader params;
    params.from_string(plant_parameters);
    _robot = std::make_shared<RobotInterface>();
    _robot->init(params);
    _validation_params.robot = _robot;

    const ros::Duration timer_duration(0.01);
    _planner_service_client = nh.serviceClient<prx_models::StelaKraft>(replanner_service);
    _planner_clock_publisher = nh.advertise<interface::PlannerClock>(planner_clock_topic, 1);
    _clock_timer = nh.createTimer(timer_duration, &This::clock_timer_callback, this);
    _control_publisher_timer = nh.createTimer(timer_duration, &This::control_timer_callback, this);

    _ekf_subscriber = nh.subscribe(ekf_topic, 1, &This::ekf_callback, this);
    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _stamped_control_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_control_topic, 1, true);

    _planner_service_call.request.condition = prx_models::StelaKraft::Request::CONDITION_TIME;
    _planner_service_call.request.solution_duration = ros::Duration(replanner_solution_duration);

    DEBUG_VARS(cycle_duration)
    _planner_clock_msg.cycle_duration = ros::Duration(cycle_duration);
    _planner_clock_msg.header.stamp = ros::Time::now();
    _planner_clock_msg.cycle_start = ros::Time::now();
    _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_start + _planner_clock_msg.cycle_duration;
  }

  void ekf_callback(const ml4kp_bridge::SpacePointStampedConstPtr& msg)
  {
    _q_hat[0] = msg->space_point.point[0];
    _q_hat[1] = msg->space_point.point[1];
    _q_hat[2] = msg->space_point.point[2];
    _qdot_hat[0] = msg->space_point.point[3];
    _qdot_hat[1] = msg->space_point.point[4];
    _qdot_hat[2] = msg->space_point.point[5];
  }

  void clock_timer_callback(const ros::TimerEvent& event)
  {
    _planner_clock_msg.state = interface::PlannerClock::LOW;
    if (_planner_clock_msg.cycle_start > ros::Time::now())
    {
      return;
    }

    _planner_clock_msg.header.stamp = ros::Time::now();
    if (_planner_clock_msg.header.stamp > _planner_clock_msg.cycle_end)
    {
      _planner_clock_msg.state = interface::PlannerClock::HIGH;
      _planner_clock_msg.cycle++;
      _planner_clock_msg.cycle_start = _planner_clock_msg.cycle_end;
      _planner_clock_msg.cycle_end = _planner_clock_msg.cycle_end + _planner_clock_msg.cycle_duration;
      // _planner_clock_msg.cycle_duration = _cycle_duration;
      _call_replanner = true;
    }
    _planner_clock_publisher.publish(_planner_clock_msg);
  }

  void get_next_root(prx_models::Node& node)
  {
    node.point.point.resize(6);
    node.point.point[0] = _q_hat[0];
    node.point.point[1] = _q_hat[1];
    node.point.point[2] = _q_hat[2];
    node.point.point[3] = _qdot_hat[0];
    node.point.point[4] = _qdot_hat[1];
    node.point.point[5] = _qdot_hat[2];
  }

  void control_timer_callback(const ros::TimerEvent& event)
  {
    if (ros::Time::now() > _next_control_dt)
    {
      _control_stamped.header.stamp = ros::Time::now();
      if (_plan.steps.size() > 0)
      {
        auto current_step = _plan.steps.front();
        _next_control_dt = _next_control_dt + current_step.duration.data;
        _control_stamped.space_point = current_step.control;
        DEBUG_VARS(current_step)
        _plan.steps.erase(_plan.steps.begin());
        _control_publisher.publish(_control_stamped.space_point);
        _stamped_control_publisher.publish(_control_stamped);
      }
      else
      {
        // PRINT_MSG("CONTINGENCY")
        // DEBUG_VARS(_qdot_hat.transpose())
        const Eigen::Vector<double, 2> ui{ _contingency->control(_qdot_hat) };
        // DEBUG_VARS(ui[0], ui[1])
        _control_stamped.space_point.point[0] = ui[0];
        _control_stamped.space_point.point[1] = ui[1];
        _control_publisher.publish(_control_stamped.space_point);
        _stamped_control_publisher.publish(_control_stamped);
        // PRINT_MSG("CONTINGENCY sent")
      }
    }
  }

  void replanner()
  {
    if (not _call_replanner)
      return;
    _planner_service_call.request.retain_plan = true;
    _planner_service_call.request.retianment_offset = _planner_clock_msg.cycle_duration;
    _planner_service_call.request.deadline = _planner_clock_msg.cycle_end;
    _planner_service_call.request.root.stamp = _planner_clock_msg.cycle_end;
    get_next_root(_planner_service_call.request.root);

    if (_planner_service_client.call(_planner_service_call))
    {
      if (_planner_service_call.response.planner_output == prx_models::StelaKraft::Response::TYPE_SUCCESS)
      {
        const std::size_t root_idx{ _planner_service_call.response.sln_tree.root };
        prx_models::tree_msg_wrapper_t wrapped_tree(_planner_service_call.response.sln_tree);
        _new_tree_available = true;

        if (_validation_plan_feasibility)
        {
          // auto graph_values = _robot->estimate_to_prior(0, , );
          _validation_params.estimates = { _q_hat, _qdot_hat };
          _validation_params.covariances = { _q_cov, _qdot_cov };
          _new_tree_available = motion_planning::check_new_tree(wrapped_tree, _validation_params);
        }

        if (_new_tree_available and _validation_collision_only)
        {
          _new_tree_available =
              _robot->propagate_plan({ _q_hat, _qdot_hat }, wrapped_tree);  // check_new_tree(wrapped_tree);
        }
        if (_new_tree_available)
        {
          _new_tree = wrapped_tree;
          if (_new_tree.nodes[_new_tree.root].children.size() > 0)
          {
            double t_accum{ 0.0 };
            while (t_accum < _planner_clock_msg.cycle_duration.toSec())
            {
              auto child = _new_tree.nodes[_new_tree.root].children[0];
              auto parent_edge = _new_tree.nodes[child].parent_edge;
              _plan.steps.push_back(_new_tree.edges[parent_edge].plan.steps[0]);
              t_accum += _new_tree.edges[parent_edge].plan.steps[0].duration.data.toSec();

              DEBUG_VARS(t_accum);
              // _planned_steps.push();
            }
            // if (_new_tree.edges[parent_edge].plan.steps.size() > 0)
            // {
            //   _control_publisher.publish(_control_stamped.space_point);
            //   _stamped_control_publisher.publish(_control_stamped);
            // }
          }

          const double dt_used_acepted{ (ros::Time::now() - _planner_clock_msg.cycle_start).toSec() };
          const double dt_remaining_accepted{ (_planner_service_call.request.deadline - ros::Time::now()).toSec() };
          // LOG_VARS(_new_tree_available, dt_used_acepted, dt_remaining_accepted)
        }
      }
    }
  }
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
  experiments_node_status = interface::node_status_t::create(nh, experiments_node_id, true);

  std::shared_ptr<mushr_sbmp_open_loop_t> sbmp_caller;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (experiments_node_status->status() != interface::NodeStatus::FINISH)
  {
    if (node_status->new_request())
    {
      node_status->status(node_status->requested_status());
    }
    if (node_status->sequence_id() != experiments_node_status->sequence_id())
    {
      sbmp_caller = nullptr;
      continue;
    }
    if (node_status->status() == interface::NodeStatus::RUNNING)
    {
      if (sbmp_caller == nullptr)
        sbmp_caller = std::make_shared<mushr_sbmp_open_loop_t>(nh);
      sbmp_caller->replanner();
    }
    else
    {
      sbmp_caller = nullptr;
    }
  }
  spinner.stop();

  return 0;
}