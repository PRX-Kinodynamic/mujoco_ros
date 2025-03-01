#include <ml4kp_bridge/defs.h>
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "prx_models/mj_mushr.hpp"
#include <control/MushrControlPropagation.h>
#include <ml4kp_bridge/TrajectoryStamped.h>
#include <prx_models/MushrObservation.h>
#include <std_msgs/Empty.h>

namespace control
{
class mushr_feedback_controller_t : public nodelet::Nodelet
{
  using Trajectory = std::shared_ptr<prx::trajectory_t>;

public:
  mushr_feedback_controller_t()
  {
    std::string params_file;
    ros::param::get("/ml4kp_params", params_file);
    std::cout << "Loading params from: " << params_file << std::endl;
    auto ml4kp_params = prx::param_loader(params_file);
    auto controller_params = prx::param_loader(ml4kp_params["feedback_controller"].as<std::string>());

    prx::simulation_step = ml4kp_params["simulation_step"].as<double>();
    std::string plant_name = ml4kp_params["/plant/name"].as<>();
    std::string plant_path = ml4kp_params["/plant/path"].as<>();
    plant = prx::system_factory_t::create_system(plant_name, plant_path);
    ROS_ASSERT(plant != nullptr);

    std::vector<double> min_state_limits = ml4kp_params["/plant/state_space/lower_bound"].as<std::vector<double>>();
    std::vector<double> max_state_limits = ml4kp_params["/plant/state_space/upper_bound"].as<std::vector<double>>();
    plant->get_state_space()->set_bounds(min_state_limits, max_state_limits);
    std::vector<double> param_values = ml4kp_params["/plant/parameter_space/values"].as<std::vector<double>>();
    plant->get_parameter_space()->copy_from(param_values);

    // Control space limits of the real MuSHR
    std::vector<double> min_control_limits = ml4kp_params["/plant/control_space/lower_bound"].as<std::vector<double>>();
    std::vector<double> max_control_limits = ml4kp_params["/plant/control_space/upper_bound"].as<std::vector<double>>();
    plant->get_control_space()->set_bounds(min_control_limits, max_control_limits);

    // prx::world_model_t control_model({ plant }, {});
    std::vector<prx::system_ptr_t> systems = { plant };
    std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = {};
    control_model = std::make_shared<prx::world_model_t>(systems, obstacles);
    control_model->create_context("control_context", { plant_name }, {});
    context = control_model->get_context("control_context");
    ROS_ASSERT(context.first != nullptr);

    current_state = plant->get_state_space()->make_point();
    empty_trajectory = std::make_shared<prx::trajectory_t>(plant->get_state_space());
    _has_experiment_started = false;

    if (controller_params["name"].as<std::string>() == "mushr_pure_pursuit")
    {
      controller = new prx::mushr_pure_pursuit_t(plant, controller_params);
    }
    else if (controller_params["name"].as<std::string>() == "mushr_stanley")
    {
      controller = new prx::mushr_stanley_t(plant, controller_params);
    }
    else
    {
      ROS_ERROR("Unknown controller type: %s", controller_params["name"].as<std::string>().c_str());
      return;
    }
    controller->set_goal(ml4kp_params["goal_state"].as<std::vector<double>>());
  }

  void onInit()
  {
    ros::NodeHandle& nh{ getPrivateNodeHandle() };
    std::string trajectory_topic, pose_topic, control_topic, reset_topic;

    nh.getParam("trajectory_topic", trajectory_topic);
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("control_topic", control_topic);
    nh.getParam("reset_topic", reset_topic);

    ros::Duration _timer_duration = ros::Duration(controller->control_duration());
    _timer = nh.createTimer(_timer_duration, &mushr_feedback_controller_t::control, this);

    _trajectory_subscriber = nh.subscribe(trajectory_topic, 1, &mushr_feedback_controller_t::get_trajectory, this);
    _pose_subscriber = nh.subscribe(pose_topic, 1, &mushr_feedback_controller_t::get_pose, this);
    _reset_subscriber = nh.subscribe(reset_topic, 1, &mushr_feedback_controller_t::reset, this);

    _control_publisher = nh.advertise<ml4kp_bridge::SpacePoint>(control_topic, 1, true);
    _ctrl.point.resize(plant->get_control_space()->get_dimension());

    _service_server =
        nh.advertiseService("/mushr/control_propagation", &mushr_feedback_controller_t::control_propagation, this);
  }

  void get_trajectory(const ml4kp_bridge::TrajectoryStamped& msg)
  {
    int trajectory_cycle_idx = msg.header.seq;
    Trajectory previous_trajectory = trajectory_map[trajectory_cycle_idx];

    if (msg.trajectory.data.size() == 0)
    {
      int current_cycle_idx = get_current_cycle_idx();
      contingency_enabled_map[current_cycle_idx] = true;
      contingency_enabled_map[trajectory_cycle_idx] = true;
      ROS_WARN("Contingency enabled for cycles %d and %d at %f", current_cycle_idx, trajectory_cycle_idx,
               (ros::Time::now() - _experiment_start_time).toSec());
    }
    else
    {
      trajectory_map[trajectory_cycle_idx] = std::make_shared<prx::trajectory_t>(plant->get_state_space());
      ml4kp_bridge::copy(trajectory_map[trajectory_cycle_idx], msg.trajectory);
      std::cout << "Received trajectory of cycle " << trajectory_cycle_idx << " at "
                << (ros::Time::now() - _experiment_start_time).toSec() << std::endl;
    }

    if (previous_trajectory != nullptr)
    {
      ROS_WARN("Trajectory already received for cycle %d", trajectory_cycle_idx);
    }

    if (!_has_experiment_started && trajectory_cycle_idx == 0)
    {
      _experiment_start_time = msg.header.stamp;
      _has_experiment_started = true;
    }
    else if (!_has_experiment_started && trajectory_cycle_idx > 0)
    {
      ROS_WARN("Experiment not started, but received trajectory of cycle %d", trajectory_cycle_idx);
    }
  }

  bool set_trajectory(int cycle_idx)
  {
    if (trajectory_map[cycle_idx] == nullptr)
    {
      ROS_WARN("Trajectory not received for cycle %d", cycle_idx);
      return false;
    }

    controller->set_points(trajectory_map[cycle_idx]);
    return true;
  }

  void get_pose(const prx_models::MushrObservation& msg)
  {
    prx_models::copy(current_state, msg);
    _pose_received = true;
  }

  void publish_zero_control()
  {
    current_control.resize(2);
    current_control[0] = 0.0;
    current_control[1] = 0.0;
    ml4kp_bridge::copy(_ctrl, current_control);
    _control_publisher.publish(_ctrl);
  }

  void reset(const std_msgs::Empty& msg)
  {
    controller->set_points(empty_trajectory);
    publish_zero_control();
    _pose_received = false;
    controller->reset();
    _has_experiment_started = false;
    _previous_cycle_idx = -1;
    trajectory_map.clear();
    contingency_enabled_map.clear();
  }

  int get_current_cycle_idx()
  {
    double time_since_experiment_start = (ros::Time::now() - _experiment_start_time).toSec();
    return std::floor(time_since_experiment_start);
  }
  
  void control(const ros::TimerEvent& event)
  {
    // Store the difference between event.current_real and experiment start time
    double time_since_experiment_start = (event.current_real - _experiment_start_time).toSec();

    if (!_has_experiment_started or time_since_experiment_start < 0)
    {
      return;
    }

    int current_cycle_idx = std::floor(time_since_experiment_start);

    if (contingency_enabled_map[current_cycle_idx] or trajectory_map[current_cycle_idx] == nullptr)
    {
      publish_zero_control();
      bool did_not_receive_trajectory = trajectory_map[current_cycle_idx] == nullptr and !contingency_enabled_map[current_cycle_idx];
      if (did_not_receive_trajectory) {
        ROS_WARN("Trajectory not received in time for cycle %d", current_cycle_idx);
      }
      contingency_enabled_map[current_cycle_idx] = true;
      return;
    }

    if (current_cycle_idx != _previous_cycle_idx)
    {
      set_trajectory(current_cycle_idx);
    }

    controller->get_control(current_state, current_control);
    plant->get_control_space()->enforce_bounds(current_control);
    ml4kp_bridge::copy(_ctrl, current_control);
    _control_publisher.publish(_ctrl);

    _previous_cycle_idx = current_cycle_idx;
  }

  bool control_propagation(control::MushrControlPropagation::Request& req,
                           control::MushrControlPropagation::Response& res)
  {
    if (!_has_experiment_started || !_pose_received)
    {
      return false;
    }
    prx::space_point_t propagation_current_state = plant->get_state_space()->clone_point(current_state);
    prx::plan_t plan(plant->get_control_space());
    plan.append_onto_back(controller->control_duration());
    double total_duration = req.duration.data.toSec();
    unsigned num_steps = total_duration / plan.back().duration;
    Eigen::VectorXd propagation_control;
    for (unsigned i = 0; i < num_steps; i++)
    {
      controller->get_control(propagation_current_state, propagation_control);
      plant->get_control_space()->copy(plan.back().control, propagation_control);
      plant->get_control_space()->enforce_bounds(plan.back().control);
      context.first->propagate(propagation_current_state, plan, propagation_current_state);
    }
    ml4kp_bridge::copy(res.final_state, propagation_current_state);
    return true;
  }

private:
  ros::Subscriber _trajectory_subscriber, _pose_subscriber, _reset_subscriber;
  ros::Publisher _control_publisher;
  ros::Timer _timer;
  ros::ServiceServer _service_server;
  ros::Time _experiment_start_time;
  prx_models::MushrObservation _pose;
  ml4kp_bridge::SpacePoint _ctrl;
  bool _has_experiment_started, _pose_received;
  int _previous_cycle_idx;

  prx::system_ptr_t plant;
  prx::feedback_controller_t<prx::trajectory_t>* controller;
  std::shared_ptr<prx::world_model_t> control_model;
  prx::world_model_context context;
  std::map<int, Trajectory> trajectory_map;
  std::map<int, bool> contingency_enabled_map;
  Trajectory empty_trajectory;
  prx::space_point_t current_state;
  Eigen::VectorXd current_control;
};
}  // namespace control
PLUGINLIB_EXPORT_CLASS(control::mushr_feedback_controller_t, nodelet::Nodelet);