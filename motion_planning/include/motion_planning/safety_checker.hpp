#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>

#include <iterator>
#include <memory>
#include <string>
#include <std_msgs/Bool.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <interface/PlannerClock.h>

#include <motion_planning/utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <prx/simulation/collision_checking/pqp_collision_checker.hpp>
#include <utils/rosparams_utils.hpp>
#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>

namespace motion_planning
{

class safety_checker_t
{
  using Trajectory = std::vector<ml4kp_bridge::SpacePointStamped>;
  using Plan = std::vector<ml4kp_bridge::PlanStepStamped>;

  // using PqpInfo = prx::collision_checking::system_pqp_info_t;
  Trajectory _trajectory;
  prx::system_ptr_t _plant;

  ros::Publisher _traj_estimation_publisher;

  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::world_model_t> _planning_model;
  std::shared_ptr<prx::collision_group_t> _collision_group;

  visualization_msgs::Marker _trajectory_marker;

  // PqpInfo _pqp_info;

  // prx::collision_checking::system_pqp_info_t _system_pqp_info;

  // PQP_CollideResult _collision_result;
  // std::vector<std::shared_ptr<prx::collision_checking::pqp_info_t>> _obstacles_pqp_infos;

public:
  safety_checker_t(ros::NodeHandle nh)
  {
    // PRX FILES
    std::string environment;
    std::string plant_parameters;

    // PRX PARAM LOADERS FOR PRX FILES
    prx::param_loader plant_params, env_params;

    using prx::simulation_step;

    GLOBAL_PARAM_BLOCKER(environment);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    GLOBAL_PARAM_BLOCKER(simulation_step);

    env_params.from_string(environment);
    plant_params.from_string(plant_parameters);

    _plant = prx::system_factory_t::create_system(plant_params);

    prx_assert(_plant != nullptr, "[mushr_sbmp_open_loop_t] prx::system_factory_t::create_system failed.");

    std::tie(_planning_model, _system_group, _collision_group) = prx::world_model_t::create(env_params, _plant);

    auto movable_object = std::dynamic_pointer_cast<prx::movable_object_t>(_plant);
    prx_assert(movable_object != nullptr, "[mushr_sbmp_open_loop_t] Couldn't cast plant to prx::movable_object.");

    // _pqp_info = prx::collision_checking::system_pqp_info_t::from_geometries(movable_object);

    // SET OBSTACLES
    prx::obstacle_loader_t obstacle_loader{ prx::obstacle_loader_t(env_params) };
    auto obstacle_list = obstacle_loader.get_obstacles();
    const std::vector<std::shared_ptr<prx::movable_object_t>> all_obstacles{ { obstacle_list } };
    // _obstacles_pqp_infos = prx::collision_checking::pqp_info_t::from_obstacles(all_obstacles);

    _trajectory_marker = ml4kp_bridge::create_marker(0.01, { 1, 1, 0, 0 });
    _trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;

    _traj_estimation_publisher = nh.advertise<visualization_msgs::Marker>("/safety/trajectory/marker", 1);
  }

  ~safety_checker_t()
  {
  }

  const Plan expand_plan(const Plan& plan)
  {
    Plan expanded_plan;

    for (auto step : plan)
    {
      const double dt{ step.plan_step.duration.data.toSec() };
      for (double ti = 0.; ti < dt; ti += prx::simulation_step)
      {
        expanded_plan.emplace_back();
        expanded_plan.back().plan_step.duration.data = ros::Duration(prx::simulation_step);
        expanded_plan.back().plan_step.control = step.plan_step.control;
      }
    }

    return expanded_plan;
  }

  template <typename X0>
  Trajectory propagate(const X0& x0, const Plan& plan)
  {
    Trajectory result;
    const std::size_t state_dim{ 6 };

    // result.push_back(x0);
    result.emplace_back();
    result.back().space_point.point.resize(state_dim);
    _system_group->get_state_space()->copy_to(result.back().space_point.point);

    for (auto step : plan)
    {
      const double dt{ step.plan_step.duration.data.toSec() };
      _system_group->get_control_space()->copy_from(step.plan_step.control.point);

      for (double ti = 0.; ti < dt; ti += prx::simulation_step)
      {
        _planning_model->step_simulation();
        result.emplace_back();
        result.back().space_point.point.resize(state_dim);
        _system_group->get_state_space()->copy_to(result.back().space_point.point);
      }
    }
    return result;
  }

  void randup(const ml4kp_bridge::SpacePointStamped& x_hat, const Plan& plan_in, const int n,  // no-lint
              Eigen::Matrix<double, 6, 6> x_cov, Eigen::Matrix<double, 2, 2> u_cov, Eigen::Matrix<double, 6, 6> w_cov)
  {
    // prx_models::multivariate_gaussian_t<6> x_gaussian_sampler(x_cov);
    // prx_models::multivariate_gaussian_t<2> u_gaussian_sampler(u_cov);
    // prx_models::multivariate_gaussian_t<6> w_gaussian_sampler(w_cov);

    // _system_group->get_state_space()->copy_to(, x_hat.space_point.point);
    // for (int i = 0; i < n; ++i)
    // {
    //   Trajectory propagate(const X0& x0, const Plan& plan);
    // }
  }

  bool is_safe(const ml4kp_bridge::SpacePointStamped& x_hat, const Plan& plan_in)
  {
    DEBUG_VARS(x_hat)
    const Plan plan{ expand_plan(plan_in) };
    ml4kp_bridge::propagate(x_hat, plan, _trajectory, _system_group);
    // _system_group->get_state_space()->copy_from(_current_trajectory.back().space_point.point);
    // const bool collision{ _collision_group->in_collision() };
    DEBUG_VARS(_trajectory.size())

    ml4kp_bridge::update_marker(_trajectory_marker, _trajectory, 0, 1, 0.0);
    _trajectory_marker.id = 0;
    _traj_estimation_publisher.publish(_trajectory_marker);

    return true;
    // std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configurations;
  }

  bool trajectory_in_collision(const Trajectory& trajectory)
  {
    int idx{ 0 };
    for (auto state : trajectory)
    {
      auto& pt = state.space_point.point;
      // _system_pqp_info.configurations[0].first = prx::axis_to_rotation_matrix(pt[2], 'Z');
      // _system_pqp_info.configurations[0].second[0] = pt[0];
      // _system_pqp_info.configurations[0].second[1] = pt[1];
      // _system_pqp_info.configurations[0].second[2] = 0.;
      // _system_pqp_info.configurations = _system->configuration(state);
      // const bool coll{ prx::collision_checking::collision(_collision_result, _pqp_info, _obstacles_pqp_infos) };
      // DEBUG_VARS(idx);
      idx++;
      // if (coll)
      //   return false;
    }
    return true;
  }

private:
};
}  // namespace motion_planning
