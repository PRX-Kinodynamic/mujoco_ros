#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <ros/subscriber.h>
#include <tf2_ros/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.hpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <prx_models/mushr.hpp>

using Pose = gtsam::Pose2;
using Velocity = Eigen::Vector3d;
using State = gtsam::ProductLieGroupV43<Pose, Velocity>;

using Control = Eigen::Vector2d;
using Step = std::tuple<State, Control, double>;

using Line = std::vector<std::string>;

// Needed because somehow a CSV sometimes has multiple separating characters i.e. comma and space
Line remove_commas(Line& line)
{
  Line no_commas_line;
  for (auto e : line)
  {
    e.erase(std::remove(e.begin(), e.end(), ','), e.end());
    if (e.size() > 0)
    {
      no_commas_line.push_back(e);
    }
  }
  return no_commas_line;
}

std::vector<Step> read_file(const std::string filename)
{
  using prx::utilities::convert_to;
  prx::utilities::csv_reader_t reader(filename);
  Line line;

  std::vector<Step> traj;
  while (reader.next_valid_line(line))
  {
    line = remove_commas(line);
    // DEBUG_VARS(line.size(), line)

    //   0. 1.   2.   3.  4.  5.    6.      7.       8
    // # x, y, theta, vx, vy, w, vel_cmd, steer_cmd, dt
    const double x{ convert_to<double>(line[0]) };
    const double y{ convert_to<double>(line[1]) };
    const double theta{ convert_to<double>(line[2]) };

    const double vx{ convert_to<double>(line[3]) };
    const double vy{ convert_to<double>(line[4]) };
    const double w{ convert_to<double>(line[5]) };

    double u0{ 0. };
    double u1{ 0. };
    double dt{ 0. };
    if (line[9] == "nan")
    {
      u0 = -1.;
      u1 = -1.;
      dt = -1.;
    }
    else
    {
      u0 = convert_to<double>(line[6]);
      u1 = convert_to<double>(line[7]);
      dt = convert_to<double>(line[8]);
    }

    const State state(Pose(x, y, theta), Eigen::Vector3d(vx, vy, w));
    const Control control{ Control(u0, u1) };
    traj.push_back(std::make_tuple(state, control, dt));
  }
  return traj;
}

Eigen::Vector<double, 6> state_to_vec(const State x)
{
  Eigen::Vector<double, 6> v;
  v[0] = x.first.x();
  v[1] = x.first.y();
  v[2] = x.first.theta();

  v.tail(3) = x.second;
  return v;
}

std::vector<State> propagate(std::vector<Step>& gt_data, std::shared_ptr<prx::system_group_t> system_group)
{
  std::vector<State> sim_trajectory;
  prx::plan_t plan(system_group->get_control_space());
  prx::trajectory_t traj(system_group->get_state_space());

  for (auto step : gt_data)
  {
    const Control u{ std::get<Control>(step) };
    const double dt{ std::get<double>(step) };
    plan.copy_onto_back(u, dt);
  }
  prx::space_point_t x0 = system_group->get_state_space()->make_point();
  Vec(x0) = state_to_vec(std::get<State>(gt_data.front()));

  plan.pop_back();  // remove the nans
  system_group->propagate(x0, plan, traj);

  std::vector<State> traj_out;
  for (auto step : traj)
  {
    auto vec = Vec(step);
    traj_out.emplace_back(Pose(vec[0], vec[2], vec[2]), vec.tail(3));
  }
  return traj_out;
}

std::pair<double, double> state_error(const State& hat, const Step& step)
{
  const Pose x_hat{ hat.first };
  const Pose x_gt{ std::get<State>(step).first };

  const Pose btw{ x_gt.between(x_hat) };
  const Eigen::Vector3d err{ gtsam::traits<Pose>::Logmap(btw) };
  return { err.head(2).norm(), err.tail(1).norm() };
}

struct traj_errors_t
{
  traj_errors_t() : total_steps(0) {};
  traj_errors_t& operator+=(const traj_errors_t& other)
  {
    *this += other.total_error;
    this->final_error.first += other.final_error.first;
    this->final_error.second += other.final_error.second;
    return *this;
  }

  traj_errors_t& operator+=(const std::pair<double, double>& other)
  {
    total_error.first += other.first;
    total_error.second += other.second;
    total_steps++;
    return *this;
  }
  static std::pair<double, double> average(std::pair<double, double>& err, int tot)
  {
    const double tot_db{ static_cast<double>(tot) };
    return { err.first / tot_db, err.second / tot_db };
  }

  std::pair<double, double> final_error;
  std::pair<double, double> total_error;
  int total_steps;

  void print()
  {
    const double final_position_error{ final_error.first };
    const double final_angle_error{ final_error.second };

    const double total_position_error{ total_error.first };
    const double total_angle_error{ total_error.second };

    DEBUG_VARS(final_position_error, final_angle_error, total_position_error, total_angle_error, total_steps);
  }
};

traj_errors_t compute_errors(std::vector<State>& sim, std::vector<Step>& gt)
{
  traj_errors_t errs;

  DEBUG_VARS(gt.size())
  for (int i = 0; i < gt.size(); ++i)
  {
    errs += state_error(sim[i], gt[i]);
  }
  errs.final_error = state_error(sim.back(), gt.back());
  errs.total_error = traj_errors_t::average(errs.total_error, errs.total_steps);

  // DEBUG_VARS(sim.back(), gt.back())
  // errs.print();
  return errs;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RealMushrDataSync");
  ros::NodeHandle nh("~");

  // PRX FILES
  std::string environment;
  std::string plant_parameters;

  prx::param_loader plant_params, env_params;

  using prx::simulation_step;

  GLOBAL_PARAM_BLOCKER(environment);
  GLOBAL_PARAM_BLOCKER(plant_parameters);
  GLOBAL_PARAM_BLOCKER(simulation_step);

  env_params.from_string(environment);
  plant_params.from_string(plant_parameters);
  auto plant = prx::system_factory_t::create_system(plant_params);
  auto [planning_model, system_group, collision_group] = prx::world_model_t::create(env_params, plant);

  std::string path;
  PARAM_SETUP(nh, path)

  traj_errors_t all_errors;
  for (const auto& entry : std::filesystem::directory_iterator(path))
  {
    const std::string filename{ entry.path() };

    std::vector<Step> gt_data{ read_file(filename) };
    std::vector<State> sim_data{ propagate(gt_data, system_group) };

    traj_errors_t traj_errors{ compute_errors(sim_data, gt_data) };
    all_errors += traj_errors;
    all_errors.print();
  }
  all_errors.total_error = traj_errors_t::average(all_errors.total_error, all_errors.total_steps);
  all_errors.final_error = traj_errors_t::average(all_errors.final_error, all_errors.total_steps);
  all_errors.print();

  return 0;
}