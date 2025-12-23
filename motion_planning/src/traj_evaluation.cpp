#include <thread>
#include "mujoco/mujoco.h"

#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include <utils/std_utils.cpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>

struct traj_t
{
  using State = prx_models::mushr_types::State::type;
  using Velocity = Eigen::Vector3d;

  std::vector<double> _timestamps;
  std::vector<State> _states;
  std::vector<Velocity> _velocities;

  using LieIntegrator = prx::fg::lie_integration_factor_t<State, Eigen::Vector3d>;

  traj_t(const std::string file)
  {
    using prx::utilities::convert_to;
    using CsvReader = prx::utilities::csv_reader_t;
    CsvReader reader(file);

    while (reader.has_next_line())
    {
      auto line = reader.next_line();

      if (line.size() == 0)
        continue;
      if (line[0][0] == '#')
        continue;

      const double dt{ convert_to<double>(line[0]) };

      const double x{ convert_to<double>(line[1]) };
      const double y{ convert_to<double>(line[2]) };
      const double th{ convert_to<double>(line[3]) };

      const double xDot{ convert_to<double>(line[4]) };
      const double yDot{ convert_to<double>(line[5]) };
      const double thDot{ convert_to<double>(line[6]) };

      if (_timestamps.size() == 0)
      {
        _timestamps.emplace_back(dt);
      }
      else
      {
        _timestamps.emplace_back(dt + _timestamps.back());
      }
      _states.emplace_back(x, y, th);
      _velocities.emplace_back(xDot, yDot, thDot);
    }
    // x0 = states[0].second;
    // const State x0_inv{ xo.inverse() };
  }

  double duration() const
  {
    return _timestamps.back();
  }

  static double error_0(const State xi, const State xj)
  {
    const Eigen::Vector3d xi_v{ State::Logmap(xi) };
    const Eigen::Vector3d xj_v{ State::Logmap(xj) };
    const Eigen::Vector3d diff{ xi_v - xj_v };
    const double err{ diff.norm() };
    return err;
  }

  std::pair<State, Velocity> operator()(const double t) const
  {
    auto const it = std::lower_bound(_timestamps.begin(), _timestamps.end(), t);
    if (it == _timestamps.begin())
    {
      const Velocity& xDot0{ _velocities.front() };
      const State& x0{ _states.front() };
      return { x0, xDot0 };
    }
    else if (it == _timestamps.end())
    {
      const Velocity& xDotF{ _velocities.back() };
      const State& xF{ _states.back() };
      return { xF, xDotF };
    }
    const std::size_t idx{ static_cast<std::size_t>(std::distance(_timestamps.begin(), it)) };

    const double t0{ _timestamps[idx - 1] };
    const double t1{ _timestamps[idx] };

    const Velocity& xDot0{ _velocities[idx - 1] };
    const Velocity& xDot1{ _velocities[idx] };

    const State& x0{ _states[idx - 1] };
    const State& x1{ _states[idx] };

    const double ti{ 1.0 - (t1 - t) / (t1 - t0) };

    const Velocity xDot_i{ xDot0 + (xDot1 - xDot0) * ti };

    const State xi{ State::interpolate(x0, x1, ti) };

    return { xi, xDot_i };
  }
};

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  using CtrlMsg = prx_models::MushrControl;
  using PlanMsg = prx_models::MushrPlan;
  const std::string node_name{ "MjFactorPlayground" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  const std::string filename{ "/Users/Gary/pracsys/catkin_ws/data/mj_mushr/sysid_v2/mj_traj_vels_e0000.txt" };

  traj_t traj(filename);

  for (double ti = 0; ti < traj.duration(); ti += 0.1)
  {
    auto [x, xdot] = traj(ti);

    // LOG_VARS(ti, x, xdot.transpose());
  }

  return 0;
}