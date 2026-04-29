#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

#include <iterator>
#include <memory>
#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/std_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <interface/node_status.hpp>
#include <interface/SensorDataStamped.h>
#include <prx_models/mushr_factors.hpp>
#include <prx_models/mushr.hpp>
#include <prx_models/PlannerStats.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <utils/dbg_utils.hpp>
#include <prx_models/planner_utils.hpp>
#include <std_msgs/Float64MultiArray.h>

using State = Eigen::Vector<double, 6>;
using Control = Eigen::Vector<double, 2>;
using Trajectory = std::vector<std::pair<State, Control>>;
using prx::utilities::convert_to;

visualization_msgs::Marker create_marker()
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "mj_trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

std::vector<Trajectory> read_trajectories(const std::string filename, const int total_trajectories)
{
  // #  0  1  2  3   4  5  6    7     8     9     10     11    12   13 14
  // # ti x0 y0 th0 x1 y1 th1 xdot0 ydot0 thdot0 xdot1 ydot1 thdot1 u0 u1
  prx::utilities::csv_reader_t reader(filename);

  std::vector<Trajectory> trajectories;
  for (int i = 0; i < total_trajectories;)
  {
    Trajectory traj;
    auto block = reader.next_block();
    if (block.empty())
    {
      continue;
    }
    ++i;

    for (auto line : block)
    {
      if (line.empty())
        continue;
      if (line[0][0] == '#')
        continue;

      const double x{ convert_to<double>(line[1]) };
      const double y{ convert_to<double>(line[2]) };
      const double z{ convert_to<double>(line[3]) };
      const double xd{ convert_to<double>(line[7]) };
      const double yd{ convert_to<double>(line[8]) };
      const double zd{ convert_to<double>(line[9]) };

      const double u0{ convert_to<double>(line[13]) };
      const double u1{ convert_to<double>(line[14]) };

      traj.emplace_back(std::make_pair(State(x, y, z, xd, yd, zd), Control(u0, u1)));
    }

    trajectories.push_back(traj);
  }
  return trajectories;
}

visualization_msgs::MarkerArray trajectories_to_markers(const std::vector<Trajectory> trajectories,
                                                        const std::string ns, const Eigen::Vector4d color)
{
  visualization_msgs::MarkerArray markers;

  auto marker = create_marker();

  marker.color.a = color[0];  // Don't forget to set the alpha!
  marker.color.r = color[1];
  marker.color.g = color[2];
  marker.color.b = color[3];
  for (auto traj : trajectories)
  {
    marker.points.clear();
    marker.ns = ns + convert_to<std::string>(marker.id);
    for (auto state_ctrl : traj)
    {
      const State x{ state_ctrl.first };
      marker.points.emplace_back();
      marker.points.back().x = x[3];
      marker.points.back().y = x[4];
      marker.points.back().z = x[5];
    }
    markers.markers.push_back(marker);
    marker.id++;
  }

  return markers;
}

struct prx_mushr_tunner_t
{
  ros::NodeHandle _nh;
  ros::Subscriber _param_sub;
  std::vector<Trajectory> _trajectories;
  ros::Publisher _pub_poly;
  prx_mushr_tunner_t(ros::NodeHandle& nh, std::vector<Trajectory> trajectories) : _nh(nh), _trajectories(trajectories)
  {
    _param_sub = nh.subscribe("/mushr/tunner", 1, &prx_mushr_tunner_t::callback, this);
    _pub_poly = _nh.advertise<visualization_msgs::MarkerArray>("/poly/trajectories", 1, true);
  }

  void callback(const std_msgs::BoolConstPtr msg)
  {
    compute_trajectories();
  }

  void compute_trajectories()
  {
    // Analytical system
    std::string plant_parameters, environment;
    using prx::simulation_step;

    GLOBAL_PARAM_BLOCKER(environment);
    GLOBAL_PARAM_BLOCKER(simulation_step);
    GLOBAL_PARAM_BLOCKER(plant_parameters)

    prx::param_loader plant_params, env_params;

    env_params.from_string(environment);
    plant_params.from_string(plant_parameters);

    auto plant = prx::system_factory_t::create_system(plant_params);
    auto [planning_model, system_group, collision_group] = prx::world_model_t::create(env_params, plant);
    DEBUG_VARS(plant);

    std::vector<Trajectory> mushr_trajectories;
    prx::space_point_t x0{ system_group->get_state_space()->make_point() };
    for (auto traj : _trajectories)
    {
      Trajectory mushr_traj;
      prx::plan_t plan(system_group->get_control_space());
      prx::trajectory_t res_traj(system_group->get_state_space());
      for (auto state_ctrl : traj)
      {
        plan.copy_onto_back(state_ctrl.second, prx::simulation_step);  // Assuming 0.1 dt
      }
      Vec(x0) = traj[0].first;
      system_group->propagate(x0, plan, res_traj);

      for (int i = 0; i < res_traj.size(); ++i)
      {
        // dummy ctrl, mushr_trajs is only to create the markers
        mushr_traj.emplace_back(std::make_pair(Vec(res_traj[i]), Control(0, 0)));
      }
      mushr_trajectories.push_back(mushr_traj);
    }
    visualization_msgs::MarkerArray poly_markers{ trajectories_to_markers(mushr_trajectories, "Poly_",
                                                                          Eigen::Vector4d(1., 0, 1., 0.)) };

    _pub_poly.publish(poly_markers);
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrExperiments" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  std::string filename;
  int total_trajectories;

  PARAM_SETUP(nh, filename)
  PARAM_SETUP(nh, total_trajectories)

  const std::vector<Trajectory> trajectories{ read_trajectories(filename, total_trajectories) };

  visualization_msgs::MarkerArray markers{ trajectories_to_markers(trajectories, "MJ_",
                                                                   Eigen::Vector4d(1., 1., 0., 1.)) };

  ros::Publisher pub{ nh.advertise<visualization_msgs::MarkerArray>("/mj/trajectories", 1, true) };
  pub.publish(markers);

  prx_mushr_tunner_t tunner(nh, trajectories);

  ros::spin();
  return 0;
}