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
#include <utils/dbg_utils.hpp>
#include <prx_models/planner_utils.hpp>
#include <prx_models/SO2_system.hpp>

template <typename DynamicalSystem>
struct collector_t
{
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;

  double dt;
  // prx::sampler_t<State> state_sampler;
  // prx::sampler_t<Control> control_sampler;
  std::shared_ptr<DynamicalSystem> _plant;

  std::ofstream _ofs;
  int total_trajectories;

  collector_t(ros::NodeHandle& nh)
  {
    // std::string environment;
    std::string plant_parameters, filename;
    // GLOBAL_PARAM_BLOCKER(environment);
    PARAM_SETUP(nh, dt);
    PARAM_SETUP(nh, total_trajectories);
    PARAM_SETUP(nh, filename);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    // env_params.from_string(environment);

    _ofs.open(filename);
    _plant = DynamicalSystem::create(plant_parameters);
  }

  void collect_data()
  {
    _ofs << "# dt X0 X1 U0\n";

    for (int i = 0; i < total_trajectories; ++i)
    {
      transition();
    }
  }

  void transition()
  {
    const State x0{ _plant->state_space()->sampler() };
    const Control u0{ _plant->control_space()->sampler() };
    const State x1{ _plant->propagate(x0, u0, dt) };
    DEBUG_VARS(x0)
    prx::to_stream(_ofs, dt);
    prx::to_stream(_ofs, x0);
    prx::to_stream(_ofs, x1);
    prx::to_stream(_ofs, u0);
    _ofs << "\n";
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrExperiments" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  collector_t<prx::SO2_system_t> collector(nh);

  collector.collect_data();
  return 0;
}