#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>

#include <iterator>
#include <memory>
#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/math/multivariate_gaussian_distribution.hpp>
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
  using StateSampler = prx::lie_group_gaussian_noise_t<State>;

  double dt;
  // prx::sampler_t<State> state_sampler;
  // prx::sampler_t<Control> control_sampler;
  std::shared_ptr<DynamicalSystem> _plant;
  StateSampler x_sampler;

  std::ofstream _ofs;
  int total_trajectories;

  collector_t(ros::NodeHandle& nh)
  {
    // std::string environment;
    std::string plant_parameters, filename;
    std::vector<double> covariance_as_vector;
    // GLOBAL_PARAM_BLOCKER(environment);
    PARAM_SETUP(nh, dt);
    PARAM_SETUP(nh, total_trajectories);
    PARAM_SETUP(nh, filename);
    PARAM_SETUP(nh, covariance_as_vector);
    GLOBAL_PARAM_BLOCKER(plant_parameters);
    // env_params.from_string(environment);

    _ofs.open(filename);
    _plant = DynamicalSystem::create(plant_parameters);

    typename StateSampler::Covariance cov_x;

    prx_assert(cov_x.rows() * cov_x.cols() == covariance_as_vector.size(), "Mismatch size on covariance vector");
    int idx{ 0 };
    for (int i = 0; i < cov_x.rows(); ++i)
    {
      for (int j = 0; j < cov_x.cols(); ++j)
      {
        cov_x(i, j) = covariance_as_vector[idx];
        idx++;
      }
    }
    DEBUG_VARS(cov_x);
    x_sampler.set(cov_x);
  }

  void collect_data()
  {
    _ofs << "# dt X0 X1 U0 X1_HAT \n";

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
    const State x1_hat{ x_sampler(x1) };
    // DEBUG_VARS(x0)
    prx::to_stream(_ofs, dt);
    prx::to_stream(_ofs, x0);
    prx::to_stream(_ofs, x1);
    prx::to_stream(_ofs, u0);
    prx::to_stream(_ofs, x1_hat);
    _ofs << "\n";
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "DynamicalSystemDataCollector" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  collector_t<prx::SO2_system_t> collector(nh);

  collector.collect_data();
  return 0;
}