#include <thread>

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// mj-ros
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <estimation/TrajectoryEstimation.h>
#include <estimation/StateEstimation.h>
// #include <estimation/fg_trajectory_estimation.hpp>
#include <analytical/fg_ltv_sde.hpp>
#include <motion_planning/sdf_factor.hpp>

// ML4KP
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include <prx/factor_graphs/factors/euler_integration_factor.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>

// GTSAM
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

int main(int argc, char** argv)
{
  using SystemInterface = prx::fg::ltv_sde_utils_t;
  using State = typename SystemInterface::State;
  using Sdf = utils::signed_distance_field_t;
  using SdfPtr = std::shared_ptr<Sdf>;
  using SdfFactor = motion_planning::sdf_factor_t<State, typename SystemInterface::ConfigFromState>;
  using SF = prx::fg::symbol_factory_t;

  const std::string node_name{ "ObstacleFactorsExample" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  int total_states;
  double safety_distance, obstacle_sigma;
  prx::param_loader params{};
  std::string sdf_file, outfile;

  ROS_PARAM_SETUP(nh, total_states);
  ROS_PARAM_SETUP(nh, safety_distance);
  ROS_PARAM_SETUP(nh, sdf_file);
  ROS_PARAM_SETUP(nh, outfile);
  ROS_PARAM_SETUP(nh, obstacle_sigma);

  params.add_file(sdf_file);
  SdfPtr sdf{ Sdf::create(params) };

  const gtsam::Key k(0);
  gtsam::GaussNewtonParams gn_params;
  gn_params.setMaxIterations(1);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(1);
  lm_params.setVerbosityLM("SUMMARY");

  std::ofstream ofs(outfile.c_str());
  auto obstacle_noise = gtsam::noiseModel::Isotropic::Sigma(1, obstacle_sigma);

  for (int i = 0; i < total_states; ++i)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const State initial{ prx::uniform_random(-5.0, 25.0), prx::uniform_random(-5.0, 25.0) };
    values.insert(k, initial);
    graph.addPrior(k, initial);
    graph.emplace_shared<SdfFactor>(k, safety_distance, sdf, obstacle_noise);
    // DEBUG_VARS(initial.transpose());
    // gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    gtsam::GaussNewtonOptimizer optimizer(graph, values, gn_params);
    graph.printErrors(values, "Graph");
    // gtsam::Values result{ optimizer.optimize() };
    gtsam::Values result{ optimizer.optimize() };

    const State final{ result.at<State>(k) };

    ofs << initial.transpose() << " ";
    ofs << final.transpose() << " ";
    ofs << "\n";
  }

  // for (int i = 0; i < total_states; ++i)
  // {
  //   const gtsam::Key k(i);
  // }
  ofs.close();

  return 0;
}