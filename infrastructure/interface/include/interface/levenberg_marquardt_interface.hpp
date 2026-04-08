#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <ros/node_handle.h>
#include <string>

#include <utils/rosparams_utils.hpp>
#include "utils/dbg_utils.hpp"

namespace interface
{

void initialize(gtsam::LevenbergMarquardtParams& lm_params, const ros::NodeHandle& nh_optimizer)
{
  int iterations;
  std::string verbosity_level{ "SILENT" };

  PARAM_SETUP(nh_optimizer, iterations);
  PARAM_SETUP_WITH_DEFAULT(nh_optimizer, verbosity_level, verbosity_level);

  DEBUG_VARS(iterations)

  lm_params.setMaxIterations(iterations);
  lm_params.setVerbosityLM(verbosity_level);
  DEBUG_VARS(lm_params.getVerbosityLM())
}
void initialize(gtsam::LevenbergMarquardtParams& lm_params, const std::string parent_namespace)
{
  ros::NodeHandle nh_optimizer(parent_namespace + "/optimizer/");
  initialize(lm_params, nh_optimizer);
}
}  // namespace interface
