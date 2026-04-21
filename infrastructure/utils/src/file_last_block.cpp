#include <filesystem>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <ros/subscriber.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <fstream>
#include <queue>
#include <thread>
#include <utility>
#include <vector>
#include <utils/dbg_utils.hpp>
#include <utils/nodelet_as_node.hpp>
#include <prx/utilities/general/csv_reader.hpp>

#include <prx/factor_graphs/lie_groups/se3.hpp>
#include <prx/factor_graphs/lie_groups/screw_axis.hpp>
#include <prx/factor_graphs/lie_groups/lie_integrator.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/plants/pusher_slider.hpp>
#include <prx/factor_graphs/utilities/values_utilities.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/utilities/general/type_conversions.hpp>

#include <gtsam/base/VectorSpace.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>

#include <interface/StampedMarkers.h>

#include <utils/rosparams_utils.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include "ml4kp_bridge/Trajectory.h"
#include <ml4kp_bridge/PlanTrajectory.h>
#include <ml4kp_bridge/lie_ode_observation.hpp>
#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/spaces/sampler.hpp>

// #include "nodelets/plant_estimator.cpp"

using prx::utilities::convert_to;
using CsvReader = prx::utilities::csv_reader_t;
using Line = std::vector<std::string>;

void write(std::string filename, std::vector<Line>& lines)
{
  std::ofstream ofs(filename);
  for (auto& line : lines)
  {
    for (auto& str : line)
    {
      ofs << str << " ";
    }
    ofs << "\n";
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FileDivider");
  ros::NodeHandle nh("~");

  std::string filename, output_filename;

  PARAM_SETUP(nh, filename)
  PARAM_SETUP(nh, output_filename)

  CsvReader reader(filename);
  std::ofstream ofs(output_filename.c_str());

  PRINT_MSG("Starting...")
  while (reader.has_next_line())
  {
    auto block = reader.next_block();
    if (block.size() > 0)
    {
      auto line = block.back();
      for (auto str : line)
      {
        ofs << str << " ";
      }
      ofs << "\n";
    }
  }
  ofs.close();
  PRINT_MSG("Done")
  return 0;
}