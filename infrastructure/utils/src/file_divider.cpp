#include <filesystem>
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

  std::string filename, output_directory, output_prefix;

  int lines_per_file;

  PARAM_SETUP(nh, filename);
  PARAM_SETUP(nh, output_prefix);
  PARAM_SETUP(nh, output_directory);
  PARAM_SETUP(nh, lines_per_file);

  int file_idx{ 0 };
  CsvReader reader(filename);
  Line line;
  std::vector<Line> buffer, current;
  prx::sampler_t<bool> sampler{};

  PRINT_MSG("Starting...")
  while (reader.next_valid_line(line))
  {
    if (sampler())
    {
      current.push_back(line);
    }
    else
    {
      buffer.push_back(line);
    }
    if (current.size() == lines_per_file)
    {
      // const std::string idx{ convert_to<std::string>(file_idx) };
      std::stringstream strstr;
      strstr << std::setfill('0') << std::setw(5) << convert_to<std::string>(file_idx);
      const std::string new_filename{ output_directory + "/" + output_prefix + "_" + strstr.str() + ".txt" };
      DEBUG_VARS(new_filename)
      write(new_filename, current);
      current.clear();
      file_idx++;
    }
  }

  const std::size_t remainig_lines{ buffer.size() };
  prx::sampler_t<int> line_sampler(0, remainig_lines);
  // while ((current.size() + buffer.size()) > lines_per_file)

  for (int i = 0; i < remainig_lines; ++i)
  {
    const int random_line{ line_sampler() % static_cast<int>(buffer.size()) };
    // auto iter = buffer.begin() + random_line;

    current.push_back(buffer[random_line]);
    buffer[random_line] = buffer.back();
    buffer.pop_back();

    if (current.size() == lines_per_file)
    {
      const ros::Time file_ti{ ros::Time::now() };
      // std::cout << std::setfill('0') << std::setw(5) << 25;
      std::stringstream strstr;
      strstr << std::setfill('0') << std::setw(5) << convert_to<std::string>(file_idx);
      const std::string new_filename{ output_directory + "/" + output_prefix + "_" + strstr.str() + ".txt" };
      const std::size_t remaining{ buffer.size() };
      DEBUG_VARS(remaining, new_filename)
      write(new_filename, current);
      current.clear();
      file_idx++;
    }

    // buffer.erase(iter);
  }

  PRINT_MSG("Done")
  return 0;
}