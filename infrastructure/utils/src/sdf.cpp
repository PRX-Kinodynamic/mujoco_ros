#include <thread>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/SetDuration.h>
#include <utils/signed_distance_field.hpp>
int main(int argc, char** argv)
{
  using SDF = utils::signed_distance_field_t;

  const std::string node_name{ "SDF" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  prx::param_loader params{ SDF::default_parameters() };

  bool force_recompute{ true };
  std::string filename{ "" };
  // PARAM_SETUP_WITH_DEFAULT(nh, filename, filename)
  // PARAM_SETUP_WITH_DEFAULT(nh, force_recompute, force_recompute)
  // if (std::filesystem::exists(filename))
  // {
  // DEBUG_VARS(filename);
  // params.add_file(filename);
  // }
  // params["force_recompute"].set(force_recompute);
  // ml4kp_bridge::check_for_ros_params(params, nh);
  // params.print();

  std::shared_ptr<SDF> sdf(SDF::create(nh));

  sdf->to_file();

  return 0;
}