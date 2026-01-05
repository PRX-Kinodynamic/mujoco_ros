#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <ros/node_handle.h>
#include <utils/rosparams_utils.hpp>

namespace estimation
{

void calibration_from_parameters(gtsam::Pose3& pose, Eigen::Vector<double, 9>& calibration,
                                 const std::string camera_namespace)
{
  ros::NodeHandle nh(camera_namespace);
  ros::NodeHandle nh_pose(camera_namespace + "/pose");

  std::vector<double> K, distortion;
  std::vector<double> quaternion, position;

  PARAM_SETUP(nh, K)
  PARAM_SETUP(nh, distortion)
  PARAM_SETUP(nh_pose, quaternion)
  PARAM_SETUP(nh_pose, position)

  const double& fx{ K[0] };
  const double& fy{ K[4] };
  const double& s{ K[1] };
  const double& u0{ K[2] };
  const double& v0{ K[5] };
  const double& k1{ distortion[0] };
  const double& k2{ distortion[1] };
  const double& p1{ distortion[2] };
  const double& p2{ distortion[3] };

  const Eigen::Vector<double, 3> cam_position{ position.data() };
  const gtsam::Rot3 cam_quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

  calibration = { fx, fy, s, u0, v0, k1, k2, p1, p2 };
  pose = gtsam::Pose3(cam_quat, cam_position);
}
}  // namespace estimation