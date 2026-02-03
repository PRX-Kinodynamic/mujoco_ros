#pragma once
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <mujoco/mujoco.h>

namespace prx_models
{
inline mjModel* init_mj_model(const std::string model_path, const mjVFS* mj_vfs = NULL)
{
  std::string error;
  error.reserve(1000);
  mjModel* mj_model{ mj_loadXML(model_path.c_str(), mj_vfs, error.data(), error.capacity()) };
  if (!mj_model or error.size() != 0)
  {
    std::cerr << "Error in loading model." << std::endl;
    std::cout << error << std::endl;
  }
  return mj_model;
}

inline mjData* init_mj_data(const mjModel* mj_model)
{
  mjData* mj_data{ mj_makeData(mj_model) };
  for (int i = 0; i < 100; i++)
  {
    mj_step(mj_model, mj_data);
  }
  return mj_data;
}

inline gtsam::Pose3 mj_copy(const Eigen::Vector<double, 7>& pose)
{
  const gtsam::Rot3 rot(gtsam::Quaternion(pose[3], pose[4], pose[5], pose[6]));
  return gtsam::Pose3(rot, pose.head(3));
}

inline Eigen::Vector<double, 7> mj_copy(const gtsam::Pose3& pose)
{
  const Eigen::Vector3d pt{ pose.translation() };
  const Eigen::Quaterniond quat{ pose.rotation().toQuaternion() };
  return (Eigen::Vector<double, 7>() << pt, quat.w(), quat.x(), quat.y(), quat.z()).finished();
}

}  // namespace prx_models