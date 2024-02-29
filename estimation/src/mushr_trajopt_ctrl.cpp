#include <thread>
#include <random>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <utils/dbg_utils.hpp>
#include <gtsam/nonlinear/ISAM2.h>

#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/factor_graphs/utilities/common_functions.hpp>
#include <prx/factor_graphs/factors/screw_axis.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using SF = prx::fg::symbol_factory_t;
auto k_Pose = [](const std::size_t& i, const std::size_t& j, const std::size_t& ti) {
  return SF::create_hashed_symbol("^{", i, "}p^{", ti, "}_{", j, "}");
};
auto k_T = [](const std::size_t& i, const std::size_t& j) { return SF::create_hashed_symbol("^{", i, "}T^{", j, "}"); };
auto k_V = [](const std::size_t& ti, const std::size_t& p) {
  return SF::create_hashed_symbol("V^{", p, "}_{", ti, "}");
};

// aPc = aTb * bPc
class transform_factor_t : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>;
  using Jacobian = Eigen::MatrixXd;
  using Error = gtsam::Vector;
  transform_factor_t(){};
  transform_factor_t(const gtsam::Key& pi, const gtsam::Key& pj, const gtsam::Key& Tij,
                     const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, pi, pj, Tij)
  {
  }

  Error evaluateError(const gtsam::Pose3& pi, const gtsam::Pose3& pj, const gtsam::Pose3& Tij,
                      boost::optional<Jacobian&> H1 = boost::none, boost::optional<Jacobian&> H2 = boost::none,
                      boost::optional<Jacobian&> H3 = boost::none) const override
  {
    const gtsam::Pose3 pj_predicted{ Tij.transformPoseFrom(pi, H1 ? &D_predict_pi : 0, H3 ? &D_predict_Tij : 0) };
    const gtsam::Pose3 error{ pj.between(pj_predicted, H2 ? &D_error_pj : 0, H1 || H3 ? &D_error_predict : 0) };

    const Eigen::Vector<double, 3> p_error{ error.translation() };
    const Eigen::Vector<double, 3> R_error{ gtsam::Rot3::Logmap(error.rotation()) };
    if (H1)
    {
      *H1 = D_error_predict * D_predict_pi;
    }
    if (H2)
    {
      *H2 = D_error_pj;
    }
    if (H3)
    {
      *H3 = D_error_predict * D_predict_Tij;
    }
    return (Eigen::Vector<double, 6>() << R_error, p_error).finished();
  }

private:
  mutable Eigen::Matrix<double, 6, 6> D_predict_pi;
  mutable Eigen::Matrix<double, 6, 6> D_predict_Tij;
  mutable Eigen::Matrix<double, 6, 6> D_error_pj, D_error_predict;
};

class pose_velocity_factor_t : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, prx::fg::screw_axis_t>
{
public:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, prx::fg::screw_axis_t>;
  using Jacobian = Eigen::MatrixXd;
  using Error = gtsam::Vector;

  pose_velocity_factor_t(const gtsam::Key& pi, const gtsam::Key& pj, const gtsam::Key& Vij, double dt,
                         const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, pi, pj, Vij), _dt(dt)
  {
  }

  Error evaluateError(const gtsam::Pose3& pi, const gtsam::Pose3& pj, const prx::fg::screw_axis_t& Vij,
                      boost::optional<Jacobian&> H1 = boost::none, boost::optional<Jacobian&> H2 = boost::none,
                      boost::optional<Jacobian&> H3 = boost::none) const override
  {
    const prx::fg::screw_axis_t screw_dt{ Vij * _dt };
    // SE3_t pi_se3{};
    const gtsam::Pose3 predicted_pose{ pi * prx::fg::screw_axis_t::exp(screw_dt) };
    // const gtsam::Pose3 predicted_pose{ gtsam::Rot3(predicted_tr.rotation()), predicted_tr.translation() };
    DEBUG_VARS(pi);
    DEBUG_VARS(pj);
    DEBUG_VARS(Vij);
    DEBUG_VARS(predicted_pose);
    const gtsam::Pose3 error{ pj.between(predicted_pose, H2 ? &D_error_pj : 0, H1 || H3 ? &D_error_predict : 0) };
    DEBUG_VARS(error);

    const Eigen::Vector<double, 3> p_error{ error.translation() };
    const Eigen::Vector<double, 3> R_error{ gtsam::Rot3::Logmap(error.rotation()) };

    if (H1)
    {
      *H1 = D_error_predict * Eigen::Matrix<double, 6, 6>::Identity();
    }
    if (H2)
    {
      *H2 = D_error_pj;
    }
    if (H3)
    {
      *H3 = _dt * Eigen::Matrix<double, 6, 6>::Identity();
    }
    DEBUG_VARS(R_error.transpose(), p_error.transpose());
    return (Eigen::Vector<double, 6>() << R_error, p_error).finished();
  }

private:
  const double _dt;
  mutable Eigen::Matrix<double, 6, 6> D_predict_pi;
  mutable Eigen::Matrix<double, 6, 6> D_predict_Tij;
  mutable Eigen::Matrix<double, 6, 6> D_error_pj, D_error_predict;
};

inline gtsam::Pose3 pose_from_transform(const geometry_msgs::TransformStamped& tf)
{
  const geometry_msgs::Vector3& t{ tf.transform.translation };
  const geometry_msgs::Quaternion& q{ tf.transform.rotation };
  const gtsam::Rot3 rot(q.w, q.x, q.y, q.z);
  const Eigen::Vector3d vec(t.x, t.y, t.z);
  return gtsam::Pose3(rot, vec);
}

inline std::string marker_frame(const std::string& camera, const std::size_t& marker)
{
  return camera + "_marker_" + std::to_string(marker);
}

struct translation_noise
{
  translation_noise() : rd(), gen(rd()), dist(0.0, 1.0)
  {
  }
  Eigen::Vector3d operator()()
  {
    return Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
  }
  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<double> dist;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrTrajoptCtrl");
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  geometry_msgs::TransformStamped transform;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  auto tf_nm = gtsam::noiseModel::Isotropic::Sigma(6, 1e0);
  auto vel_prior_nm = gtsam::noiseModel::Isotropic::Sigma(6, 1e-5);
  auto pose_vel_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e0);

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam2(parameters);

  prx::simulation_step = simulation_step;
  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  prx::system_ptr_t plant = prx::system_factory_t::create_system(_plant_name, _plant_path);
  prx_assert(_plant != nullptr, "Failed to create plant");

  prx::world_model_t world_model({ _plant }, {});
  world_model.create_context("estimator_context", { _plant_name }, {});
  auto context = world_model.get_context("estimator_context");
  auto system_group = context.first;
  auto collision_group = context.second;
  prx::space_t* state_space = _system_group->get_state_space();
  prx::space_t* control_space = _system_group->get_control_space();
  prx::space_t* parameter_space = _system_group->get_parameter_space();

  prx::space_point_t start_state{ ss->make_point() };
  Vec(start_state) = Eigen::Vector4d::Zero();

  prx::plan_t plan{ control_space };
  prx::trajectory_t traj_gt{ state_space };

  plan.copy_onto_back(Eigen::Vector2d(0, 1), 10);
  system_group->propagate(start_state, plan, traj_gt);

  ros::Publisher trajectory_pub = private_nh.advertise<ml4kp_bridge::TrajectoryStamped>("/mushr/ml4kp_traj", 1);
  ros::Publisher estimated_traj_pub = private_nh.advertise<ml4kp_bridge::TrajectoryStamped>("/mushr/ctrl_traj", 1);

  ml4kp_bridge::TrajectoryStamped traj_msg_gt, estimated_traj_msg;
  ml4kp_bridge::copy(gt_traj_msg.trajectory, traj_gt);
  ml4kp_bridge::copy(estimated_traj_msg.trajectory, traj_gt);

  ros::Rate rate(1);
  while (ros::ok())
  {
    trajectory_pub.publish(traj_msg);
    estimated_traj_pub.publish(estimated_traj_msg);
  }
  return 0;
}