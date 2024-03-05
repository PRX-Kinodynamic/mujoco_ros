#include <thread>
#include <random>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <utils/dbg_utils.hpp>
#include <gtsam/nonlinear/ISAM2.h>

#include <ml4kp_bridge/defs.h>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/factor_graphs/utilities/common_functions.hpp>
#include <prx/factor_graphs/factors/screw_axis.hpp>
#include <prx/factor_graphs/factors/mushr_factors.hpp>
#include <prx/factor_graphs/factors/mushr_types.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <prx_models/mj_mushr.hpp>
#include <tf2_ros/transform_listener.h>

#include <utils/rosparams_utils.hpp>

// using prx::utilities::convert_to;
using SF = prx::fg::symbol_factory_t;

auto k_X = [](const std::size_t& ti) { return SF::create_hashed_symbol("x_{", ti, "}"); };
auto k_U = [](const std::size_t& ti) { return SF::create_hashed_symbol("u_{", ti, "}"); };
auto k_Z = [](const std::size_t& ti) { return SF::create_hashed_symbol("z_{", ti, "}"); };
auto k_P = [](const std::size_t& ti) { return SF::create_hashed_symbol("theta_{", ti, "}"); };

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

bool get_latest_transform(const std::string& target, const std::string& source, tf2_ros::Buffer& buffer,
                          geometry_msgs::TransformStamped& transform)
{
  try
  {
    transform = buffer.lookupTransform(target, source, ros::Time(0));
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  return false;
}

void get_new_trajectory(std::shared_ptr<prx::system_group_t> sg, const prx::space_point_t start_state,
                        const prx::plan_t& plan, prx::trajectory_t& trajectory,
                        ml4kp_bridge::TrajectoryStamped& ros_traj)
{
  sg->propagate(start_state, plan, trajectory);
  ml4kp_bridge::copy(ros_traj.trajectory, trajectory);
}

void init_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, prx::trajectory_t& traj, prx::plan_t plan,
             const unsigned traj_freq, const unsigned ctrl_freq)
{
  prx_assert(traj_freq >= ctrl_freq, "Trajectory frequency must be greater than control frequency.");

  const unsigned step{ static_cast<unsigned int>(traj_freq / ctrl_freq) };
  const double dt{ 1.0 / static_cast<double>(ctrl_freq) };
  prx::fg::mushrTypes::Parameters params{};
  prx::fg::mushrTypes::vel_delta_max(params) = 1.0;
  prx::fg::mushrTypes::vel_delta_min(params) = -1.0;
  prx::fg::mushrTypes::steering_gain(params) = 1.0;
  prx::fg::mushrTypes::steering_offset(params) = 0.10;
  prx::fg::mushrTypes::velocity_min(params) = -10.0;
  prx::fg::mushrTypes::velocity_max(params) = 10.0;

  values.insert(k_P(0), params);
  graph.addPrior(k_P(0), params);
  auto mushr_prop_nm = gtsam::noiseModel::Isotropic::Sigma(4, 1e-1);
  auto priors = gtsam::noiseModel::Isotropic::Sigma(4, 1e0);

  std::size_t j{ 0 };
  DEBUG_VARS(traj.size());

  std::size_t curr_ctrl{ 0 };
  Eigen::Vector2d ctrl{ Vec(plan[curr_ctrl].control) };
  double ctrl_dur{ plan[curr_ctrl].duration };
  for (unsigned i = 0; i < traj.size() - step; i += step)
  {
    graph.emplace_shared<prx::fg::mushr_factor_t>(k_X(j), k_X(j + 1), k_U(j), k_P(0), dt, mushr_prop_nm);
    const Eigen::Vector4d state{ Vec(traj[i]) };
    // graph.addPrior(k_X(j), state, priors);
    values.insert(k_X(j), state);

    // DEBUG_VARS(i, dt, j, j * dt, ctrl_dur, step);

    if (j * dt > ctrl_dur)
    {
      ctrl_dur = plan[curr_ctrl].duration;
    }
    values.insert(k_U(j), ctrl);
    j++;
  }
  const Eigen::Vector4d last_state{ traj.back()->as<Eigen::Vector4d>() };
  values.insert(k_X(j), last_state);
  // graph.addPrior(k_X(j), last_state, gtsam::noiseModel::Isotropic::Sigma(4, 1e-4));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "MushrTrajoptCtrl");
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };

  std::string world_frame{};
  std::string robot_frame{};

  ROS_PARAM_SETUP(nh, world_frame);
  ROS_PARAM_SETUP(nh, robot_frame);

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

  prx::simulation_step = 0.01;
  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant != nullptr, "Failed to create plant");

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("estimator_context", { plant_name }, {});
  auto context = world_model.get_context("estimator_context");
  auto system_group = context.first;
  auto collision_group = context.second;
  prx::space_t* state_space = system_group->get_state_space();
  prx::space_t* control_space = system_group->get_control_space();
  prx::space_t* parameter_space = system_group->get_parameter_space();

  prx::space_point_t start_state{ state_space->make_point() };

  prx::plan_t plan{ control_space };
  prx::trajectory_t traj_gt{ state_space };

  // plan.copy_onto_back(Eigen::Vector2d(0, 1), 0.1);
  plan.copy_onto_back(Eigen::Vector2d(0, 1), 1);
  plan.copy_onto_back(Eigen::Vector2d(0.5, 1), 1);
  plan.copy_onto_back(Eigen::Vector2d(-0.5, 1), 1);
  // system_group->propagate(start_state, plan, traj_gt);

  ros::Publisher trajectory_pub{ nh.advertise<ml4kp_bridge::TrajectoryStamped>("/mushr/ml4kp_traj", 1) };
  ros::Publisher estimated_traj_pub{ nh.advertise<ml4kp_bridge::TrajectoryStamped>("/mushr/ctrl_traj", 1) };

  ml4kp_bridge::TrajectoryStamped gt_traj_msg, estimated_traj_msg;
  // ml4kp_bridge::copy(gt_traj_msg.trajectory, traj_gt);
  // ml4kp_bridge::copy(estimated_traj_msg.trajectory, traj_gt);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped robot_transform;

  ros::Rate rate(1);
  bool location_received{ false };
  while (!location_received)
  {
    location_received = get_latest_transform(world_frame, robot_frame, tf_buffer, robot_transform);
    rate.sleep();
  }
  prx_models::copy(*start_state, robot_transform);
  get_new_trajectory(system_group, start_state, plan, traj_gt, gt_traj_msg);
  trajectory_pub.publish(gt_traj_msg);

  init_fg(graph, initial_values, traj_gt, plan, 100, 30);

  // graph.print("FG", prx::fg::symbol_factory_t::formatter);
  // prx::fg::symbol_factory_t::symbols_to_file();

  // graph.orderingCOLAMD().print("Ordering: ", prx::fg::symbol_factory_t::formatter);
  // DEBUG_VARS(graph.linearize(initial_values)->jacobian().first);
  // DEBUG_VARS(graph.linearize(initial_values)->augmentedHessian());

  // isam2.update(graph, initial_values);
  // auto result = isam2.calculateEstimate();
  //
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values results = prx::fg::optimize_and_log(optimizer, lm_params);

  // results.print("Results ", prx::fg::symbol_factory_t::formatter);
  // while (ros::ok())
  // {
  //   // estimated_traj_pub.publish(estimated_traj_msg);
  //   rate.sleep();
  // }
  return 0;
}