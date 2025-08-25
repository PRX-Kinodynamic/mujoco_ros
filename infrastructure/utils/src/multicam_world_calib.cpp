#include <filesystem>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>

#include <utils/rosparams_utils.hpp>
// #include "nodelets/plant_estimator.cpp"
using SE3 = prx::fg::se3_t;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using SF = prx::fg::symbol_factory_t;

// a_T_b * b_T_c = a_T_c;
// a_T_b ~ NOT Known
// b_T_c ~ Known
// a_T_c ~ Known
class compose_factor_t : public gtsam::NoiseModelFactor1<SE3>
{
  using Base = gtsam::NoiseModelFactor1<SE3>;

public:
  compose_factor_t(const gtsam::Key& aTbKey, const SE3 bTc, const SE3 aTc, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, aTbKey), _bTc(bTc), _aTc(aTc)
  {
  }

  virtual Eigen::VectorXd evaluateError(const SE3& aTb,
                                        boost::optional<Eigen::MatrixXd&> H = boost::none) const override
  {
    Eigen::Matrix<double, 6, 6> atcp_H_atb, err_H_atcp;
    const SE3 aTc_pred{ aTb.compose(_bTc, atcp_H_atb) };
    const Eigen::VectorXd error{ aTc_pred.logmap(_aTc, err_H_atcp) };

    if (H)
    {
      // ROS_INFO_STREAM("Computing H");
      *H = err_H_atcp * atcp_H_atb;
      // PRX_DBG_VARS(*H);
    }

    return error;
  }

  const SE3 _bTc;
  const SE3 _aTc;
};

// Mi_Z_cj     = Mi_T_O * O_T_cj;
// Observation =  SE3   *   SE3
// O = origin frame
// However, all transforms are on camera frame.
class compose_factor2_t : public gtsam::NoiseModelFactor1<SE3, SE3>
{
  using Base = gtsam::NoiseModelFactor1<SE3, SE3>;

public:
  compose_factor2_t(const gtsam::Key key_mi_T_O, const gtsam::Key key_cj_T_O, const SE3 Mi_Z_cj,
                    const NoiseModel& cost_model = nullptr)
    : Base(cost_model, key_mi_T_O, key_cj_T_O), _Mi_Z_cj(Mi_Z_cj)
  {
  }

  virtual Eigen::VectorXd evaluateError(const SE3& mi_T_O, const SE3& cj_T_O,
                                        boost::optional<Eigen::MatrixXd&> HmiO = boost::none,
                                        boost::optional<Eigen::MatrixXd&> HcjO = boost::none) const override
  {
    Eigen::MatrixXd Ocj_H_cjO;
    Eigen::MatrixXd pred_H_miO, pred_H_Ocj;
    Eigen::MatrixXd err_H_pred;
    const SE3 O_T_cj{ cj_T_O.inverse(Ocj_H_cjO) };
    const SE3 pred_Mi_T_cj{ mi_T_O.compose(O_T_cj, pred_H_miO, pred_H_Ocj) };

    // Eigen::Matrix<double, 6, 6> atcp_H_atb, err_H_atcp;
    // const SE3 aTc_pred{ aTb.compose(_bTc, atcp_H_atb) };
    const Eigen::VectorXd error{ pred_Mi_T_cj.logmap(_Mi_Z_cj, err_H_pred) };

    if (HmiO)
    {
      // ROS_INFO_STREAM("Computing H");
      *HmiO = err_H_pred * pred_H_miO;
    }
    if (HcjO)
    {
      *HcjO = err_H_pred * pred_H_Ocj * Ocj_H_cjO;
    }

    return error;
  }

  const SE3 _Mi_Z_cj;
};

bool get_observation(const std::string camera_frame, const std::string marker_frame, SE3& res,
                     tf2_ros::Buffer& tf_buffer)
{
  try
  {
    // geometry_msgs::TransformStamped tf_msg{ tf_buffer.lookupTransform(camera_frame, marker_frame, ros::Time(0)) };
    geometry_msgs::TransformStamped tf_msg{ tf_buffer.lookupTransform(marker_frame, camera_frame, ros::Time(0)) };
    // _tf_out.transform.translation = _tf_in_1.transform.translation;
    // _quat = to_quat(_tf_in_1);
    const geometry_msgs::Vector3& t{ tf_msg.transform.translation };
    const geometry_msgs::Quaternion& q{ tf_msg.transform.rotation };

    const Eigen::Vector3d pt{ t.x, t.y, t.z };
    const Eigen::Quaterniond qt{ q.w, q.x, q.y, q.z };
    res = SE3(qt, pt);

    ROS_INFO_STREAM(camera_frame << " " << marker_frame << " " << res);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_INFO_STREAM("TF between frame " << camera_frame << " and " << marker_frame << " Not found");
    return false;
  }
  return false;
}

// Assuming only two cameras
SE3 run_calibration(const std::vector<std::string> cameras,
                    const std::vector<std::pair<std::string, std::string>>& frames, tf2_ros::Buffer& tf_buffer)
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  // const gtsam::Key c0_T_c1{ gtsam::Symbol('T', 0) };

  prx_assert(cameras.size() == 2, "Only two camers supported");
  const std::string cA_frame{ cameras[0] };
  const std::string cB_frame{ cameras[1] };

  SE3 T_initial{};

  SE3 mi_T_cA;
  SE3 mi_T_cB;
  SE3 cA_T_O;
  SE3 cB_T_O;

  PRX_DBG_VARS(cA_frame, cB_frame)
  const gtsam::Key key_cA_T_O{ SF::create_hashed_symbol("Camera", cA_frame) };
  const gtsam::Key key_cB_T_O{ SF::create_hashed_symbol("Camera_", cB_frame) };
  PRINT_MSG("First");
  PRX_DBG_VARS(cA_frame, cB_frame)
  PRINT_KEYS(key_cA_T_O, key_cB_T_O)

  bool first{ true };
  int idx{ 0 };
  for (auto& pair : frames)
  {
    const std::string cA_marker_frame{ pair.first };
    const std::string cB_marker_frame{ pair.second };
    PRX_DBG_VARS(cA_marker_frame, cB_marker_frame);

    const bool ca_valid{ get_observation(cA_frame, cA_marker_frame, mi_T_cA, tf_buffer) };
    const bool cb_valid{ get_observation(cB_frame, cB_marker_frame, mi_T_cB, tf_buffer) };

    if (ca_valid and cb_valid)
    {
      if (first)
      {
        PRINT_MSG("First");
        // fix the origin to be the first Marker
        cA_T_O = mi_T_cA.inverse();
        cB_T_O = mi_T_cB.inverse();

        PRINT_KEYS(key_cA_T_O, key_cB_T_O)
        initial_values.insert(key_cA_T_O, cA_T_O);
        initial_values.insert(key_cB_T_O, cB_T_O);
        first = false;
      }

      const gtsam::Key key_mA_T_O{ SF::create_hashed_symbol("M_", idx) };
      PRINT_KEYS(key_mA_T_O)
      // const gtsam::Key key_mB_T_O{ SF::create_hashed_symbol(cB_marker_frame) };
      // PRX_DBG_VARS(cA_frame, mi_T_cA)
      // PRX_DBG_VARS(cB_frame, mi_T_cB)
      // compose_factor2_t(const gtsam::Key key_mi_T_O, const gtsam::Key key_cj_T_O, const SE3 Mi_Z_cj,
      //             const NoiseModel& cost_model = nullptr);
      graph.emplace_shared<compose_factor2_t>(key_mA_T_O, key_cA_T_O, mi_T_cA);
      graph.emplace_shared<compose_factor2_t>(key_mA_T_O, key_cB_T_O, mi_T_cB);

      PRINT_MSG("Adding to values")
      initial_values.insert(key_mA_T_O, mi_T_cA * cA_T_O);
      idx++;
      // initial_values.insert(key_mB_T_O, mi_T_cB * cB_T_O);
    }
    else
    {
      PRX_DBG_VARS(ca_valid, cA_frame, cA_marker_frame);
      PRX_DBG_VARS(cb_valid, cB_frame, cB_marker_frame);
    }
    PRINT_MSG("Next")
  }
  // initial_values.insert(c0_T_c1, T_initial);

  // PRX_DBG_VARS(T_initial);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  ROS_INFO_STREAM("Setting optimizer");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);

  gtsam::Values result{ optimizer.optimize() };
  cA_T_O = result.at<SE3>(key_cA_T_O);
  cB_T_O = result.at<SE3>(key_cB_T_O);

  const SE3 cA_T_cB{ cA_T_O * cB_T_O.inverse() };
  PRX_DBG_VARS(cA_T_cB);

  return cA_T_cB;
}

bool load_prev_calibration(const std::string& calib_file, SE3& calib)
{
  if (std::filesystem::exists(calib_file))
  {
    prx::param_loader params(calib_file);

    params.print();

    const double tx{ params["/translation/x"].as<double>() };
    const double ty{ params["/translation/y"].as<double>() };
    const double tz{ params["/translation/z"].as<double>() };

    const double qw{ params["/quaternion/w"].as<double>() };
    const double qx{ params["/quaternion/x"].as<double>() };
    const double qy{ params["/quaternion/y"].as<double>() };
    const double qz{ params["/quaternion/z"].as<double>() };

    const Eigen::Vector3d pt{ tx, ty, tz };
    const Eigen::Quaterniond qt{ qw, qx, qy, qz };
    calib = SE3(qt, pt);
    return true;
  }

  return false;
}

void calibration_to_file(const std::string filename, const SE3& calib)
{
  std::ofstream ofs(filename);

  ofs << "translation: \n";
  ofs << "  x: " << calib.position()[0] << "\n";
  ofs << "  y: " << calib.position()[1] << "\n";
  ofs << "  z: " << calib.position()[2] << "\n";

  ofs << "quaternion: \n";
  ofs << "  w: " << calib.quaternion().w() << "\n";
  ofs << "  x: " << calib.quaternion().x() << "\n";
  ofs << "  y: " << calib.quaternion().y() << "\n";
  ofs << "  z: " << calib.quaternion().z() << "\n";
  ofs.close();
}

geometry_msgs::TransformStamped calib_to_tfmsg(const SE3& calib, const std::string c0_frame, const std::string c1_frame)
{
  geometry_msgs::TransformStamped static_tf;
  static_tf.header.stamp = ros::Time::now();
  static_tf.header.frame_id = c0_frame;
  static_tf.child_frame_id = c1_frame;
  static_tf.transform.translation.x = calib.position()[0];
  static_tf.transform.translation.y = calib.position()[1];
  static_tf.transform.translation.z = calib.position()[2];
  static_tf.transform.rotation.x = calib.quaternion().x();
  static_tf.transform.rotation.y = calib.quaternion().y();
  static_tf.transform.rotation.z = calib.quaternion().z();
  static_tf.transform.rotation.w = calib.quaternion().w();
  return static_tf;
}

void publish_calibration(geometry_msgs::TransformStamped& static_transform_stamped)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  const std::string c0_frame{ static_transform_stamped.header.frame_id };
  const std::string c1_frame{ static_transform_stamped.child_frame_id };

  static_broadcaster.sendTransform(static_transform_stamped);
  PRX_DBG_VARS(static_transform_stamped);
  ROS_INFO_STREAM("Calibration " << c0_frame << " " << c1_frame << " published.");
}

// std::messages

// Calibrate the world T=[R|t] of the world.
// This is, find the T that transforms C0 to C1, which can be used by ros::Tf to transform between frames
int main(int argc, char** argv)
{
  ros::init(argc, argv, "world_calibration");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  XmlRpc::XmlRpcValue calibration_frames;
  std::string calibration_file;
  std::vector<std::string> camera_frames;
  bool load_previous_calibration;

  ROS_PARAM_SETUP(nh, calibration_frames);
  ROS_PARAM_SETUP(nh, camera_frames);
  ROS_PARAM_SETUP(nh, calibration_file);
  ROS_PARAM_SETUP(nh, load_previous_calibration);

  std::vector<std::pair<std::string, std::string>> frames;

  for (int i = 0; i < calibration_frames.size(); ++i)
  {
    auto topic_i = calibration_frames[i];
    const std::string c0_marker_frame{ std::string(topic_i["c0_marker_frame"]) };
    const std::string c1_marker_frame{ std::string(topic_i["c1_marker_frame"]) };
    frames.push_back({ c0_marker_frame, c1_marker_frame });
  }

  SE3 calib;
  geometry_msgs::TransformStamped static_transform_stamped{};

  if (load_previous_calibration and load_prev_calibration(calibration_file, calib))
  {
    const std::string msg{ "[Calibration] Loading previous calibration" };
    PRX_DBG_VARS(msg);
    static_transform_stamped = calib_to_tfmsg(calib, camera_frames[0], camera_frames[1]);
  }
  else
  {
    ros::Rate r(1);
    const std::string msg{ "[Calibration] Waiting 10 seconds for tf data... " };
    PRX_DBG_VARS(msg);

    for (int i = 0; i < 20; ++i)
    {
      ros::spinOnce();
      r.sleep();
    }
    calib = run_calibration(camera_frames, frames, tf_buffer);
    static_transform_stamped = calib_to_tfmsg(calib, camera_frames[0], camera_frames[1]);
    calibration_to_file(calibration_file, calib);
  }

  publish_calibration(static_transform_stamped);

  ros::spin();

  return 0;
}