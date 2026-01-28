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
#include <interface/StampedMarkers.h>

#include <utils/rosparams_utils.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <estimation/camera_calibration_factors.hpp>
// #include "nodelets/plant_estimator.cpp"
// using SE3 = prx::fg::se3_t;
using SE3 = gtsam::Pose3;
using CameraCalibration = gtsam::Cal3DS2;
using Camera = gtsam::PinholeCamera<CameraCalibration>;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using SF = prx::fg::symbol_factory_t;

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
    res = SE3(gtsam::Rot3(qt), pt);

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
                    const std::vector<std::tuple<std::string, std::string, bool>>& frames, tf2_ros::Buffer& tf_buffer)
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
  const gtsam::Key key_cA_T_O{ SF::create_hashed_symbol("Camera_", cA_frame) };
  const gtsam::Key key_cB_T_O{ SF::create_hashed_symbol("Camera_", cB_frame) };
  PRINT_MSG("First");
  PRX_DBG_VARS(cA_frame, cB_frame)
  PRINT_KEYS(key_cA_T_O)
  PRINT_KEYS(key_cB_T_O)

  bool first{ true };
  int idx{ 0 };
  for (auto& tuple : frames)
  {
    const std::string cA_marker_frame{ std::get<0>(tuple) };
    const std::string cB_marker_frame{ std::get<1>(tuple) };
    const bool cB_marker_on_ground{ std::get<2>(tuple) };
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

        PRINT_KEYS(key_cA_T_O)
        PRINT_KEYS(key_cB_T_O)
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
      // graph.emplace_shared<compose_factor2_t>(key_mA_T_O, key_cA_T_O, mi_T_cA);
      // graph.emplace_shared<compose_factor2_t>(key_mA_T_O, key_cB_T_O, mi_T_cB);
      // if (cB_marker_on_ground)
      // graph.emplace_shared<OnGroundFactor>(key_mA_T_O);

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

  graph.print("Graph", SF::formatter);
  // PRX_DBG_VARS(T_initial);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  ROS_INFO_STREAM("Setting optimizer");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);

  gtsam::Values result{ optimizer.optimize() };
  cA_T_O = result.at<SE3>(key_cA_T_O);
  cB_T_O = result.at<SE3>(key_cB_T_O);

  result.print("Result", SF::formatter);
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
    calib = SE3(gtsam::Rot3(qt), pt);
    return true;
  }

  return false;
}

void calibration_to_file(const std::string filename, const SE3& calib)
{
  std::ofstream ofs(filename);

  ofs << "translation: \n";
  ofs << "  x: " << calib.translation()[0] << "\n";
  ofs << "  y: " << calib.translation()[1] << "\n";
  ofs << "  z: " << calib.translation()[2] << "\n";

  ofs << "quaternion: \n";
  ofs << "  w: " << calib.rotation().toQuaternion().w() << "\n";
  ofs << "  x: " << calib.rotation().toQuaternion().x() << "\n";
  ofs << "  y: " << calib.rotation().toQuaternion().y() << "\n";
  ofs << "  z: " << calib.rotation().toQuaternion().z() << "\n";
  ofs.close();
}

geometry_msgs::TransformStamped calib_to_tfmsg(const SE3& calib, const std::string c0_frame, const std::string c1_frame)
{
  geometry_msgs::TransformStamped static_tf;
  static_tf.header.stamp = ros::Time::now();
  static_tf.header.frame_id = c0_frame;
  static_tf.child_frame_id = c1_frame;
  static_tf.transform.translation.x = calib.translation()[0];
  static_tf.transform.translation.y = calib.translation()[1];
  static_tf.transform.translation.z = calib.translation()[2];

  const Eigen::Quaterniond q{ calib.rotation().toQuaternion() };

  static_tf.transform.rotation.x = q.x();
  static_tf.transform.rotation.y = q.y();
  static_tf.transform.rotation.z = q.z();
  static_tf.transform.rotation.w = q.w();
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

struct calibrator_t
{
  using This = calibrator_t;
  using MarkersMap = std::map<std::size_t, interface::Marker>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using ArucoMarkerFactor = estimation::aruco_marker_factor_t<Camera>;
  using OnGroundFactor = estimation::on_ground_factor_t;

  std::string _world_frame;
  std::vector<ros::Subscriber> _marker_subscribers;
  std::vector<ros::Publisher> _cam_info_pubs;

  std::map<std::size_t, MarkersMap> _markers;

  ros::Timer _fg_timer;

  int _wait_cycles;
  double _marker_size;

  int _origin_marker;
  int _height;

  std::vector<int> _markers_on_ground;
  std::vector<std::string> _cameras;
  gtsam::LevenbergMarquardtParams _lm_params;

  tf2_ros::StaticTransformBroadcaster _static_broadcaster;

  calibrator_t(ros::NodeHandle& nh) : _wait_cycles(5), _lm_params(prx::fg::default_levenberg_marquardt_parameters())
  {
    std::vector<std::string> aruco_topics;
    double& marker_size{ _marker_size };
    int& origin_marker{ _origin_marker };
    int& height{ _height };

    std::string& world_frame{ _world_frame };

    std::vector<int>& markers_on_ground{ _markers_on_ground };
    std::vector<std::string>& cameras{ _cameras };

    PARAM_SETUP(nh, world_frame);
    PARAM_SETUP(nh, aruco_topics);
    PARAM_SETUP(nh, marker_size);
    PARAM_SETUP(nh, origin_marker);
    PARAM_SETUP(nh, height);
    PARAM_SETUP(nh, markers_on_ground);
    PARAM_SETUP(nh, cameras);

    DEBUG_VARS(marker_size);
    DEBUG_VARS(aruco_topics);

    // _lm_params.setMaxIterations(1);
    _lm_params.setMaxIterations(100);

    // for (auto& topic_name : aruco_topics)
    for (int i = 0; i < aruco_topics.size(); ++i)
    {
      _marker_subscribers.push_back(nh.subscribe<interface::StampedMarkers>(
          aruco_topics[i], 1, boost::bind(&This::marker_callback, this, _1, i)));
    }

    for (auto cam : cameras)
    {
      const std::string camera_name_topic{ cam + "/info" };
      _cam_info_pubs.push_back(nh.advertise<sensor_msgs::CameraInfo>(camera_name_topic, 1, true));
    }

    const ros::Duration freq_timer(1.0);
    _fg_timer = nh.createTimer(freq_timer, &This::timer_function, this);
  }

  void marker_callback(const interface::StampedMarkersConstPtr msg, const std::size_t idx)
  {
    // _markers[idx] = *msg;
    for (int i = 0; i < msg->markers.size(); ++i)
    {
      const interface::Marker& marker{ msg->markers[i] };
      _markers[idx][marker.id] = marker;
    }
    // DEBUG_VARS(*msg);
  }

  void timer_function(const ros::TimerEvent& event)
  {
    if (_wait_cycles == 0)
    {
      // run_calibration();
      publish_calibration();
    }
    _wait_cycles--;
  }

  void publish_calibration()
  {
    // PARAM_SETUP(nh, cameras);

    // for (auto cam : _cameras)
    for (int ci = 0; ci < _cameras.size(); ++ci)
    {
      const std::string& camera_name{ _cameras[ci] };
      ros::NodeHandle nh("~/" + camera_name);
      // std::vector<std::string> keys;
      // nh.getParamNames(keys);
      // DEBUG_VARS(keys);

      std::vector<double> K;
      std::vector<double> distortion;
      double height;
      double width;

      PARAM_SETUP(nh, K)
      PARAM_SETUP(nh, distortion)
      PARAM_SETUP(nh, height)
      PARAM_SETUP(nh, width)

      Eigen::Matrix3d cam_K;  //{ K.data() };
      Eigen::Vector<double, 5> cam_distortion{ distortion.data() };
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          cam_K(i, j) = K[i * 3 + j];
        }
      }
      // DEBUG_VARS(cam_distortion);
      sensor_msgs::CameraInfo msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = camera_name;

      msg.width = width;
      msg.height = height;
      msg.distortion_model = "plumb_bob";

      msg.D = distortion;
      std::copy(K.begin(), K.end(), msg.K.begin());
      // msg.K = K;
      msg.R = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
      std::copy(K.begin(), K.begin() + 3, msg.P.begin());
      std::copy(K.begin() + 3, K.begin() + 6, msg.P.begin() + 4);
      std::copy(K.begin() + 6, K.begin() + 9, msg.P.begin() + 8);

      _cam_info_pubs[ci].publish(msg);
      // msg.P = {};

      const double& fx{ K[0] };
      const double& fy{ K[4] };
      const double& s{ K[1] };
      const double& u0{ K[2] };
      const double& v0{ K[5] };
      const double& k1{ distortion[0] };
      const double& k2{ distortion[1] };
      const double& p1{ distortion[2] };
      const double& p2{ distortion[3] };

      gtsam::Pose3 pose(gtsam::Rot3(0, 0, -1, 0), Eigen::Vector3d(1.0, 0.0, 2.0));
      CameraCalibration calibration(fx, fy, s, u0, v0, k1, k2, p1, p2);
      gtsam::PinholeCamera camera(pose, calibration);

      publish_camera_tf(camera_name, camera);
    }
  }

  void publish_camera_tf(const std::string camera_frame, const Camera& camera)
  {
    geometry_msgs::TransformStamped static_transform_stamped;
    static_transform_stamped.header.frame_id = _world_frame;
    static_transform_stamped.child_frame_id = camera_frame;

    static_transform_stamped.header.stamp = ros::Time::now();

    const gtsam::Pose3& pose{ camera.pose() };
    static_transform_stamped.transform.translation.x = pose.x();
    static_transform_stamped.transform.translation.y = pose.y();
    static_transform_stamped.transform.translation.z = pose.z();

    // quat = tf.transformations.quaternion_from_matrix(mat)
    const gtsam::Quaternion quat{ pose.rotation().toQuaternion() };
    static_transform_stamped.transform.rotation.w = quat.w();
    static_transform_stamped.transform.rotation.x = quat.x();
    static_transform_stamped.transform.rotation.y = quat.y();
    static_transform_stamped.transform.rotation.z = quat.z();

    _static_broadcaster.sendTransform(static_transform_stamped);
  }

  void run_calibration()
  {
    gtsam::Values initial_values;
    gtsam::NonlinearFactorGraph graph;

    // gtsam::Pose3 pose(gtsam::Rot3(0, 0, -1, 0), Eigen::Vector3d(0.115589, 0.0677932, 1.81156));
    gtsam::Pose3 pose(gtsam::Rot3(0, 0, -1, 0), Eigen::Vector3d(1.0, 0.0, 2.0));

    // CameraCalibration calibration(3699.77, 1892.07, -2.95666, 1418.38, 51.6949, 53.2221187, -1416.32391, -3.2468501,
    //                               -2.03022483);

    // CameraCalibration calibration;
    CameraCalibration calibration(1.06662602e+03, 1.06702175e+03, 0.0, 9.34438762e+02, 5.58367833e+02,  // no-lint
                                  0.15133433, -0.3597004, -0.00049652, -0.00261917);
    // CameraCalibration calibration(1.06662602e+03, 1.06702175e+03, 1.0, 9.34438762e+02, 5.58367833e+02,  // no-lint
    //                               0.15133433, -0.3597004, -0.00049652, -0.00261917);
    // 0.15133433, -0.3597004, -0.00049652, -0.00261917, 0.17594971
    gtsam::PinholeCamera camera(pose, calibration);

    // int i{ 0 };
    const gtsam::Key origin_marker{ SF::create_hashed_symbol("marker_{", _origin_marker, "}") };
    for (int i = 0; i < _markers.size(); ++i)
    {
      const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", i) };

      initial_values.insert(key_cam_i, camera);

      NoiseModel aruco_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };  // in pixels

      // const interface::Marker marker{ _markers[0].front() };
      for (auto& marker_pair : _markers[i])
      {
        const interface::Marker& marker{ marker_pair.second };

        // if (marker.id != 121)
        // if (marker.id != 121 and marker.id != 1 and marker.id != 80)
        if (marker.id != 121 and marker.id != 1)
        {
          continue;
        }
        const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker.id, "}") };

        initial_values.insert_or_assign(key_marker_i, gtsam::Pose3());

        for (int j = 0; j < 4; ++j)
        {
          // const Eigen::Vector2d meassurement(marker.corners[j].x, _height - marker.corners[j].y);
          const Eigen::Vector2d meassurement(marker.corners[j].x, marker.corners[j].y);
          graph.emplace_shared<ArucoMarkerFactor>(key_cam_i, key_marker_i, meassurement, _marker_size, j, aruco_nm);
        }
      }

      NoiseModel ground_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };  // in cm
      for (auto& marker_id : _markers_on_ground)
      {
        const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker_id, "}") };
        graph.emplace_shared<OnGroundFactor>(origin_marker, key_marker_i, ground_nm);
      }
    }
    NoiseModel origin_nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-4) };
    graph.addPrior(origin_marker, gtsam::Pose3(), origin_nm);

    // graph.print("Graph", SF::formatter);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, _lm_params);
    gtsam::Values result{ optimizer.optimize() };

    result.print("Result", SF::formatter);

    graph.printErrors(result, "Graph", SF::formatter);

    project_result(result);
  }

  void project_result(const gtsam::Values values)
  {
    int i{ 0 };
    const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", i) };

    const Camera cam{ values.at<Camera>(key_cam_i) };

    for (auto& marker_pair : _markers[i])
    {
      const interface::Marker& marker{ marker_pair.second };
      const int marker_id{ marker.id };
      const gtsam::Key marker_key{ SF::create_hashed_symbol("marker_{", marker_id, "}") };

      // if (marker.id != 121 and marker.id != 1 and marker.id != 80)
      if (marker.id != 121 and marker.id != 1)
      {
        continue;
      }
      const gtsam::Pose3 marker_pose{ values.at<gtsam::Pose3>(marker_key) };

      const Eigen::Vector3d offset{ Eigen::Vector3d::Zero() };
      const Eigen::Vector2d pixel{ ArucoMarkerFactor::predict(cam, marker_pose, offset) };
      DEBUG_VARS(marker_id, pixel[0], pixel[1])
    }
  }
};

// std::messages

// Calibrate the world T=[R|t] of the world.
// This is, find the T that transforms C0 to C1, which can be used by ros::Tf to transform between frames
int main(int argc, char** argv)
{
  ros::init(argc, argv, "world_calibration");
  ros::NodeHandle nh("~");

  calibrator_t calib(nh);

  ros::spin();

  // tf2_ros::Buffer tf_buffer;
  // tf2_ros::TransformListener tf_listener(tf_buffer);

  // XmlRpc::XmlRpcValue calibration_frames;
  // std::string calibration_file;
  // std::vector<std::string> camera_frames;
  // bool load_previous_calibration;

  // ROS_PARAM_SETUP(nh, calibration_frames);
  // ROS_PARAM_SETUP(nh, camera_frames);
  // ROS_PARAM_SETUP(nh, calibration_file);
  // ROS_PARAM_SETUP(nh, load_previous_calibration);

  // std::vector<std::tuple<std::string, std::string, bool>> frames;

  // for (int i = 0; i < calibration_frames.size(); ++i)
  // {
  //   auto topic_i = calibration_frames[i];
  //   const std::string c0_marker_frame{ std::string(topic_i["c0_marker_frame"]) };
  //   const std::string c1_marker_frame{ std::string(topic_i["c1_marker_frame"]) };
  //   const bool c1_marker_on_ground{ static_cast<bool>(topic_i["c1_marker_on_gound"]) };
  //   DEBUG_VARS(c0_marker_frame, c1_marker_frame, c1_marker_on_ground);
  //   frames.push_back({ c0_marker_frame, c1_marker_frame, c1_marker_on_ground });
  // }

  // SE3 calib;
  // geometry_msgs::TransformStamped static_transform_stamped{};

  // if (load_previous_calibration and load_prev_calibration(calibration_file, calib))
  // {
  //   const std::string msg{ "[Calibration] Loading previous calibration" };
  //   PRX_DBG_VARS(msg);
  //   static_transform_stamped = calib_to_tfmsg(calib, camera_frames[0], camera_frames[1]);
  // }
  // else
  // {
  //   ros::Rate r(1);
  //   const std::string msg{ "[Calibration] Waiting 10 seconds for tf data... " };
  //   PRX_DBG_VARS(msg);

  //   for (int i = 0; i < 20; ++i)
  //   {
  //     ros::spinOnce();
  //     r.sleep();
  //   }
  //   calib = run_calibration(camera_frames, frames, tf_buffer);
  //   static_transform_stamped = calib_to_tfmsg(calib, camera_frames[0], camera_frames[1]);
  //   calibration_to_file(calibration_file, calib);
  // }

  // publish_calibration(static_transform_stamped);

  // ros::spin();

  return 0;
}