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

struct calibrator_t
{
  using This = calibrator_t;

  using SE3 = gtsam::Pose3;
  using CameraCalibration = gtsam::Cal3DS2;
  using Camera = gtsam::PinholePose<CameraCalibration>;  // Camera with a *fix* calibration
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;
  using MarkersMap = std::map<std::size_t, interface::Marker>;
  using ArucoMarkerFactor = estimation::aruco_marker_factor_t;
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

  std::vector<Camera> _cameras;
  std::vector<int> _markers_on_ground;
  std::vector<std::string> _camera_frames;
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
    std::vector<std::string>& camera_frames{ _camera_frames };

    PARAM_SETUP(nh, world_frame);
    PARAM_SETUP(nh, aruco_topics);
    PARAM_SETUP(nh, marker_size);
    PARAM_SETUP(nh, origin_marker);
    PARAM_SETUP(nh, height);
    PARAM_SETUP(nh, markers_on_ground);
    PARAM_SETUP(nh, camera_frames);

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

    for (auto cam : _camera_frames)
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
      const std::string& camera_name{ _camera_frames[ci] };
      ros::NodeHandle nh("~/" + camera_name);
      ros::NodeHandle nh_pose("~/" + camera_name + "/pose");
      // std::vector<std::string> keys;
      // nh.getParamNames(keys);
      // DEBUG_VARS(keys);

      double height, width;
      std::vector<double> K, distortion;
      std::vector<double> quaternion, position;

      PARAM_SETUP(nh, K)
      PARAM_SETUP(nh, distortion)
      PARAM_SETUP(nh, height)
      PARAM_SETUP(nh, width)
      PARAM_SETUP(nh_pose, quaternion)
      PARAM_SETUP(nh_pose, position)

      // Eigen::Matrix3d cam_K;  //{ K.data() };
      // const Eigen::Vector<double, 5> cam_distortion{ distortion.data() };

      // for (int i = 0; i < 3; ++i)
      // {
      //   for (int j = 0; j < 3; ++j)
      //   {
      //     cam_K(i, j) = K[i * 3 + j];
      //   }
      // }
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

      const Eigen::Vector<double, 3> cam_position{ position.data() };
      const gtsam::Rot3 cam_quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

      const gtsam::Pose3 pose(cam_quat, cam_position);
      // gtsam::Pose3 pose(gtsam::Rot3(0, 0, -1, 0), Eigen::Vector3d(0.0, 0.0, 2.0));

      const CameraCalibration calibration(fx, fy, s, u0, v0, k1, k2, p1, p2);
      _cameras.emplace_back(pose, calibration);
      // gtsam::PinholeCamera camera(pose, calibration);
    }
    publish_cameras_tf();
  }

  void publish_cameras_tf()  //
  {
    for (int i = 0; i < _camera_frames.size(); ++i)
    {
      const std::string& camera_frame{ _camera_frames[i] };
      const Camera& camera{ _cameras[i] };

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
  }

  void run_calibration()
  {
    // gtsam::Values initial_values;
    // gtsam::NonlinearFactorGraph graph;

    // // gtsam::Pose3 pose(gtsam::Rot3(0, 0, -1, 0), Eigen::Vector3d(0.115589, 0.0677932, 1.81156));
    // gtsam::Pose3 pose(gtsam::Rot3(0, 0, -1, 0), Eigen::Vector3d(1.0, 0.0, 2.0));

    // // CameraCalibration calibration(3699.77, 1892.07, -2.95666, 1418.38, 51.6949, 53.2221187, -1416.32391,
    // -3.2468501,
    // //                               -2.03022483);

    // // CameraCalibration calibration;
    // CameraCalibration calibration(1.06662602e+03, 1.06702175e+03, 0.0, 9.34438762e+02, 5.58367833e+02,  // no-lint
    //                               0.15133433, -0.3597004, -0.00049652, -0.00261917);
    // // CameraCalibration calibration(1.06662602e+03, 1.06702175e+03, 1.0, 9.34438762e+02, 5.58367833e+02,  // no-lint
    // //                               0.15133433, -0.3597004, -0.00049652, -0.00261917);
    // // 0.15133433, -0.3597004, -0.00049652, -0.00261917, 0.17594971
    // gtsam::PinholeCamera camera(pose, calibration);

    // // int i{ 0 };
    // const gtsam::Key origin_marker{ SF::create_hashed_symbol("marker_{", _origin_marker, "}") };
    // for (int i = 0; i < _markers.size(); ++i)
    // {
    //   const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", i) };

    //   initial_values.insert(key_cam_i, camera);

    //   NoiseModel aruco_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };  // in pixels

    //   // const interface::Marker marker{ _markers[0].front() };
    //   for (auto& marker_pair : _markers[i])
    //   {
    //     const interface::Marker& marker{ marker_pair.second };

    //     // if (marker.id != 121)
    //     // if (marker.id != 121 and marker.id != 1 and marker.id != 80)
    //     if (marker.id != 121 and marker.id != 1)
    //     {
    //       continue;
    //     }
    //     const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker.id, "}") };

    //     initial_values.insert_or_assign(key_marker_i, gtsam::Pose3());

    //     for (int j = 0; j < 4; ++j)
    //     {
    //       // const Eigen::Vector2d meassurement(marker.corners[j].x, _height - marker.corners[j].y);
    //       const Eigen::Vector2d meassurement(marker.corners[j].x, marker.corners[j].y);
    //       graph.emplace_shared<ArucoMarkerFactor>(key_cam_i, key_marker_i, meassurement, _marker_size, j, aruco_nm);
    //     }
    //   }

    //   NoiseModel ground_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };  // in cm
    //   for (auto& marker_id : _markers_on_ground)
    //   {
    //     const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker_id, "}") };
    //     graph.emplace_shared<OnGroundFactor>(origin_marker, key_marker_i, ground_nm);
    //   }
    // }
    // NoiseModel origin_nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-4) };
    // graph.addPrior(origin_marker, gtsam::Pose3(), origin_nm);

    // // graph.print("Graph", SF::formatter);
    // gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, _lm_params);
    // gtsam::Values result{ optimizer.optimize() };

    // result.print("Result", SF::formatter);

    // graph.printErrors(result, "Graph", SF::formatter);

    // project_result(result);
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

      // const Eigen::Vector3d offset{ Eigen::Vector3d::Zero() };
      // const Eigen::Vector2d pixel{ ArucoMarkerFactor::predict(cam, marker_pose, offset) };
      // DEBUG_VARS(marker_id, pixel[0], pixel[1])
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

  return 0;
}