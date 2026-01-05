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
#include <prx/utilities/general/type_conversions.hpp>

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
  using ArucoMarkerFactor = estimation::aruco_marker_factor_t<Camera>;
  using OnGroundFactor = estimation::on_ground_factor_t;
  // using prx::utilities::convert_to;

  std::string _world_frame;
  std::vector<ros::Subscriber> _marker_subscribers;
  std::vector<ros::Publisher> _cam_info_pubs;

  std::map<std::string, MarkersMap> _markers;

  ros::Timer _fg_timer;

  int _wait_cycles;
  double _marker_size;

  int _origin_marker;
  int _height;

  std::vector<Camera> _cameras;
  std::vector<int> _markers_on_ground;
  std::vector<std::string> _camera_frames;
  std::map<int, gtsam::Pose3> _markers_poses;
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

    // DEBUG_VARS(marker_size);
    // DEBUG_VARS(aruco_topics);

    ros::NodeHandle nh_optimizer("~/optimizer/");
    int iterations;
    PARAM_SETUP(nh_optimizer, iterations);

    _lm_params.setMaxIterations(iterations);
    // _lm_params.setMaxIterations(100);

    // for (auto& topic_name : aruco_topics)
    prx_assert(aruco_topics.size() == _camera_frames.size(), "camera frames and topics must be same size");
    for (int i = 0; i < aruco_topics.size(); ++i)
    {
      // DEBUG_VARS(aruco_topics[i], _camera_frames[i]);
      _marker_subscribers.push_back(nh.subscribe<interface::StampedMarkers>(
          aruco_topics[i], 1, boost::bind(&This::marker_callback, this, _1, _camera_frames[i])));
    }

    for (auto cam : _camera_frames)
    {
      const std::string camera_name_topic{ cam + "/info" };
      _cam_info_pubs.push_back(nh.advertise<sensor_msgs::CameraInfo>(camera_name_topic, 1, true));
    }

    const ros::Duration freq_timer(1.0);
    _fg_timer = nh.createTimer(freq_timer, &This::timer_function, this);
  }

  void marker_callback(const interface::StampedMarkersConstPtr msg, const std::string cam_id)
  {
    // _markers[idx] = *msg;
    for (int i = 0; i < msg->markers.size(); ++i)
    {
      const interface::Marker& marker{ msg->markers[i] };
      _markers[cam_id][marker.id] = marker;
      // DEBUG_VARS(cam_id, marker);
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
    for (int ci = 0; ci < _camera_frames.size(); ++ci)
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

      const Eigen::Vector<double, 9> calibration({ fx, fy, s, u0, v0, k1, k2, p1, p2 });
      // const CameraCalibration calibration(fx, fy, s, u0, v0, k1, k2, p1, p2);
      _cameras.emplace_back(pose, calibration);
      // gtsam::PinholeCamera camera(pose, calibration);
    }

    // run_calibration();
    run_incremental_calibration();
    publish_cameras_tf();
  }

  static void pose_to_tf(geometry_msgs::TransformStamped& msg, const gtsam::Pose3& pose)
  {
    msg.transform.translation.x = pose.x();
    msg.transform.translation.y = pose.y();
    msg.transform.translation.z = pose.z();

    // quat = tf.transformations.quaternion_from_matrix(mat)
    const gtsam::Quaternion quat{ pose.rotation().toQuaternion() };
    msg.transform.rotation.w = quat.w();
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
  }

  void publish_cameras_tf()  //
  {
    for (int i = 0; i < _camera_frames.size(); ++i)
    {
      const std::string& camera_frame{ _camera_frames[i] };
      const Camera& camera{ _cameras[i] };

      camera.print(camera_frame);

      geometry_msgs::TransformStamped static_transform_stamped;
      static_transform_stamped.header.frame_id = _world_frame;
      static_transform_stamped.child_frame_id = camera_frame;

      static_transform_stamped.header.stamp = ros::Time::now();

      const gtsam::Pose3& pose{ camera.pose() };
      pose_to_tf(static_transform_stamped, pose);

      // static_transform_stamped.transform.translation.x = pose.x();
      // static_transform_stamped.transform.translation.y = pose.y();
      // static_transform_stamped.transform.translation.z = pose.z();

      // // quat = tf.transformations.quaternion_from_matrix(mat)
      // const gtsam::Quaternion quat{ pose.rotation().toQuaternion() };
      // static_transform_stamped.transform.rotation.w = quat.w();
      // static_transform_stamped.transform.rotation.x = quat.x();
      // static_transform_stamped.transform.rotation.y = quat.y();
      // static_transform_stamped.transform.rotation.z = quat.z();

      _static_broadcaster.sendTransform(static_transform_stamped);
    }

    // for (int i = 0; i < _markers_poses.size(); ++i)
    for (auto pair : _markers_poses)
    {
      // DEBUG_VARS(pair.first);
      // pair.second.print();
      geometry_msgs::TransformStamped static_transform_stamped;
      static_transform_stamped.header.frame_id = _world_frame;
      static_transform_stamped.child_frame_id = "Marker_" + prx::utilities::convert_to<std::string>(pair.first);

      static_transform_stamped.header.stamp = ros::Time::now();

      // DEBUG_VARS();
      pose_to_tf(static_transform_stamped, pair.second);
      _static_broadcaster.sendTransform(static_transform_stamped);
    }
  }

  void run_incremental_calibration()
  {
    gtsam::Values initial_values;
    gtsam::NonlinearFactorGraph graph;

    const NoiseModel origin_nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-4) };
    const NoiseModel aruco_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };    // in pixels
    const NoiseModel ground_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };  // in cm

    const gtsam::Key origin_marker{ SF::create_hashed_symbol("marker_{", _origin_marker, "}") };

    initial_values.insert(origin_marker, gtsam::Pose3());
    graph.addPrior(origin_marker, gtsam::Pose3(), origin_nm);

    for (int i = 0; i < _camera_frames.size(); ++i)
    {
      const std::string frame{ _camera_frames[i] };
      const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", frame) };
      initial_values.insert(key_cam_i, _cameras[i]);
      for (int j = 0; j < 4; ++j)
      {
        const interface::Marker& marker{ _markers[frame][_origin_marker] };
        const Eigen::Vector2d meassurement(marker.corners[j].x, marker.corners[j].y);
        graph.emplace_shared<ArucoMarkerFactor>(key_cam_i, origin_marker, meassurement, _marker_size, j, aruco_nm);
      }
    }

    for (int i = 0; i < _camera_frames.size(); ++i)
    {
      const std::string frame{ _camera_frames[i] };
      const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", frame) };

      // DEBUG_PRINT;

      for (auto& marker_pair : _markers[frame])
      {
        const interface::Marker& marker{ marker_pair.second };

        // std::vector<int> valid_markers = { 121, 36, 14 };
        // if (std::find(valid_markers.begin(), valid_markers.end(), marker.id) == valid_markers.end())
        // {
        //   continue;
        // }
        const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker.id, "}") };

        for (int j = 0; j < 4; ++j)
        {
          const Eigen::Vector2d meassurement(marker.corners[j].x, marker.corners[j].y);
          graph.emplace_shared<ArucoMarkerFactor>(key_cam_i, key_marker_i, meassurement, _marker_size, j, aruco_nm);
        }

        if (std::find(_markers_on_ground.begin(), _markers_on_ground.end(), marker.id) != _markers_on_ground.end())
        {
          graph.emplace_shared<OnGroundFactor>(origin_marker, key_marker_i, ground_nm);
        }  // if (initial_values.exists(key_marker_i))
        // {
        // }
        if (not initial_values.exists(key_marker_i))
        {
          initial_values.insert(key_marker_i, gtsam::Pose3());
        }

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, _lm_params);
        initial_values = optimizer.optimize();
        update_estimation(initial_values);
        publish_cameras_tf();
        graph.printErrors(initial_values, "Graph", SF::formatter);
        // PRINT_KEY(key_cam_i);
        // PRINT_KEY(key_marker_i);
        // int dummy;
        // std::cin >> dummy;
      }
    }
  }

  void run_calibration()
  {
    gtsam::Values initial_values;
    gtsam::NonlinearFactorGraph graph;

    const gtsam::Key origin_marker{ SF::create_hashed_symbol("marker_{", _origin_marker, "}") };
    for (int i = 0; i < _camera_frames.size(); ++i)
    {
      const std::string frame{ _camera_frames[i] };
      const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", frame) };
      // DEBUG_VARS(frame, key_cam_i);

      // PRINT_KEY(key_cam_i)
      initial_values.insert(key_cam_i, _cameras[i]);
      // DEBUG_PRINT;
      NoiseModel aruco_nm{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };  // in pixels

      for (auto& marker_pair : _markers[frame])
      {
        const interface::Marker& marker{ marker_pair.second };

        //     // if (marker.id != 121)
        //     // if (marker.id != 121 and marker.id != 1 and marker.id != 80)
        // if (marker.id != 121 and marker.id != 1)
        //
        // if (marker.id != 121 and marker.id != 1 and marker.id != 80 and )
        std::vector<int> valid_markers = { 121, 36, 14 };
        if (std::find(valid_markers.begin(), valid_markers.end(), marker.id) == valid_markers.end())
        {
          continue;
        }
        const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker.id, "}") };

        // Eigen::Vector2d z_avg{ Eigen::Vector2d::Zero() };
        for (int j = 0; j < 4; ++j)
        {
          //       // const Eigen::Vector2d meassurement(marker.corners[j].x, _height - marker.corners[j].y);
          const Eigen::Vector2d meassurement(marker.corners[j].x, marker.corners[j].y);
          graph.emplace_shared<ArucoMarkerFactor>(key_cam_i, key_marker_i, meassurement, _marker_size, j, aruco_nm);
          // z_avg += meassurement;
        }
        // const Eigen::Vector3d position_init{ camera.backproject(z_avg / 4.0, camera.pose().z()) };
        // initial_values.insert_or_assign(key_marker_i, gtsam::Pose3(gtsam::Rot3(), position_init));
        initial_values.insert_or_assign(key_marker_i, gtsam::Pose3());
      }

      NoiseModel ground_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };  // in cm
      for (auto& marker_id : _markers_on_ground)
      {
        const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker_id, "}") };
        if (initial_values.exists(key_marker_i))
        {
          graph.emplace_shared<OnGroundFactor>(origin_marker, key_marker_i, ground_nm);
        }
      }
    }
    NoiseModel origin_nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-4) };
    graph.addPrior(origin_marker, gtsam::Pose3(), origin_nm);

    // // graph.print("Graph", SF::formatter);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, _lm_params);
    gtsam::Values result{ optimizer.optimize() };

    // result.print("Result", SF::formatter);

    graph.printErrors(result, "Graph", SF::formatter);

    update_estimation(result);

    // project_result(result);
  }

  void update_estimation(const gtsam::Values& result)
  {
    for (int i = 0; i < _camera_frames.size(); ++i)
    {
      const std::string frame{ _camera_frames[i] };
      const gtsam::Key key_cam_i{ SF::create_hashed_symbol("Camera_", frame) };

      _cameras[i] = result.at<Camera>(key_cam_i);
      for (auto& marker_pair : _markers[frame])
      {
        const interface::Marker& marker{ marker_pair.second };

        const gtsam::Key key_marker_i{ SF::create_hashed_symbol("marker_{", marker.id, "}") };

        if (result.exists(key_marker_i))
        {
          _markers_poses[marker.id] = result.at<gtsam::Pose3>(key_marker_i);
        }
      }
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