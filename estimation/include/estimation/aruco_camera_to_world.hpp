#include <unordered_set>
#include <vector>

#include <ml4kp_bridge/gtsam_bridge.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <prx_models/Graph.h>

#include <ml4kp_bridge/defs.h>
// #include <prx/utilities/general/type_conversions.hpp>

#include <ros/node_handle.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

#include <utils/rosparams_utils.hpp>
#include <interface/StampedMarkers.h>
#include <interface/levenberg_marquardt_interface.hpp>

#include <estimation/camera_utils.hpp>
#include <estimation/camera_calibration_factors.hpp>
#include "utils/dbg_utils.hpp"
#include <interface/Marker.h>

namespace estimation
{

template <class Base>
class aruco_camera_to_world_t : public Base
{
  using Derived = aruco_camera_to_world_t<Base>;

  using CameraCalibration = gtsam::Cal3DS2;
  using Camera = gtsam::PinholePose<CameraCalibration>;  // Camera with a *fix* calibration

  using ArucoFixCamFactor = aruco_marker_fix_camera_factor_t<Camera>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using GraphValues = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>;

  // using MarkersMap = std::map<std::size_t, interface::StampedMarkers>;

public:
  aruco_camera_to_world_t() : _cameras(), _lm_params(prx::fg::default_levenberg_marquardt_parameters())
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::vector<std::string> aruco_topics;

    int& origin_marker{ _origin_marker };
    double& marker_size{ _marker_size };
    std::string& world_frame{ _world_frame };
    std::vector<std::string>& camera_frames{ _camera_frames };
    std::vector<int>& markers_on_ground{ _markers_on_ground };

    double estimation_frequency;

    PARAM_SETUP(private_nh, marker_size);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, aruco_topics);
    PARAM_SETUP(private_nh, origin_marker);
    PARAM_SETUP(private_nh, camera_frames);
    PARAM_SETUP(private_nh, markers_on_ground);
    PARAM_SETUP(private_nh, estimation_frequency)

    get_initial_poses(private_nh);

    _tf.header.frame_id = _world_frame;

    const std::string current_namespace{ private_nh.getNamespace() };
    interface::initialize(_lm_params, current_namespace);

    prx_assert(aruco_topics.size() == _camera_frames.size(), "camera frames and topics must be same size");

    for (int ci = 0; ci < _camera_frames.size(); ++ci)
    {
      gtsam::Pose3 cam_pose;
      Eigen::Vector<double, 9> cam_calibration;
      const std::string cam_namespace{ current_namespace + "/" + _camera_frames[ci] };
      calibration_from_parameters(cam_pose, cam_calibration, cam_namespace);
      _cameras[_camera_frames[ci]] = Camera(cam_pose, cam_calibration);

      _tf.child_frame_id = _camera_frames[ci];
      ml4kp_bridge::copy(_tf, _cameras[_camera_frames[ci]].pose());
      _static_broadcaster.sendTransform(_tf);
    }
    for (int i = 0; i < aruco_topics.size(); ++i)
    {
      _aruco_subscribers.push_back(private_nh.subscribe<interface::StampedMarkers>(
          aruco_topics[i], 1, boost::bind(&Derived::aruco_callback, this, _1, _camera_frames[i])));
    }

    const ros::Duration freq_timer(1.0 / estimation_frequency);
    _estimation_timer = private_nh.createTimer(freq_timer, &Derived::timer_function, this);
  }

  void get_initial_poses(ros::NodeHandle& nh_parent)
  {
    // ros::NodeHandle nh_poses(nh_parent.getNamespace());
    XmlRpc::XmlRpcValue initial_poses;

    PARAM_SETUP_WITH_DEFAULT(nh_parent, initial_poses, initial_poses);

    // DEBUG_VARS(initial_poses.valid());
    // DEBUG_VARS(initial_poses.size());

    for (int i = 0; i < initial_poses.size(); ++i)
    {
      const XmlRpc::XmlRpcValue pose_in{ initial_poses[i] };
      // const std::string marker_id_str(pose_in["id"]);
      const int marker_id(pose_in["id"]);
      // const int marker_id(prx::utilities::convert_to<int>(marker_id_str));
      const gtsam::Key key_marker{ gtsam::Symbol('M', marker_id) };

      // DEBUG_VARS(marker_id)
      gtsam::Pose3 pose;
      utils::get_value(pose, pose_in);
      // pose.print();

      _markers_initial_poses[marker_id] = pose;
      _prev_sln.insert(key_marker, pose);
    }
  }

protected:
  gtsam::NonlinearFactorGraph marker_to_graph(gtsam::Values& values, const interface::Marker& marker,
                                              const std::string cam_idx, const double dt)
  {
    // DEBUG_VARS(dt)
    gtsam::NonlinearFactorGraph graph;
    const NoiseModel aruco_nm{ gtsam::noiseModel::Isotropic::Sigma(2, dt) };
    const NoiseModel on_ground_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };
    const NoiseModel above_ground_nm{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-2) };

    const gtsam::Key key_marker{ gtsam::Symbol('M', marker.id) };
    const gtsam::Key key_origin_marker{ gtsam::Symbol('M', _origin_marker) };

    for (int j = 0; j < 4; ++j)
    {
      const Eigen::Vector2d meassurement(marker.corners[j].x, marker.corners[j].y);
      graph.emplace_shared<ArucoFixCamFactor>(key_marker, _cameras[cam_idx], meassurement, _marker_size, j, aruco_nm);
    }

    // if (marker.id == _origin_marker)
    // {
    //   graph.addPrior(key_origin_marker, gtsam::Pose3());
    // }

    if (std::find(_markers_on_ground.begin(), _markers_on_ground.end(), marker.id) != _markers_on_ground.end())
    {
      graph.emplace_shared<on_ground_factor_t>(key_origin_marker, key_marker, on_ground_nm);
      insert_values(values, key_origin_marker);
    }
    else
    {
      graph.emplace_shared<above_ground_factor_t>(key_marker, above_ground_nm);
    }

    insert_values(values, key_marker);

    return graph;
  }

  void insert_values(gtsam::Values& values, const gtsam::Key& key)
  {
    if (not values.exists(key))
    {
      if (_prev_sln.exists(key))
      {
        values.insert(key, _prev_sln.at<gtsam::Pose3>(key));
      }
      else
      {
        values.insert(key, gtsam::Pose3());
      }
    }
  }

  void timer_function(const ros::TimerEvent& event)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    std::set<int> markers_ids;

    if (_markers.size() == 0)
    {
      return;
    }

    for (auto& markers_map_pair : _markers)
    {
      const std::string cam_idx{ markers_map_pair.first };
      const double dt{ (event.current_real - markers_map_pair.second.header.stamp).toSec() };
      for (auto& marker : markers_map_pair.second.markers)
      {
        markers_ids.emplace(marker.id);
        graph.push_back(marker_to_graph(values, marker, cam_idx, dt));
      }
    }

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, _lm_params);
    const gtsam::Values result{ optimizer.optimize() };

    for (auto& id : markers_ids)
    {
      const gtsam::Key key_marker{ gtsam::Symbol('M', id) };
      const gtsam::Pose3 x{ result.at<gtsam::Pose3>(key_marker) };
      const std::string frame{ "Marker_" + prx::utilities::convert_to<std::string>(id) };

      _markers_initial_poses[id] = x;
      _tf.child_frame_id = frame;
      ml4kp_bridge::copy(_tf, x);

      _tf_broadcaster.sendTransform(_tf);
      if (id != _origin_marker)
        _prev_sln.insert_or_assign(key_marker, x);
    }
    _markers.clear();
  }

  void aruco_callback(const interface::StampedMarkersConstPtr msg, const std::string cam_id)
  {
    _markers[cam_id] = *msg;
  }

  int _origin_marker;

  std::string _world_frame;

  std::map<int, gtsam::Pose3> _markers_initial_poses;
  std::map<std::string, interface::StampedMarkers> _markers;

  std::map<std::string, Camera> _cameras;

  gtsam::Values _prev_sln;
  gtsam::LevenbergMarquardtParams _lm_params;

  const NoiseModel _aruco_nm;  // in pixels

  double _marker_size;

  geometry_msgs::TransformStamped _tf;
  tf2_ros::TransformBroadcaster _tf_broadcaster;
  tf2_ros::StaticTransformBroadcaster _static_broadcaster;

  std::vector<int> _markers_on_ground;
  std::vector<std::string> _camera_frames;

  // Subscribers
  std::vector<ros::Subscriber> _aruco_subscribers;

  // Publishers
  ros::Publisher _viz_edges_publisher;
  ros::Publisher _viz_nodes_publisher;

  // Timers
  ros::Timer _estimation_timer;
};
}  // namespace estimation