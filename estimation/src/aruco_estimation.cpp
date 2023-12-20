#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>

#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/type_conversions.hpp>
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/factors/camera_calibration_factor.hpp>
#include <prx/factor_graphs/factors/positive_vector_factor.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/factor_graphs/utilities/common_functions.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Rot3.h>
#include <aruco/aruco_nano.h>
#include <interface/stamped_markers.h>
#include <interface/utils.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <thread>

#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;
using namespace prx;

using sf = prx::fg::symbol_factory_t;
using Symbol = prx::fg::symbol_t;
using projection_factor = typename prx::fg::camera_projection_factor_t;
using camera_phi_factor = typename prx::fg::camera_phi_factor_t;

using Pixel = Eigen::Vector2d;
using Position = Eigen::Vector3d;
using CameraMatrix = Eigen::Matrix<double, 3, 4>;     // camera projection matrix
using IntrinsicMatrix = Eigen::Matrix<double, 3, 3>;  // camera projection matrix
using RotationMatrix = Eigen::Matrix<double, 3, 3>;   // camera projection matrix

using Values = gtsam::Values;
using FactorGraph = gtsam::NonlinearFactorGraph;
using NoiseModels = std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr>;
using MarkersPositions = std::unordered_map<std::size_t, Position>;
const Eigen::Vector3d Zero_3{ Eigen::Vector3d::Zero() };
const Eigen::Vector3d Ones_3{ Eigen::Vector3d::Ones() };
const Eigen::Vector4d Ones_4{ Eigen::Vector4d::Ones() };

const Eigen::Vector4d quat_init{ 0.5, 0.5, 0.5, 0.5 };

// Create symbol of marker's corner from markers id & corner id
auto sy_w_corner = [](std::size_t mid, std::size_t cid) {
  return sf::create_hashed_symbol("W_AR_P", mid, "CORNER", cid);
};

auto sy_p_center = [](std::size_t cam_id, std::size_t mid) {
  return sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", mid);
};

auto sym_cam = [](std::size_t cam_id) { return sf::create_hashed_symbol("Cam", cam_id); };

auto sy_marker_position = [](std::size_t ci, std::size_t mid) {
  return sf::create_hashed_symbol("Ms^{", ci, "}_{", mid, "}");
};
auto dyn_marker_position = [](std::size_t mid, std::size_t ci, std::size_t t) {
  return sf::create_hashed_symbol("M^{", t, "}_{", mid, "_", ci, "}");
};

auto sy_projection = [](std::size_t cam_id) { return sf::create_hashed_symbol("C^{PROJECTION}_", cam_id); };
// Create symbol of marker's center from markers id
// auto sy_w_center = [](std::size_t mid) { return sf::create_hashed_symbol("W_AR_P", mid); };
auto sy_scale = [](std::size_t cam_id) { return sf::create_hashed_symbol("s_", cam_id); };

std::size_t total_iterations{ 0 };
double marker_length{ 0 };  // in meters
int robot_id;

struct marker_t
{
  marker_t(const std::size_t idx, const std::size_t t, const Pixel& c0, const Pixel& c1, const Pixel& c2,
           const Pixel& c3)
    : id(idx), ti(t), corners({ c0, c1, c2, c3 }), center((c0 + c1 + c2 + c3) / 4.0)
  {
  }
  const std::array<Pixel, 4> corners;
  const std::size_t id;
  const std::size_t ti;
  const Pixel center;
};
std::vector<marker_t> markers_recieved;
void markers_callback(const interface::stamped_markersConstPtr markers)
{
  // for (int i = 0; i < 3; ++i)
  for (int i = 0; i < markers->markers.size(); ++i)
  {
    const interface::marker& marker{ markers->markers[i] };
    const int idx{ marker.id };
    markers_recieved.emplace_back(idx, total_iterations,  // no-lint
                                  Pixel{ { marker.x1, marker.y1 } }, Pixel{ { marker.x2, marker.y2 } },
                                  Pixel{ { marker.x3, marker.y3 } }, Pixel{ { marker.x4, marker.y4 } });
  }
  total_iterations++;
}

void get_static_markers(ros::NodeHandle& nh, MarkersPositions& markers_positions)
{
  XmlRpc::XmlRpcValue values;
  nh.getParam("markers_positions", values);
  for (int i = 0; i < values.size(); ++i)
  {
    const int id{ static_cast<int>(values[i]["id"]) };
    const double x{ static_cast<double>(values[i]["position"][0]) };
    const double y{ static_cast<double>(values[i]["position"][1]) };
    const double z{ static_cast<double>(values[i]["position"][2]) };
    markers_positions.emplace(std::make_pair(id, Position{ { x, y, z } }));
  }
}

void bootstap(NoiseModels& noise_models, FactorGraph& graph, Values& values, MarkersPositions& markers_positions)
{
  for (const auto& markers : markers_recieved)
  {
    if (markers.id == robot_id)
    {
      // PRX_DEBUG_VARS(markers.id, markers.center);
      continue;
    }
    const Symbol proj_sym{ sy_projection(0) };
    for (int ci = 0; ci < 4; ++ci)
    {
      const Symbol marker_sym{ sy_marker_position(ci, markers.id) };
      graph.add(projection_factor(marker_sym, proj_sym, markers.corners[ci], noise_models["projection"]));
      PRX_DEBUG_VARS(markers.id, ci, markers.corners[ci].transpose());
    }
  }
  std::array<Position, 4> marker_corners{ Position(marker_length / 2.0, -marker_length / 2.0, 0.0),
                                          Position(marker_length / 2.0, marker_length / 2.0, 0.0),
                                          Position(-marker_length / 2.0, marker_length / 2.0, 0.0),
                                          Position(-marker_length / 2.0, -marker_length / 2.0, 0.0) };

  PRX_DEBUG_VARS("Values:");
  for (auto marker : markers_positions)
  {
    const std::size_t id{ marker.first };
    for (int ci = 0; ci < 4; ++ci)
    {
      const Position position{ marker.second + marker_corners[ci] };
      const Symbol marker_sym{ sy_marker_position(ci, id) };
      graph.addPrior(marker_sym, position, noise_models["markers_prior"]);
      values.insert(marker_sym, position);
      PRX_DEBUG_VARS(id, ci, position.transpose());
    }
    // const Position position{ Position::Random() };
  }
}

// Go from P to A*[R|t]
void camera_matrix_to_matrices(const CameraMatrix& P, IntrinsicMatrix& A, RotationMatrix& R, Position& t)
{
  const IntrinsicMatrix B{ P.block<3, 3>(0, 0) };
  const Position b{ P.block<3, 1>(0, 3) };
  const IntrinsicMatrix K{ B * B.transpose() };

  // Eigen::Matrix3d sK;
  // Eigen::Vector3d xyz;
  // std::tie(sK, xyz) = gtsam::RQ(B);
  // PRX_DEBUG_VARS(sK);
  // PRX_DEBUG_VARS(xyz);

  // double s = sK(2, 2);
  // A = sK / s;

  // // Recover cRw itself, and its inverse
  // gtsam::Rot3 cRw = gtsam::Rot3::RzRyRx(xyz);
  // gtsam::Rot3 wRc = cRw.inverse();
  // R = wRc.matrix();
  // // Now, recover T from a = - s K cRw T = - A T
  // t = -(B.inverse() * b);

  const double u0{ K(0, 2) };
  const double v0{ K(1, 2) };

  const double beta{ std::sqrt(K(1, 1) - v0 * v0) };
  const double gamma{ (K(0, 1) - u0 * v0) / beta };
  const double alpha2{ K(0, 0) - std::pow(u0, 2) - std::pow(gamma, 2) };
  const double alpha{ std::abs(alpha2) < 0.01 ? 0.0 : std::sqrt(alpha2) };
  PRX_DEBUG_VARS(K(0, 0), std::pow(u0, 2), std::pow(gamma, 2));

  PRX_DEBUG_VARS(K);
  PRX_DEBUG_VARS(beta, gamma, alpha);

  A << alpha, gamma, u0, 0.0, beta, v0, 0.0, 0.0, 1.0;
  PRX_DEBUG_VARS(A.inverse());
  R = A.inverse() * B;
  t = A.inverse() * b;
}

int main(int argc, char** argv)
{
  const std::string node_name{ "ArucoEstimationNode" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  // FG vars
  Values values_markers;
  FactorGraph graph_markers;
  FactorGraph graph;
  Values values;
  NoiseModels noise_models;
  MarkersPositions static_markers_positions;
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  // lm_params.setUseFixedLambdaFactor(false);
  lm_params.setMaxIterations(100);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  // std::unordered_map<Symbol, Pixel> symbol_positions;

  // Camera vars
  Eigen::Matrix3d camera_matrix;
  Eigen::Vector<double, 5> distortion_coeff;

  // Params
  int cam_id{ 0 };  // For multiple cameras, setting to 0 for now

  ROS_PARAM_SETUP(nh, marker_length);
  ROS_PARAM_SETUP(nh, robot_id);

  projection_factor::Projection camera_projection_init{ projection_factor::Projection::Ones() };
  // camera_projection_init[8] = 0.5;
  // camera_projection_init[9] = 0.7;
  // camera_projection_init[10] = 0.5;
  const double marker_to_center{ marker_length / std::sqrt(2.0) };

  noise_models["scale"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);
  noise_models["projection"] = gtsam::noiseModel::Isotropic::Sigma(2, 1e0);
  noise_models["pos_projection"] = gtsam::noiseModel::Isotropic::Sigma(12, 1e0);
  noise_models["norm_projection"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["markers_prior"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
  noise_models["camera_phi"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);

  // ROS subscribers/publishers
  const std::string markers_topic{ ros::this_node::getNamespace() + "/markers" };
  ros::Subscriber markers_subscriber{ nh.subscribe(markers_topic, 1, &markers_callback) };

  get_static_markers(nh, static_markers_positions);

  while (ros::ok())
  {
    ros::spinOnce();
    if (total_iterations == 1)
    {
      bootstap(noise_models, graph_markers, values_markers, static_markers_positions);
      // values.insert(values_markers);
      // graph.add(graph_markers);

      values_markers.insert(sy_projection(cam_id), camera_projection_init);
      graph_markers.add(prx::fg::positive_vector_factor_t<12>(sy_projection(cam_id), noise_models["pos_projection"]));
      graph_markers.add(prx::fg::camera_projection_norm_factor_t(sy_projection(cam_id), noise_models["norm_"
                                                                                                     "projection"]));
      gtsam::LevenbergMarquardtOptimizer optimizer(graph_markers, values_markers, lm_params);
      graph_markers.printErrors(values_markers, "Markers", prx::fg::symbol_factory_t::formatter);
      values_markers.print("Values", prx::fg::symbol_factory_t::formatter);
      values = prx::fg::optimize_and_log(optimizer, lm_params);

      results.print("Results", prx::fg::symbol_factory_t::formatter);
      const CameraMatrix P{
        results.at<projection_factor::Projection>(sy_projection(cam_id)).reshaped<Eigen::RowMajor>(3, 4)
      };
      IntrinsicMatrix A{};
      RotationMatrix R{};
      Position tvec{};
      camera_matrix_to_matrices(P, A, R, tvec);
      PRX_DEBUG_VARS(P);
      PRX_DEBUG_VARS(A);
      PRX_DEBUG_VARS(R);
      PRX_DEBUG_VARS(tvec.transpose());
      markers_recieved.clear();
      // ros::shutdown();
    }
    else
    {
      Symbol marker_pos{};
      for (const auto& markers : markers_recieved)
      {
        for (int ci = 0; ci < 4; ++ci)
        {
          if (markers.id == robot_id)
          {
            marker_pos = dyn_marker_position(markers.id, ci, markers.ti);
            const Eigen::Vector3d init_pos{ Eigen::Vector3d::Zero() };
            values.insert(marker_pos, init_pos);
          }
          else
          {
            marker_pos = sy_marker_position(ci, markers.id);
          }

          const Pixel pixel{ markers.corners[ci] };
          graph_2.add(camera_phi_factor(marker_pos, sy_projection(cam_id), sy_scale(cam_id), pixel,
                                        noise_models["camera_phi"]));
        }
      }
    }
  };
  exit(0);
  ////
  prx::param_loader params{ "executables/perception/camera_to_world.yaml", argc, argv };
  std::vector<std::string> images_paths;

  std::string markers_file{ params["markers_file"].as<std::string>() };
  ////

  std::vector<std::vector<aruconano::Marker>> markers_set;

  // values.insert(values_markers);
  // graph.add(graph_markers);

  // const projection_factor::Projection camera_projection_init{ projection_factor::Projection::Ones() };
  // values.insert(sy_projection(cam_id), camera_projection_init);
  // graph.add(prx::fg::positive_vector_factor_t<12>(sy_projection(cam_id), noise_models["pos_projection"]));
  // graph.add(prx::fg::camera_projection_norm_factor_t(sy_projection(cam_id), noise_models["norm_projection"]));
  // graph.add(prx::fg::normalize_factor_t<12>(sy_projection(cam_id), noise_models["norm_projection"]));
  // cam_id++;

  // prx::fg::fg_to_csv<2>(graph, values, prx::out_path + "/mj_ctw_in.txt", variables_positions);

  prx::fg::symbol_factory_t::symbols_to_file();
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  auto results = prx::fg::optimize_and_log(optimizer, lm_params);

  gtsam::NonlinearFactorGraph graph_2;
  gtsam::Values values_2;
  cam_id = 0;

  projection_factor::Projection P0_vec{ results.at<projection_factor::Projection>(sy_projection(cam_id)) };
  std::size_t t_marker{ 0 };
  // logger_t logger(prx::out_path + "/ctw_traj.txt", ' ');
  // values_2.insert(sy_projection(cam_id), results.at<projection_factor::Projection>(sy_projection(cam_id)));

  for (const auto& markers : markers_set)
  {
    for (const auto& m : markers)
    {
      Eigen::Vector2d center{ Eigen::Vector2d::Zero() };

      for (auto pt : m)
      {
        const Eigen::Vector2d pixel{ pt.x, pt.y };
        center += pixel;
      }
      center = center / 4.0;
      // logger("UV", m.id, center.transpose());
    }
  }
  results.insert(sy_scale(cam_id), Eigen::Vector<double, 1>(1.0));
  graph_2.add(graph_markers);
  graph_2.add(prx::fg::positive_vector_factor_t<1>(sy_scale(cam_id), noise_models["scale"]));
  graph_2.addPrior(sy_scale(cam_id), Eigen::Vector<double, 1>(1.0), noise_models["scale"]);
  graph_2.add(prx::fg::positive_vector_factor_t<12>(sy_projection(cam_id), noise_models["pos_projection"]));
  prx::fg::symbol_factory_t::symbols_to_file();
  gtsam::LevenbergMarquardtOptimizer optimizer_2(graph_2, results, lm_params);
  auto results_2 = prx::fg::optimize_and_log(optimizer_2, lm_params);
  graph_2.printErrors(results_2, "graph", prx::fg::symbol_factory_t::formatter);
  // results_2.print("ctw", prx::symbol_factory_t::formatter);

  // prx::fg::fg_to_csv<2>(graph_2, results_2, prx::out_path + "/mj_ctw_out_2.txt", variables_positions);

  // for (std::size_t i = 0; i < t_marker; ++i)
  // {
  //   camera_phi_factor::Position position{ results_2.at<camera_phi_factor::Position>(dyn_marker_position(0, i)) };
  //   logger("R", i, position.transpose());
  // }
  // for (std::size_t idx : { 62, 50, 51, 54, 64, 55 })
  // {
  //   camera_phi_factor::Position position{ results_2.at<camera_phi_factor::Position>(sy_marker_position(idx)) };
  //   logger("M", idx, position.transpose());
  // }

  return 0;
}