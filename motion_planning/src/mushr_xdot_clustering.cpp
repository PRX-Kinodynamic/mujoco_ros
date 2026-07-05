#include <limits>
#include <prx/simulation/system.hpp>
#include <thread>
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/defs.hpp>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>
#include <motion_planning/clustering.hpp>
#include "utils/dbg_utils.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <interface/gaussian_to_ellipse_marker.hpp>
#include <motion_planning/nonlinear_clustering.hpp>
#include <prx_models/linear_mixture_model.hpp>

// using XDdot = Eigen::Vector<double, 3>;
using State = Eigen::Vector3d;
using Control = Eigen::Vector<double, 2>;
using Element = gtsam::ProductLieGroupV43<State, Control>;
using Data = std::tuple<State, Control, State>;

static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };
static constexpr Eigen::Index DimZ{ DimX + DimU };

using Zmatrix = Eigen::Matrix<double, DimX, DimZ>;
using Covariance = Eigen::Matrix<double, DimZ, DimZ>;
using IsotropicNM = gtsam::noiseModel::Isotropic;
using DiagonalNM = gtsam::noiseModel::Diagonal;
using Line = std::vector<std::string>;
using prx::utilities::convert_to;

void data_to_markers(visualization_msgs::Marker& marker, const std::vector<Data>& data)
{
  for (auto [x0, u0, x1] : data)
  {
    marker.points.emplace_back();
    marker.points.back().x = x0[0];
    marker.points.back().y = x0[1];
    marker.points.back().z = x0[2];
  }
}

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrClustering" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  visualization_msgs::MarkerArray x0_marker_array, x0_ellipses_array;
  visualization_msgs::MarkerArray x1_marker_array;
  visualization_msgs::MarkerArray x1_pred_marker_array;

  ros::Publisher x0_markers_publisher{ nh.advertise<visualization_msgs::MarkerArray>("/clustering/x0/markers", 1,
                                                                                     true) };
  ros::Publisher x0_ellipses_publisher{ nh.advertise<visualization_msgs::MarkerArray>("/clustering/x0/ellipses/markers",
                                                                                      1, true) };
  ros::Publisher x1_markers_publisher{ nh.advertise<visualization_msgs::MarkerArray>("/clustering/x1/markers", 1,
                                                                                     true) };
  ros::Publisher x1_pred_markers_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/x1/prediction/markers", 1, true) };
  ros::Publisher x0_rejected_markers_publisher{ nh.advertise<visualization_msgs::Marker>(
      "/clustering/x0/rejected/markers", 1, true) };
  // ros::Publisher markers_ellipses_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
  //     "/clustering/ellipses/markers", 1, true) };
  // ros::Publisher markers_controls_data_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
  //     "/clustering/controls/data/markers", 1, true) };
  // ros::Publisher markers_controls_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
  //     "/clustering/controls/markers", 1, true) };

  std::string data_file, output_dir;
  int batch_size, test_size;

  PARAM_SETUP(nh, data_file)
  PARAM_SETUP(nh, output_dir)
  PARAM_SETUP_WITH_DEFAULT(nh, batch_size, 1e3);
  PARAM_SETUP_WITH_DEFAULT(nh, test_size, batch_size);

  prx::utilities::csv_reader_t reader(data_file);

  // motion_planning::cluster_in_out_t<Element, Data> input, output;
  // motion_planning::nonlinear_cluster_values_t<Element, Data> input, output;

  // cout << "The orthogonal matrix U is:" << endl << schur.matrixU() << endl;
  // cout << "The quasi-triangular matrix T is:" << endl << schur.matrixT() << endl << endl;

  Line line;
  // Element element;
  // auto nm = IsotropicNM::Sigma(8, 0.10);
  Eigen::VectorXd sigmas(5);
  // sigmas << 0.05, 0.05, 0.05, 0.05, 0.05;
  // sigmas << 0.125, 0.125, 0.125, 0.1, 0.1;
  sigmas << 0.25, 0.25, 0.25, 0.25, 0.25;
  // sigmas << 0.1, 0.1, 0.1, 0.1, 0.1;
  // sigmas << 1, 1, 1, 1, 1;
  auto nm = DiagonalNM::Sigmas(sigmas);

  std::vector<Data> input_data, test_set;
  // ----------------: 0  1  2   3  4  5   6    7     8     9     10    11     12   13 14
  // Files with lines: ti x0 y0 th0 x1 y1 th1 xdot0 ydot0 thdot0 xdot1 ydot1 thdot1 u0 u1
  while (reader.next_valid_line(line))
  {
    const double xdot0{ convert_to<double>(line[7]) };   // Q0
    const double ydot0{ convert_to<double>(line[8]) };   // Q0
    const double thdot0{ convert_to<double>(line[9]) };  // Q0

    const double xdot1{ convert_to<double>(line[10]) };   // Q1
    const double ydot1{ convert_to<double>(line[11]) };   // Q1
    const double thdot1{ convert_to<double>(line[12]) };  // Q1

    const double u0{ convert_to<double>(line[13]) };  // control
    const double u1{ convert_to<double>(line[14]) };  // control

    const State xd0{ State(xdot0, ydot0, thdot0) };
    const State xd1{ State(xdot1, ydot1, thdot1) };
    const Control u{ Control(u0, u1) };
    // const Data data{ std::make_tuple(xd0, u, xd1) };
    // const Element element{ { xd0, u } };

    // input.push_back(element, data, nm);
    input_data.emplace_back(xd0, u, xd1);
  }
  std::shuffle(input_data.begin(), input_data.end(), prx::global_generator);
  for (int i = 0; i < test_size; ++i)
  {
    test_set.push_back(input_data.back());
    input_data.pop_back();
  }
  motion_planning::nonlinear_cluster_values_t<Element, Data> input, output;
  using Cluster = motion_planning::cluster_t<Element, Data>;
  // using Covariance = Eigen::Matrix<double, DimX, DimX>;
  std::map<std::size_t, std::tuple<Cluster, Covariance, Zmatrix>> cluster_map;

  bool converged{ false };
  std::size_t iter{ 0 };
  std::size_t max_iterations{ 10 };
  std::size_t prev_output{ 0 };
  while (not converged and iter < max_iterations)
  {
    iter++;
    output.clear();
    if (input_data.size() > 0)
    {
      iter--;
      const std::size_t tot_elements_to_cluster{ std::min(static_cast<std::size_t>(batch_size), input_data.size()) };
      // for (int i = initial_size; i < tot_elements_to_cluster; ++i)
      while (input.clusters.size() < tot_elements_to_cluster)
      {
        const Data& data{ input_data.back() };
        auto& [x0, u0, x1] = data;
        const Element element(x0, u0);

        input_data.pop_back();

        input.push_back(element, data, nm);
      }
    }
    std::shuffle(input.clusters.begin(), input.clusters.end(), prx::global_generator);

    int max_steps{ -1 };
    cluster_multiple_iterations(output, input, max_steps);

    converged = prev_output == output.clusters.size();
    prev_output = output.clusters.size();
    DEBUG_VARS(max_steps, converged, input.clusters.size(), output.clusters.size());

    input.clear();

    using LGM = prx_models::linear_gaussian_model_t<State, Control>;
    // Eigen::Matrix<double, DimX, DimX + DimU> A{ LGM::LSE_AB(all_zts, all_xdots) };
    for (int i = 0; i < output.clusters.size(); ++i)
    {
      const motion_planning::cluster_t<Element, Data> cluster{ output.clusters[i] };
      const std::size_t total_clustered{ cluster.total_clustered };
      if (cluster.locked)
      {
        input.clusters.emplace_back(cluster);
        continue;
      }
      if (total_clustered < 2 * (DimX + DimU))
      {
        const std::string rejected_key{ gtsam::DefaultKeyFormatter(cluster.key) };
        DEBUG_VARS(i, rejected_key, total_clustered)
        continue;
      }

      const Element& element{ cluster.element };
      const std::vector<Element>& clustered_elements{ cluster.clustered_elements };
      const Covariance cov{ motion_planning::compute_cluster_covariance(element, clustered_elements) };

      const Eigen::Matrix<double, DimX, DimZ> Zmat{ LGM::LSE_AB(cluster.data) };
      LGM lgm(Zmat, cov, element);
      const auto [ebA, ebB] = lgm.compute_error_bounds(0.05, 20, 0.001);

      double mean_error{ 0 };
      for (auto [x0, u0, x1] : cluster.data)
      {
        const State x1p{ lgm.evaluate(x0, u0) };
        const Eigen::Vector<double, DimX> v_err{ prx::TangentBetween(x1, x1p) };
        mean_error += v_err.norm();
      }
      mean_error = mean_error / total_clustered;
      if (mean_error > std::max(ebA, ebB))
      {
        const std::size_t& rejected{ cluster.idx };
        DEBUG_VARS(rejected, mean_error, ebA, ebB);

        for (auto [x0, u0, x1] : cluster.data)
        {
          const Element ei(x0, u0);
          input.push_back(ei, { x0, u0, x1 }, nm);
        }
      }
      else
      {
        const std::size_t& accepted{ cluster.idx };
        DEBUG_VARS(accepted, mean_error, ebA, ebB);
        input.clusters.emplace_back(cluster);
        input.clusters.back().locked = true;
        cluster_map.emplace(cluster.idx, std::make_tuple(input.clusters.back(), cov, Zmat));
      }
    }
  }

  // int max_steps{ -1 };
  // cluster_multiple_iterations(output, input, max_steps);

  // DEBUG_VARS(max_steps, output.clusters.size());
  // DEBUG_VARS(output.total_clustered.size(), output.data.size());
  // DEBUG_VARS(output.factor_graphs.size(), output.keys.size());

  // const std::string clusters_filename{ output_dir + "/clusters.txt" };
  // const std::string values_filename{ output_dir + "/values.txt" };
  // const std::string covs_filename{ output_dir + "/covariances.txt" };
  // const std::string covs_Ais{ output_dir + "/linear_systems.txt" };
  // std::ofstream ofs_values(values_filename.c_str());
  // std::ofstream ofs_covs(covs_filename.c_str());
  // std::ofstream ofs_clusters(clusters_filename.c_str());
  // std::ofstream ofs_Ais(covs_Ais.c_str());
  prx_models::linear_mixture_model_t<State, Control> lmm;

  interface::gaussian_params_t gauss_pts_params, marker_ctrl_params;

  int rejected{ 0 };
  int clustered{ 0 };
  visualization_msgs::Marker marker_rejected_pts{ ml4kp_bridge::create_marker(0.3, { 0.8, 1.0, 0., 0. }, 0, "rejected",
                                                                              "world") };
  marker_rejected_pts.type = visualization_msgs::Marker::POINTS;
  for (int i = 0; i < input.clusters.size(); ++i)
  {
    const motion_planning::cluster_t<Element, Data>& cluster{ input.clusters[i] };

    if (cluster.locked)
    {
      continue;
    }

    const std::size_t total_clustered{ cluster.total_clustered };
    const std::string rejected_key{ gtsam::DefaultKeyFormatter(cluster.key) };

    data_to_markers(marker_rejected_pts, cluster.data);
    // rejected++;
  }
  DEBUG_VARS(cluster_map.size())
  for (auto& [key, cluster_covariance_Zmat] : cluster_map)
  {
    const Cluster cluster{ std::get<Cluster>(cluster_covariance_Zmat) };
    const Covariance cov{ std::get<Covariance>(cluster_covariance_Zmat) };
    const Zmatrix Zmat{ std::get<Zmatrix>(cluster_covariance_Zmat) };
    const std::size_t i{ cluster.idx };
    const std::size_t total_clustered{ cluster.total_clustered };
    // const motion_planning::cluster_t<Element, Data> cluster{ output.clusters[i] };

    clustered++;
    const Element& element{ cluster.element };
    const std::vector<Element>& clustered_elements{ cluster.clustered_elements };
    // const Eigen::Matrix<double, 5, 5> cov{ motion_planning::compute_cluster_covariance(element, clustered_elements)
    // };

    const double ei_r{ std::fabs(std::cos(element.first[0] + element.second[0])) };
    const double ei_g{ std::fabs(std::sin(element.first[1] + element.second[1])) };
    const double ei_b{ std::fabs(std::cos(element.first[2])) };
    const std::vector marker_color{ { 0.8, ei_r, ei_g, ei_b } };
    const std::string marker_ns{ "cluster_" + convert_to<std::string>(i) };
    const std::string marker_frame_id{ "world" };
    visualization_msgs::Marker marker_x0_pts{ ml4kp_bridge::create_marker(0.3, marker_color, i, marker_ns,
                                                                          marker_frame_id) };
    visualization_msgs::Marker marker_x1_pts{ ml4kp_bridge::create_marker(0.3, marker_color, i, marker_ns,
                                                                          marker_frame_id) };
    visualization_msgs::Marker marker_x1_predict_pts{ ml4kp_bridge::create_marker(0.3, marker_color, i, marker_ns,
                                                                                  marker_frame_id) };

    marker_x0_pts.id = marker_x1_pts.id = marker_x1_predict_pts.id = i;
    marker_x0_pts.type = marker_x1_pts.type = marker_x1_predict_pts.type = visualization_msgs::Marker::POINTS;

    const Eigen::Matrix<double, 3, 3> covX{ cov.block<3, 3>(0, 0) };
    gauss_pts_params.idx = i;
    gauss_pts_params.ns = "cluster_" + convert_to<std::string>(i);
    gauss_pts_params.position = element.first;
    gauss_pts_params.color = { 0.5, ei_r, ei_g, ei_b };
    gauss_pts_params.cov_to_3Dellipse(covX);

    // std::vector<Eigen::Vector<double, DimX>> all_x1;
    // std::vector<Eigen::Vector<double, DimX + DimU>> all_zs;

    data_to_markers(marker_x0_pts, cluster.data);

    lmm.emplace(Zmat, cov, element, i);

    // for (auto [x0, u0, x1] : cluster.data)
    // {
    //   const Element z0{ Element(x0, u0) };
    //   const Eigen::Vector<double, DimZ> tg0{ gtsam::traits<Element>::Logmap(z0) };
    //   const Eigen::Vector<double, DimX> tg1{ gtsam::traits<State>::Logmap(x1) };
    //   all_zs.push_back(tg0);
    //   all_x1.push_back(tg1);
    // }

    // using LGM = prx_models::linear_gaussian_model_t<State, Control>;
    // Eigen::Matrix<double, DimX, DimX + DimU> A{ LGM::LSE_AB(all_zs, all_x1) };
    // LGM lgm(A, cov, element);

    // double mean_error{ 0 };
    // for (auto [x0, u0, x1] : cluster.data)
    // {
    //   const State x1p{ lgm.evaluate(x0, u0) };

    //   marker_x1_predict_pts.points.emplace_back();
    //   marker_x1_predict_pts.points.back().x = x1p[0];
    //   marker_x1_predict_pts.points.back().y = x1p[1];
    //   marker_x1_predict_pts.points.back().z = x1p[2];
    //   // if (i == element_to_debug)
    //   // {
    //   //   LOG_VARS(x1, x1p);
    //   // }
    //   const Eigen::Vector<double, DimX> v_err{ prx::TangentBetween(x1, x1p) };
    //   mean_error += v_err.norm();
    // }
    // // double total_clustered = cluster.data.size();
    // mean_error = mean_error / total_clustered;
    // DEBUG_VARS(i, element, total_clustered, mean_error)

    const visualization_msgs::Marker euclidean_ellipse{ interface::gaussian_to_ellipse_marker(gauss_pts_params) };

    // euclidean_ellipses_array.markers.push_back(euclidean_ellipse);

    x0_marker_array.markers.push_back(marker_x0_pts);
    x1_marker_array.markers.push_back(marker_x1_pts);
    x1_pred_marker_array.markers.push_back(marker_x1_predict_pts);
    x0_ellipses_array.markers.push_back(euclidean_ellipse);
  }

  const double rate_clustered{ clustered / static_cast<double>(clustered + rejected) };
  DEBUG_VARS(clustered, rejected, rate_clustered)

  x0_markers_publisher.publish(x0_marker_array);
  x1_markers_publisher.publish(x1_marker_array);
  x1_pred_markers_publisher.publish(x1_pred_marker_array);
  x0_ellipses_publisher.publish(x0_ellipses_array);
  x0_rejected_markers_publisher.publish(marker_rejected_pts);

  double total_mean_error{ 0. };
  double total_test{ 0. };
  for (auto& [x0, u0, x1] : test_set)
  {
    const auto [valid, x1p] = lmm.predict_safe(x0, u0);
    // marker.points.emplace_back();
    // marker.points.back().x = x1p.first.theta();
    // marker.points.back().y = x1p.second;
    // marker.points.back().z = 0.0;
    if (valid)
    {
      const Eigen::Vector<double, DimX> v_err{ prx::TangentBetween(x1, x1p) };
      total_mean_error += v_err.norm();
      total_test++;
    }
    // }

    // x1_lmm_euclidean_marker_array.markers.push_back(marker);
  }
  total_mean_error = total_mean_error / total_test;
  DEBUG_VARS(total_mean_error)

  ros::spin();
  return 0;
}
