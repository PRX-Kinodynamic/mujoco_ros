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

#include <prx/utilities/math/lie_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <motion_planning/clustering.hpp>
#include "utils/dbg_utils.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <interface/gaussian_to_ellipse_marker.hpp>
#include <prx_models/SO2_system.hpp>
#include <motion_planning/nonlinear_clustering.hpp>
#include <prx_models/linear_mixture_model.hpp>

// Objective: Obtain a model x1 = f(x0,u0) from data (x0, u0, x1), assuming known and constant dt
// This is done in two steps: Clustering then linear system synthesis
// 1. Clustering: given data (x0, u0) -- assumed to be a lie manifold (composition of manifolds)
//                and given a *belief* on each data point -- (x0,u0)~N(0,Sigma), where Sigma is the belief
//                the objective is to create N clusters. N is NOT an input, it is found by the clustering algorithm
//                While only the pair (x0,u0) is being clustered, the clusters need to *remember* x1 for the next step
//                This assumes that x1 is *close-enough* to x0 or in other words, that clustering x1 is *equivalent* to
//                clustering x0. This may seem weird but is a consequence of assuming X to be a lie group: it is smooth
//                manifold after all.
// 2. Linear System Synthesis: This assumes a linear system of the form x1 = A * x0 + B * u0 (LTI), and the objective of
//                             the synthesis is to *learn* (aka compute) A and B for each cluster. This is done via
//                             Least Squares b = A'*z where A'=[A|B], z = [x0|u0], b=x1 (from [1]).
//                             However, if X is not an euclidean vector but another lie group, A*x0 is not correct.
//                             Instead, for x1 = f(x0, u0) do: dx1 = A * dx0 + B * du, and apply dx1 as: x1 = x0 (+) dx1
//                             where dx = x' (-) x0 and du = u' (-) u0; and (x', u') is the mean  of the cluster.
//
// [1] Dean, Sarah, Horia Mania, Nikolai Matni, Benjamin Recht, and Stephen Tu. "On the sample
//     complexity of the linear quadratic regulator." Foundations of Computational Mathematics
//     20, no. 4 (2020): 633-679.

using State = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
using Control = double;

static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };
static constexpr Eigen::Index DimZ{ DimX + DimU };

using Element = gtsam::ProductLieGroupV43<State, Control>;
using Data = std::tuple<State, Control, State>;

// using Element = gtsam::Pose2;
// using Data = std::pair<double, Element>;
using Zmatrix = Eigen::Matrix<double, DimX, DimZ>;
using Covariance = Eigen::Matrix<double, DimX + DimU, DimX + DimU>;
using IsotropicNM = gtsam::noiseModel::Isotropic;
using DiagonalNM = gtsam::noiseModel::Diagonal;
using Line = std::vector<std::string>;
using prx::utilities::convert_to;

int main(int argc, char** argv)
{
  const std::string node_name{ "SO2Clustering" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  visualization_msgs::MarkerArray data_marker_array, data_euclidean_marker_array;
  visualization_msgs::MarkerArray x1_marker_array, x1_euclidean_marker_array;
  visualization_msgs::MarkerArray x1_lmm_euclidean_marker_array;
  visualization_msgs::MarkerArray x1_predict_euclidean_marker_array;
  visualization_msgs::MarkerArray lie_ellipses_array, euclidean_ellipses_array;
  visualization_msgs::MarkerArray marker_controls, marker_controls_data;

  ros::Publisher markers_publisher{ nh.advertise<visualization_msgs::MarkerArray>("/clustering/lie/points/markers", 1,
                                                                                  true) };
  ros::Publisher markers_euclidean_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/euclidean/points/markers", 1, true) };
  ros::Publisher markers_euclidean_ellipses_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/euclidean/ellipses/markers", 1, true) };
  ros::Publisher markers_lie_ellipses_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/lie/ellipses/markers", 1, true) };
  ros::Publisher markers_controls_data_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/controls/data/markers", 1, true) };
  ros::Publisher markers_controls_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/controls/markers", 1, true) };
  ros::Publisher markers_x1_euclidean_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/x1/euclidean/markers", 1, true) };
  ros::Publisher markers_x1_predict_euclidean_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/x1/predict/euclidean/markers", 1, true) };
  ros::Publisher markers_x1_lmm_euclidean_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/x1/lmm/euclidean/markers", 1, true) };

  std::string data_file, output_dir;
  int batch_size, test_size;

  PARAM_SETUP(nh, data_file)
  PARAM_SETUP(nh, output_dir)
  PARAM_SETUP_WITH_DEFAULT(nh, batch_size, 1e3);
  PARAM_SETUP_WITH_DEFAULT(nh, test_size, batch_size * 0.1);

  DEBUG_VARS(batch_size, test_size)
  prx::utilities::csv_reader_t reader(data_file);

  Line line;
  // Data data;
  // Element element;
  Eigen::VectorXd sigmas(3);
  // sigmas << 0.1, 0.1, 0.1;
  sigmas << 1, 1, 0.1;
  auto nm = DiagonalNM::Sigmas(sigmas);

  std::vector<Data> input_data, test_set;
  // std::vector<Element> input_elements;

  // Files with lines: dt X0 X1 U0
  while (reader.next_valid_line(line))
  {
    const double theta0{ convert_to<double>(line[1]) };  // X0
    const double thdot0{ convert_to<double>(line[2]) };  // X0

    const double theta1{ convert_to<double>(line[3]) };  // X1
    const double thdot1{ convert_to<double>(line[4]) };  // X1

    const Control u0{ convert_to<double>(line[5]) };  // X1

    const State x0{ State(theta0, thdot0) };
    const State x1{ State(theta1, thdot1) };

    // const Element element(x0, u0);
    const Data data{ std::make_tuple(x0, u0, x1) };

    // input.push_back(element, data, nm);
    input_data.emplace_back(x0, u0, x1);
    // input_elements.emplace_back(x0, u0);
  }
  std::shuffle(input_data.begin(), input_data.end(), prx::global_generator);
  for (int i = 0; i < test_size; ++i)
  {
    test_set.push_back(input_data.back());
    input_data.pop_back();
  }

  motion_planning::nonlinear_cluster_values_t<Element, Data> input, output;
  using Cluster = motion_planning::cluster_t<Element, Data>;
  using Covariance = Eigen::Matrix3d;
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

        // const Element& element{ input_elements.back() };

        input_data.pop_back();
        // input_elements.pop_back();

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
      const Eigen::Matrix3d cov{ motion_planning::compute_cluster_covariance(element, clustered_elements) };

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
        // lmm.emplace(A, cov, element, i);
        const std::size_t& accepted{ cluster.idx };
        DEBUG_VARS(accepted, mean_error, ebA, ebB);
        input.clusters.emplace_back(cluster);
        input.clusters.back().locked = true;
        cluster_map.emplace(cluster.idx, std::make_tuple(input.clusters.back(), cov, Zmat));
      }
    }

    // converged = true;
  }

  DEBUG_VARS(cluster_map.size());

  // int max_steps{ -1 };
  // cluster_multiple_iterations(output, input, max_steps);

  // DEBUG_VARS(output.total_clustered.size(), output.data.size());
  // DEBUG_VARS(output.factor_graphs.size(), output.keys.size());

  // int kidx{ 0 };
  // for (auto& fg : output.factor_graphs)
  // {
  //   // DEBUG_VARS(kidx);
  //   fg.print("kidx ");
  // }
  // kidx = 0;
  // for (auto& k : output.keys)
  // {
  //   const std::string key{ gtsam::DefaultKeyFormatter(k) };
  //   DEBUG_VARS(kidx, key);
  //   kidx++;
  // }
  // const std::string clusters_filename{ output_dir + "/clusters.txt" };
  // const std::string values_filename{ output_dir + "/values.txt" };
  // const std::string covs_filename{ output_dir + "/covariances.txt" };
  // const std::string covs_Ais{ output_dir + "/linear_systems.txt" };
  // std::ofstream ofs_values(values_filename.c_str());
  // std::ofstream ofs_covs(covs_filename.c_str());
  // std::ofstream ofs_clusters(clusters_filename.c_str());
  // std::ofstream ofs_Ais(covs_Ais.c_str());

  interface::gaussian_params_t gauss_lie_params, gauss_euclidean_params;
  gauss_euclidean_params.frame_id = "world";

  prx_models::linear_mixture_model_t<State, Control> lmm;
  int element_to_debug{ 0 };

  int rejected{ 0 };
  // for (int i = 0; i < output.clusters.size(); ++i)
  for (auto& [key, cluster_covariance_Zmat] : cluster_map)
  {
    const Cluster cluster{ std::get<Cluster>(cluster_covariance_Zmat) };
    const Covariance cov{ std::get<Covariance>(cluster_covariance_Zmat) };
    const Zmatrix Zmat{ std::get<Zmatrix>(cluster_covariance_Zmat) };

    const std::size_t i{ cluster.idx };
    const std::size_t total_clustered{ cluster.total_clustered };
    // if (total_clustered < 2 * DimZ)
    // {
    //   const std::string rejected_key{ gtsam::DefaultKeyFormatter(cluster.key) };
    //   DEBUG_VARS(i, rejected_key, total_clustered)
    //   continue;
    // }

    const Element& element{ cluster.element };
    const std::vector<Element>& clustered_elements{ cluster.clustered_elements };

    // const Eigen::Matrix3d cov{ motion_planning::compute_cluster_covariance(element, clustered_elements) };

    if (i == element_to_debug)
    {
      DEBUG_VARS(cov)
    }
    const Eigen::Vector2d tg{ gtsam::traits<State>::Logmap(element.first) };
    const double th{ element.first.first.theta() };
    const double thdot{ element.first.second };
    const double u0{ element.second };
    const double ei_r{ std::fabs(std::cos(th)) };
    const double ei_g{ std::fabs(std::sin(thdot)) };
    const double ei_b{ std::fabs(std::cos(u0)) };
    visualization_msgs::Marker marker_pts{ ml4kp_bridge::create_marker(0.3, { 0.8, ei_r, ei_g, ei_b }) };
    visualization_msgs::Marker marker_euclidean_pts{ ml4kp_bridge::create_marker(0.3, { 0.8, ei_r, ei_g, ei_b }) };
    visualization_msgs::Marker marker_x1_euclidean_pts{ ml4kp_bridge::create_marker(0.3, { 0.8, ei_r, ei_g, ei_b }) };
    visualization_msgs::Marker marker_x1_predict_euclidean_pts{ ml4kp_bridge::create_marker(
        0.3, { 0.95, ei_r, ei_g, ei_b }) };

    marker_pts.id = marker_euclidean_pts.id = marker_x1_euclidean_pts.id = marker_x1_predict_euclidean_pts.id = i;
    marker_pts.type = marker_euclidean_pts.type = marker_x1_euclidean_pts.type = marker_x1_predict_euclidean_pts.type =
        visualization_msgs::Marker::POINTS;
    marker_pts.action = marker_euclidean_pts.action = marker_x1_euclidean_pts.action =
        marker_x1_predict_euclidean_pts.action = visualization_msgs::Marker::ADD;
    marker_pts.ns = marker_euclidean_pts.ns = marker_x1_euclidean_pts.ns = marker_x1_predict_euclidean_pts.ns =
        "cluster_" + convert_to<std::string>(i);
    marker_pts.header.frame_id = marker_euclidean_pts.header.frame_id = marker_x1_euclidean_pts.header.frame_id =
        marker_x1_predict_euclidean_pts.header.frame_id = "world";

    gauss_lie_params.idx = gauss_euclidean_params.idx = i;
    gauss_lie_params.ns = gauss_euclidean_params.ns = "cluster_" + convert_to<std::string>(i);

    gauss_euclidean_params.position = { tg[0], tg[1], u0 };
    gauss_euclidean_params.orientation = Eigen::Quaterniond::Identity();
    gauss_euclidean_params.color = { 0.8, ei_r, ei_g, ei_b };
    gauss_euclidean_params.cov_to_3Dellipse(cov, i == element_to_debug);
    if (i == element_to_debug)
    {
      DEBUG_VARS(gauss_euclidean_params.axis);
    }

    gauss_lie_params.position = { std::sin(th), std::cos(th), thdot };
    gauss_lie_params.orientation =
        prx::euler_to_rotation<Eigen::Quaterniond>(std::vector<double>({ prx::constants::pi / 2. }), "z");

    gauss_lie_params.color = { 0.8, ei_r, ei_g, ei_b };
    Eigen::Matrix2d cov2{ gauss_lie_params.marginal<2>(cov, 0) };
    gauss_lie_params.cov_to_3Dellipse(cov2);
    gauss_lie_params.axis[2] = 0.00001;

    marker_pts.pose.position.x = 0.0;
    marker_pts.pose.position.y = 0.0;
    marker_pts.pose.position.z = 0.0;

    marker_euclidean_pts.pose.position.x = 0.0;
    marker_euclidean_pts.pose.position.y = 0.0;
    marker_euclidean_pts.pose.position.z = 0.0;

    marker_x1_euclidean_pts.pose.position.x = 0.0;
    marker_x1_euclidean_pts.pose.position.y = 0.0;
    marker_x1_euclidean_pts.pose.position.z = 0.0;

    // std::vector<Eigen::Vector2d> all_xdots;
    // std::vector<Eigen::Vector3d> all_zts;

    // const State& xmean{ element.first };
    for (auto [x0, u0, x1] : cluster.data)
    {
      const double th0{ x0.first.theta() };
      const double thdot0{ x0.second };

      const double th1{ x1.first.theta() };
      const double thdot1{ x1.second };

      // const Eigen::Vector3d tg0{ prx::TangentBetween(element, Element(x0, u0)) };
      // const Element z0{ Element(x0, u0) };
      // const Eigen::Vector3d tg0{ gtsam::traits<Element>::Logmap(z0) };
      // const Eigen::Vector2d tg1{ gtsam::traits<State>::Logmap(x1) };

      marker_pts.points.emplace_back();
      marker_euclidean_pts.points.emplace_back();
      marker_x1_euclidean_pts.points.emplace_back();

      marker_pts.points.back().x = std::sin(th0);
      marker_pts.points.back().y = std::cos(th0);
      marker_pts.points.back().z = thdot0;

      marker_euclidean_pts.points.back().x = th0;
      marker_euclidean_pts.points.back().y = thdot0;
      marker_euclidean_pts.points.back().z = u0;

      marker_x1_euclidean_pts.points.back().x = th1;
      marker_x1_euclidean_pts.points.back().y = thdot1;
      marker_x1_euclidean_pts.points.back().z = 0.0;

      // all_zts.push_back(tg0);
      // all_xdots.push_back(tg1);

      // if (i == element_to_debug)
      // {
      //   LOG_VARS(tg0, tg1);
      // }
    }

    // using LGM = prx_models::linear_gaussian_model_t<State, Control>;
    // Eigen::Matrix<double, DimX, DimX + DimU> A{ LGM::LSE_AB(all_zts, all_xdots) };
    // const Eigen::Matrix<double, DimX, DimZ> A{ LGM::LSE_AB(cluster.data) };

    // LGM lgm(A, cov, element);

    // lgm.verbose(i == element_to_debug);
    // const auto [ebA, ebB] = lgm.compute_error_bounds(0.05, 20, 0.001);

    // if (i == element_to_debug)
    // {
    //   LOG_VARS(A);
    // }
    // double mean_error{ 0 };
    // // for (auto& [x0, u] : all_zts)
    // for (auto [x0, u0, x1] : cluster.data)
    // {
    //   const State x1p{ lgm.evaluate(x0, u0) };

    //   marker_x1_predict_euclidean_pts.points.emplace_back();
    //   marker_x1_predict_euclidean_pts.points.back().x = x1p.first.theta();
    //   marker_x1_predict_euclidean_pts.points.back().y = x1p.second;
    //   marker_x1_predict_euclidean_pts.points.back().z = 0.0;
    //   if (i == element_to_debug)
    //   {
    //     LOG_VARS(x1, x1p);
    //   }
    //   const Eigen::Vector<double, DimX> v_err{ prx::TangentBetween(x1, x1p) };
    //   mean_error += v_err.norm();
    // }
    // double total_clustered = cluster.data.size();
    // mean_error = mean_error / total_clustered;
    // if (mean_error > std::max(ebA, ebB))
    // {
    //   const std::size_t& i_rejected{ i };
    //   DEBUG_VARS(i_rejected, mean_error, ebA, ebB);
    // }
    // else
    // {
    lmm.emplace(Zmat, cov, element, i);
    //   const std::size_t& i_accepted{ i };
    //   DEBUG_VARS(i_accepted, mean_error, ebA, ebB);
    // }

    // gtsam::Values values;
    // values.insert(cluster.key, cluster.element);
    // const double fg_error{ cluster.factor_graph.error(values) };
    // DEBUG_VARS(i, element, total_clustered, fg_error, mean_error)

    const visualization_msgs::Marker euclidean_ellipse{ interface::gaussian_to_ellipse_marker(gauss_euclidean_params) };
    const visualization_msgs::Marker lie_ellipse{ interface::gaussian_to_ellipse_marker(gauss_lie_params) };

    euclidean_ellipses_array.markers.push_back(euclidean_ellipse);
    lie_ellipses_array.markers.push_back(lie_ellipse);
    data_marker_array.markers.push_back(marker_pts);
    data_euclidean_marker_array.markers.push_back(marker_euclidean_pts);
    x1_euclidean_marker_array.markers.push_back(marker_x1_euclidean_pts);
    x1_predict_euclidean_marker_array.markers.push_back(marker_x1_predict_euclidean_pts);
  }
  // ofs_clusters.close();
  markers_lie_ellipses_publisher.publish(lie_ellipses_array);
  markers_euclidean_publisher.publish(data_euclidean_marker_array);
  markers_publisher.publish(data_marker_array);
  markers_euclidean_ellipses_publisher.publish(euclidean_ellipses_array);
  markers_controls_publisher.publish(marker_controls);
  markers_controls_data_publisher.publish(marker_controls_data);
  markers_x1_euclidean_publisher.publish(x1_euclidean_marker_array);
  markers_x1_predict_euclidean_publisher.publish(x1_predict_euclidean_marker_array);

  ros::spinOnce();

  double total_mean_error{ 0. };
  double total_test{ 0. };

  // for (int i = 0; i < output.clusters.size(); ++i)
  // for (auto& [key, cluster_covariance] : cluster_map)
  visualization_msgs::Marker marker{ ml4kp_bridge::create_marker(0.3, { 0.95, 0.2, 0.2, 0.8 }) };
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  // marker.ns = "cluster_" + convert_to<std::string>(cluster.idx);
  marker.ns = "llm";
  marker.header.frame_id = "world";
  for (auto& [x0, u0, x1] : test_set)
  {
    // const Cluster cluster{ std::get<Cluster>(cluster_covariance) };

    // lmm.verbose(i == element_to_debug);

    // if (cluster.idx == element_to_debug)
    // {
    //   const Element z_mean{ cluster.element };
    //   LOG_VARS(z_mean);
    // }

    // for (auto [x0, u0, x1] : cluster.data)
    // {
    const State x1p{ lmm.predict(x0, u0) };
    marker.points.emplace_back();
    marker.points.back().x = x1p.first.theta();
    marker.points.back().y = x1p.second;
    marker.points.back().z = 0.0;

    // if (i == element_to_debug)
    // {
    //   LOG_VARS(x1, x1p);
    // }
    const Eigen::Vector<double, DimX> v_err{ prx::TangentBetween(x1, x1p) };
    total_mean_error += v_err.norm();
    total_test++;
    // }

    x1_lmm_euclidean_marker_array.markers.push_back(marker);
  }
  total_mean_error = total_mean_error / total_test;
  DEBUG_VARS(total_mean_error)

  markers_x1_lmm_euclidean_publisher.publish(x1_lmm_euclidean_marker_array);

  ros::spin();
  return 0;
}
