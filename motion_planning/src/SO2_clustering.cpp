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
#include <prx_models/SO2_system.hpp>

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

using Element = gtsam::ProductLieGroupV43<State, Control>;
using Data = std::tuple<State, Control, State>;

// using Element = gtsam::Pose2;
// using Data = std::pair<double, Element>;
using Covariance = Eigen::Matrix<double, DimX + DimU, DimX + DimU>;
using IsotropicNM = gtsam::noiseModel::Isotropic;
using DiagonalNM = gtsam::noiseModel::Diagonal;
using Line = std::vector<std::string>;
using prx::utilities::convert_to;

//  (1x1)   = (1x3)*(3x1) + (0x0)*(0x0)
// Duration = A*x + B*u
// using Xdot = Eigen::Vector<double, 3>;
// using Amat = Eigen::Matrix<double, 2, 3>;
// using Ele = Eigen::Matrix<double, 3, 1>;
// using Durations = Eigen::Vector<double, 1>;
// using Bmat = Eigen::Matrix<double, 3, 2>;

// Amat compute_linear_system(const std::vector<Element> all_zts, const std::vector<Xdot> all_xdots)
// {
//   Eigen::MatrixXd theta(all_zts.size(), 5);  // (Nx5)
//   Eigen::MatrixXd xs1(all_xdots.size(), 3);  // (Nx3)

//   for (int i = 0; i < all_zts.size(); ++i)
//   {
//     theta.row(i) = all_zts[i];
//     xs1.row(i) = all_xdots[i];
//   }
//   //                                       (5xN)              (Nx5)               (5xN)             (Nx3)
//   const Eigen::MatrixXd theta_estimate{ (theta.transpose() * theta).inverse() * theta.transpose() * xs1 };

//   // DEBUG_VARS(theta_estimate.transpose())
//   const Amat A{ theta_estimate.transpose() };
//   // const Eigen::Matrix<double, 3, 2> B{ theta_estimate.transpose().block<3, 2>(0, 3) };
//   return A;
// }

int main(int argc, char** argv)
{
  const std::string node_name{ "SO2Clustering" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  visualization_msgs::MarkerArray data_marker_array, data_euclidean_marker_array;
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

  std::string data_file, output_dir;

  PARAM_SETUP(nh, data_file)
  PARAM_SETUP(nh, output_dir)

  prx::utilities::csv_reader_t reader(data_file);

  motion_planning::cluster_in_out_t<Element, Data> input, output;

  Line line;
  // Data data;
  // Element element;
  Eigen::VectorXd sigmas(3);
  // sigmas << 0.1, 0.1, 0.1;
  sigmas << 1, 1, 1;
  auto nm = DiagonalNM::Sigmas(sigmas);

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

    const Element element(x0, u0);
    const Data data{ std::make_tuple(x0, u0, x1) };

    input.push_back(element, nm, data);
  }
  DEBUG_VARS(input.original_elements.size());

  int max_steps{ -1 };
  cluster_multiple_iterations(output, input, 7.815, max_steps);

  DEBUG_VARS(max_steps, output.values.size());
  DEBUG_VARS(output.total_clustered.size(), output.original_elements.size());

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

  int rejected{ 0 };
  for (int i = 0; i < output.original_elements.size(); ++i)
  {
    if (output.total_clustered[i] < 10)
    {
      // rejected++;
      continue;
    }

    const Element element{ output.values[i] };
    const Eigen::Matrix<double, 3, 3> R{
      dynamic_cast<gtsam::noiseModel::Gaussian*>(output.noise_models[i].get())->R()
    };
    const Eigen::Matrix<double, 3, 3> cov{ (R.transpose() * R).inverse() };

    if (i == 0)
    {
      DEBUG_VARS(cov)
    }
    const Eigen::Vector2d tg{ gtsam::traits<State>::Logmap(element.first) };
    const double th{ element.first.first.theta() };
    const double thdot{ element.first.second };
    const double u0{ element.second };
    const double ei_r{ std::fabs(std::sin(th)) };
    const double ei_g{ std::fabs(std::sin(thdot)) };
    const double ei_b{ std::fabs(std::sin(u0)) };
    visualization_msgs::Marker marker_pts{ ml4kp_bridge::create_marker(0.1, { 0.8, ei_r, ei_g, ei_b }) };
    visualization_msgs::Marker marker_euclidean_pts{ ml4kp_bridge::create_marker(0.1, { 0.8, ei_r, ei_g, ei_b }) };

    marker_pts.id = marker_euclidean_pts.id = i;
    marker_pts.type = marker_euclidean_pts.type = visualization_msgs::Marker::POINTS;
    marker_pts.action = marker_euclidean_pts.action = visualization_msgs::Marker::ADD;
    marker_pts.ns = marker_euclidean_pts.ns = "cluster_" + convert_to<std::string>(i);
    marker_pts.header.frame_id = marker_euclidean_pts.header.frame_id = "world";

    gauss_lie_params.idx = gauss_euclidean_params.idx = i;
    gauss_lie_params.ns = gauss_euclidean_params.ns = "cluster_" + convert_to<std::string>(i);

    gauss_euclidean_params.position = { tg[0], tg[1], u0 };
    gauss_euclidean_params.orientation = Eigen::Quaterniond::Identity();
    gauss_euclidean_params.color = { 0.8, ei_r, ei_g, ei_b };
    gauss_euclidean_params.cov_to_3Dellipse(cov);

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
    for (auto [x0, u0, x1] : output.original_elements[i])
    {
      const double th0{ x0.first.theta() };
      const double thdot0{ x0.second };
      const Eigen::Vector2d tg0{ gtsam::traits<State>::Logmap(x0) };

      marker_pts.points.emplace_back();
      marker_euclidean_pts.points.emplace_back();

      marker_pts.points.back().x = std::sin(th0);
      marker_pts.points.back().y = std::cos(th0);
      marker_pts.points.back().z = thdot0;

      marker_euclidean_pts.points.back().x = tg0[0];
      marker_euclidean_pts.points.back().y = tg0[1];
      marker_euclidean_pts.points.back().z = u0;
    }

    const visualization_msgs::Marker euclidean_ellipse{ interface::gaussian_to_ellipse_marker(gauss_euclidean_params) };
    const visualization_msgs::Marker lie_ellipse{ interface::gaussian_to_ellipse_marker(gauss_lie_params) };

    euclidean_ellipses_array.markers.push_back(euclidean_ellipse);
    lie_ellipses_array.markers.push_back(lie_ellipse);
    data_marker_array.markers.push_back(marker_pts);
    data_euclidean_marker_array.markers.push_back(marker_euclidean_pts);
    ////////////////////////////////////////
    // visualization_msgs::Marker marker_ctrl_data;
    // marker_ctrl_data.type = visualization_msgs::Marker::POINTS;
    // marker_ctrl_data.action = visualization_msgs::Marker::ADD;
    // marker_ctrl_data.header.frame_id = "world";

    // if (output.total_clustered[i] < 10)
    // {
    //   rejected++;
    //   continue;
    // }
    // const Eigen::MatrixXd R{ dynamic_cast<gtsam::noiseModel::Gaussian*>(output.noise_models[i].get())->R() };

    // ofs_values << i << " ";
    // ofs_values << output.total_clustered[i] << " ";
    // prx::to_stream(ofs_values, output.values[i]);
    // ofs_values << "\n";

    // std::stringstream strstr;
    // strstr << std::setfill('0') << std::setw(5) << convert_to<std::string>(i);

    // const std::string filename{ output_dir + "/cluster_" + strstr.str() + ".txt" };
    // std::ofstream ofs(filename.c_str());
    // std::vector<Element> clustered_elements;

    // std::vector<Xdot> all_xdots;
    // for (auto ei : output.original_elements[i])
    // {
    //   marker.points.emplace_back();
    //   marker_ctrl_data.points.emplace_back();

    //   marker.points.back().x = ei[0];
    //   marker.points.back().y = ei[1];
    //   marker.points.back().z = ei[2];

    //   marker_ctrl_data.points.back().x = ei[6];
    //   marker_ctrl_data.points.back().y = ei[7];
    //   marker_ctrl_data.points.back().z = 0.0;

    //   clustered_elements.emplace_back(ei[0], ei[1], ei[2], ei[6], ei[7]);
    //   ofs << ei.head(3).transpose() << " ";
    //   ofs << ei.tail(2).transpose() << " ";
    //   ofs << "\n";

    //   all_xdots.emplace_back(ei[3], ei[4], ei[5]);
    // }
    // const Eigen::MatrixXd cov{ (R.transpose() * R).inverse() };

    // auto A_xdot = cov.block<3, 3>(0, 0);
    // auto B_xdot = cov.block<3, 2>(0, 3);
    // auto C_xdot = cov.block<2, 2>(3, 3);

    // auto A_ctrl = cov.block<2, 2>(3, 3);
    // auto B_ctrl = cov.block<2, 3>(3, 0);
    // auto C_ctrl = cov.block<3, 3>(0, 0);

    // Eigen::Matrix3d cov_xdot{ A_xdot - B_xdot * C_xdot.inverse() * B_xdot.transpose() };
    // Eigen::Matrix2d cov_ctrl{ A_ctrl - B_ctrl * C_ctrl.inverse() * B_ctrl.transpose() };

    // ofs_covs << i << "\n";
    // ofs_covs << cov << "\n";

    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_xdot(cov_xdot);
    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_ctrl(cov_ctrl);
    // Eigen::VectorXd D_xdot = es_xdot.eigenvalues();
    // Eigen::VectorXd D_ctrl = es_ctrl.eigenvalues();
    // Eigen::Matrix<double, 3, 3> V_xdot = es_xdot.eigenvectors();
    // Eigen::Matrix<double, 2, 2> V_ctrl = es_ctrl.eigenvectors();

    // marker.id = i;
    // marker_ctrl_data.id = i;
    // marker_params.idx = i;
    // marker_ctrl_params.idx = i;

    // marker.ns = "Data_" + strstr.str();
    // marker_params.ns = "Xdot_" + strstr.str();
    // marker_ctrl_data.ns = "Data_" + strstr.str();
    // marker_ctrl_params.ns = "Ctrl_" + strstr.str();

    // const double xdot{ output.values[i][0] };
    // const double ydot{ output.values[i][1] };
    // const double thdot{ output.values[i][2] };
    // const double u0{ output.values[i][3] };
    // const double u1{ output.values[i][4] };
    // marker.color.a = 1.0;
    // marker.color.r = std::fabs(std::sin(xdot));
    // marker.color.g = std::fabs(std::sin(ydot));
    // marker.color.b = std::fabs(std::sin(thdot));

    // marker_ctrl_data.color.a = 1.0;
    // marker_ctrl_data.color.r = std::abs(u0);
    // marker_ctrl_data.color.g = std::abs(u1);
    // marker_ctrl_data.color.b = .00;

    // marker.scale.x = 0.1;  // is point width,
    // marker.scale.y = 0.1;  // is point height
    // marker.scale.z = 0.1;  // is point height
    // marker_ctrl_data.scale.x = 0.1;
    // marker_ctrl_data.scale.y = 0.1;
    // marker_ctrl_data.scale.z = 0.1;

    // marker_params.position[0] = xdot;
    // marker_params.position[1] = ydot;
    // marker_params.position[2] = thdot;
    // marker_ctrl_params.position[0] = u0;
    // marker_ctrl_params.position[1] = u1;
    // marker_ctrl_params.position[2] = 0.0;

    // marker.pose.position.x = 0.0;
    // marker.pose.position.y = 0.0;
    // marker.pose.position.z = 0.0;
    // marker_ctrl_data.pose.position.x = 0.0;
    // marker_ctrl_data.pose.position.y = 0.0;
    // marker_ctrl_data.pose.position.z = 0.0;

    // Eigen::Matrix3d u_mat{ Eigen::Matrix3d::Identity() };
    // u_mat.block<2, 2>(0, 0) = V_ctrl;
    // marker_params.orientation = Eigen::Quaterniond(V_xdot);
    // marker_ctrl_params.orientation = Eigen::Quaterniond(u_mat);
    // // DEBUG_VARS(marker_params.orientation)
    // marker.pose.orientation.w = 1.0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker_ctrl_data.pose.orientation = marker.pose.orientation;

    // marker_params.color = Eigen::Vector4d(0.8, std::sin(xdot), std::sin(ydot), std::sin(thdot)).cwiseAbs();
    // marker_ctrl_params.color = Eigen::Vector4d(0.5, std::abs(u0), std::abs(u1), 0.0);

    // marker_params.axis = D_xdot;
    // marker_ctrl_params.axis[2] = 0.01;
    // marker_ctrl_params.axis.head(2) = D_ctrl;
    // visualization_msgs::Marker marker_ellipse{ interface::gaussian_to_ellipse_marker(marker_params) };
    // visualization_msgs::Marker marker_ellipse_ctrl{ interface::gaussian_to_ellipse_marker(marker_ctrl_params) };
    // marker_msg.markers.push_back(marker);
    // marker_ellipses.markers.push_back(marker_ellipse);
    // marker_controls.markers.push_back(marker_ellipse_ctrl);
    // marker_controls_data.markers.push_back(marker_ctrl_data);

    // const Amat Ai{ compute_linear_system(clustered_elements, all_xdots) };
    // ofs_Ais << i << " ";
    // ofs_Ais << Ai.reshaped().transpose() << "\n";
  }
  // ofs_clusters.close();
  markers_lie_ellipses_publisher.publish(lie_ellipses_array);
  markers_euclidean_publisher.publish(data_euclidean_marker_array);
  markers_publisher.publish(data_marker_array);
  markers_euclidean_ellipses_publisher.publish(euclidean_ellipses_array);
  markers_controls_publisher.publish(marker_controls);
  markers_controls_data_publisher.publish(marker_controls_data);

  // ofs_values.close();
  // ofs_covs.close();
  // ofs_clusters.close();
  // ofs_Ais.close();
  // DEBUG_VARS(rejected);
  // PRINT_MSG("DONE!");
  ros::spin();
  return 0;
}
