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

// using XDdot = Eigen::Vector<double, 3>;
using Control = Eigen::Vector<double, 2>;
using Element = Eigen::Vector<double, 5>;
using Data = Eigen::Vector<double, 8>;

// using Element = gtsam::Pose2;
// using Data = std::pair<double, Element>;
using Covariance = Eigen::Matrix<double, 5, 5>;
using IsotropicNM = gtsam::noiseModel::Isotropic;
using DiagonalNM = gtsam::noiseModel::Diagonal;
using Line = std::vector<std::string>;
using prx::utilities::convert_to;

//  (1x1)   = (1x3)*(3x1) + (0x0)*(0x0)
// Duration = A*x + B*u
using Xdot = Eigen::Vector<double, 3>;
using Amat = Eigen::Matrix<double, 3, 5>;
// using Ele = Eigen::Matrix<double, 3, 1>;
// using Durations = Eigen::Vector<double, 1>;
// using Bmat = Eigen::Matrix<double, 3, 2>;

Amat compute_linear_system(const std::vector<Element> all_zts, const std::vector<Xdot> all_xdots)
{
  Eigen::MatrixXd theta(all_zts.size(), 5);  // (Nx5)
  Eigen::MatrixXd xs1(all_xdots.size(), 3);  // (Nx3)

  for (int i = 0; i < all_zts.size(); ++i)
  {
    theta.row(i) = all_zts[i];
    xs1.row(i) = all_xdots[i];
  }
  //                                       (5xN)              (Nx5)               (5xN)             (Nx3)
  const Eigen::MatrixXd theta_estimate{ (theta.transpose() * theta).inverse() * theta.transpose() * xs1 };

  // DEBUG_VARS(theta_estimate.transpose())
  const Amat A{ theta_estimate.transpose() };
  // const Eigen::Matrix<double, 3, 2> B{ theta_estimate.transpose().block<3, 2>(0, 3) };
  return A;
}

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrClustering" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  visualization_msgs::MarkerArray marker_msg, marker_ellipses, marker_controls, marker_controls_data;

  ros::Publisher markers_publisher{ nh.advertise<visualization_msgs::MarkerArray>("/clustering/points/markers", 1,
                                                                                  true) };
  ros::Publisher markers_ellipses_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/ellipses/markers", 1, true) };
  ros::Publisher markers_controls_data_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/controls/data/markers", 1, true) };
  ros::Publisher markers_controls_publisher{ nh.advertise<visualization_msgs::MarkerArray>(
      "/clustering/controls/markers", 1, true) };

  std::string data_file, output_dir;

  PARAM_SETUP(nh, data_file)
  PARAM_SETUP(nh, output_dir)

  prx::utilities::csv_reader_t reader(data_file);

  motion_planning::cluster_in_out_t<Element, Data> input, output;

  // cout << "The orthogonal matrix U is:" << endl << schur.matrixU() << endl;
  // cout << "The quasi-triangular matrix T is:" << endl << schur.matrixT() << endl << endl;

  Line line;
  Data data;
  Element element;
  // auto nm = IsotropicNM::Sigma(8, 0.10);
  Eigen::VectorXd sigmas(5);
  // sigmas << 0.05, 0.05, 0.05, 0.05, 0.05;
  sigmas << 0.1, 0.1, 0.1, 0.1, 0.1;
  auto nm = DiagonalNM::Sigmas(sigmas);

  // ----------------: 0  1  2   3  4  5   6    7     8     9     10    11     12   13 14
  // Files with lines: ti x0 y0 th0 x1 y1 th1 xdot0 ydot0 thdot0 xdot1 ydot1 thdot1 u0 u1
  while (reader.next_valid_line(line))
  {
    // { LCA::convert_to<Xdot>(xd0_str) };
    // ti = convert_to<double>(line[0]);
    data[0] = convert_to<double>(line[7]);  // Q0
    data[1] = convert_to<double>(line[8]);  // Q0
    data[2] = convert_to<double>(line[9]);  // Q0

    data[3] = convert_to<double>(line[10]);  // Q1
    data[4] = convert_to<double>(line[11]);  // Q1
    data[5] = convert_to<double>(line[12]);  // Q1

    data[6] = convert_to<double>(line[13]);  // control
    data[7] = convert_to<double>(line[14]);  // control

    element[0] = convert_to<double>(line[7]);  // Q0
    element[1] = convert_to<double>(line[8]);  // Q0
    element[2] = convert_to<double>(line[9]);  // Q0

    element[3] = convert_to<double>(line[13]);  // control
    element[4] = convert_to<double>(line[14]);  // control

    input.push_back(element, nm, data);
    // input.values.push_back(element);
    // input.noise_models.push_back(IsotropicNM::Sigma(8, .10));
    // input.total_clustered.push_back(1);
  }
  DEBUG_VARS(input.original_elements.size());

  int max_steps{ -1 };
  cluster_multiple_iterations(output, input, 7.815, max_steps);

  DEBUG_VARS(max_steps, output.values.size());
  DEBUG_VARS(output.total_clustered.size(), output.original_elements.size());

  const std::string clusters_filename{ output_dir + "/clusters.txt" };
  const std::string values_filename{ output_dir + "/values.txt" };
  const std::string covs_filename{ output_dir + "/covariances.txt" };
  const std::string covs_Ais{ output_dir + "/linear_systems.txt" };
  std::ofstream ofs_values(values_filename.c_str());
  std::ofstream ofs_covs(covs_filename.c_str());
  std::ofstream ofs_clusters(clusters_filename.c_str());
  std::ofstream ofs_Ais(covs_Ais.c_str());

  interface::gaussian_params_t marker_params, marker_ctrl_params;

  int rejected{ 0 };
  for (int i = 0; i < output.original_elements.size(); ++i)
  {
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_ctrl_data;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "world";
    marker_ctrl_data.type = visualization_msgs::Marker::POINTS;
    marker_ctrl_data.action = visualization_msgs::Marker::ADD;
    marker_ctrl_data.header.frame_id = "world";

    if (output.total_clustered[i] < 10)
    {
      rejected++;
      continue;
    }
    const Eigen::MatrixXd R{ dynamic_cast<gtsam::noiseModel::Gaussian*>(output.noise_models[i].get())->R() };

    // DEBUG_VARS(i)
    // DEBUG_VARS(R)
    ofs_values << i << " ";
    ofs_values << output.total_clustered[i] << " ";
    ofs_values << output.values[i].transpose() << " ";
    ofs_values << "\n";
    // ofs_values << output.values[i].y() << " ";
    // ofs_values << output.values[i].theta() << " ";
    // ofs_values << "\n";

    std::stringstream strstr;
    strstr << std::setfill('0') << std::setw(5) << convert_to<std::string>(i);
    // std::stringstream strstr;
    // strstr << output_dir << "/cluster_" << std::setw(5) << std::setfill('0') << i << ".txt";
    const std::string filename{ output_dir + "/cluster_" + strstr.str() + ".txt" };
    std::ofstream ofs(filename.c_str());
    // ofs << "# Clustered_element " << "\n";
    // // ofs << output.values[i].transpose() << "\n";
    // std::vector<Eigen::Vector3d> all_zts;
    // std::vector<Durations> all_durations;
    // double min_duration{ std::numeric_limits<double>::max() };
    // double max_duration{ 0.0 };
    std::vector<Element> clustered_elements;

    // const std::vector<Element> all_zts;
    std::vector<Xdot> all_xdots;
    for (auto ei : output.original_elements[i])
    {
      marker.points.emplace_back();
      marker_ctrl_data.points.emplace_back();

      marker.points.back().x = ei[0];
      marker.points.back().y = ei[1];
      marker.points.back().z = ei[2];

      marker_ctrl_data.points.back().x = ei[6];
      marker_ctrl_data.points.back().y = ei[7];
      marker_ctrl_data.points.back().z = 0.0;

      clustered_elements.emplace_back(ei[0], ei[1], ei[2], ei[6], ei[7]);
      ofs << ei.head(3).transpose() << " ";
      ofs << ei.tail(2).transpose() << " ";
      ofs << "\n";

      all_xdots.emplace_back(ei[3], ei[4], ei[5]);
      //   ofs << ei.second.x() << " ";
      //   ofs << ei.second.y() << " ";
      //   ofs << ei.second.theta() << " ";
      //   ofs << "\n";
    }
    const Eigen::MatrixXd cov{ (R.transpose() * R).inverse() };
    // Eigen::MatrixXd cov{ Eigen::Matrix<double, 5, 5>::Identity() };
    // motion_planning::compute_cluster_covariance(cov, output.values[i], clustered_elements, 0);

    // const double avg_duration{ accum / output.original_elements[i].size() };
    // // R is upper triangular
    // ofs_clusters << R(0, 0) << " ";
    // ofs_clusters << R(0, 1) << " ";
    // ofs_clusters << R(0, 2) << " ";
    // ofs_clusters << R(1, 1) << " ";
    // ofs_clusters << R(1, 2) << " ";
    // ofs_clusters << R(2, 2) << " ";
    // // ofs_clusters << output.values[i].x() << " ";
    // // ofs_clusters << output.values[i].y() << " ";
    // // ofs_clusters << output.values[i].theta() << " ";
    // ofs_clusters << min_duration << " ";
    // ofs_clusters << max_duration << " ";
    // ofs_clusters << avg_duration << " ";
    // ofs_clusters << "\n";

    auto A_xdot = cov.block<3, 3>(0, 0);
    auto B_xdot = cov.block<3, 2>(0, 3);
    auto C_xdot = cov.block<2, 2>(3, 3);

    auto A_ctrl = cov.block<2, 2>(3, 3);
    auto B_ctrl = cov.block<2, 3>(3, 0);
    auto C_ctrl = cov.block<3, 3>(0, 0);

    Eigen::Matrix3d cov_xdot{ A_xdot - B_xdot * C_xdot.inverse() * B_xdot.transpose() };
    Eigen::Matrix2d cov_ctrl{ A_ctrl - B_ctrl * C_ctrl.inverse() * B_ctrl.transpose() };

    ofs_covs << i << "\n";
    ofs_covs << cov << "\n";
    // DEBUG_VARS(cov)
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_xdot(cov_xdot);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_ctrl(cov_ctrl);
    Eigen::VectorXd D_xdot = es_xdot.eigenvalues();
    Eigen::VectorXd D_ctrl = es_ctrl.eigenvalues();
    Eigen::Matrix<double, 3, 3> V_xdot = es_xdot.eigenvectors();
    Eigen::Matrix<double, 2, 2> V_ctrl = es_ctrl.eigenvectors();
    // Eigen::Quaterniond q_cov(V);
    // DEBUG_VARS(D.transpose())
    // DEBUG_VARS(V)

    marker.id = i;
    marker_ctrl_data.id = i;
    marker_params.idx = i;
    marker_ctrl_params.idx = i;

    marker.ns = "Data_" + strstr.str();
    marker_params.ns = "Xdot_" + strstr.str();
    marker_ctrl_data.ns = "Data_" + strstr.str();
    marker_ctrl_params.ns = "Ctrl_" + strstr.str();

    const double xdot{ output.values[i][0] };
    const double ydot{ output.values[i][1] };
    const double thdot{ output.values[i][2] };
    const double u0{ output.values[i][3] };
    const double u1{ output.values[i][4] };
    marker.color.a = 1.0;
    marker.color.r = std::fabs(std::sin(xdot));
    marker.color.g = std::fabs(std::sin(ydot));
    marker.color.b = std::fabs(std::sin(thdot));

    marker_ctrl_data.color.a = 1.0;
    marker_ctrl_data.color.r = std::abs(u0);
    marker_ctrl_data.color.g = std::abs(u1);
    marker_ctrl_data.color.b = .00;

    marker.scale.x = 0.1;  // is point width,
    marker.scale.y = 0.1;  // is point height
    marker.scale.z = 0.1;  // is point height
    marker_ctrl_data.scale.x = 0.1;
    marker_ctrl_data.scale.y = 0.1;
    marker_ctrl_data.scale.z = 0.1;

    marker_params.position[0] = xdot;
    marker_params.position[1] = ydot;
    marker_params.position[2] = thdot;
    marker_ctrl_params.position[0] = u0;
    marker_ctrl_params.position[1] = u1;
    marker_ctrl_params.position[2] = 0.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker_ctrl_data.pose.position.x = 0.0;
    marker_ctrl_data.pose.position.y = 0.0;
    marker_ctrl_data.pose.position.z = 0.0;

    Eigen::Matrix3d u_mat{ Eigen::Matrix3d::Identity() };
    u_mat.block<2, 2>(0, 0) = V_ctrl;
    marker_params.orientation = Eigen::Quaterniond(V_xdot);
    marker_ctrl_params.orientation = Eigen::Quaterniond(u_mat);
    // DEBUG_VARS(marker_params.orientation)
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker_ctrl_data.pose.orientation = marker.pose.orientation;

    marker_params.color = Eigen::Vector4d(0.8, std::sin(xdot), std::sin(ydot), std::sin(thdot)).cwiseAbs();
    marker_ctrl_params.color = Eigen::Vector4d(0.5, std::abs(u0), std::abs(u1), 0.0);

    // marker_params.color[0] = std::min(1.0, (0.1 + min_duration) / 20.0);  // alpha first: ARGB
    marker_params.axis = D_xdot;
    marker_ctrl_params.axis[2] = 0.01;
    marker_ctrl_params.axis.head(2) = D_ctrl;
    visualization_msgs::Marker marker_ellipse{ interface::gaussian_to_ellipse_marker(marker_params) };
    visualization_msgs::Marker marker_ellipse_ctrl{ interface::gaussian_to_ellipse_marker(marker_ctrl_params) };
    marker_msg.markers.push_back(marker);
    marker_ellipses.markers.push_back(marker_ellipse);
    marker_controls.markers.push_back(marker_ellipse_ctrl);
    marker_controls_data.markers.push_back(marker_ctrl_data);

    const Amat Ai{ compute_linear_system(clustered_elements, all_xdots) };
    ofs_Ais << i << " ";
    ofs_Ais << Ai.reshaped().transpose() << "\n";
    // const Amat Ai{ compute_linear_system(all_zts, all_durations) };
    // DEBUG_VARS(i, all_zts.size(), Ai);
  }
  ofs_clusters.close();
  markers_publisher.publish(marker_msg);
  markers_ellipses_publisher.publish(marker_ellipses);
  markers_controls_publisher.publish(marker_controls);
  markers_controls_data_publisher.publish(marker_controls_data);

  ofs_values.close();
  ofs_covs.close();
  ofs_clusters.close();
  ofs_Ais.close();
  DEBUG_VARS(rejected);
  PRINT_MSG("DONE!");
  ros::spin();
  return 0;
}
