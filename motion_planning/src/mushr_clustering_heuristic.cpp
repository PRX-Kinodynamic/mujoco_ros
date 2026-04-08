#include <limits>
#include <prx/simulation/system.hpp>
#include <thread>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
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

using Element = gtsam::Pose2;
using Data = std::pair<double, Element>;
using Covariance = Eigen::Matrix<double, 3, 3>;
using IsotropicNM = gtsam::noiseModel::Isotropic;
using DiagonalNM = gtsam::noiseModel::Diagonal;
using Line = std::vector<std::string>;
using prx::utilities::convert_to;

//  (1x1)   = (1x3)*(3x1) + (0x0)*(0x0)
// Duration = A*x + B*u
using Amat = Eigen::Matrix<double, 1, 3>;
using ExpPose = Eigen::Matrix<double, 3, 1>;
using Durations = Eigen::Vector<double, 1>;
// using Bmat = Eigen::Matrix<double, 3, 2>;

Amat compute_linear_system(const std::vector<ExpPose> all_zts, const std::vector<Durations> all_durations)
{
  Eigen::MatrixXd theta(all_zts.size(), 3);      // (Nx3)
  Eigen::MatrixXd xs1(all_durations.size(), 1);  // (Nx1)

  for (int i = 0; i < all_zts.size(); ++i)
  {
    theta.row(i) = all_zts[i];
    xs1.row(i) = all_durations[i];
  }
  //                                    ( (3xN)              (Nx3))               (3xN)             (Nx1)
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

  visualization_msgs::MarkerArray marker_msg;

  ros::Publisher markers_publisher{ nh.advertise<visualization_msgs::MarkerArray>("/clustering/markers", 1, true) };

  std::string data_file, output_dir;

  PARAM_SETUP(nh, data_file)
  PARAM_SETUP(nh, output_dir)

  prx::utilities::csv_reader_t reader(data_file);

  motion_planning::cluster_in_out_t<Element, Data> input, output;

  // cout << "The orthogonal matrix U is:" << endl << schur.matrixU() << endl;
  // cout << "The quasi-triangular matrix T is:" << endl << schur.matrixT() << endl << endl;

  Line line;
  // Data data;
  // auto nm = IsotropicNM::Sigma(8, 0.10);
  Eigen::VectorXd sigmas(3);
  sigmas << 0.1, 0.1, 0.1;
  auto nm = DiagonalNM::Sigmas(sigmas);
  while (reader.next_valid_line(line))
  {
    // { LCA::convert_to<Xdot>(xd0_str) };

    const double x{ convert_to<double>(line[0]) };
    const double y{ convert_to<double>(line[1]) };
    const double theta{ convert_to<double>(line[2]) };

    const Element element(x, y, theta);
    const Data data{ convert_to<double>(line[4]), element };

    input.push_back(element, nm, data);
  }
  DEBUG_VARS(input.original_elements.size());

  int max_steps{ -1 };
  cluster_multiple_iterations(output, input, 7.815, max_steps);

  DEBUG_VARS(max_steps, output.values.size());
  DEBUG_VARS(output.total_clustered.size(), output.original_elements.size());

  const std::string clusters_filename{ output_dir + "/clusters.txt" };
  const std::string values_filename{ output_dir + "/values.txt" };
  const std::string covs_filename{ output_dir + "/covariances.txt" };
  std::ofstream ofs_values(values_filename.c_str());
  std::ofstream ofs_covs(covs_filename.c_str());
  std::ofstream ofs_clusters(clusters_filename.c_str());
  for (int i = 0; i < output.original_elements.size(); ++i)
  {
    const Eigen::MatrixXd R{ dynamic_cast<gtsam::noiseModel::Gaussian*>(output.noise_models[i].get())->R() };
    ofs_values << i << " ";
    ofs_values << output.total_clustered[i] << " ";
    ofs_values << output.values[i].x() << " ";
    ofs_values << output.values[i].y() << " ";
    ofs_values << output.values[i].theta() << " ";
    ofs_values << "\n";
    ofs_covs << i << "\n";
    ofs_covs << R << "\n";

    std::stringstream strstr;
    strstr << output_dir << "/cluster_" << std::setw(6) << std::setfill('0') << i << ".txt";

    std::ofstream ofs(strstr.str().c_str());
    ofs << "# Clustered_element " << "\n";
    // ofs << output.values[i].transpose() << "\n";
    std::vector<Eigen::Vector3d> all_zts;
    std::vector<Durations> all_durations;
    double min_duration{ std::numeric_limits<double>::max() };
    double max_duration{ 0.0 };
    double accum{ 0.0 };
    for (auto ei : output.original_elements[i])
    {
      ofs << ei.first << " ";
      ofs << ei.second.x() << " ";
      ofs << ei.second.y() << " ";
      ofs << ei.second.theta() << " ";
      ofs << "\n";
      min_duration = std::min(min_duration, ei.first);
      max_duration = std::max(max_duration, ei.first);
      accum += ei.first;
      all_durations.emplace_back(ei.first);
      all_zts.push_back(gtsam::traits<gtsam::Pose2>::Logmap(ei.second));
    }
    const double avg_duration{ accum / output.original_elements[i].size() };
    // R is upper triangular
    ofs_clusters << R(0, 0) << " ";
    ofs_clusters << R(0, 1) << " ";
    ofs_clusters << R(0, 2) << " ";
    ofs_clusters << R(1, 1) << " ";
    ofs_clusters << R(1, 2) << " ";
    ofs_clusters << R(2, 2) << " ";
    ofs_clusters << output.values[i].x() << " ";
    ofs_clusters << output.values[i].y() << " ";
    ofs_clusters << output.values[i].theta() << " ";
    ofs_clusters << min_duration << " ";
    ofs_clusters << max_duration << " ";
    ofs_clusters << avg_duration << " ";
    ofs_clusters << "\n";

    Eigen::MatrixXd cov{ (R.transpose() * R).inverse() };
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
    Eigen::VectorXd D = es.eigenvalues();
    Eigen::Matrix<double, 3, 3> V = es.eigenvectors();
    Eigen::Quaterniond q_cov(V);
    // DEBUG_VARS(D.transpose())
    DEBUG_VARS(q_cov)

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "clustering";
    marker.id = i;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = output.values[i].x();
    marker.pose.position.y = output.values[i].y();
    marker.pose.position.z = output.values[i].theta();
    marker.pose.orientation.x = q_cov.x();
    marker.pose.orientation.y = q_cov.y();
    marker.pose.orientation.z = q_cov.z();
    marker.pose.orientation.w = q_cov.w();
    marker.color.a = std::min(1.0, (0.1 + min_duration) / 20.0);  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 7.815 * std::sqrt(D[0]);  // Ros needs diameter, prx in rad
    marker.scale.y = 7.815 * std::sqrt(D[1]);  // Ros needs diameter, prx in rad
    marker.scale.z = 7.815 * std::sqrt(D[2]);
    marker_msg.markers.push_back(marker);
    // const Amat Ai{ compute_linear_system(all_zts, all_durations) };
    // DEBUG_VARS(i, all_zts.size(), Ai);
  }
  ofs_clusters.close();
  markers_publisher.publish(marker_msg);
  ros::spin();
  return 0;
}
