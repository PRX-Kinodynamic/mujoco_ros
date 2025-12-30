#include <gtest/gtest.h>
#include <ros/init.h>
#include <estimation/camera_calibration_factors.hpp>
#include <gtsam/base/numericalDerivative.h>

TEST(TestOnGroundFactors, testOnGroundFactorsDerivatives)
{
  using SE3 = gtsam::Pose3;
  using OnGroundFactor = estimation::on_ground_factor_t;

  const double tolerance{ 1e-3 };

  const SE3 x{ SE3(gtsam::Rot3(), Eigen::Vector3d(1, 2, 3)) };
  const SE3 w{ SE3(gtsam::Rot3(0.707, 0.0, 0.0, 0.707), Eigen::Vector3d(-1, -2, 0)) };

  // Check jacobians
  Eigen::MatrixXd actualHw, expectedHw;
  Eigen::MatrixXd actualHx, expectedHx;
  // Eigen::MatrixXd actualHdt, expectedHdt;
  // Eigen::MatrixXd actualHu, expectedHu;

  const OnGroundFactor factor(0, 1, nullptr);

  std::function<gtsam::Vector(const SE3&, const SE3&)> err_proxy = [&factor](const SE3& _w, const SE3& _x) {
    return factor.evaluateError(_w, _x);
  };

  factor.evaluateError(w, x, actualHw, actualHx);
  expectedHw = gtsam::numericalDerivative21(err_proxy, w, x);
  expectedHx = gtsam::numericalDerivative22(err_proxy, w, x);

  PRX_DBG_VARS(expectedHw);
  PRX_DBG_VARS(actualHw);

  PRX_DBG_VARS(expectedHx);
  PRX_DBG_VARS(actualHx);

  PRX_DBG_VARS(expectedHw - actualHw);
  PRX_DBG_VARS(expectedHx - actualHx);

  const bool expectedHw_isApprox_actualHw{ expectedHw.isApprox(actualHw, tolerance) };
  const bool expectedHx_isApprox_actualHx{ expectedHx.isApprox(actualHx, tolerance) };

  PRX_DBG_VARS(expectedHw_isApprox_actualHw);
  PRX_DBG_VARS(expectedHx_isApprox_actualHx);

  ASSERT_TRUE(expectedHw_isApprox_actualHw);
  ASSERT_TRUE(expectedHx_isApprox_actualHx);
}

TEST(TestArucoMarkerFactors, testArucoMarkerFactorDerivatives)
{
  using SE3 = gtsam::Pose3;
  using CameraCalibration = gtsam::Cal3DS2;
  using Camera = gtsam::PinholeCamera<CameraCalibration>;
  using ArucoMarkerFactor = estimation::aruco_marker_factor_t;
  using Pixel = Eigen::Vector2d;

  const double tolerance{ 1e-5 };

  const Pixel z{ Pixel(100, 50) };
  const SE3 x{ SE3(gtsam::Rot3(), Eigen::Vector3d(1, 2, 3)) };  // Using Beta=0.2;Vin=0.25; w=0.1
  Camera camera;
  // const double dt{ 0.5 };
  // const Control u{ .25, 0.5 };
  // const Params params{ 0.8, 0.8, 0.8 };
  // const StateDot xd1{ MushrCtrl::predict(xd0, dt, u, params) };

  // Check jacobians
  Eigen::MatrixXd actualHcam, expectedHcam;
  Eigen::MatrixXd actualHx, expectedHx;
  // Eigen::MatrixXd actualHdt, expectedHdt;
  // Eigen::MatrixXd actualHu, expectedHu;

  const ArucoMarkerFactor factor(0, 1, z, 0.165, 0, nullptr);

  std::function<gtsam::Vector(const Camera&, const SE3&)> err_proxy  // no-lint
      = [&factor](const Camera& cam, const SE3& x)                   // no-lint
  { return factor.evaluateError(cam, x); };

  factor.evaluateError(camera, x, actualHcam, actualHx);
  expectedHcam = gtsam::numericalDerivative21(err_proxy, camera, x);
  expectedHx = gtsam::numericalDerivative22(err_proxy, camera, x);

  PRX_DBG_VARS(expectedHcam);
  PRX_DBG_VARS(actualHcam);

  PRX_DBG_VARS(expectedHx);
  PRX_DBG_VARS(actualHx);

  const bool expectedHcam_isApprox_actualHcam{ expectedHcam.isApprox(actualHcam, tolerance) };
  const bool expectedHx_isApprox_actualHx{ expectedHx.isApprox(actualHx, tolerance) };

  PRX_DBG_VARS(expectedHcam_isApprox_actualHcam);
  PRX_DBG_VARS(expectedHx_isApprox_actualHx);

  ASSERT_TRUE(expectedHcam_isApprox_actualHcam);
  ASSERT_TRUE(expectedHx_isApprox_actualHx);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}