#include <filesystem>

#include <utils/dbg_utils.hpp>

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

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>

// using SE3 = gtsam::Pose3;
// using CameraCalibration = gtsam::Cal3DS2;
// using Camera = gtsam::PinholeCamera<CameraCalibration>;
// using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

namespace estimation
{
class on_ground_factor_t : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
  using SE3 = gtsam::Pose3;
  using Base = gtsam::NoiseModelFactorN<SE3, SE3>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using GroundMat = Eigen::Matrix<double, 3, 6>;

  using SF = prx::fg::symbol_factory_t;

public:
  on_ground_factor_t(const gtsam::Key key_world, const gtsam::Key key_x, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, key_world, key_x)
    , _ground_mat((GroundMat() << 1, 0, 0, 0, 0, 0, /**/
                   0, 1, 0, 0, 0, 0,                /**/
                   0, 0, 0, 0, 0, 1)
                      .finished())
  {
  }

  virtual Eigen::VectorXd evaluateError(const SE3& w, const SE3& x,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> Hw = boost::none,
                                        boost::optional<Eigen::MatrixXd&> Hx = boost::none) const override
  {
    Eigen::Matrix<double, 6, 6> xInv_H_x, twx_H_w, twx_H_xInv, l_H_twx;
    // const Eigen::Vector3d translation{ x.translation(t_H_x) };
    // const Eigen::Vector<double, 1> z_value{ _z_only_mat * translation };

    const SE3 x_inv{ x.inverse(xInv_H_x) };
    const SE3 Twx{ w.compose(x_inv, twx_H_w, twx_H_xInv) };

    const Eigen::Vector<double, 6> logmap{ gtsam::Pose3::Logmap(Twx, l_H_twx) };
    const Eigen::Vector3d error(_ground_mat * logmap);

    // DEBUG_VARS(Twx);
    // DEBUG_VARS(logmap.transpose());
    // DEBUG_VARS(error.transpose());
    const GroundMat& err_H_l{ _ground_mat };
    if (Hw)
    {
      *Hw = err_H_l * l_H_twx * twx_H_w;
    }
    if (Hx)
    {
      *Hx = err_H_l * l_H_twx * twx_H_xInv * xInv_H_x;
    }
    return error;
  }

  void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = SF::formatter) const override
  {
    std::cout << s << "Ground Factor: ";
    std::cout << keyFormatter(this->template key<1>()) << " ";
    std::cout << keyFormatter(this->template key<2>()) << "\n";
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
    std::cout << "\n";
  }

private:
  const GroundMat _ground_mat;
};

class aruco_marker_factor_t : public gtsam::NoiseModelFactorN<gtsam::PinholeCamera<gtsam::Cal3DS2>, gtsam::Pose3>
{
  using SE3 = gtsam::Pose3;
  using CameraCalibration = gtsam::Cal3DS2;
  using Camera = gtsam::PinholeCamera<CameraCalibration>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using SF = prx::fg::symbol_factory_t;
  using Pixel = Eigen::Vector2d;

  using Base = gtsam::NoiseModelFactor1<Camera, SE3>;

public:
  aruco_marker_factor_t(const gtsam::Key key_camera, const gtsam::Key key_marker, const Pixel meassurement,
                        const double marker_size, const int corner, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, key_camera, key_marker)
    , _z(meassurement)
    , _marker_size(marker_size)
    , _corner(corner)
    , _offset(compute_corner_offset(corner, marker_size))
  {
    // DEBUG_VARS(_corner, _offset.transpose())
  }
  static Eigen::Vector3d compute_corner_offset(const int corner, const double marker_size)
  {
    Eigen::Vector3d offset;
    switch (corner)
    {
      case 0:
        offset = Eigen::Vector3d(-marker_size / 2.0, marker_size / 2.0, 0);
        break;
      case 1:
        offset = Eigen::Vector3d(marker_size / 2.0, marker_size / 2.0, 0);
        break;
      case 2:
        offset = Eigen::Vector3d(marker_size / 2.0, -marker_size / 2.0, 0);
        break;
      case 3:
        offset = Eigen::Vector3d(-marker_size / 2.0, -marker_size / 2.0, 0);
        break;
      default:
        prx_throw("Invalid corner");
    }
    return offset;
  }

  static Eigen::Vector2d predict(const Camera& camera, const SE3& marker, const Eigen::Vector3d& offset,
                                 boost::optional<Eigen::MatrixXd&> Hcam = boost::none,
                                 boost::optional<Eigen::MatrixXd&> Hmarker = boost::none)
  {
    Eigen::Matrix<double, 3, 6> cW_H_m;
    Eigen::Matrix<double, 2, Camera::dimension> pPT_H_cam;
    Eigen::Matrix<double, 2, 3> pPT_H_cW;
    // Hx ? &p_H_x : nullptr,  // no-lint
    // Hxdot ? &p_H_xdot : nullptr) };
    const Eigen::Vector3d corner_world{ marker.transformFrom(offset, Hmarker ? &cW_H_m : nullptr) };

    // DEBUG_VARS(corner_world.transpose());
    const Eigen::Vector2d pred_img_pt{ camera.project2(corner_world,                 // no-lint
                                                       Hcam ? &pPT_H_cam : nullptr,  // no-lint
                                                       Hmarker ? &pPT_H_cW : nullptr) };

    // DEBUG_VARS(marker);
    // DEBUG_VARS(corner_world.transpose());
    // DEBUG_VARS(pred_img_pt.transpose());
    if (Hcam)
    {
      *Hcam = pPT_H_cam;
    }
    if (Hmarker)
    {
      *Hmarker = pPT_H_cW * cW_H_m;
    }

    return pred_img_pt;
  }

  virtual Eigen::VectorXd evaluateError(const Camera& camera, const SE3& marker,
                                        boost::optional<Eigen::MatrixXd&> Hcam = boost::none,
                                        boost::optional<Eigen::MatrixXd&> Hmarker = boost::none) const override
  {
    const Eigen::Vector2d pt_pred{ predict(camera, marker, _offset, Hcam, Hmarker) };
    const Eigen::Vector2d error{ pt_pred - _z };

    // DEBUG_VARS(pt_pred.transpose(), _z.transpose());

    return error;
  }

  void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = SF::formatter) const override
  {
    std::cout << s << "Aruco Marker: ";
    std::cout << keyFormatter(this->template key<1>()) << " ";
    std::cout << keyFormatter(this->template key<2>()) << "\n";
    std::cout << "  Z: " << _z.transpose() << "\n";
    std::cout << "  Marker size: " << _marker_size << "\n";
    std::cout << "  Corner: " << _corner << "\n";
    std::cout << "  Offset: " << _offset.transpose() << "\n";
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
    std::cout << "\n";
  }

private:
  const Eigen::Vector2d _z;
  const double _marker_size;
  const int _corner;
  const Eigen::Vector3d _offset;
};

// Mi_Z_cj     = Mi_T_O * O_T_cj;
// Observation =  SE3   *   SE3
// O = origin frame
// However, all transforms are on camera frame.
class compose_factor2_t : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
  using SE3 = gtsam::Pose3;
  using Base = gtsam::NoiseModelFactorN<SE3, SE3>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  compose_factor2_t(const gtsam::Key key_mi_T_O, const gtsam::Key key_cj_T_O, const SE3 Mi_Z_cj,
                    const NoiseModel& cost_model = nullptr)
    : Base(cost_model, key_mi_T_O, key_cj_T_O), _Mi_Z_cj(Mi_Z_cj)
  {
  }

  virtual Eigen::VectorXd evaluateError(const SE3& mi_T_O, const SE3& cj_T_O,
                                        boost::optional<Eigen::MatrixXd&> HmiO = boost::none,
                                        boost::optional<Eigen::MatrixXd&> HcjO = boost::none) const override
  {
    Eigen::MatrixXd Ocj_H_cjO;
    Eigen::MatrixXd pred_H_miO, pred_H_Ocj;
    Eigen::MatrixXd err_H_pred;
    const SE3 O_T_cj{ cj_T_O.inverse(Ocj_H_cjO) };
    // const SE3 pred_Mi_T_cj{ mi_T_O.compose(O_T_cj, pred_H_miO, pred_H_Ocj) };
    const SE3 pred_Mi_T_cj{ mi_T_O.compose(O_T_cj, pred_H_miO, pred_H_Ocj) };

    // Eigen::Matrix<double, 6, 6> atcp_H_atb, err_H_atcp;
    // const SE3 aTc_pred{ aTb.compose(_bTc, atcp_H_atb) };
    const Eigen::VectorXd error{ pred_Mi_T_cj.logmap(_Mi_Z_cj, err_H_pred) };

    if (HmiO)
    {
      // ROS_INFO_STREAM("Computing H");
      *HmiO = err_H_pred * pred_H_miO;
    }
    if (HcjO)
    {
      *HcjO = err_H_pred * pred_H_Ocj * Ocj_H_cjO;
    }

    return error;
  }

  const SE3 _Mi_Z_cj;
};
}  // namespace estimation