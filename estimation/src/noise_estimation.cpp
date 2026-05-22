#include <cstddef>
#include <fstream>
#include <iterator>
#include <memory>
#include <prx/factor_graphs/lie_groups/se2.hpp>
#include <prx/simulation/playback/plan.hpp>
#include <prx/simulation/system.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/general/prx_assert.hpp>
#include <prx/utilities/general/random.hpp>
#include <prx/utilities/spaces/space.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <prx/simulation/controllers/pid.hpp>
#include <prx/simulation/controllers/lqr.hpp>
#include <thread>
#include <ml4kp_bridge/defs.h>
#include "prx_models/MushrPlanner.h"
#include "prx_models/mj_mushr.hpp"
#include "utils/dbg_utils.hpp"
#include <utils/std_utils.hpp>
#include <prx_models/mushr.hpp>

#include <prx_models/defs.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>
#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
#include <gtsam/geometry/Pose2.h>
// #include <gtsam/base/ProductLieGroup.h>
#include <ml4kp_bridge/product_lie_group.hpp>
#include <interface/node_status.hpp>
#include <interface/gaussian_to_ellipse_marker.hpp>
#include <estimation/dynamical_system_predict_factor.hpp>

using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

template <typename DynamicalSystem>
struct mushr_model_noise_estimator_t
{
  using This = mushr_model_noise_estimator_t;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;

  std::shared_ptr<DynamicalSystem> _plant;

  State _xhat0, _xhat1;
  Control _ut;
  ros::Time _t0, _t1;

  Eigen::Matrix<double, 6, 6> _xhat0_cov, _xhat1_cov;

  bool _ctrl_set;

  gtsam::JacobianFactor::shared_ptr _prior;
  const gtsam::Key _key_x0, _key_x1;
  gtsam::Values _values;

  gtsam::Ordering _key_ordering;

  NoiseModel _dynamical_system_nm;

  gtsam::JacobianFactor::shared_ptr _x1p_prior;

  bool _x0_set;
  double _predict_error;
  ///////////////////
  // ros::Subscriber _sensor_subscriber, _control_subscriber;
  // ros::Publisher _estimation_publisher, _covariance_publisher;

  // interface::gaussian_params_t _gaussian_params;
  // visualization_msgs::Marker _marker_cov;

  mushr_model_noise_estimator_t(ros::NodeHandle& nh)
    : _ctrl_set(false), _prior(nullptr), _key_x0(gtsam::Symbol('X', 0)), _key_x1(gtsam::Symbol('X', 1))
  {
    _dynamical_system_nm = gtsam::noiseModel::Isotropic::Sigma(6, 1);
    // _key_ordering += _key;

    std::string plant_parameters;

    GLOBAL_PARAM_BLOCKER(plant_parameters);

    // prx::param_loader
    // plant_params.from_string(plant_parameters);

    _plant = DynamicalSystem::create(plant_parameters);
  }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr msg)
  {
    ml4kp_bridge::copy(_ut, msg->point);
    _ctrl_set = true;
  }

  void estimated_state_callback(const ml4kp_bridge::SpacePointStampedConstPtr msg)
  {
    using StateExpresion = gtsam::Expression<State>;
    ml4kp_bridge::copy(_xhat1, msg->space_point.point);
    ml4kp_bridge::copy(_xhat1_cov, msg->space_point.covariance);

    _t1 = msg->header.stamp;

    if (_x0_set)
    {
      fwd_prop();
    }
    else
    {
      _xhat0 = _xhat1;
      _xhat0_cov = _xhat1_cov;
      _x0_set = true;
      _t0 = _t1;
    }
  }

  // Obtain the value x1 from f(x0, u, dt), where x0 is another prior
  void fwd_prop()
  {
    gtsam::GaussianFactorGraph linear_fg;
    // _values.insert(_key, _xt0);

    const double dt{ (_t1 - _t0).toSec() };
    estimation::model_predict_factor_t predict_factor(_key_x0, _key_x1, _ut, dt, _plant, _dynamical_system_nm);

    // linearFactorGraph.push_back(priorFactor_);
    gtsam::Values linearizationPoint;

    const gtsam::PriorFactor<State> curr_prior(_key_x0, _xhat0, _xhat0_cov);  // key, zi, z_noise);
    linear_fg.push_back(curr_prior.linearize(linearizationPoint));
    // linear_fg.addPrior(_key_x0, _xhat0_cov);

    linearizationPoint.insert(_key_x0, _xhat0);
    linearizationPoint.insert(_key_x1, _xhat1);
    linear_fg.push_back(predict_factor.linearize(linearizationPoint));

    gtsam::Ordering lastKeyAsOrdering;
    lastKeyAsOrdering += _key_x1;
    const gtsam::GaussianConditional::shared_ptr marginal{
      linear_fg.marginalMultifrontalBayesNet(lastKeyAsOrdering)->front()
    };
    const gtsam::VectorValues result{ marginal->solve(gtsam::VectorValues()) };
    _predict_error = linear_fg.error(result);

    _x1p_prior = boost::make_shared<gtsam::JacobianFactor>(                      // no-lint
        marginal->keys().front(),                                                // no-lint
        marginal->getA(marginal->begin()),                                       // no-lint
        marginal->getb() - marginal->getA(marginal->begin()) * result[_key_x1],  // no-lint
        marginal->get_model());

    const State& current{ linearizationPoint.at<State>(_key_x1) };
    const State x1{ gtsam::traits<State>::Retract(current, result[_key_x1]) };
    DEBUG_VARS(_xhat1, x1);

    // const State x1_model{ _plant->propagate(_xt0, _ut, dt) };
  }

  // static compute_w()
  // {
  //   gtsam::GaussianFactorGraph linear_fg;

  //   // if (_prior != nullptr)
  //   // {
  //   //   linear_fg.push_back(_prior);
  //   //   _values.insert(key, _xt1);
  //   // }

  //   _values.insert(_key, _xhat1);
  //   const gtsam::PriorFactor<State> curr_prior(_key, _xhat1, _xhat1_cov);
  //   linear_fg.push_back(curr_prior.linearize(_values));

  //   const gtsam::GaussianConditional::shared_ptr marginal{
  //     linear_fg.marginalMultifrontalBayesNet(_key_ordering)->front()
  //   };
  // }
};

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MuSHRKalman" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  mushr_model_noise_estimator_t<prx::mushrPolynomial_t> estimator(nh);
  ros::spin();

  return 0;
}