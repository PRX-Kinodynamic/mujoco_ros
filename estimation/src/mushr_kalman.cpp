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

#include <prx_models/defs.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/rosparams_utils.hpp>
#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
#include <gtsam/geometry/Pose2.h>
// #include <gtsam/base/ProductLieGroup.h>
#include <ml4kp_bridge/product_lie_group.hpp>

using Pose = gtsam::Pose2;
using Velocity = Eigen::Vector3d;
using MushrState = gtsam::ProductLieGroupV43<Pose, Velocity>;
using Control = prx_models::mushr_types::Control::type;

class mushr_kalman_predict_t : public gtsam::NoiseModelFactorN<MushrState, MushrState>
{
  // static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  // static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };
  using Polynomial = prx_models::mushr_types::Control::Poly;
  using Parameters = prx_models::mushr_types::Control::params;
  using Base = gtsam::NoiseModelFactorN<MushrState, MushrState>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  // using MushrCtrlAccel = mushr_CtrlAccel_t<>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  // mushr_kalman_predict_t() = delete;
  // mushr_kalman_predict_t(const mushr_kalman_predict_t& other) = delete;

public:
  mushr_kalman_predict_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const Control ui, const Polynomial poly,
                         const Parameters params, const NoiseModel& cost_model)
    : Base(cost_model, key_x0, key_x1), _ui(ui), _poly(poly), _params(params)

  {
  }

  ~mushr_kalman_predict_t() override
  {
  }

  virtual Eigen::VectorXd evaluateError(const MushrState& x0, const MushrState& x1, OptDeriv Hx0 = boost::none,
                                        OptDeriv Hx1 = boost::none) const override
  {
    const bool compute_derivs{ Hx0 or Hx1 };

    // x = [q, qdot]
    Eigen::Matrix<double, 3, 3> q1p_H_q0, q1p_H_qd0, qd1p_H_qd0;
    Eigen::Matrix<double, 6, 6> b_H_x1, b_H_x1p, err_H_b;

    const Pose x1p{ prx_models::mushr_x_xdot_t::predict(x0.first, x0.second, prx::simulation_step, q1p_H_q0,
                                                        q1p_H_qd0) };
    const Velocity x1dot_p{ prx_models::mushr_CtrlAccel_t<>::predict(x0.second, _ui, prx::simulation_step, _params,
                                                                     _poly, qd1p_H_qd0) };
    const MushrState predicted(x1p, x1dot_p);
    const MushrState between{ x1.between(predicted,                           // no-lint
                                         compute_derivs ? &b_H_x1 : nullptr,  // no-lint
                                         compute_derivs ? &b_H_x1p : nullptr) };
    const Eigen::Vector<double, 6> error{ MushrState::Logmap(between, compute_derivs ? &err_H_b : nullptr) };

    if (Hx0)
    {
      const Eigen::Matrix<double, 3, 3> qd1_H_q0{ Eigen::Matrix<double, 3, 3>::Zero() };

      Eigen::Matrix<double, 6, 6> x1p_H_x0;
      x1p_H_x0.block<3, 6>(0, 0) << q1p_H_q0, qd1_H_q0;  // no-lint
      x1p_H_x0.block<3, 6>(3, 0) << q1p_H_qd0, qd1p_H_qd0;

      *Hx0 = err_H_b * b_H_x1p * x1p_H_x0;
    }
    if (Hx1)
    {
      *Hx1 = err_H_b * b_H_x1;
    }
    return error;
  }

private:
  const Control _ui;
  const Polynomial _poly;
  const Parameters _params;

  // const double _dt;
};

class mushr_kalman_update_t : public gtsam::NoiseModelFactorN<MushrState>
{
  // static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  // static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };

  using Base = gtsam::NoiseModelFactorN<MushrState>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  // using MushrCtrlAccel = mushr_CtrlAccel_t<>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  using LieIntegrator = prx::fg::lie_integrator_t<Pose, Velocity>;

  // mushr_kalman_predict_t() = delete;
  // mushr_kalman_predict_t(const mushr_kalman_predict_t& other) = delete;

public:
  mushr_kalman_update_t(const gtsam::Key key_x, const Pose zi, const double dt, const NoiseModel& cost_model)
    : Base(cost_model, key_x), _zi(zi), _dt(dt)

  {
  }

  ~mushr_kalman_update_t() override
  {
  }

  virtual Eigen::VectorXd evaluateError(const MushrState& x, OptDeriv Hx = boost::none) const override
  {
    Eigen::Matrix<double, 3, 3> qb_H_qdt, err_H_qb, qdt_H_q, qdt_H_qdot;

    const Pose& q{ x.first };
    const Velocity& qdot{ x.second };
    const Pose qdt{ LieIntegrator::integrate(q, qdot, _dt, Hx ? &qdt_H_q : nullptr, Hx ? &qdt_H_qdot : nullptr) };

    const Pose qb{ qdt.between(_zi,  // no-lint
                               Hx ? &qb_H_qdt : nullptr) };
    const Eigen::Vector<double, 3> error{ Pose::Logmap(qb, Hx ? &err_H_qb : nullptr) };
    //////////

    if (Hx)
    {
      *Hx = Eigen::Matrix<double, 3, 6>::Zero();
      // Block of size (p,q), starting at (i,j)
      // matrix.block(i,j,p,q);

      Hx->block<3, 3>(0, 0) = err_H_qb * qb_H_qdt * qdt_H_q;
      Hx->block<3, 3>(0, 3) = err_H_qb * qb_H_qdt * qdt_H_qdot;
    }
    return error;
  }

private:
  const double _dt;
  const Pose _zi;
};

struct mushr_kalman_t
{
  using This = mushr_kalman_t;
  // using State = gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d>;
  using EKF = gtsam::ExtendedKalmanFilter<MushrState>;
  using Polynomial = prx_models::mushr_types::Control::Poly;
  using Parameters = prx_models::mushr_types::Control::params;

  ros::Subscriber _sensor_subscriber, _control_subscriber;
  ros::Publisher _estimation_publisher;
  // gtsam::Pose2 _observation;

  std::shared_ptr<prx::world_model_t> _planning_model;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<EKF> _ekf;
  std::size_t _x_idx;

  ros::Time _prev_z_dt;

  Polynomial _poly;
  Parameters _params;

  MushrState _current_estimate;

  gtsam::SharedDiagonal _predict_nm, _update_nm;

  mushr_kalman_t(ros::NodeHandle& nh)
    : _x_idx(0)
    , _predict_nm(gtsam::noiseModel::Isotropic::Sigma(6, 1))
    , _update_nm(gtsam::noiseModel::Isotropic::Sigma(3, 1))
  {
    std::string sensor_topic_name, control_topic_name;
    std::string plant_params_filename, stamped_estimation_topic;
    PARAM_SETUP(nh, plant_params_filename);
    PARAM_SETUP(nh, sensor_topic_name);
    PARAM_SETUP(nh, control_topic_name);
    PARAM_SETUP(nh, stamped_estimation_topic);

    prx::simulation_step = 0.1;
    // create_plan(nh);
    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
    _control_subscriber = nh.subscribe(control_topic_name, 1, &This::control_callback, this);
    _estimation_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(stamped_estimation_topic, 1, true);

    prx::param_loader plant_params(plant_params_filename);
    const std::vector<double> values{ plant_params["parameter_space/values"].as<std::vector<double>>() };
    for (int i = 0; i < _params.size(); ++i)
    {
      _params[i] = values[i];
    }
    for (int i = 0; i < _poly.size(); ++i)
    {
      _poly[i] = values[5 + i];
    }

    // ExtendedKalmanFilter<State> ekf(x0, x_initial, P_initial);
  }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr msg)
  {
    if (_ekf)
    {
      const gtsam::Symbol x0('x', _x_idx);
      const gtsam::Symbol x1('x', _x_idx + 1);

      Control ui{ msg->point[0], msg->point[1] };

      mushr_kalman_predict_t predict_factor(x0, x1, ui, _poly, _params, _predict_nm);

      // DEBUG_VARS(_x_idx, ui.transpose());
      _x_idx++;
      auto previous_estimate = _current_estimate;
      try
      {
        _current_estimate = _ekf->predict(predict_factor);
      }
      catch (gtsam::IndeterminantLinearSystemException exception)
      {
        gtsam::GaussianFactorGraph linearFactorGraph;

        gtsam::Values linearizationPoint;
        linearizationPoint.insert(x0, previous_estimate);
        linearizationPoint.insert(x1, previous_estimate);
        linearFactorGraph.push_back(predict_factor.linearize(linearizationPoint));
        // linearFactorGraph.push_back(predict_factor);
        auto Ab = linearFactorGraph.jacobian();
        auto A = Ab.first;
        auto b = Ab.second;
        LOG_VARS(A)
        LOG_VARS(b)

        const std::string exception_nearby_variable{ gtsam::DefaultKeyFormatter(exception.nearbyVariable()) };
        LOG_VARS(exception_nearby_variable);
        LOG_VARS(exception.what());
        // prx::fg::indeterminant_linear_system_helper(, _values, dbg::variables::ofs_log);
        throw;
      }
    }
  }

  void publish_estimate()
  {
    ml4kp_bridge::SpacePointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.space_point.point.push_back(_current_estimate.first.x());
    msg.space_point.point.push_back(_current_estimate.first.y());
    msg.space_point.point.push_back(_current_estimate.first.theta());
    msg.space_point.point.push_back(_current_estimate.second[0]);
    msg.space_point.point.push_back(_current_estimate.second[1]);
    msg.space_point.point.push_back(_current_estimate.second[2]);

    _estimation_publisher.publish(msg);
  }

  void sensor_callback(const interface::SensorDataStampedConstPtr msg)
  {
    const std::vector<double>& sensor_data{ msg->raw_sensor_data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6]) };
    const double x{ sensor_data[0] };
    const double y{ sensor_data[1] };
    const double theta{ prx::quaternion_to_euler(q)[2] };
    // _observation = gtsam::Pose2(x, y, theta);
    const gtsam::Symbol xi('x', _x_idx);
    if (not _ekf)
    {
      // Point2 x_initial(0.0, 0.0);
      MushrState x_initial(gtsam::Pose2(x, y, theta), Eigen::Vector3d::Zero());
      gtsam::SharedDiagonal P_initial{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };
      // Create an ExtendedKalmanFilter object
      // ExtendedKalmanFilter<Point2> ekf(x0, x_initial, P_initial);
      _ekf = std::make_shared<EKF>(xi, x_initial, P_initial);
      // _current_estimate = x0;
      _prev_z_dt = msg->header.stamp;
    }

    const double dt{ (msg->header.stamp - _prev_z_dt).toSec() };
    const gtsam::Pose2 zi(x, y, theta);
    const mushr_kalman_update_t update_factor(xi, zi, dt, _update_nm);

    _current_estimate = _ekf->update(update_factor);

    // DEBUG_VARS(x, y, theta, dt)
    const Pose qhat{ _current_estimate.first };
    // DEBUG_VARS(qhat.x(), qhat.y(), qhat.theta())
    publish_estimate();
    _prev_z_dt = msg->header.stamp;
  }
};

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MuSHRKalman" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  mushr_kalman_t estimator(nh);
  ros::spin();

  return 0;
}