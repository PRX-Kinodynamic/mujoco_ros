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
#include <interface/node_status.hpp>

using Pose = gtsam::Pose2;
using Velocity = Eigen::Vector3d;
using MushrState = gtsam::ProductLieGroupV43<Pose, Velocity>;
using Control = prx_models::mushr_types::Control::type;

using Polynomial = prx_models::mushr_types::Control::Poly;
using Parameters = prx_models::mushr_types::Control::params;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

class mushr_kalman_predict_t : public gtsam::NoiseModelFactorN<MushrState, MushrState>
{
  // static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  // static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };
  using Base = gtsam::NoiseModelFactorN<MushrState, MushrState>;

  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  // using MushrCtrlAccel = mushr_CtrlAccel_t<>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;
  using LieIntegrator = prx::fg::lie_integrator_t<Pose, Velocity>;

  // mushr_kalman_predict_t() = delete;
  // mushr_kalman_predict_t(const mushr_kalman_predict_t& other) = delete;

public:
  mushr_kalman_predict_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const Control ui, const double dt,
                         const Polynomial poly, const Parameters params, const NoiseModel& cost_model,
                         const bool implicit)
    : Base(cost_model, key_x0, key_x1), _ui(ui), _poly(poly), _params(params), _dt(dt), _implicit(implicit)

  {
  }

  ~mushr_kalman_predict_t() override
  {
  }

  static Eigen::Vector<double, 6> error_implicit_model(const MushrState& x0, const MushrState& x1, const double dt,
                                                       OptDeriv Hx0 = boost::none, OptDeriv Hx1 = boost::none)
  {
    const bool deriv{ Hx0 or Hx1 };
    Eigen::Matrix3d qb_H_qdt, qErr_H_qb, qdt_H_q0, qdt_H_qdot1, qb_H_q1;

    const Pose& q0{ x0.first };
    const Velocity& qdot0{ x0.second };

    const Pose& q1{ x1.first };
    const Velocity& qdot1{ x1.second };

    const Pose qdt{ LieIntegrator::integrate(q0, qdot1, dt,                // no-lint
                                             deriv ? &qdt_H_q0 : nullptr,  // no-lint
                                             deriv ? &qdt_H_qdot1 : nullptr) };

    const Pose qb{ qdt.between(q1,                           // no-lint
                               deriv ? &qb_H_qdt : nullptr,  // no-lint
                               deriv ? &qb_H_q1 : nullptr) };
    const Eigen::Vector<double, 3> q_error{ Pose::Logmap(qb, deriv ? &qErr_H_qb : nullptr) };
    const Velocity& vel_error{ qdot1 - qdot0 };
    const Eigen::Vector<double, 6> predict_error{ (Eigen::Vector<double, 6>() << q_error, vel_error).finished() };

    if (Hx0)
    {
      const Eigen::Matrix3d velErr_H_qdot0{ -Eigen::Matrix3d::Identity() };
      *Hx0 = Eigen::Matrix<double, 6, 6>::Zero();

      // dqerr / dq0
      Hx0->block<3, 3>(0, 0) = qErr_H_qb * qb_H_qdt * qdt_H_q0;
      // dvelErr / dq0
      Hx0->block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
      // dqerr / dqdot0
      Hx0->block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();  // qErr_H_qb * qb_H_qdt * qdt_H_qdot0;
      // dvelErr / dqdot0
      Hx0->block<3, 3>(3, 3) = velErr_H_qdot0;
    }
    if (Hx1)
    {
      // const Eigen::Matrix3d qErr_H_qdot1{ -Eigen::Matrix3d::Identity() };
      const Eigen::Matrix3d velErr_H_qdot1{ Eigen::Matrix3d::Identity() };
      *Hx1 = Eigen::Matrix<double, 6, 6>::Zero();
      // dqerr / dq1
      Hx1->block<3, 3>(0, 0) = qErr_H_qb * qb_H_q1;
      // dvelErr / dq1
      Hx1->block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
      // dqerr / dqdot1
      Hx1->block<3, 3>(0, 3) = qErr_H_qb * qb_H_qdt * qdt_H_qdot1;
      // dvelErr / dqdot1
      Hx1->block<3, 3>(3, 3) = velErr_H_qdot1;
    }

    return predict_error;
  }

  static Eigen::Vector<double, 6> error_explicit_model(const MushrState& x0, const MushrState& x1, const double dt,
                                                       OptDeriv Hx0 = boost::none, OptDeriv Hx1 = boost::none)
  {
    const bool deriv{ Hx0 or Hx1 };
    Eigen::Matrix3d qb_H_qdt, qErr_H_qb, qdt_H_q0, qdt_H_qdot0, qb_H_q1;

    const Pose& q0{ x0.first };
    const Velocity& qdot0{ x0.second };

    const Pose& q1{ x1.first };
    const Velocity& qdot1{ x1.second };

    const Pose qdt{ LieIntegrator::integrate(q0, qdot0, dt,                // no-lint
                                             deriv ? &qdt_H_q0 : nullptr,  // no-lint
                                             deriv ? &qdt_H_qdot0 : nullptr) };

    const Pose qb{ qdt.between(q1,                           // no-lint
                               deriv ? &qb_H_qdt : nullptr,  // no-lint
                               deriv ? &qb_H_q1 : nullptr) };
    const Eigen::Vector<double, 3> q_error{ Pose::Logmap(qb, deriv ? &qErr_H_qb : nullptr) };
    const Velocity& vel_error{ qdot1 - qdot0 };
    const Eigen::Vector<double, 6> predict_error{ (Eigen::Vector<double, 6>() << q_error, vel_error).finished() };

    // LOG_MSG("------ PREDICT ------")
    // auto error = predict_error.transpose();
    // LOG_VARS(_dt)
    // LOG_VARS(q)
    // LOG_VARS(qdot.transpose())
    // LOG_VARS(qdt)
    // LOG_VARS(qb)
    // LOG_VARS(error)

    if (Hx0)
    {
      const Eigen::Matrix3d velErr_H_qdot0{ -Eigen::Matrix3d::Identity() };
      *Hx0 = Eigen::Matrix<double, 6, 6>::Zero();

      // dqerr / dq0
      Hx0->block<3, 3>(0, 0) = qErr_H_qb * qb_H_qdt * qdt_H_q0;
      // dvelErr / dq0
      Hx0->block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
      // dqerr / dqdot0
      Hx0->block<3, 3>(0, 3) = qErr_H_qb * qb_H_qdt * qdt_H_qdot0;
      // dvelErr / dqdot0
      Hx0->block<3, 3>(3, 3) = velErr_H_qdot0;
    }
    if (Hx1)
    {
      // const Eigen::Matrix3d qErr_H_qdot1{ -Eigen::Matrix3d::Identity() };
      const Eigen::Matrix3d velErr_H_qdot1{ Eigen::Matrix3d::Identity() };
      *Hx1 = Eigen::Matrix<double, 6, 6>::Zero();
      // dqerr / dq1
      Hx1->block<3, 3>(0, 0) = qErr_H_qb * qb_H_q1;
      // dvelErr / dq1
      Hx1->block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
      // dqerr / dqdot1
      Hx1->block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
      // dvelErr / dqdot1
      Hx1->block<3, 3>(3, 3) = velErr_H_qdot1;
    }

    return predict_error;
  }

  virtual Eigen::VectorXd evaluateError(const MushrState& x0, const MushrState& x1, OptDeriv Hx0 = boost::none,
                                        OptDeriv Hx1 = boost::none) const override
  {
    if (_implicit)
    {
      return error_implicit_model(x0, x1, _dt, Hx0, Hx1);
    }
    // else
    return error_explicit_model(x0, x1, _dt, Hx0, Hx1);
  }

private:
  const bool _implicit;
  const double _dt;
  const Control _ui;
  const Polynomial _poly;
  const Parameters _params;
};

class mushr_kalman_update_t : public gtsam::NoiseModelFactorN<MushrState>
{
  // static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  // static constexpr Eigen::Index DimXdot{ gtsam::traits<StateDot>::dimension };

  using Base = gtsam::NoiseModelFactorN<MushrState>;

  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  // using MushrCtrlAccel = mushr_CtrlAccel_t<>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

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
    Eigen::Matrix3d qBtw_H_q, qErr_H_qBtw;
    const Pose& q{ x.first };

    const Pose q_btw{ q.between(_zi, Hx ? &qBtw_H_q : nullptr) };
    const Eigen::Vector<double, 3> update_error{ Pose::Logmap(q_btw, Hx ? &qErr_H_qBtw : nullptr) };
    if (Hx)
    {
      *Hx = Eigen::Matrix<double, 3, 6>::Zero();
      // dqerr / dq0
      Hx->block<3, 3>(0, 0) = qErr_H_qBtw * qBtw_H_q;
      // dqerr / dqdot0 = Zero;
    }
    return update_error;
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

  Control _ui;
  Polynomial _poly;
  Parameters _params;

  MushrState _current_estimate;
  ros::Time _prev_z_dt;

  gtsam::SharedDiagonal _predict_nm, _update_nm;
  std::shared_ptr<interface::node_status_t> _simulator_node_status;

  bool _predict_implicit;
  mushr_kalman_t(ros::NodeHandle& nh) : _x_idx(0), _ui(0., 0.), _predict_implicit(false)
  {
    std::string sensor_topic_name, control_topic;
    std::string plant_parameters, estimation_topic, simulator_node_id;

    bool& implicit{ _predict_implicit };

    PARAM_SETUP(nh, sensor_topic_name);
    PARAM_SETUP(nh, control_topic);
    PARAM_SETUP(nh, simulator_node_id);
    PARAM_SETUP(nh, estimation_topic);
    PARAM_SETUP(nh, implicit);
    GLOBAL_PARAM_SETUP_DEFAULT(plant_parameters, plant_parameters);

    _update_nm = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    _predict_nm = gtsam::noiseModel::Isotropic::Sigma(6, 1);

    _simulator_node_status = interface::node_status_t::create(nh, simulator_node_id, true);
    prx::simulation_step = 0.1;

    _sensor_subscriber = nh.subscribe(sensor_topic_name, 1, &This::sensor_callback, this);
    _control_subscriber = nh.subscribe(control_topic, 1, &This::control_callback, this);
    _estimation_publisher = nh.advertise<ml4kp_bridge::SpacePointStamped>(estimation_topic, 1, true);

    prx::param_loader plant_params;
    plant_params.from_string(plant_parameters);
    // DEBUG_VARS(plant_params)
    const std::vector<double> values{ plant_params["parameter_space/values"].as<std::vector<double>>() };
    // DEBUG_VARS(values)
    for (int i = 0; i < _params.size(); ++i)
    {
      _params[i] = values[i];
    }
    for (int i = 0; i < _poly.size(); ++i)
    {
      _poly[i] = values[5 + i];
    }

    PRINT_MSG("EKF initialized")
  }

  // void create_plant(ros::NodeHandle& nh)
  // {
  //   std::string plant_params_filename;
  //   PARAM_SETUP(nh, plant_params_filename);
  //   prx::param_loader plant_params(plant_params_filename);
  //   auto plant = prx::system_factory_t::create_system(plant_params);
  //   prx_assert(plant != nullptr, "Error loading plant");
  //   std::tie(_planning_model, _system_group, _collision_group) = prx::world_model_t::create(plant);
  // }

  void control_callback(const ml4kp_bridge::SpacePointConstPtr msg)
  {
    _ui = Control(msg->point[0], msg->point[1]);
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
    if (_simulator_node_status->status() == interface::NodeStatus::RESET)
    {
      _ekf = nullptr;
      return;
    }
    const std::vector<double>& sensor_data{ msg->raw_sensor_data };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6]) };
    const double x{ sensor_data[0] };
    const double y{ sensor_data[1] };
    const double theta{ prx::quaternion_to_euler(q)[2] };
    // _observation = gtsam::Pose2(x, y, theta);
    // const gtsam::Symbol xi('x', _x_idx);
    if (not _ekf)
    {
      const gtsam::Symbol x0('x', _x_idx);
      // Point2 x_initial(0.0, 0.0);
      MushrState x_initial(gtsam::Pose2(x, y, theta), Eigen::Vector3d::Zero());
      gtsam::SharedDiagonal P_initial{ gtsam::noiseModel::Isotropic::Sigma(6, 0.1) };
      // Create an ExtendedKalmanFilter object
      // ExtendedKalmanFilter<Point2> ekf(x0, x_initial, P_initial);
      _ekf = std::make_shared<EKF>(x0, x_initial, P_initial);
      // _current_estimate = x0;
      _prev_z_dt = msg->header.stamp;
      // _x_idx++;
    }
    else
    {
      const double dt{ (msg->header.stamp - _prev_z_dt).toSec() };
      const gtsam::Pose2 zi(x, y, theta);

      estimate(zi, dt);

      publish_estimate();
      _prev_z_dt = msg->header.stamp;
    }
  }

  void estimate(const gtsam::Pose2& zi, const double& dt)
  {
    const gtsam::Symbol x0('x', _x_idx);
    const gtsam::Symbol x1('x', _x_idx + 1);
    auto previous_estimate = _current_estimate;
    const mushr_kalman_predict_t predict_factor(x0, x1, _ui, dt, _poly, _params, _predict_nm, _predict_implicit);
    const mushr_kalman_update_t update_factor(x1, zi, dt, _update_nm);
    bool update_exception{ false };
    try
    {
      _current_estimate = _ekf->predict(predict_factor);
      update_exception = true;
      _current_estimate = _ekf->update(update_factor);
      _x_idx++;
    }
    catch (gtsam::IndeterminantLinearSystemException exception)
    {
      gtsam::GaussianFactorGraph linearFactorGraph;

      gtsam::Values linearizationPoint;
      linearizationPoint.insert(x0, previous_estimate);
      linearizationPoint.insert(x1, previous_estimate);
      if (update_exception)
      {
        linearFactorGraph.push_back(update_factor.linearize(linearizationPoint));
      }
      else
      {
        linearFactorGraph.push_back(predict_factor.linearize(linearizationPoint));
      }
      // linearFactorGraph.push_back(predict_factor);
      auto Ab = linearFactorGraph.jacobian();
      auto A = Ab.first;
      auto b = Ab.second;
      LOG_VARS(update_exception, _x_idx)
      LOG_VARS(zi)
      LOG_VARS(dt)
      LOG_VARS(_ui.transpose())
      LOG_VARS(_poly.transpose(), _params.transpose())
      LOG_VARS(A)
      LOG_VARS(b)

      const std::string exception_nearby_variable{ gtsam::DefaultKeyFormatter(exception.nearbyVariable()) };
      LOG_VARS(exception_nearby_variable);
      LOG_VARS(exception.what());
      // prx::fg::indeterminant_linear_system_helper(, _values, dbg::variables::ofs_log);
      throw;
    }
  }
};

void test_predict_factor()
{
  using Factor = mushr_kalman_predict_t;
  const gtsam::Key key_x0{ 0 };
  const gtsam::Key key_x1{ 1 };
  Eigen::Vector2d u{ Eigen::Vector2d(0, 0) };
  const double dt{ 0.1 };
  NoiseModel nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };

  // mushr_kalman_predict_t predict_factor(key_x0, key_x1, u, 0.1, Polynomial(), Parameters(), nm);
  std::function<gtsam::Vector(const MushrState& x0, const MushrState& x1)> fn_proxy =
      [&](const MushrState& x0, const MushrState& x1) { return Factor::error_explicit_model(x0, x1, dt); };

  const MushrState x0(gtsam::Pose2(0, 0, 0), Eigen::Vector3d::Zero());
  const MushrState x1(gtsam::Pose2(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1));

  Eigen::MatrixXd actualHx0, expectedHx0;
  Eigen::MatrixXd actualHx1, expectedHx1;

  Factor::error_explicit_model(x0, x1, dt, actualHx0, actualHx1);

  expectedHx0 = gtsam::numericalDerivative21(fn_proxy, x0, x1);
  expectedHx1 = gtsam::numericalDerivative22(fn_proxy, x0, x1);

  const double tolerance{ 1e-5 };

  const bool test_x0_passed{ expectedHx0.isApprox(actualHx0, tolerance) };
  if (not test_x0_passed)
  {
    DEBUG_VARS(expectedHx0);
    DEBUG_VARS(actualHx0);
    prx_throw("Update factor Hx0 test error");
  }

  const bool test_x1_passed{ expectedHx1.isApprox(actualHx1, tolerance) };
  if (not test_x1_passed)
  {
    DEBUG_VARS(expectedHx1);
    DEBUG_VARS(actualHx1);
    prx_throw("Update factor Hx1 test error");
  }
}

void test_update_factor()
{
  const gtsam::Key key_x0{ 0 };
  Eigen::Vector2d u{ Eigen::Vector2d(0, 0) };
  NoiseModel nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };
  const gtsam::Pose2 zi(0.1, 0.1, 0.1);

  mushr_kalman_update_t update_factor(key_x0, zi, 0.1, nm);
  std::function<gtsam::Vector(const MushrState& x0)> fn_proxy = [&](const MushrState& x0) {
    return update_factor.evaluateError(x0);
  };

  const MushrState x0(gtsam::Pose2(0, 0, 0), Eigen::Vector3d::Zero());

  Eigen::MatrixXd actualHx0, expectedHx0;

  update_factor.evaluateError(x0, actualHx0);

  // MushrCtrl::velocity_delta(xd0, dt, xdotDesired, K, &actualHxd0, &actualHdt, &actualHxdotd, &actualHK);
  expectedHx0 = gtsam::numericalDerivative11(fn_proxy, x0);

  const double tolerance{ 1e-5 };
  const bool test_passed{ expectedHx0.isApprox(actualHx0, tolerance) };
  if (not test_passed)
  {
    DEBUG_VARS(expectedHx0);
    DEBUG_VARS(actualHx0);
    prx_throw("Update factor Hx0 test error");
  }
}

void test_predict_implicit_error()
{
  using Factor = mushr_kalman_predict_t;
  const gtsam::Key key_x0{ 0 };
  const gtsam::Key key_x1{ 1 };
  Eigen::Vector2d u{ Eigen::Vector2d(0, 0) };
  const double dt{ 0.1 };
  NoiseModel nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };

  // mushr_kalman_predict_t predict_factor(key_x0, key_x1, u, dt, Polynomial(), Parameters(), nm);
  std::function<gtsam::Vector(const MushrState& x0, const MushrState& x1)> fn_proxy =
      [&](const MushrState& x0, const MushrState& x1) { return Factor::error_implicit_model(x0, x1, dt); };

  const MushrState x0(gtsam::Pose2(0, 0, 0), Eigen::Vector3d::Zero());
  const MushrState x1(gtsam::Pose2(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1));

  Eigen::MatrixXd actualHx0, expectedHx0;
  Eigen::MatrixXd actualHx1, expectedHx1;

  Factor::error_implicit_model(x0, x1, dt, actualHx0, actualHx1);

  expectedHx0 = gtsam::numericalDerivative21(fn_proxy, x0, x1);
  expectedHx1 = gtsam::numericalDerivative22(fn_proxy, x0, x1);

  const double tolerance{ 1e-5 };

  const bool test_x0_passed{ expectedHx0.isApprox(actualHx0, tolerance) };
  if (not test_x0_passed)
  {
    DEBUG_VARS(expectedHx0);
    DEBUG_VARS(actualHx0);
    prx_throw("Update implicit Hx0 test error");
  }

  const bool test_x1_passed{ expectedHx1.isApprox(actualHx1, tolerance) };
  if (not test_x1_passed)
  {
    DEBUG_VARS(expectedHx1);
    DEBUG_VARS(actualHx1);
    prx_throw("Update implicit Hx1 test error");
  }
}

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  const std::string node_name{ "MuSHRKalman" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  test_predict_factor();
  test_update_factor();
  test_predict_implicit_error();

  mushr_kalman_t estimator(nh);
  ros::spin();

  return 0;
}