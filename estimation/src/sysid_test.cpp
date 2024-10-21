#include <filesystem>
#include <set>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <estimation/MushrEstimatorConfig.h>
#include <ml4kp_bridge/Trajectory.h>

#include <prx/utilities/general/csv_reader.hpp>
#include <utils/rosparams_utils.hpp>

#include <prx_models/mushr.hpp>
#include <prx/factor_graphs/utilities/values_utilities.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx_models/mushr_factors.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using SF = prx::fg::symbol_factory_t;
using Values = gtsam::Values;
using FactorGraph = gtsam::NonlinearFactorGraph;
using GraphValues = std::pair<FactorGraph, Values>;
using CsvReader = prx::utilities::csv_reader_t;
using MushrUtils = prx_models::mushr_utils_t;
using prx::utilities::convert_to;

using State = prx_models::mushr_types::State::type;
using Statedot = prx_models::mushr_types::StateDot::type;
using Control = prx_models::mushr_types::Control::type;
using Ubar = prx_models::mushr_types::Ubar::type;
using Params = prx_models::mushr_types::Control::params;

std::vector<double> dynr_params;
const gtsam::Key k_params{ SF::create_hashed_symbol("params") };
std::unordered_set<gtsam::Key> xdot_keys;

void read_tf(const std::string filename, std::vector<State>& trajectory, std::vector<double>& dts)
{
  prx_assert(std::filesystem::exists(filename), "Filename [" << filename << "] does not exists.");
  CsvReader reader(filename, ' ');

  double t_prev{ -1 };
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;

    const double t{ convert_to<double>(line[0]) };
    // const double t{ convert_to<double>(line[2]) };

    const double x{ convert_to<double>(line[3]) };
    const double y{ convert_to<double>(line[4]) };
    const double z{ convert_to<double>(line[5]) };

    const double qw{ convert_to<double>(line[6]) };
    const double qx{ convert_to<double>(line[7]) };
    const double qy{ convert_to<double>(line[8]) };
    const double qz{ convert_to<double>(line[9]) };
    const Eigen::Quaterniond q{ Eigen::Quaterniond(qw, qx, qy, qz) };

    const double angle{ prx::quaternion_to_euler(q)[2] };
    const State state{ x, y, angle };

    trajectory.push_back(state);

    dts.push_back(t);
    // dts.push_back(t_prev < 0 ? 0 : (t - t_prev));
    // t_prev = t;
  }
}

void read_controls(const std::string filename, prx::plan_t& plan, std::vector<double>& dts)
{
  prx_assert(std::filesystem::exists(filename), "Filename [" << filename << "] does not exists.");
  CsvReader reader(filename, ' ');
  double dt{ 0 };
  double t_prev{ -1 };
  std::vector<Control> controls;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;

    const double t{ convert_to<double>(line[0]) };

    const double u0{ convert_to<double>(line[3]) };
    const double u1{ convert_to<double>(line[4]) };

    Control control;
    control[prx_models::mushr_t::control::velocity_idx] = u0;
    control[prx_models::mushr_t::control::steering_idx] = u1;

    controls.push_back(control);

    dts.push_back(t);
    // dts.push_back(t_prev < 0 ? 0 : (t - t_prev));
    // t_prev = t;
  }
  for (int i = 1; i < dts.size(); ++i)
  {
    dt = dts[i] - dts[i - 1];
    plan.copy_onto_back(controls[i], dt);
  }
}

void sync_ctrls_observations(std::vector<State>& trajectory, std::vector<double>& xdts,  // no-lint
                             std::vector<double>& udts)
{
  while (udts[0] > xdts[1])
  {
    xdts.erase(xdts.begin());
    trajectory.erase(trajectory.begin());
  }
  // for (int i = 1; i < udts.size(); ++i)
  // {
  //   // DEBUG_VARS(udts[i] - xdts[0]);
  //   udts[i] = udts[i] - xdts[i - 1];
  //   // DEBUG_VARS(udts[i]);
  // }
  // udts[0] = udts[0] - xdts[0];
  for (int i = xdts.size() - 1; i > 0; --i)
  {
    xdts[i] = xdts[i] - xdts[i - 1];
  }
  xdts[0] = 0.0;
}
template <class Basis, typename Type>
class manifold_evaluation_t
  : public gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<gtsam::traits<Type>::dimension>, Type>
{
  static constexpr Eigen::Index Dim{ gtsam::traits<Type>::dimension };

  using Base = gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<Dim>, Type>;
  using Derived = manifold_evaluation_t<Basis, Type>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using JacobianXX = Eigen::Matrix<double, Dim, Dim>;
  using JacobianXP = Eigen::Matrix<double, Dim, -1>;

public:
  manifold_evaluation_t(const gtsam::Key keyPolyParams, const gtsam::Key keyXdot, const Type xi,
                        const NoiseModel& cost_model, const size_t N, double x, double a, double b)
    : Base(cost_model, keyPolyParams, keyX), _evaluation_function(N, x, a, b), _x(xi)
  {
  }

  virtual Eigen::VectorXd evaluateError(const gtsam::ParameterMatrix<Dim>& P,  // no-lint
                                        OptDeriv Hp = boost::none) const override
  {
    const bool compute_derivs{ (Hx or Hp) };
    // BASIS::template ManifoldEvaluationFunctor<T>(N, x);
    const Type predicted{ _evaluation_function(P, Hp ? &dxp_H_P : nullptr) };

    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const Type between{ _x.between(predicted,  // no-lint
                                   nullptr,    // no-lint
                                   Hp ? &b_H_xp : nullptr) };
    const Eigen::VectorXd error{ Type::Logmap(between, Hp ? &err_H_b : nullptr) };

    if (Hp)
    {
      *Hp = err_H_b * b_H_xp * dxp_H_P;
    }
    return error;
  }

private:
  const typename Basis::template ManifoldEvaluationFunctor<Type> _evaluation_function;

  mutable JacobianXX err_H_b;  // Deriv error wrt between

  mutable JacobianXX b_H_x;   // Deriv between wrt x
  mutable JacobianXX b_H_xp;  // Deriv between wrt x_{predicted}

  mutable Eigen::MatrixXd dxp_H_P;  // Deriv \dot{predicted} wrt Params
  const Type _x;
};

class mushr_sysid_t : public gtsam::NoiseModelFactorN<Statedot, Statedot, Params>
{
  using Base = gtsam::NoiseModelFactorN<Statedot, Statedot, Params>;
  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

public:
  mushr_sysid_t(gtsam::Key k_xdot1, gtsam::Key k_xdot0, gtsam::Key k_params, const Control& u, const double& dt,
                const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, k_xdot1, k_xdot0, k_params), _u(u), _dt(dt)
  {
  }

  virtual Statedot predict(const Statedot& xd0, const Params& params,  // no-lint
                           gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,
                           gtsam::OptionalJacobian<3, 2> Hparams = boost::none) const
  {
    return prx_models::mushr_CtrlAccel_t::predict(xd0, _u, _dt, params, Hxd0, boost::none, boost::none, Hparams);
  }

  virtual Error evaluateError(const Statedot& xd1, const Statedot& xd0, const Params& params,  // no-lint
                              OptDeriv Hxd1 = boost::none, OptDeriv Hxd0 = boost::none,
                              OptDeriv Hparams = boost::none) const
  {
    if (Hxd1)
    {
      *Hxd1 = -Eigen::Matrix<double, 3, 3>::Identity();
    }
    return predict(xd0, params, Hxd0, Hparams) - xd1;
  }

private:
  const Control _u;
  const double _dt;
};

class mushr_xdot_fix_z_t : public gtsam::NoiseModelFactorN<Statedot>
{
  using Base = gtsam::NoiseModelFactorN<Statedot>;
  using LieIntegrator = prx::fg::lie_integration_factor_t<prx_models::mushr_types::State::type, Statedot, double>;

public:
  mushr_xdot_fix_z_t(gtsam::Key xdot, const State x1, const State x0, const double dt,
                     const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, xdot), _x1(x1), _x0(x0), _dt(dt)
  {
  }

  virtual Eigen::VectorXd evaluateError(const Statedot& xdot,
                                        boost::optional<Eigen::MatrixXd&> Hxdot = boost::none) const override
  {
    // const X x1p{ mushr_x_xdot_t::predict(_x0, xdot, _dt, nullptr, Hxdot, nullptr) };
    return LieIntegrator::error(_x1, _x0, xdot, _dt, boost::none, boost::none, Hxdot, boost::none);
  }

  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = SF::formatter) const override
  {
    std::cout << s << "  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
    std::cout << "X0: " << _x0 << " ";
    std::cout << "X1: " << _x1 << " ";
    std::cout << "dt: " << _dt << std::endl;
  }

private:
  const State _x0;
  const State _x1;
  const double _dt;
};

GraphValues root_fg(std::size_t traj_id, std::size_t parent, const Params initial_params)
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  GraphValues graph_values;
  const gtsam::Key k_xdot{ prx_models::mushr_utils_t::keyXdot(traj_id, parent) };
  // const gtsam::Key k_ubar{ prx_models::mushr_utils_t::keyUbar(traj_id, parent) };

  NoiseModel xdot_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e0) };
  // NoiseModel ubar_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };

  const Statedot xdot{ Statedot::Zero() };
  // const Ubar ubar{ Ubar::Zero() };

  graph_values.first.addPrior(k_xdot, xdot, xdot_prior_noise);
  // graph_values.first.addPrior(k_ubar, ubar, ubar_prior_noise);

  graph_values.second.insert_or_assign(k_xdot, xdot);
  // graph_values.second.insert(k_ubar, ubar);

  return graph_values;
}

GraphValues node_edge_to_fg(std::size_t traj_id, std::size_t parent, std::size_t child, const State x1, const State x0,
                            const double dt, const Control u, Ubar& ubar0, const Params initial_params)
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  // mushr_xdot_fix_z_t(gtsam::Key xdot, const X x1, const X x0, const double dt,
  // mushr_sysid_t(gtsam::Key ubar1, gtsam::Key ubar0, gtsam::Key k_params, const U& u, const double& dt,
  const gtsam::Key k_xdot0{ prx_models::mushr_utils_t::keyXdot(traj_id, parent) };
  const gtsam::Key k_xdot1{ prx_models::mushr_utils_t::keyXdot(traj_id, child) };
  // const gtsam::Key k_ubar0{ prx_models::mushr_utils_t::keyUbar(traj_id, parent) };
  // const gtsam::Key k_ubar1{ prx_models::mushr_utils_t::keyUbar(traj_id, child) };

  xdot_keys.insert(k_xdot0);
  xdot_keys.insert(k_xdot1);
  GraphValues graph_values;

  NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 5e-1) };
  NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
  NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-0) };
  NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-0) };

  graph_values.first.emplace_shared<mushr_xdot_fix_z_t>(k_xdot0, x1, x0, dt, integration_noise);
  graph_values.first.emplace_shared<mushr_sysid_t>(k_xdot1, k_xdot0, k_params, u, dt, nullptr);
  // graph_values.first.emplace_shared<prx_models::mushr_xdot_ub_t>(k_xdot, k_ubar1, nullptr);
  // aux_graph.first.emplace_shared<XdotIntegrationFactor>(k_xdot1, k_xdot0, k_u01, k_t01, integration_noise,
  //                                                       default_params);
  // const Ubar ubar1{ prx_models::mushr_ub_u_xdot_param_t::dynamics(u, ubar0, initial_params, dt) };
  // const Statedot xdot{ prx_models::mushr_xdot_ub_t::dynamics(ubar1) };
  const Statedot xdot0{ State::Logmap(x1 * x0.inverse()) };
  // const Statedot xdot{ mushr_sysid_t::predict(xdot0,u, dt, ) };
  // graph_values.second.insert(k_ubar0, );
  graph_values.second.insert_or_assign(k_xdot0, xdot0);
  graph_values.second.insert_or_assign(k_xdot1, xdot0);
  // graph_values.second.insert(k_xdot, xdot);

  // ubar0 = ubar1;

  return graph_values;
}

template <typename States>
void states_to_file(std::ofstream& ofs, const States traj, const std::size_t min_idx)
{
  // for (auto state : traj)
  for (int i = 0; i < min_idx; ++i)
  {
    ofs << traj[i] << "\n";
  }
  ofs << "\n";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mushrTunning");
  ros::NodeHandle nh("~");

  const std::string path_tf{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/" };
  const std::string path_plan{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/" };
  std::vector<std::string> files_suffixes = {
    // no-lint
    "plan_fwd_241021_024821.txt",  // no-lint
    // "plan_fwd_left_241021_103849.txt",
    "plan_fwd_left_241021_104703.txt"
    // "analytical.txt"
  };
  prx::simulation_step = 0.01;

  auto obstacles = prx::load_obstacles("environments/empty.yaml");
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  const std::string plant_name{ "mushrFG" };
  const std::string plant_path{ "mushrFG" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_t* ps{ sg->get_parameter_space() };

  std::vector<prx::plan_t> plans;
  std::vector<State> start_states;

  double p0, p1, p2;
  PARAM_SETUP(nh, p0);
  PARAM_SETUP(nh, p1);
  // PARAM_SETUP(nh, p2);

  const std::string zout_filename{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/sysid_in_trajectories.txt" };
  std::ofstream ofs_z(zout_filename);

  const Params initial_params{ Params(p0, p1) };
  // std::ofstream ofs_plan_gt(path_tf + "/gt_plan.txt");
  // std::ofstream ofs_traj_gt(path_tf + "/gt_traj.txt");
  // std::ofstream ofs_traj_z(path_tf + "/z_traj.txt");
  Values values;
  FactorGraph factor_graph;
  values.insert(k_params, initial_params);

  for (int i = 0; i < files_suffixes.size(); ++i)
  {
    const std::string tf_filename{ path_tf + "mj_sensor_" + files_suffixes[i] };
    const std::string control_filename{ path_plan + "executed_control_" + files_suffixes[i] };

    DEBUG_VARS(files_suffixes[i]);

    ml4kp_bridge::Plan plan, gt_plan;
    std::vector<State> z_traj;
    // std::vector<Control> controls;
    // ml4kp_bridge::Trajectory z_traj, gt_traj;
    std::vector<double> x_dts;
    std::vector<double> u_dts;

    plans.emplace_back(cs);
    read_tf(tf_filename, z_traj, x_dts);
    read_controls(control_filename, plans.back(), u_dts);
    sync_ctrls_observations(z_traj, x_dts, u_dts);

    using prx::utilities::convert_to;
    const std::string xt0{ convert_to<std::string>(x_dts[0]) };
    const std::string ut0{ convert_to<std::string>(u_dts[0]) };
    DEBUG_VARS(xt0, ut0);

    const GraphValues root_graph_values{ root_fg(i, 0, initial_params) };

    values.insert(root_graph_values.second);
    factor_graph.push_back(root_graph_values.first);

    std::size_t idx{ 0 };
    Ubar ubar0{ Ubar::Zero() };
    const std::size_t min_idx{ std::min(x_dts.size(), u_dts.size()) };
    states_to_file(ofs_z, z_traj, min_idx);
    State x0{ z_traj[0] };
    start_states.emplace_back(x0);

    std::size_t factor_idx{ 1 };
    const double edge_duration{ 0.5 };
    // DEBUG_VARS(min_time);
    double T{ 0.0 };
    double dt{ 0.0 };
    DEBUG_VARS(x_dts.size());
    for (int idx = 0; idx < min_idx; ++idx)
    {
      if (dt < edge_duration)
      {
        dt += x_dts[idx];
        continue;
      }
      T += dt;
      const State x1{ z_traj[idx] };
      const Control u{ Vec(plans.back().at(T)) };

      DEBUG_VARS(dt, x0, x1, u.transpose());
      GraphValues graph_values{ node_edge_to_fg(i, factor_idx - 1, factor_idx, x1, x0, dt, u, ubar0, initial_params) };

      x0 = x1;
      factor_idx++;
      factor_graph.push_back(graph_values.first);
      values.insert_or_assign(graph_values.second);
      dt = 0;
    }
  }
  prx::fg::SF::symbols_to_file();
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  // const std::string init_file{ "/Users/Gary/pracsys/catkin_ws/dbg/sysid_init.txt" };
  // std::ofstream ofs_init(init_file.c_str());

  // factor_graph.printErrors(values, "", SF::formatter);
  // factor_graph.printErrors(values, "Initial", SF::formatter);
  // values.print("Initial Values", SF::formatter);
  gtsam::LevenbergMarquardtOptimizer optimizer(factor_graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  const Params params_result{ result.at<Params>(k_params).transpose() };
  DEBUG_VARS(params_result.transpose());
  const std::string res_file{ "/Users/Gary/pracsys/catkin_ws/dbg/sysid_result.txt" };
  std::ofstream ofs_res(res_file.c_str());

  for (auto key : xdot_keys)
  {
    const std::string K{ prx::fg::SF::formatter(key) };
    const Statedot xdot{ values.at<Statedot>(key) };
    ofs_res << K << " " << xdot.transpose() << "\n";
  }
  ofs_res.close();

  const std::string outfilename{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/sysid_out_trajectories.txt" };

  prx::trajectory_t traj(ss);
  prx::space_point_t x0{ ss->make_point() };
  traj.to_file(outfilename);

  (*ps)[0] = params_result[0];
  (*ps)[1] = params_result[1];

  DEBUG_VARS(*ps);
  for (int i = 0; i < plans.size(); ++i)
  {
    traj.clear();
    Vec(x0).head(3) = start_states[i].vector();
    prx::plan_t& plan{ plans[i] };
    // DEBUG_VARS(plan);
    sg->propagate(x0, plan, traj);
    // DEBUG_VARS(traj);
    traj.to_file(outfilename, std::ofstream::app);
  }

  return 0;
}