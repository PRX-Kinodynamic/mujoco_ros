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

    // const double t{ convert_to<double>(line[0]) };
    const double t{ convert_to<double>(line[2]) };

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
  // DEBUG_VARS(trajectory.size());
  while (udts[0] > xdts[1])
  {
    xdts.erase(xdts.begin());
    trajectory.erase(trajectory.begin());
  }
  // const std::string ut0{ convert_to<std::string>(udts[0]) };
  // const std::string xt0{ convert_to<std::string>(xdts[0]) };
  // const std::string xt1{ convert_to<std::string>(xdts[1]) };
  // DEBUG_VARS(ut0, xt0, xt1)
  // DEBUG_VARS(trajectory);
  // for (int i = 1; i < udts.size(); ++i)
  // {
  const std::string dtu0{ convert_to<std::string>(udts[0]) };
  const std::string dtx0{ convert_to<std::string>(xdts[0]) };
  DEBUG_VARS(dtu0, dtx0);
  //   udts[i] = udts[i] - xdts[i - 1];
  //   // DEBUG_VARS(udts[i]);
  // }
  // const double total_duration{  };
  for (int i = xdts.size() - 1; i > 0; --i)
  {
    xdts[i] = xdts[i] - xdts[i - 1];
  }
  xdts[0] = 0.0;
  // DEBUG_VARS(xdts);
}

class mushr_sysid_t : public gtsam::NoiseModelFactorN<Statedot, Statedot, Params>
{
  using Base = gtsam::NoiseModelFactorN<Statedot, Statedot, Params>;
  using Error = Eigen::VectorXd;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  static constexpr Eigen::Index DimParams{ prx_models::mushr_types::Control::ParamsDim };

public:
  mushr_sysid_t(gtsam::Key k_xdot1, gtsam::Key k_xdot0, gtsam::Key k_params, const Control& u, const double& dt,
                const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, k_xdot1, k_xdot0, k_params), _u(u), _dt(dt)
  {
  }

  virtual Statedot predict(const Statedot& xd0, const Params& params,  // no-lint
                           gtsam::OptionalJacobian<3, 3> Hxd0 = boost::none,
                           gtsam::OptionalJacobian<3, DimParams> Hparams = boost::none) const
  {
    return prx_models::mushr_CtrlAccel_t::predict(xd0, _u, _dt, params, prx_models::mushr_utils_t::default_poly, Hxd0,
                                                  boost::none, boost::none, Hparams);
  }

  virtual Error evaluateError(const Statedot& xd1, const Statedot& xd0, const Params& params,  // no-lint
                              OptDeriv Hxd1 = boost::none, OptDeriv Hxd0 = boost::none,
                              OptDeriv Hparams = boost::none) const override
  {
    if (Hxd1)
    {
      *Hxd1 = -Eigen::Matrix<double, 3, 3>::Identity();
    }
    // DEBUG_VARS(xd0.transpose(), params[0]);
    const Statedot prediction{ predict(xd0, params, Hxd0, Hparams) };
    // DEBUG_VARS(prediction.transpose(), xd1.transpose());
    return prediction - xd1;
  }

  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = SF::formatter) const override
  {
    std::cout << s << "  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
    std::cout << "u: " << _u.transpose() << " ";
    std::cout << "dt: " << _dt << std::endl;
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
    const Statedot error{ LieIntegrator::error(_x1, _x0, xdot, _dt, boost::none, boost::none, Hxdot) };

    // DEBUG_VARS(_x1, _x0);
    // DEBUG_VARS(xdot.transpose(), error.transpose());
    return error;
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

Statedot compute_vel(const State x1, const State x0)
{
  const Statedot xdot0{ State::Logmap(x0.inverse() * x1) };
  return xdot0;
}

GraphValues node_edge_to_fg(std::size_t traj_id, std::size_t parent, std::size_t child, const State x1, const State x0,
                            const double dt, const Control u, const Params initial_params, const bool last)
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  // mushr_xdot_fix_z_t(gtsam::Key xdot, const X x1, const X x0, const double dt,
  // mushr_sysid_t(gtsam::Key ubar1, gtsam::Key ubar0, gtsam::Key k_params, const U& u, const double& dt,
  const gtsam::Key k_xdot0{ prx_models::mushr_utils_t::keyXdot(traj_id, parent) };
  const gtsam::Key k_xdot1{ prx_models::mushr_utils_t::keyXdot(traj_id, child) };
  // const gtsam::Key k_ubar0{ prx_models::mushr_utils_t::keyUbar(traj_id, parent) };
  // const gtsam::Key k_ubar1{ prx_models::mushr_utils_t::keyUbar(traj_id, child) };

  xdot_keys.insert(k_xdot0);
  // xdot_keys.insert(k_xdot1);
  GraphValues graph_values;

  NoiseModel prior_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 5e-1) };
  NoiseModel u_prior_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e0) };
  NoiseModel dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-0) };
  NoiseModel integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-4) };

  // const Statedot xdot0{ State::Logmap(x0.inverse() * x1) };
  const Statedot xdot0{ compute_vel(x1, x0) };

  // const Statedot xdot0{ State::Logmap(x1 * x0.inverse()) };
  // DEBUG_VARS(x0, x1);
  DEBUG_VARS(xdot0.transpose());
  // DEBUG_VARS(xdot0_X.transpose());
  graph_values.second.insert_or_assign(k_xdot0, xdot0);

  graph_values.first.emplace_shared<mushr_xdot_fix_z_t>(k_xdot0, x1, x0, dt, integration_noise);
  if (not last)
  {
    graph_values.first.emplace_shared<mushr_sysid_t>(k_xdot1, k_xdot0, k_params, u, dt, nullptr);
  }

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

  std::string path_tf{};
  std::string path_plan{};
  std::vector<std::string> files_suffixes;
  PARAM_SETUP(nh, path_tf);
  PARAM_SETUP(nh, path_plan);
  PARAM_SETUP(nh, files_suffixes);
  // const std::string path_tf{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/" };
  // const std::string path_plan{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/" };
  // no-lint
  // "left_00_241028_141102.txt",  // no-lint
  // "left_01_241028_141600.txt",  // no-lint
  // "left_02_241028_141639.txt",  // no-lint
  // "left_03_241028_141721.txt",  // no-lint
  // "left_04_241028_141805.txt",  // no-lint

  // "b_241025_184902_241025_211707.txt",  // no-lint
  //
  // "b_241025_102825_241025_103147.txt",  // no-lint
  // "plan_fwd_241021_024821.txt",  // no-lint
  // "plan_fwd_left_241021_103849.txt",
  // "plan_fwd_left_241021_104703.txt"
  // "analytical.txt"
  // };
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

  double p0, p1, p2, p3, p4;
  PARAM_SETUP(nh, p0);
  PARAM_SETUP(nh, p1);
  PARAM_SETUP(nh, p2);
  PARAM_SETUP(nh, p3);
  PARAM_SETUP(nh, p4);

  const std::string zout_filename{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/sysid_in_trajectories.txt" };
  std::ofstream ofs_z(zout_filename);

  // const std::string zout_filename{ "/Users/Gary/pracsys/catkin_ws/data/estimation/mushr/sysid_in_trajectories.txt" };
  // std::ofstream ofs_z(zout_filename);

  const Params initial_params{ Params(p0, p1, p2, p3, p4) };
  // std::ofstream ofs_plan_gt(path_tf + "/gt_plan.txt");
  // std::ofstream ofs_traj_gt(path_tf + "/gt_traj.txt");
  // std::ofstream ofs_traj_z(path_tf + "/z_traj.txt");
  Values values;
  FactorGraph factor_graph;
  values.insert(k_params, initial_params);
  const std::string fg_states_file{ "/Users/Gary/pracsys/catkin_ws/dbg/fg_states.txt" };
  std::ofstream ofs_fg_states(fg_states_file.c_str());

  const std::string vels_file{ "/Users/Gary/pracsys/catkin_ws/dbg/mushr_vels.txt" };
  std::ofstream ofs_vels(vels_file.c_str());

  for (int i = 0; i < files_suffixes.size(); ++i)
  {
    // const std::string tf_filename{ path_tf + "mj_sensor_" + files_suffixes[i] };
    const std::string tf_filename{ path_tf + "tf_" + files_suffixes[i] };
    const std::string control_filename{ path_plan + "executed_control_stamped_" + files_suffixes[i] };

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

    // const GraphValues root_graph_values{ root_fg(i, 0, initial_params) };

    // values.insert(root_graph_values.second);
    // factor_graph.push_back(root_graph_values.first);

    // std::size_t idx{ 0 };
    Ubar ubar0{ Ubar::Zero() };
    // const std::size_t min_idx{ std::min(x_dts.size(), u_dts.size()) };
    const std::size_t min_idx{ x_dts.size() };
    // states_to_file(ofs_z, z_traj, min_idx);
    State x0{ z_traj[0] };
    State x0_inv{ x0.inverse() };
    // State x0_inv{ 0, 0, 0 };
    start_states.emplace_back(x0_inv * x0);

    const double edge_duration{ 0.5 };

    // DEBUG_VARS(min_time);
    double T{ 0.0 };
    double dt{ 0.0 };
    const double total_duration{ plans.back().duration() };
    DEBUG_VARS(x_dts.size(), total_duration);
    std::size_t idx = 0;

    std::vector<State> fg_traj;
    std::vector<Control> fg_plan;
    fg_traj.push_back(x0);
    fg_plan.push_back(Vec(plans.back().at(0)));

    while (T + dt < total_duration)
    {
      // break;
      if (dt < edge_duration)
      {
        dt += x_dts[idx];
        idx++;
        continue;
      }
      T += dt;
      DEBUG_VARS(idx, dt, T, total_duration);
      const State x0{ z_traj[idx] };
      const Control u{ Vec(plans.back().at(T)) };
      ofs_z << x0_inv * x0 << "\n";
      fg_traj.push_back(x0);
      fg_plan.push_back(u);

      dt = 0;
      idx++;
    }
    ofs_z << "\n\n";
    states_to_file(ofs_fg_states, fg_traj, fg_traj.size());
    Statedot xdot_prev{ Statedot::Zero() };
    double xddot{ 0.0 };
    for (std::size_t factor_idx = 1; factor_idx < fg_traj.size(); ++factor_idx)
    {
      const std::size_t parent{ factor_idx - 1 };
      const std::size_t child{ factor_idx };

      DEBUG_VARS(parent, child);

      const Control u{ fg_plan[parent] };
      const State x0{ fg_traj[parent] };
      const State x1{ fg_traj[child] };

      GraphValues graph_values{ node_edge_to_fg(i, parent, child, x1, x0, edge_duration, u, initial_params,
                                                (child + 1) == fg_traj.size()) };
      factor_graph.push_back(graph_values.first);
      values.insert_or_assign(graph_values.second);

      const Statedot xdot0{ compute_vel(x1, x0) };

      const double delta{ u[prx_models::mushr_types::Control::steering] };
      ofs_vels << edge_duration * (factor_idx - 1) << " ";
      ofs_vels << xdot0.norm() << " " << xddot << " " << xdot_prev.norm() << " ";
      ofs_vels << delta << " ";
      ofs_vels << prx_models::mushr_types::Control::evaluate_polynomial(prx_models::mushr_utils_t::default_poly, delta);
      ofs_vels << "\n";
      xddot = (xdot0 - xdot_prev).norm();
      xdot_prev = xdot0;
    }
    ofs_vels << "\n";
    // const gtsam::Key k_xdotT{ prx_models::mushr_utils_t::keyXdot(i, fg_traj.size() - 1) };
    DEBUG_VARS(fg_traj.size());
    // values.insert_or_assign(k_xdotT, fg_traj.back());
  }
  ofs_vels.close();

  ofs_fg_states.close();
  ofs_z.close();
  prx::fg::SF::symbols_to_file();
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  gtsam::LevenbergMarquardtOptimizer optimizer(factor_graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };
  result.print("Result", SF::formatter);

  factor_graph.printErrors(result, "Result", SF::formatter);

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

  (*ps)[0] = p0;  // params_result[prx_models::mushr_types::Control::vel_desired];
  (*ps)[1] = p1;  // params_result[prx_models::mushr_types::Control::steering];
  (*ps)[2] = p2;  // params_result[prx_models::mushr_types::Control::friction];
  (*ps)[3] = p3;  // params_result[prx_models::mushr_types::Control::friction];
  (*ps)[4] = p4;  // params_result[prx_models::mushr_types::Control::friction];

  DEBUG_VARS(*ps);
  prx::simulation_step = 0.01;
  for (int i = 0; i < plans.size(); ++i)
  {
    traj.clear();
    // Vec(x0).head(3) = Eigen::Vector3d(0, 0, prx::constants::pi);
    Vec(x0).head(3) = start_states[i].vector();
    prx::plan_t& plan{ plans[i] };
    // DEBUG_VARS(plan.duration());
    // DEBUG_VARS(plan);
    sg->propagate(x0, plan, traj);
    // DEBUG_VARS(traj);
    traj.to_file(outfilename, std::ofstream::app);
  }

  return 0;
}