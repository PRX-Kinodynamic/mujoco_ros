#include <thread>
#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/nodelet_as_node.hpp>

#include <analytical/ltv_sde.hpp>
#include <analytical/simulator.hpp>
#include <analytical/fg_ltv_sde.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <prx/factor_graphs/utilities/symbols_factory.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>

using SF = prx::fg::symbol_factory_t;

auto keyX = [](const std::size_t& xi, const std::size_t& ti) {
  return SF::create_hashed_symbol("x^{", ti, "}_{", xi, "}");
};
auto keyU = [](const std::size_t& ui, const std::size_t& ti) {
  return SF::create_hashed_symbol("u^{", ti, "}_{", ui, "}");
};
auto keyH = [](const std::size_t& hi, const std::size_t& ti) {
  return SF::create_hashed_symbol("h^{", ti, "}_{", hi, "}");
};
auto keyW = [](const std::size_t& wi, const std::size_t& ti) {
  return SF::create_hashed_symbol("w^{", ti, "}_{", wi, "}");
};
auto keyZ = [](const std::size_t& xi, const std::size_t& ti) {
  return SF::create_hashed_symbol("Z^{", ti, "}_{", xi, "}");
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simplified_tensegrity");
  ros::NodeHandle nh("~");

  std::string prx_config_file;
  PARAM_SETUP(nh, prx_config_file);

  DEBUG_VARS(prx_config_file);

  prx::param_loader params;
  params.set_input_path("/");
  params.add_file(prx_config_file);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<double>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };

  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant != nullptr, "Plant is nullptr!");

  plant->init(params["plant"]);

  gtsam::Values initial_values;
  gtsam::NonlinearFactorGraph graph;

  std::string filename{ params["/out/filename"].as<>() };
  prx::utilities::csv_reader_t reader(filename, ' ');

  std::vector<double> control_in{ params["/plant/control"].as<std::vector<double>>() };
  std::vector<double> params_in{ params["/plant/parameters"].as<std::vector<double>>() };

  const double traj_duration{ 10.0 };
  prx::fg::ltv_sde_factor_t::MatrixA A{};
  prx::fg::ltv_sde_factor_t::MatrixB B{};
  prx::fg::ltv_sde_factor_t::MatrixG G{};
  A << 1, 0, traj_duration, 0, 0, 1, 0, traj_duration, 0, 0, 1, 0, 0, 0, 0, 1;

  B << traj_duration * traj_duration / 2.0, 0, 0, traj_duration * traj_duration / 2.0, traj_duration, 0, 0,
      traj_duration;

  G = Eigen::DiagonalMatrix<double, 4>(params_in[4], params_in[5], params_in[6], params_in[7]);

  const Eigen::Vector4d zero4d{ Eigen::Vector4d::Zero() };
  const Eigen::Vector4d w_sigmas{ Eigen::Vector4d(params_in[0], params_in[1], params_in[2], params_in[3]) };
  const Eigen::Vector2d control{ Eigen::Vector2d(control_in[0], control_in[1]) };

  auto xprop_nm = gtsam::noiseModel::Isotropic::Sigma(4, 1e-5);
  auto xzero_nm = gtsam::noiseModel::Isotropic::Sigma(4, 1e-5);
  // auto xend_nm = gtsam::noiseModel::Isotropic::Sigma(4, 1e);
  auto u_nm = gtsam::noiseModel::Isotropic::Sigma(2, 1e-4);
  auto xend_nm = gtsam::noiseModel::Diagonal::Sigmas(w_sigmas);
  auto wprior_nm = gtsam::noiseModel::Diagonal::Sigmas(w_sigmas);

  Eigen::Vector4d x_predicted{ Eigen::Vector4d::Zero() };

  std::size_t ti{ 0 };
  x_predicted = prx::fg::ltv_sde_factor_t::predict(x_predicted, control, zero4d, zero4d, A, B, G);
  DEBUG_VARS(x_predicted);
  while (reader.has_next_line())
  {
    auto line = reader.next_line<double>();
    if (line.size() == 0)
      continue;

    const Eigen::Vector4d start_state(line[0], line[1], line[2], line[3]);
    const Eigen::Vector4d end_state(line[4], line[5], line[6], line[7]);

    // PRX_DEBUG_VAR_1(keyX(1, ti));
    // graph.addPrior(keyW(0, 0), zero4d, wprior_nm);
    // for (double i = 0; i < traj_duration; i += prx::simulation_step)
    // {
    graph.emplace_shared<prx::fg::ltv_sde_observation_factor_t>(keyX(1, 0), end_state, xend_nm);
    // graph.addPrior(keyX(1, 0), end_state);
    break;
    // graph.addPrior(keyX(1, ti), end_state, xend_nm);
    ti++;
  }
  graph.emplace_shared<prx::fg::ltv_sde_factor_t>(keyX(1, 0), keyX(0, 0), keyU(0, 0), keyH(0, 0), keyW(0, 0), xprop_nm,
                                                  A, B, G);
  initial_values.insert(keyX(1, 0), x_predicted);

  graph.addPrior(keyX(0, 0), zero4d, xzero_nm);
  // graph.addPrior(keyW(0, 0), zero4d, xzero_nm);
  graph.addPrior(keyH(0, 0), zero4d, xzero_nm);
  graph.addPrior(keyU(0, 0), control, u_nm);

  initial_values.insert(keyW(0, 0), zero4d);
  initial_values.insert(keyX(0, 0), zero4d);
  initial_values.insert(keyU(0, 0), control);
  initial_values.insert(keyH(0, 0), zero4d);

  prx::fg::symbol_factory_t::symbols_to_file();

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values results = optimizer.optimize();

  DEBUG_VARS(graph.linearize(results)->jacobian().first);
  graph.printErrors(results, "Errors: ", prx::fg::symbol_factory_t::formatter);
  // results.print("results", prx::fg::symbol_factory_t::formatter);
  gtsam::Marginals marginals(graph, results, gtsam::Marginals::CHOLESKY);

  marginals.print("marginals", prx::fg::symbol_factory_t::formatter);

  gtsam::JointMarginal joint = marginals.jointMarginalCovariance({ keyX(0, 0), keyX(1, 0) });
  DEBUG_VARS(joint.fullMatrix());
  for (int i = 0; i < 1; ++i)
  {
    std::cout << prx::fg::symbol_factory_t::formatter(keyX(1, i)) << ":\n";
    std::cout << results.at<Eigen::Vector4d>(keyX(1, i)) << "\n";
    std::cout << marginals.marginalCovariance(keyX(1, i)) << "\n";
  }
  std::cout << prx::fg::symbol_factory_t::formatter(keyX(0, 0)) << ":\n";
  std::cout << marginals.marginalCovariance(keyX(0, 0)) << "\n";
  std::cout << prx::fg::symbol_factory_t::formatter(keyU(0, 0)) << ":\n";
  std::cout << marginals.marginalCovariance(keyU(0, 0)) << "\n";
  std::cout << prx::fg::symbol_factory_t::formatter(keyH(0, 0)) << ":\n";
  std::cout << marginals.marginalCovariance(keyH(0, 0)) << "\n";
  std::cout << prx::fg::symbol_factory_t::formatter(keyW(0, 0)) << ":\n";
  std::cout << results.at<Eigen::Vector4d>(keyW(0, 0)) << "\n";
  Eigen::Matrix4d w_cov{ marginals.marginalCovariance(keyW(0, 0)) };
  std::cout << w_cov << "\n";
  // DEBUG_VARS(std::sqrt(w_cov(0, 0)));
  // DEBUG_VARS(std::sqrt(w_cov(1, 1)));
  return 0;
}