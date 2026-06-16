#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>
#include <utils/dbg_utils.hpp>

#include <gtsam/base/Testable.h>

#include <gtsam/config.h>

// #include <gtsam/nonlinear/Expression.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <prx/utilities/math/chi_squared.hpp>
#include <motion_planning/error_covariance_estimation.hpp>
// #include "utils/dbg.hpp"

// LieClusteringAlgorithm
namespace motion_planning
{

template <typename Element, typename Data = Element>
struct nonlinear_cluster_values_t
{
  static constexpr int Dim{ gtsam::traits<Element>::dimension };
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  std::vector<std::vector<Data>> data;
  std::vector<std::vector<Element>> clustered_elements;
  std::vector<gtsam::Key> keys;
  std::vector<Element> values;
  std::vector<gtsam::NonlinearFactorGraph> factor_graphs;

  std::vector<NoiseModel> noise_models;
  std::vector<std::size_t> total_clustered;

  double alpha;
  gtsam::GaussNewtonParams optimizer_params;

  nonlinear_cluster_values_t() : alpha(0.1)
  {
    optimizer_params.setMaxIterations(1);
  }

  friend void swap(nonlinear_cluster_values_t<Element, Data>& lhs, nonlinear_cluster_values_t<Element, Data>& rhs)
  {
    lhs.values.swap(rhs.values);
    lhs.factor_graphs.swap(rhs.factor_graphs);
    lhs.noise_models.swap(rhs.noise_models);
    lhs.total_clustered.swap(rhs.total_clustered);
    lhs.clustered_elements.swap(rhs.clustered_elements);
    lhs.data.swap(rhs.data);
    lhs.keys.swap(rhs.keys);

    std::swap(lhs.alpha, rhs.alpha);
    std::swap(lhs.optimizer_params, rhs.optimizer_params);
  }

  void push_back(const Element element, const Data input_data, NoiseModel nm = nullptr)
  {
    // values.emplace_back();
    factor_graphs.emplace_back();

    // noise_models.push_back(nm);
    total_clustered.push_back(1);
    data.push_back({ input_data });
    clustered_elements.push_back({ element });

    const std::size_t idx{ keys.size() };
    const gtsam::Key kx{ gtsam::Symbol('X', idx) };
    keys.push_back(kx);
    values.push_back(element);
    factor_graphs.back().addPrior(kx, element, nm);
  }

  void clear()
  {
    values.clear();
    factor_graphs.clear();
    noise_models.clear();
    total_clustered.clear();
    data.clear();
    keys.clear();
    clustered_elements.clear();
  }
};

template <typename Element, int Dim = gtsam::traits<Element>::dimension>
Eigen::Matrix<double, Dim, Dim> compute_cluster_covariance(const Element& centroid,
                                                           const std::vector<Element>& clustered_elements)
{
  Eigen::Matrix<double, Dim, Dim> cov{ Eigen::Matrix<double, Dim, Dim>::Zero() };

  for (auto xi : clustered_elements)
  {
    const Element btw{ gtsam::traits<Element>::Between(xi, centroid) };
    const Eigen::Vector<double, Dim> diff{ gtsam::traits<Element>::Logmap(btw) };
    // const Element diff{ gtsam::traits<Element>::Logmap(xi, mean) };  //(X-MU)*(X-MU)';
    //  gtsam::traits<X>:
    cov += diff * diff.transpose();
  }

  cov = cov / clustered_elements.size();
  return cov;
}

template <typename Element>
std::tuple<bool, gtsam::NonlinearFactorGraph, Element>
merge_factor_graphs(const gtsam::NonlinearFactorGraph& fg0, const gtsam::NonlinearFactorGraph& fg1,
                    const gtsam::Key& k0,
                    const gtsam::Key& k1,                  // no-lint
                    const Element& e0, const Element& e1,  // no-lint
                    prx::chi_squared& chi2, const double alpha, const gtsam::GaussNewtonParams& params)
{
  // rekey_mapping is a map of old->new keys
  const std::map<gtsam::Key, gtsam::Key> rekey_mapping{ { k1, k0 } };

  gtsam::NonlinearFactorGraph graph{ fg1.rekey(rekey_mapping) };
  graph.push_back(fg0);
  gtsam::Values values;
  values.insert(k0, e0);

  gtsam::Values result{ gtsam::GaussNewtonOptimizer(graph, values, params).optimize() };
  // result.print("result");
  const double error{ graph.error(result) };

  const double chi2_critical_value{ chi2.critical_value(graph.size(), alpha) };
  if (error > chi2_critical_value)
  {
    return { false, gtsam::NonlinearFactorGraph(), Element() };
  }
  else
  {
    return { true, graph, result.at<Element>(k0) };
  }
}

template <typename Element, typename Data>
static void cluster(nonlinear_cluster_values_t<Element, Data>& output,  // no-lint
                    const nonlinear_cluster_values_t<Element, Data>& input, prx::chi_squared& chi2)
{
  static constexpr Eigen::Index DimElement{ gtsam::traits<Element>::dimension };
  using ClusterValues = nonlinear_cluster_values_t<Element, Data>;
  using Covariance = Eigen::Matrix<double, DimElement, DimElement>;
  using GaussianNM = gtsam::noiseModel::Gaussian;

  // DEBUG_VARS(input.)
  if (input.values.size() == 0)
    return;

  gtsam::Values values;

  double prev_error{ 0 };
  double adjusted_error{ 0 };

  nonlinear_cluster_values_t<Element, Data> rejected;

  std::size_t clustered{ input.total_clustered.front() };
  Element e0{ input.values.front() };
  gtsam::Key k0{ input.keys.front() };
  gtsam::NonlinearFactorGraph fg0{ input.factor_graphs.front() };
  std::vector<Data> data{ input.data.front() };
  std::vector<Element> clustered_elements{ input.clustered_elements.front() };

  // input.values.erase(input.values.begin());
  // input.keys.erase(input.keys.begin());
  // input.data.erase(input.data.begin());
  // input.factor_graphs.erase(input.factor_graphs.begin());
  // input.clustered_elements.erase(input.clustered_elements.begin());

  for (int i = 1; i < input.factor_graphs.size(); ++i)
  {
    const gtsam::Key& k1{ input.keys[i] };
    const Element& e1{ input.values[i] };
    const gtsam::NonlinearFactorGraph& fg1{ input.factor_graphs[i] };

    auto [merge, new_graph, cluster_element] =
        merge_factor_graphs(fg0, fg1, k0, k1, e0, e1, chi2, input.alpha, input.optimizer_params);

    if (merge)
    {
      fg0 = new_graph;
      e0 = cluster_element;
      clustered_elements.insert(clustered_elements.end(),  // no-lint
                                input.clustered_elements[i].begin(), input.clustered_elements[i].end());
      data.insert(data.end(), input.data[i].begin(), input.data[i].end());
      clustered += input.total_clustered[i];
    }
    else
    {
      rejected.keys.push_back(k1);
      rejected.values.push_back(e1);
      rejected.factor_graphs.push_back(fg1);
      // rejected.noise_models.push_back(input.noise_models[i]);
      rejected.total_clustered.push_back(input.total_clustered[i]);
      rejected.data.push_back(input.data[i]);
      rejected.clustered_elements.push_back(input.clustered_elements[i]);
    }
  }

  output.factor_graphs.push_back(fg0);
  output.keys.push_back(k0);
  output.values.push_back(e0);
  output.total_clustered.push_back(clustered);
  output.data.push_back(data);
  output.clustered_elements.push_back(clustered_elements);

  cluster(output, rejected, chi2);
}

template <typename Element, typename Data>
static void cluster_multiple_iterations(nonlinear_cluster_values_t<Element, Data>& output,       // no-lint
                                        const nonlinear_cluster_values_t<Element, Data>& input,  // no-lint
                                        int& max_steps)
{
  if (max_steps == 0)
    return;
  prx::chi_squared chi2(0.001);

  std::size_t prev_size{ input.values.size() };

  nonlinear_cluster_values_t<Element, Data> output_aux, input_aux;

  cluster(output_aux, input, chi2);

  // DEBUG_VARS(output_aux.noise_models);
  max_steps--;
  while (max_steps != 0 and prev_size > output_aux.values.size())
  {
    input_aux.clear();

    swap(output_aux, input_aux);

    prev_size = input_aux.values.size();

    cluster(output_aux, input_aux, chi2);

    max_steps--;
    // DEBUG_VARS(max_steps, prev_size, output_aux.values.size())
  }

  swap(output, output_aux);
}

}  // namespace motion_planning