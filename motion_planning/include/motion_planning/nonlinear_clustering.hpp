#pragma once
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
struct cluster_t
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  std::size_t idx;

  std::vector<Data> data;
  std::vector<Element> clustered_elements;

  gtsam::Key key;
  Element element;
  gtsam::NonlinearFactorGraph factor_graph;

  // If true, the data might increase, but the element (~mean) will remain fixed
  // This is, no more updates to this cluster
  bool locked;

  // NoiseModel noise_models;
  std::size_t total_clustered;

  inline static std::size_t next_idx{ 0 };

  cluster_t() = delete;

  cluster_t(const Element& element_, const Data& input_data, NoiseModel nm)
    : idx(next_idx)
    , total_clustered(1)
    , key(gtsam::Symbol('X', idx))
    , element(element_)
    , data({ input_data })
    , clustered_elements({ element_ })
    , locked(false)
  {
    next_idx++;
    factor_graph.addPrior(key, element, nm);
  }

  cluster_t(const std::size_t idx_, const gtsam::NonlinearFactorGraph& fg_, const gtsam::Key& key_,
            const Element& element_, const std::vector<Data>& data_, const std::vector<Element>& clustered_elements_,
            const std::size_t total_clustered_)
    : idx(idx_)
    , factor_graph(fg_)
    , key(key_)
    , element(element_)
    , total_clustered(total_clustered_)
    , data(data_)
    , clustered_elements(clustered_elements_)
  {
    // data. .swap(data_);
    // clustered_elements.swap(clustered_elements);
  }

  void clear()
  {
    // values.clear();
    total_clustered = 0;

    factor_graph = gtsam::NonlinearFactorGraph();
    data.clear();
    clustered_elements.clear();
    locked = false;
  }

  // friend void swap(cluster_t<Element, Data>& lhs, cluster_t<Element, Data>& rhs)
  // {
  //   lhs.values.swap(rhs.values);
  //   lhs.factor_graphs.swap(rhs.factor_graphs);
  //   std::swap(lhs.total_clustered, rhs.total_clustered);
  //   lhs.clustered_elements.swap(rhs.clustered_elements);
  //   lhs.data.swap(rhs.data);
  //   lhs.keys.swap(rhs.keys);
  // }
};

template <typename Element, typename Data = Element>
struct nonlinear_cluster_values_t
{
  static constexpr int Dim{ gtsam::traits<Element>::dimension };
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  // std::vector<std::vector<Data>> data;
  // std::vector<std::vector<Element>> clustered_elements;
  // std::vector<gtsam::Key> keys;
  // std::vector<Element> values;
  // std::vector<gtsam::NonlinearFactorGraph> factor_graphs;

  // std::vector<NoiseModel> noise_models;
  // std::vector<std::size_t> total_clustered;
  std::vector<cluster_t<Element, Data>> clusters;

  double alpha;
  gtsam::GaussNewtonParams optimizer_params;

  nonlinear_cluster_values_t() : alpha(0.1)
  {
    optimizer_params.setMaxIterations(1);
  }

  friend void swap(nonlinear_cluster_values_t<Element, Data>& lhs, nonlinear_cluster_values_t<Element, Data>& rhs)
  {
    // lhs.values.swap(rhs.values);
    // lhs.factor_graphs.swap(rhs.factor_graphs);
    // lhs.noise_models.swap(rhs.noise_models);
    // lhs.total_clustered.swap(rhs.total_clustered);
    // lhs.clustered_elements.swap(rhs.clustered_elements);
    // lhs.data.swap(rhs.data);
    // lhs.keys.swap(rhs.keys);
    lhs.clusters.swap(rhs.clusters);

    std::swap(lhs.alpha, rhs.alpha);
    std::swap(lhs.optimizer_params, rhs.optimizer_params);
  }

  void push_back(const Element element, const Data input_data, NoiseModel nm = nullptr)
  {
    clusters.emplace_back(element, input_data, nm);
  }

  void clear()
  {
    clusters.clear();
    // values.clear();
    // factor_graphs.clear();
    // noise_models.clear();
    // total_clustered.clear();
    // data.clear();
    // keys.clear();
    // clustered_elements.clear();
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
template <typename Element, typename Data>
bool merge_clusters(cluster_t<Element, Data>& c0, const cluster_t<Element, Data>& c1,  // no-lint
                    prx::chi_squared& chi2, const double alpha, const gtsam::GaussNewtonParams& params)
{
  // if (c0.total_clustered + c1.total_clustered < 2)
  // {

  // }
  const std::map<gtsam::Key, gtsam::Key> rekey_mapping{ { c1.key, c0.key } };

  gtsam::NonlinearFactorGraph graph{ c1.factor_graph.rekey(rekey_mapping) };
  graph.push_back(c0.factor_graph);
  gtsam::Values values;
  values.insert(c0.key, c0.element);

  double error{ 0.0 };
  gtsam::Values result;
  try
  {
    result = gtsam::GaussNewtonOptimizer(graph, values, params).optimize();
    error = graph.error(result);
  }
  catch (gtsam::IndeterminantLinearSystemException e)
  {
    DEBUG_VARS(c0.total_clustered, c1.total_clustered)
    DEBUG_VARS(c0.element, c1.element)
    PRINT_MSG(e.what());
    return false;
  }
  const double chi2_critical_value{ chi2.critical_value(graph.size(), alpha) };
  // if (error > chi2_critical_value)
  // {
  //   return { false, gtsam::NonlinearFactorGraph(), Element() };
  // }
  // else
  // {
  //   return { true, graph, result.at<Element>(c0.key) };
  // }
  // if (c0.locked and error < chi2_critical_value)
  // {
  //   return true;
  // }

  if (error < chi2_critical_value)
  {
    if (not c0.locked)
    {
      // if (notc0.locked)
      // {
      c0.factor_graph = graph;
      c0.element = result.at<Element>(c0.key);
      // }

      c0.clustered_elements.insert(c0.clustered_elements.end(),  // no-lint
                                   c1.clustered_elements.begin(), c1.clustered_elements.end());
      c0.data.insert(c0.data.end(), c1.data.begin(), c1.data.end());
      c0.total_clustered += c1.total_clustered;
    }
    return true;
  }
  return false;
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
static void cluster_recursive(nonlinear_cluster_values_t<Element, Data>& output,  // no-lint
                              const nonlinear_cluster_values_t<Element, Data>& input, prx::chi_squared& chi2)
{
  static constexpr Eigen::Index DimElement{ gtsam::traits<Element>::dimension };
  using ClusterValues = nonlinear_cluster_values_t<Element, Data>;
  using Cluster = cluster_t<Element, Data>;
  using Covariance = Eigen::Matrix<double, DimElement, DimElement>;
  using GaussianNM = gtsam::noiseModel::Gaussian;

  // DEBUG_VARS(input.)
  if (input.clusters.size() == 0)
    return;

  // gtsam::Values values;

  double prev_error{ 0 };
  double adjusted_error{ 0 };

  nonlinear_cluster_values_t<Element, Data> rejected;

  Cluster c0{ input.clusters.front() };

  // std::size_t cluster_idx{ cluster.idx };
  // std::size_t clustered{ cluster.total_clustered };
  // Element e0{ cluster.element };
  // gtsam::Key k0{ cluster.key };
  // gtsam::NonlinearFactorGraph fg0{ cluster.factor_graph };
  // std::vector<Data> data{ cluster.data };
  // std::vector<Element> clustered_elements{ cluster.clustered_elements };

  for (int i = 1; i < input.clusters.size(); ++i)
  {
    const Cluster& ci{ input.clusters[i] };
    // const gtsam::Key& k1{ cl_i.key };
    // const Element& e1{ cl_i.element };
    // const gtsam::NonlinearFactorGraph& fg1{ cl_i.factor_graph };

    // auto [merge, new_graph, cluster_element] =
    //     merge_factor_graphs(fg0, fg1, k0, k1, e0, e1, chi2, input.alpha, input.optimizer_params);

    // const bool merged{ false };
    const bool merged{ merge_clusters(c0, ci, chi2, input.alpha, input.optimizer_params) };
    // if (merge)
    // {
    //   fg0 = new_graph;
    //   e0 = cluster_element;
    //   clustered_elements.insert(clustered_elements.end(),  // no-lint
    //                             cl_i.clustered_elements.begin(), cl_i.clustered_elements.end());
    //   data.insert(data.end(), cl_i.data.begin(), cl_i.data.end());
    //   clustered += cl_i.total_clustered;
    // }
    // else
    if (not merged)
    {
      // const std::size_t rejected_idx{ cl_i.idx };
      // const std::size_t rejected_total_clustered{ input.clusters[i].total_clustered };
      rejected.clusters.emplace_back(ci);
      // rejected.clusters.emplace_back(cl_i.idx, fg1, k1, e1, data, clustered_elements, cl_i.total_clustered);

      // rejected.keys.push_back(k1);
      // rejected.values.push_back(e1);
      // rejected.factor_graphs.push_back(fg1);
      // // rejected.noise_models.push_back(input.noise_models[i]);
      // rejected.total_clustered.push_back(input.total_clustered[i]);
      // rejected.data.push_back(input.data[i]);
      // rejected.clustered_elements.push_back(input.clustered_elements[i]);
    }
  }

  output.clusters.emplace_back(c0);
  // output.clusters.emplace_back(cluster_idx, fg0, k0, e0, data, clustered_elements, clustered);
  // output.factor_graphs.push_back(fg0);
  // output.keys.push_back(k0);
  // output.values.push_back(e0);
  // output.total_clustered.push_back(clustered);
  // output.data.push_back(data);
  // output.clustered_elements.push_back(clustered_elements);

  cluster_recursive(output, rejected, chi2);
}

template <typename Element, typename Data>
static void cluster_multiple_iterations(nonlinear_cluster_values_t<Element, Data>& output,       // no-lint
                                        const nonlinear_cluster_values_t<Element, Data>& input,  // no-lint
                                        int& max_steps)
{
  if (max_steps == 0)
    return;
  prx::chi_squared chi2(0.001);

  std::size_t prev_size{ input.clusters.size() };

  nonlinear_cluster_values_t<Element, Data> output_aux, input_aux;

  DEBUG_VARS(input.clusters.size())
  cluster_recursive(output_aux, input, chi2);
  DEBUG_VARS(output_aux.clusters.size())

  // DEBUG_VARS(output_aux.noise_models);
  max_steps--;
  while (max_steps != 0 and prev_size > output_aux.clusters.size())
  {
    input_aux.clear();

    swap(output_aux, input_aux);

    prev_size = input_aux.clusters.size();

    DEBUG_VARS(input_aux.clusters.size())
    cluster_recursive(output_aux, input_aux, chi2);
    DEBUG_VARS(output_aux.clusters.size())

    max_steps--;
    // DEBUG_VARS(max_steps, prev_size, output_aux.values.size())
  }

  swap(output, output_aux);
}

}  // namespace motion_planning