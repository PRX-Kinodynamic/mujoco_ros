#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>

#include <gtsam/base/Testable.h>

#include <gtsam/config.h>

// #include <gtsam/nonlinear/Expression.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// #include "utils/dbg.hpp"

// LieClusteringAlgorithm
namespace motion_planning
{

template <typename Element, typename Data = Element>
struct cluster_in_out_t
{
  std::vector<std::vector<Data>> original_elements;
  std::vector<Element> values;
  std::vector<gtsam::JacobianFactor::shared_ptr> priors;

  std::vector<gtsam::SharedGaussian> noise_models;
  std::vector<std::size_t> total_clustered;

  friend void swap(cluster_in_out_t<Element, Data>& lhs, cluster_in_out_t<Element, Data>& rhs)
  {
    lhs.values.swap(rhs.values);
    lhs.priors.swap(rhs.priors);
    lhs.noise_models.swap(rhs.noise_models);
    lhs.total_clustered.swap(rhs.total_clustered);
    lhs.original_elements.swap(rhs.original_elements);
  }

  template <typename NoiseModel>
  void push_back(const Element element, NoiseModel nm, const Data data)
  {
    values.push_back(element);
    priors.push_back(nullptr);
    noise_models.push_back(nm);
    total_clustered.push_back(1);
    original_elements.push_back({ data });
  }

  void clear()
  {
    values.clear();
    priors.clear();
    noise_models.clear();
    total_clustered.clear();
    original_elements.clear();
  }
};

template <typename Covariance, typename Element>
void compute_cluster_covariance(Covariance& cov, const Element& mean, const std::vector<Element>& clustered_elements,
                                const int prev_clustered, const bool verbose = false)
{
  static constexpr Eigen::Index DimElement{ gtsam::traits<Element>::dimension };
  if (verbose)
    DEBUG_VARS(cov);

  cov = cov * prev_clustered;

  if (verbose)
    DEBUG_VARS(cov);

  for (auto xi : clustered_elements)
  {
    const Element btw{ gtsam::traits<Element>::Between(xi, mean) };
    const Eigen::Vector<double, DimElement> diff{ gtsam::traits<Element>::Logmap(btw) };
    // const Element diff{ gtsam::traits<Element>::Logmap(xi, mean) };  //(X-MU)*(X-MU)';
    //  gtsam::traits<X>:
    cov += diff * diff.transpose();

    if (verbose)
    {
      // DEBUG_VARS(btw)
      DEBUG_VARS(diff.transpose());
    }
  }
  if (verbose)
  {
    DEBUG_VARS(cov, clustered_elements.size(), prev_clustered);
  }

  cov = cov / (clustered_elements.size() + prev_clustered);
  if (verbose)
    DEBUG_VARS(cov);
}

template <typename Element, typename Data>
static void cluster(cluster_in_out_t<Element, Data>& output,       // no-lint
                    const cluster_in_out_t<Element, Data>& input,  // no-lint
                    const double confidence)
{
  static constexpr Eigen::Index DimElement{ gtsam::traits<Element>::dimension };
  using Covariance = Eigen::Matrix<double, DimElement, DimElement>;
  using GaussianNM = gtsam::noiseModel::Gaussian;

  if (input.values.size() == 0)
    return;

  gtsam::Values values;

  double prev_error{ 0 };
  double adjusted_error{ 0 };

  cluster_in_out_t<Element, Data> rejected;

  const gtsam::Key key{ gtsam::Symbol('X', 0) };
  values.insert(key, input.values[0]);
  std::size_t clustered{ 0 };

  // DEBUG_VARS(input.priors.size());
  gtsam::JacobianFactor::shared_ptr prior{ input.priors[0] };
  gtsam::Ordering key_ordering;
  key_ordering += key;

  std::vector<Element> clustered_elements;
  std::vector<Data> original_elements;

  int prev_clustered{ 0 };
  gtsam::SharedGaussian prev_noise = nullptr;
  for (int i = 0; i < input.values.size(); ++i)
  {
    const Element& zi{ input.values[i] };
    const gtsam::SharedGaussian z_noise{ input.noise_models[i] };

    gtsam::GaussianFactorGraph linear_fg;

    if (prior != nullptr)
    {
      linear_fg.push_back(prior);
    }

    const gtsam::PriorFactor<Element> curr_prior(key, zi, z_noise);
    linear_fg.push_back(curr_prior.linearize(values));

    const gtsam::GaussianConditional::shared_ptr marginal{
      linear_fg.marginalMultifrontalBayesNet(key_ordering)->front()
    };

    const gtsam::VectorValues result{ marginal->solve(gtsam::VectorValues()) };
    const double error{ linear_fg.error(result) };

    if (error > confidence)
    {
      rejected.values.push_back(zi);
      rejected.priors.push_back(input.priors[i]);
      rejected.noise_models.push_back(input.noise_models[i]);
      rejected.total_clustered.push_back(input.total_clustered[i]);
      rejected.original_elements.push_back(input.original_elements[i]);
    }
    else
    {
      const Element& current{ values.at<Element>(key) };
      const Element x{ gtsam::traits<Element>::Retract(current, result[key]) };

      values.update(key, x);
      prior = boost::make_shared<gtsam::JacobianFactor>(
          marginal->keys().front(), marginal->getA(marginal->begin()),
          marginal->getb() - marginal->getA(marginal->begin()) * result[key], marginal->get_model());
      prev_error = error;
      clustered += input.total_clustered[i];

      original_elements.insert(original_elements.end(),  // no-lint
                               input.original_elements[i].begin(), input.original_elements[i].end());

      clustered_elements.push_back(zi);
      if (not prev_noise)
      {
        prev_noise = z_noise;
        prev_clustered = input.total_clustered[i];
      }
    }
  }

  const Element& res{ values.at<Element>(key) };
  // Covariance cov{ prev_noise->covariance() };
  // const bool verbose{ output.values.size() == 0 };
  // compute_cluster_covariance(cov, res, clustered_elements, prev_clustered, false);
  // const bool valid_cov{ cov.inverse().allFinite() };

  // gtsam::SharedGaussian res_nm;
  // if (valid_cov)
  // {
  //   res_nm = GaussianNM::Covariance(cov);
  // }
  // else
  // {
  //   res_nm = prev_noise;
  // }

  output.priors.push_back(prior);
  output.values.push_back(res);
  output.noise_models.push_back(prev_noise);
  output.total_clustered.push_back(clustered);
  output.original_elements.push_back(original_elements);

  // DEBUG_VARS(output.values.size());
  // DEBUG_VARS(output.priors.size());
  cluster(output, rejected, confidence);
}

template <typename Element, typename Data>
static void cluster_multiple_iterations(cluster_in_out_t<Element, Data>& output,       // no-lint
                                        const cluster_in_out_t<Element, Data>& input,  // no-lint
                                        const double confidence, int& max_steps)
{
  if (max_steps == 0)
    return;

  std::size_t prev_size{ input.values.size() };

  cluster_in_out_t<Element, Data> output_aux, input_aux;

  cluster(output_aux, input, confidence);

  // DEBUG_VARS(output_aux.noise_models);
  max_steps--;
  while (max_steps != 0 and prev_size > output_aux.values.size())
  {
    input_aux.clear();

    swap(output_aux, input_aux);

    prev_size = input_aux.values.size();

    cluster(output_aux, input_aux, confidence);

    max_steps--;
    // DEBUG_VARS(max_steps, prev_size, output_aux.values.size())
  }

  swap(output, output_aux);
}

template <typename Element, typename Data>
static std::vector<Data> query(const cluster_in_out_t<Element, Data>& input,  // no-lint
                               const Element query_element, const double confidence)
{
  static constexpr Eigen::Index DimElement{ gtsam::traits<Element>::dimension };
  using Covariance = Eigen::Matrix<double, DimElement, DimElement>;
  using GaussianNM = gtsam::noiseModel::Gaussian;

  if (input.values.size() == 0)
    return {};

  gtsam::Values values;

  double prev_error{ 0 };
  double adjusted_error{ 0 };

  double min_error{ std::numeric_limits<double>::max() };
  std::vector<Data> best_data;

  const gtsam::Key key{ gtsam::Symbol('X', 0) };
  values.insert(key, query_element);

  gtsam::Ordering key_ordering;
  key_ordering += key;

  auto nm = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  const gtsam::PriorFactor<Element> curr_prior(key, query_element, nm);
  auto linearized_factor = curr_prior.linearize(values);
  for (int i = 0; i < input.values.size(); ++i)
  {
    const Element& zi{ input.values[i] };
    const gtsam::SharedGaussian z_noise{ input.noise_models[i] };

    gtsam::GaussianFactorGraph linear_fg;

    linear_fg.push_back(linearized_factor);

    const gtsam::PriorFactor<Element> proposed_prior(key, zi, z_noise);
    linear_fg.push_back(proposed_prior.linearize(values));

    const gtsam::GaussianConditional::shared_ptr marginal{
      linear_fg.marginalMultifrontalBayesNet(key_ordering)->front()
    };

    const gtsam::VectorValues result{ marginal->solve(gtsam::VectorValues()) };
    const double error{ linear_fg.error(result) };

    // DEBUG_VARS(zi)
    // DEBUG_VARS(error)
    if (error < confidence)
    {
      return input.original_elements[i];
    }
    if (min_error > error)
    {
      best_data = input.original_elements[i];
      min_error = error;
    }
  }
  // DEBUG_VARS(min_error)
  // return { 20.0 };
  return best_data;
  // const Element& res{ values.at<Element>(key) };
  // Covariance cov{ prev_noise->covariance() };
  // // const bool verbose{ output.values.size() == 0 };
  // compute_cluster_covariance(cov, res, clustered_elements, prev_clustered, false);
  // const bool valid_cov{ cov.inverse().allFinite() };
}
}  // namespace motion_planning