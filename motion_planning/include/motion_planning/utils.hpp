#pragma once
#include <prx/factor_graphs/utilities/dbg_utills.hpp>

namespace motion_planning
{
using SF = prx::fg::symbol_factory_t;

template <typename StateType>
void estimate_to_stream(std::ostream& ofs, const gtsam::Key& key, const StateType& state)
{
  using SF = prx::fg::symbol_factory_t;

  ofs << SF::formatter(key) << " ";
  for (int i = 0; i < state.size(); ++i)
  {
    ofs << state[i] << " ";
  }
}

template <std::size_t I, typename StateEstimates, typename StateKeys,
          std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>
inline void update_estimates(StateEstimates& state_estimates, gtsam::ISAM2& isam, const StateKeys& keys)
{
}

// StateEstimates is a tuple (std::tuple<...>) and I is the id of an element in the tuple
template <std::size_t I, typename StateEstimates, typename StateKeys,
          std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
inline void update_estimates(StateEstimates& state_estimates, gtsam::ISAM2& isam, const StateKeys& keys)
{
  using SF = prx::fg::symbol_factory_t;
  using EstimateType = typename std::tuple_element<I, StateEstimates>::type;
  try
  {
    std::get<I>(state_estimates) = isam.calculateEstimate<EstimateType>(keys[I]);
    // values.insert_or_assign(keys[I], state_estimates);
  }
  catch (std::out_of_range e)
  {
    DEBUG_VARS(I, keys[I], SF::formatter(keys[I]));
    DEBUG_VARS(e.what());
    throw e;
  }
  update_estimates<I + 1>(state_estimates, isam, keys);
}

template <std::size_t I, typename StateEstimates, typename StateKeys,
          std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>
inline void update_values(gtsam::Values& values, gtsam::ISAM2& isam, const StateKeys& keys)
{
}

// StateEstimates is a tuple (std::tuple<...>) and I is the id of an element in the tuple
template <std::size_t I, typename StateEstimates, typename StateKeys,
          std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
inline void update_values(gtsam::Values& values, gtsam::ISAM2& isam, const StateKeys& keys)
{
  using SF = prx::fg::symbol_factory_t;
  using EstimateType = typename std::tuple_element<I, StateEstimates>::type;
  try
  {
    values.update(keys[I], isam.calculateEstimate<EstimateType>(keys[I]));
  }
  catch (std::out_of_range e)
  {
    DEBUG_VARS(I, keys[I], SF::formatter(keys[I]));
    DEBUG_VARS(e.what());
    throw e;
  }
  update_values<I + 1, StateEstimates>(values, isam, keys);
}

template <std::size_t I, typename StateEstimates, typename StateKeys,
          std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
double compute_error(const StateEstimates& estimates, const StateKeys& keys, gtsam::ISAM2& isam,
                     gtsam::Values& sbmp_values)
{
  return 0;
}

template <std::size_t I, typename StateEstimates, typename StateKeys,
          std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
double compute_error(const StateEstimates& estimates, const StateKeys& keys, gtsam::ISAM2& isam,
                     gtsam::Values& sbmp_values)
{
  using StateType = typename std::tuple_element<I, StateEstimates>::type;
  static constexpr Eigen::Index N{ gtsam::traits<StateType>::dimension };

  const gtsam::Key& key{ keys[I] };
  const StateType& x{ std::get<I>(estimates) };
  const StateType x_sbmp{ sbmp_values.at<StateType>(key) };

  const Eigen::MatrixXd cov{ isam.marginalCovariance(key) };
  const Eigen::MatrixXd S{ cov.inverse() };

  // const StateType diff{ x - x_sbmp };
  const StateType between{ gtsam::traits<StateType>::Between(x, x_sbmp) };  //
  const Eigen::VectorXd diff{ gtsam::traits<StateType>::Logmap(between) };

  const Eigen::VectorXd cost{ diff.transpose() * S * diff };
  return cost[0] + compute_error<I + 1>(estimates, keys, isam, sbmp_values);
}

void covariance_diagonal_to_stream(std::ostream& ofs, const gtsam::Key& key, gtsam::ISAM2& isam)
{
  try
  {
    // DEBUG_VARS(SF::formatter(key));
    const Eigen::MatrixXd cov{ isam.marginalCovariance(key) };
    // DEBUG_VARS(cov);
    // const Eigen::VectorXd diagonal{ cov.diagonal() };
    for (auto row : cov.rowwise())
    {
      for (auto e : row)
      {
        ofs << e << " ";
      }
    }
  }
  catch (std::out_of_range e)
  {
    const std::string problem_key{ SF::formatter(key) };
    PRINT_MSG_VARS("Can't compute covariance", problem_key);
    DEBUG_VARS(e.what());
  }
}

template <typename StateEstimates, std::size_t I, typename StateKeys,
          std::enable_if_t<(I == std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
void estimates_to_file(std::ostream& ofs, const gtsam::Values& estimate, const StateKeys& keys, gtsam::ISAM2& isam,
                       const bool with_covariance)
{
  ofs << "\n";
}

template <typename StateEstimates, std::size_t I, typename StateKeys,
          std::enable_if_t<(I < std::tuple_size<StateEstimates>{}), bool> = true>  // no-lint
void estimates_to_file(std::ostream& ofs, const gtsam::Values& estimate, const StateKeys& keys, gtsam::ISAM2& isam,
                       const bool with_covariance = true)
{
  using StateType = typename std::tuple_element<I, StateEstimates>::type;
  const gtsam::Key key{ keys[I] };
  const StateType state{ estimate.at<StateType>(key) };

  estimate_to_stream(ofs, key, state);
  if (with_covariance)
    covariance_diagonal_to_stream(ofs, key, isam);

  estimates_to_file<StateEstimates, I + 1>(ofs, estimate, keys, isam, with_covariance);
}

}  // namespace motion_planning