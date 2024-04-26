#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace estimation
{
namespace fg
{

class trajectory_estimation
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  /**
    Factor graph at some time t:
                                 (Integration)
                           (x_t) ----- * ----- (x_{t+1})
                                       |
                     (Dynamic)         |
    (\dot{x_{t}}) ----- * ----- (\dot{x_{t+1}})
                        |
                        |
                (\ddot{x_t} = u_t)
   */
public:
  /**
   * @brief Creates the small factor graph above: between t and t+1
   * @details Create a small factor graph x_{t+1} <- x_t + xdot * dt AND \dot{x}_{t+1} <- \dot{x}_t + \ddot{x} * dt  AND
   * \hat{x}_t <- x_t + noise ~= z intended to be an update to isam2, accounting for a new observation of a state.
   *
   * @param idx index of the observation. First observation is always 0
   * @param dt The time between x[idx] and x[idx+1]
   * @param keyX A function that produces a key for X taking as input idx
   * @param keyXdot A function that produces a key for Xdot taking as input idx
   * @param integration_noise Noise model to use for integration
   * @param observation_noise Noise model to use for observation
   * @tparam IntegrationFactor Integration class that does x_{t+1} <- x_t + xdot * dt in that order: (x(idx+1), x(idx) +
   * xdot(idx))
   * @return A factor graph
   */
  template <typename IntegrationFactor, typename DynamicFactor, typename ObervationFactor, typename Z, typename KeyFnX,
            typename KeyFnXdot, typename KeyFnXdotdot>
  static gtsam::NonlinearFactorGraph get_dyn_factor(const std::size_t idx, const double dt, const Z& z, KeyFnX& keyX,
                                                    KeyFnXdot& keyXdot, KeyFnXdotdot& keyXdotdot,
                                                    const NoiseModel& integration_noise = nullptr,
                                                    const NoiseModel& dynamic_noise = nullptr,
                                                    const NoiseModel& observation_noise = nullptr)
  {
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<IntegrationFactor>(keyX(idx + 1), keyX(idx), keyXdot(idx), integration_noise, dt);
    graph.emplace_shared<DynamicFactor>(keyXdot(idx + 1), keyXdot(idx), keyXdotdot(idx), dynamic_noise, dt);
    graph.emplace_shared<ObervationFactor>(keyX(idx), z, observation_noise);
    return graph;
  }

  template <typename X, typename Xdot, typename Xdotdot, typename KeyFnX, typename KeyFnXdot, typename KeyFnXdotdot>
  static gtsam::Values get_dyn_values(const std::size_t idx, const X& x1, const Xdot& xdot, const Xdotdot& xdotdot,
                                      KeyFnX& keyX, KeyFnXdot& keyXdot, KeyFnXdotdot& keyXdotdot)
  {
    gtsam::Values values;
    values.insert(keyXdotdot(idx), xdotdot);
    values.insert(keyXdot(idx + 1), xdot);
    values.insert(keyX(idx + 1), x1);
    // values.insert(keyX(idx), value);
    return values;
  }
};

}  // namespace fg
}  // namespace estimation