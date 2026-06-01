#pragma once

#include <prx/utilities/spaces/sampler.hpp>
#include <ml4kp_bridge/product_lie_group.hpp>
#include "utils/dbg_utils.hpp"

namespace prx
{

template <typename G, typename H>
class sampler_t<gtsam::ProductLieGroupV43<G, H>>
{
public:
  using Sampler = sampler_t<gtsam::ProductLieGroup<G, H>>;
  using Element = gtsam::ProductLieGroup<G, H>;
  using GBounds = typename sampler_t<G>::Bounds;
  using HBounds = typename sampler_t<H>::Bounds;
  using Bounds = std::pair<GBounds, HBounds>;

  // template <typename GBounds, typename HBounds>
  sampler_t(const GBounds Gmin, const GBounds Gmax, const HBounds Hmin, const HBounds Hmax)
    : _G_sampler(Gmin, Gmax), _H_sampler(Hmin, Hmax)
  {
  }

  sampler_t() : _G_sampler(), _H_sampler()
  {
  }

  static prx::param_loader advance(prx::param_loader& param, int distance)
  {
    auto iter = param.begin();
    // for (auto iter = params.begin(); iter != params.end(); iter++)
    for (int i = 0; i < distance; ++i)
    {
      iter++;
    }
    PRX_DBG_VARS(*iter)
    return *iter;
  }

  sampler_t(prx::param_loader params) : sampler_t(advance(params, 0), advance(params, 1))
  {
    // std::advance(params.begin(), 1);
    // prx::param_loader next(params.begin()++, params.end());
    // PRX_DBG_VARS(params);
    // // PRX_DBG_VARS(next);
    // // PRX_DBG_VARS(next_0);
    // for (auto iter = params.begin(); iter != params.end(); iter++)
    // {
    //   prx::param_loader next(*iter);  //, params.end());
    //   PRX_DBG_VARS(next);
    // }
  }

  sampler_t(prx::param_loader Gparams, prx::param_loader Hparams) : _G_sampler(Gparams), _H_sampler(Hparams)
  {
    PRX_DBG_VARS(Gparams);
    PRX_DBG_VARS(Hparams);
  }

  Element operator()()
  {
    return std::move(Element(_G_sampler(), _H_sampler()));
  }

  void bounds(const GBounds Gmin, const GBounds Gmax, const HBounds Hmin, const HBounds Hmax)
  {
    _G_sampler.bounds(Gmin, Gmax);
    _H_sampler.bounds(Hmin, Hmax);
  }

  std::pair<Bounds, Bounds> bounds()
  {
    return { _G_sampler.bounds(), _H_sampler.bounds() };
  }

  // void bounds(const GBounds Gmin, const GBounds Gmax, const HBounds Hmin, const HBounds Hmax)

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    prx::streamer_t<sampler_t<G>>::to_stream(os, obj._G_sampler);
    prx::streamer_t<sampler_t<H>>::to_stream(os, obj._H_sampler);
    return os;
  }

protected:
  sampler_t<G> _G_sampler;
  sampler_t<H> _H_sampler;
};
}  // namespace prx