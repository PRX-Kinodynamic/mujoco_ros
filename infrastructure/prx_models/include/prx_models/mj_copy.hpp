#pragma once

namespace prx_models
{
/**
 * Mj copy to avoid extra copies to mj_data->ctrl (or other data structure).
 * For each controller, is only necesary to implement a copy function that takes a
 * reference to the control message (no pointer):
 *   template <typename SpacePoint>
 *   inline void copy(SpacePoint SpacePoint_out, const mj_models::RobotSpacePoint& msg)
 *   {
 *     (...)
 *   }
 */

template <typename SpacePoint, typename Msg>
inline void copy(SpacePoint SpacePoint_out, const boost::shared_ptr<Msg> msg)
{
  prx_models::copy(SpacePoint_out, *msg);
}

template <typename SpacePoint, typename Msg>
inline void copy(SpacePoint SpacePoint_out, const boost::shared_ptr<Msg const> msg)
{
  prx_models::copy(SpacePoint_out, *msg);
}

template <typename SpacePoint, typename Msg>
inline void copy(boost::shared_ptr<Msg> msg, SpacePoint SpacePoint_out)
{
  prx_models::copy(SpacePoint_out, *msg);
}

template <typename SpacePoint, typename Msg>
inline void copy(boost::shared_ptr<Msg const> msg, SpacePoint SpacePoint_out)
{
  prx_models::copy(SpacePoint_out, *msg);
}

}  // namespace prx_models