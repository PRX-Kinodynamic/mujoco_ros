#pragma once

namespace prx_models
{
/**
 * Mj copy to avoid extra copies to mj_data->ctrl (or other data structure).
 * For each controller, is only necesary to implement a copy function that takes a
 * reference to the control message (no pointer):
 *   template <typename Ctrl>
 *   inline void copy(Ctrl ctrl_out, const mj_models::RobotCtrl& msg)
 *   {
 *     (...)
 *   }
 */

template <typename Ctrl, typename Msg>
inline void copy(Ctrl ctrl_out, const boost::shared_ptr<Msg> msg)
{
  copy(ctrl_out, *msg);
}

template <typename Ctrl, typename Msg>
inline void copy(Ctrl ctrl_out, const boost::shared_ptr<Msg const> msg)
{
  copy(ctrl_out, *msg);
}
}  // namespace prx_models