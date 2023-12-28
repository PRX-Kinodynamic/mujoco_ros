#pragma once

#include <ml4kp_bridge/template_utils.hpp>

namespace interface
{
// Copy implementations:
// 1) copy(Obj, Obj) --> Implemented in another, obj-specific file
// 2) copy(Obj, Ptr)
// 3) copy(Ptr, Obj)
// 4) copy(Ptr, Ptr)

// Version 2
template <typename To, std::enable_if_t<not ml4kp_bridge::is_any_ptr<To>::value, bool> = true,  // no-lint
          typename From, std::enable_if_t<ml4kp_bridge::is_any_ptr<From>::value, bool> = true>
inline void translate_msg(To& to, const From& from)
{
  translate_msg(to, *from);
}

// Version 3
template <typename To, std::enable_if_t<ml4kp_bridge::is_any_ptr<To>::value, bool> = true,  // no-lint
          typename From, std::enable_if_t<not ml4kp_bridge::is_any_ptr<From>::value, bool> = true>
inline void translate_msg(To& to, const From& from)
{
  translate_msg(*to, from);
}

// Version 4
template <typename To, std::enable_if_t<ml4kp_bridge::is_any_ptr<To>::value, bool> = true,  // no-lint
          typename From, std::enable_if_t<ml4kp_bridge::is_any_ptr<From>::value, bool> = true>
inline void translate_msg(To& to, const From& from)
{
  translate_msg(*to, *from);
}
}  // namespace interface