#pragma once

#include <prx/utilities/spaces/space.hpp>
#include <ml4kp_bridge/template_utils.hpp>

namespace ml4kp_bridge
{

// Copy implementations:
// 1) copy(Obj, Obj) --> Implemented in another, obj-specific file
// 2) copy(Obj, Ptr)
// 3) copy(Ptr, Obj)
// 4) copy(Ptr, Ptr)

// Version 2
template <typename In, std::enable_if_t<not is_any_ptr<In>::value, bool> = true,  // no-lint
          typename Out, std::enable_if_t<is_any_ptr<Out>::value, bool> = true>
inline void copy(In& msg, const Out state)
{
  copy(msg, *state);
}

// Version 3
template <typename In, std::enable_if_t<is_any_ptr<In>::value, bool> = true,  // no-lint
          typename Out, std::enable_if_t<not is_any_ptr<Out>::value, bool> = true>
inline void copy(In msg, const Out& state)
{
  copy(*msg, state);
}

// Version 4
template <typename In, std::enable_if_t<is_any_ptr<In>::value, bool> = true,  // no-lint
          typename Out, std::enable_if_t<is_any_ptr<Out>::value, bool> = true>
inline void copy(In msg, const Out state)
{
  copy(*msg, *state);
}

template <typename Msg>
void to_file(const Msg& msg, const std::string filename, const std::ios_base::openmode mode = std::ofstream::trunc)
{
  std::ofstream ofs;
  ofs.open(filename.c_str(), mode);
  to_file(msg, ofs);
  ofs.close();
}
}  // namespace ml4kp_bridge