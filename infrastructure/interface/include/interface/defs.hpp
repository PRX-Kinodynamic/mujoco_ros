#pragma once

#include <ros/ros.h>

namespace interface
{
template <typename To, typename From>
void translate_msg(To& to_msg, const From& from_msg)
{
}
}  // namespace interface