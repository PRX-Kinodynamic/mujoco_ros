#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include "ml4kp_bridge/product_lie_group.hpp"
namespace ml4kp_bridge
{

template <class T>
struct is_gtsam_type : std::false_type
{
};

template <>
struct is_gtsam_type<gtsam::Rot2> : std::true_type
{
};

template <>
struct is_gtsam_type<gtsam::Pose2> : std::true_type
{
};

template <typename... A>
struct is_gtsam_type<gtsam::ProductLieGroupV43<A...>> : std::true_type
{
};

}  // namespace ml4kp_bridge