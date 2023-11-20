#pragma once

#include "prx/utilities/general/template_utils.hpp"
#include <boost/shared_ptr.hpp>
namespace ml4kp_bridge
{

template <class T>
struct is_boost_ptr : std::false_type
{
};

template <class T>
struct is_boost_ptr<boost::shared_ptr<T>> : std::true_type
{
};

template <class T>
struct is_boost_ptr<boost::shared_ptr<T> const> : std::true_type
{
};

template <class T>
struct is_boost_ptr<boost::shared_ptr<T const>> : std::true_type
{
};

template <class T>
struct is_any_ptr : std::integral_constant<bool, prx::utilities::is_any_ptr<T>::value || is_boost_ptr<T>::value>
{
};

}  // namespace ml4kp_bridge