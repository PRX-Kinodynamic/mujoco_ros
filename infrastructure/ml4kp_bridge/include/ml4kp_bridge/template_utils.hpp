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

// template <>
template <class T>
struct is_pair : std::false_type
{
};

template <class A, class B>
struct is_pair<std::pair<A, B>> : std::true_type
{
};

// template <typename>
// struct is_pair_derived : std::false_type
// {
// }

// template <class Type, class A, class B>
// struct is_pair_derived : std::is_base_of<Type, std::pair<A, B>>::value
// {
// };

// template <typename... T>
// struct is_pair : std::integral_constant<bool, std::is_base_of<T, std::pair<T...>>::value>
// {
// };

template <typename>
struct is_tuple : std::false_type
{
};

template <typename... T>
struct is_tuple<std::tuple<T...>> : std::true_type
{
};

}  // namespace ml4kp_bridge