// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#ifndef SENSOR_MSGS__TYPE_TRAITS_HPP_
#define SENSOR_MSGS__TYPE_TRAITS_HPP_

#include <type_traits>

namespace sensor_msgs
{
namespace detail
{

// TODO(igor): this is a general trait, should it live elsewhere?
template<typename Test, template<typename ...> class Ref>
struct is_specialization : std::false_type {};

template<template<typename ...> class Ref, typename ... Args>
struct is_specialization<Ref<Args...>, Ref>: std::true_type {};

// Taken from https://stackoverflow.com/a/35207812/1763680
template<class T, class EqualTo>
struct has_operator_equals_impl
{
  template<class U, class V>
  static auto test(U *)->decltype(std::declval<U>() == std::declval<V>());
  template<typename, typename>
  static auto test(...)->std::false_type;

  using type = typename std::is_same<bool, decltype(test<T, EqualTo>(0))>::type;
};

template<class T, class EqualTo = T>
struct has_operator_equals : has_operator_equals_impl<T, EqualTo>::type {};

/// Detect the template type used under the hood of the given allocator type.
/// Provided std::allocator<int> type will be std::allocator.
template<class AllocatorT>
struct underlying_allocator_template;

template<class ValueT, template<class> class AllocatorT>
struct underlying_allocator_template<AllocatorT<ValueT>>
{
  template<class T>
  using type = AllocatorT<T>;
};


/// Detect the template type used under the hood.
/// Provided std::vector<int> type will be std::vector.
template<class ContainerT>
struct underlying_container_template;

/// An overload for a container like an std::vector<int, std::allocator<int>>.
template<class ValueT, class AllocatorT, template<class, class> class ContainerT>
struct underlying_container_template<ContainerT<ValueT, AllocatorT>>
{
  template<class T, std::size_t UNUSED_UPPER_BOUND, class A>
  using type = ContainerT<T, A>;
};

/// An overload for containers like a bounded_vector<int, 100U, std::allocator<int>>.
template<
  class ValueT,
  std::size_t UPPER_BOUND,
  class AllocatorT,
  template<class, std::size_t, class> class ContainerT>
struct underlying_container_template<ContainerT<ValueT, UPPER_BOUND, AllocatorT>>
{
  template<class T, std::size_t BOUND, class A>
  using type = ContainerT<T, BOUND, A>;
};

}  // namespace detail
}  // namespace sensor_msgs

#endif  // SENSOR_MSGS__TYPE_TRAITS_HPP_
