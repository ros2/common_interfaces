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


#ifndef SENSOR_MSGS__FIELD_GENERATORS_HPP_
#define SENSOR_MSGS__FIELD_GENERATORS_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/field_properties.hpp>

namespace sensor_msgs
{

namespace detail
{
template<typename ClassT, typename ReturnT>
inline size_t offset_of(ReturnT ClassT::* member)
{
  ClassT object{};
  return size_t(&(object.*member)) - size_t(&object);
}

template<class ClassT, class ReturnT>
size_t offset_of_return_ref(ReturnT (ClassT::* member_func)(void))
{
  ClassT object;
  const auto & result_ref = (object.*member_func)();
  return size_t(&result_ref) - size_t(&object);
}

template<typename ClassT, typename MemberT>
void push_back_field(
  sensor_msgs::msg::PointCloud2::_fields_type & fields,
  const sensor_msgs::msg::PointField::_name_type & name,
  MemberT ClassT::* member)
{
  using sensor_msgs::get_field_count;
  using sensor_msgs::get_field_datatype;
  fields.emplace_back();
  fields.back()
  .set__name(name)
  .set__offset(offset_of(member))
  .set__datatype(get_field_datatype<MemberT>())
  .set__count(get_field_count<MemberT>());
}

template<typename ClassT, typename ReturnT>
void push_back_field(
  sensor_msgs::msg::PointCloud2::_fields_type & fields,
  const sensor_msgs::msg::PointField::_name_type & name,
  ReturnT (ClassT::* member_func)(void))
{
  using sensor_msgs::get_field_count;
  using sensor_msgs::get_field_datatype;
  using MemberT = std::decay_t<ReturnT>;
  fields.emplace_back();
  fields.back()
  .set__name(name)
  .set__offset(offset_of_return_ref(member_func))
  .set__datatype(get_field_datatype<MemberT>())
  .set__count(get_field_count<MemberT>());
}

/// End of recursion for applying all field adders to a given point type.
template<class PointT, class FieldAdders, size_t N>
constexpr std::enable_if_t < (N >= std::tuple_size<FieldAdders>{}) >
apply_field_adder_to_generate_field_if_needed(sensor_msgs::msg::PointCloud2::_fields_type &) {
}

/// Standard case to recursively apply all field adders for a given point type.
template<class PointT, class FieldAdders, size_t N = 0>
constexpr std::enable_if_t < (N < std::tuple_size<FieldAdders>{}) >
apply_field_adder_to_generate_field_if_needed(sensor_msgs::msg::PointCloud2::_fields_type & fields)
{
  using CurrentFieldAdder = std::tuple_element_t<N, FieldAdders>;
  CurrentFieldAdder::template push_back_field_if_needed<PointT>(fields);
  apply_field_adder_to_generate_field_if_needed<PointT, FieldAdders, N + 1>(fields);
}

}  // namespace detail


/// A macro that defines a struct that allows to query for the MEMBER member in any given struct.
/// The intended usecase for this struct is (example for MEMBER 'x'):
/// has_member_x<PointXYZ>::value
#define IMPL__LIDAR_UTILS__DEFINE_MEMBER_CHECKER(MEMBER) \
  template<typename T, typename V = bool> \
  struct has_member_ ## MEMBER : std::false_type {}; \
  template<typename T> \
  struct has_member_ ## MEMBER< \
    T, typename std::enable_if< \
      !std::is_same<decltype(std::declval<T>().MEMBER), void>::value, \
      bool>::type>: std::true_type {}

#define IMPL__LIDAR_UTILS__DEFINE_MEMBER_FUNCTION_CHECKER(FUNCTION_NAME) \
  template<typename T, typename = void> \
  struct has_function_ ## FUNCTION_NAME ## _returning_non_const_ref : std::false_type {}; \
  template<typename T> \
  struct has_function_ ## FUNCTION_NAME ## _returning_non_const_ref< \
    T, std::enable_if_t< \
      std::is_reference<decltype(std::declval<T>().FUNCTION_NAME())>::value && \
      !std::is_const<std::remove_reference_t<decltype( \
        std::declval<T>().FUNCTION_NAME())>>::value>>: std::true_type {}

/// A Macro that defines a generator struct for a given field name.
///
/// The generator structs take a vector of PointFields and push_back a new
/// PointField into it if the underlying class has the specified field. The
/// existence of the field is checked by the has_member_FIELD<PointT>::value.
#define LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(FIELD) \
  struct field_ ## FIELD ## _generator { \
    IMPL__LIDAR_UTILS__DEFINE_MEMBER_CHECKER(FIELD); \
    IMPL__LIDAR_UTILS__DEFINE_MEMBER_FUNCTION_CHECKER(FIELD); \
    template<class PointT> \
    static inline std::enable_if_t< \
      !has_member_ ## FIELD<PointT>::value && \
      !has_function_ ## FIELD ## _returning_non_const_ref<PointT>::value> \
    push_back_field_if_needed(sensor_msgs::msg::PointCloud2::_fields_type &) { \
    } \
    template<class PointT> \
    static inline std::enable_if_t< \
      has_member_ ## FIELD<PointT>::value || \
      has_function_ ## FIELD ## _returning_non_const_ref<PointT>::value> \
    push_back_field_if_needed(sensor_msgs::msg::PointCloud2::_fields_type & fields) { \
      sensor_msgs::detail::push_back_field(fields, #FIELD, &PointT::FIELD); \
    } \
  }

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(x);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(y);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(z);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(id);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(ring);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(intensity);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(timestamp);

/// A help function to start iterating over all field adders and return the appropritate fields
/// generated by those.
///
/// @tparam     PointT       Type of point
/// @tparam     FieldAdders  A tuple of field adder types, generated by
///                          LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER
///
/// @return     All generated PointField structs.
///
template<class PointT, class FieldAdders>
inline sensor_msgs::msg::PointCloud2::_fields_type generate_fields_from_point()
{
  sensor_msgs::msg::PointCloud2::_fields_type fields;
  detail::apply_field_adder_to_generate_field_if_needed<PointT, FieldAdders, 0>(fields);
  return fields;
}

}  // namespace sensor_msgs


#endif  // SENSOR_MSGS__FIELD_GENERATORS_HPP_
