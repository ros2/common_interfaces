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

#ifndef SENSOR_MSGS__FIELD_PROPERTIES_HPP_
#define SENSOR_MSGS__FIELD_PROPERTIES_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstdint>

namespace sensor_msgs
{

inline std::int32_t sizeof_field(const std::uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 2;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
  }
  throw std::runtime_error("Unexpected datatype provided");
}

template<typename T>
inline constexpr std::uint32_t get_field_count()
{
  static_assert(sizeof(T) == -1, "Only specializations of this function are expected");
  return 0U;
}

template<typename T>
inline constexpr std::uint8_t get_field_datatype()
{
  static_assert(sizeof(T) == -1, "Only specializations of this function are expected");
  return 0U;
}

template<>
inline constexpr std::uint32_t get_field_count<float>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<double>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::int8_t>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::uint8_t>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::int16_t>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::uint16_t>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::int32_t>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::uint32_t>() {return 1U;}
template<>
inline constexpr std::uint32_t get_field_count<std::int64_t>() {return 2U;}
template<>
inline constexpr std::uint32_t get_field_count<std::uint64_t>() {return 2U;}

template<>
inline constexpr std::uint8_t get_field_datatype<float>()
{
  return sensor_msgs::msg::PointField::FLOAT32;
}
template<>
inline constexpr std::uint8_t get_field_datatype<double>()
{
  return sensor_msgs::msg::PointField::FLOAT64;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::int8_t>()
{
  return sensor_msgs::msg::PointField::INT8;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::uint8_t>()
{
  return sensor_msgs::msg::PointField::UINT8;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::int16_t>()
{
  return sensor_msgs::msg::PointField::INT16;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::uint16_t>()
{
  return sensor_msgs::msg::PointField::UINT16;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::int32_t>()
{
  return sensor_msgs::msg::PointField::INT32;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::uint32_t>()
{
  return sensor_msgs::msg::PointField::UINT32;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::int64_t>()
{
  return sensor_msgs::msg::PointField::INT32;
}
template<>
inline constexpr std::uint8_t get_field_datatype<std::uint64_t>()
{
  return sensor_msgs::msg::PointField::UINT32;
}

}  // namespace sensor_msgs

#endif  // SENSOR_MSGS__FIELD_PROPERTIES_HPP_
