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

#include <sensor_msgs/point_cloud_msg_wrapper.hpp>
#include <sensor_msgs/field_generators.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <tuple>
#include <cmath>
#include <limits>

namespace
{
struct PointX;
struct PointXYZI;
struct PointXYWithVirtualDestructor;
class ClassPointXY;
using GeometryPointXYZ = geometry_msgs::msg::Point32;

template<typename T,
  std::enable_if_t<std::is_floating_point<T>::value> * = nullptr>
constexpr bool nearly_equal(
  const T & a,
  const T & b,
  const T & epsilon = std::numeric_limits<T>::epsilon()) noexcept
{
  return std::fabs(a - b) <=
         (epsilon * std::max(std::fabs(a), std::fabs(b)));
}


struct PointWithCustomField
{
  float x;
  double non_standard_test_field;
  std::int32_t y;
  friend bool operator==(const PointWithCustomField & p1, const PointWithCustomField & p2) noexcept
  {
    return
      ::nearly_equal(p1.x, p2.x) &&
      ::nearly_equal(p1.non_standard_test_field, p2.non_standard_test_field) &&
      (p1.y == p2.y);
  }
};

struct CustomAlignedPoint
{
  float x;
  float y;
  float z;
  alignas(double) std::uint8_t intensity;
  double timestamp;
  friend bool operator==(const CustomAlignedPoint & p1, const CustomAlignedPoint & p2) noexcept
  {
    return
      ::nearly_equal(p1.x, p2.x) &&
      ::nearly_equal(p1.y, p2.y) &&
      ::nearly_equal(p1.z, p2.z) &&
      ::nearly_equal(p1.timestamp, p2.timestamp) &&
      (p1.intensity == p2.intensity);
  }
};

struct PointNotPresentInAllPointTypes {std::int8_t x;};

template<typename PointT>
PointT create_point() {return PointT{};}
template<>
PointX create_point();
template<>
PointXYZI create_point();
template<>
ClassPointXY create_point();
template<>
PointXYWithVirtualDestructor create_point();
template<>
GeometryPointXYZ create_point();
template<>
CustomAlignedPoint create_point();

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(non_standard_test_field);

}  // namespace

template<typename T>
class PointCloudMsgWrapperTest : public testing::Test
{
public:
  using PointT = T;
};

using AllPointTypes = ::testing::Types<
  PointX,
  PointXYZI,
  ClassPointXY,
  PointXYWithVirtualDestructor,
  CustomAlignedPoint,
  GeometryPointXYZ>;
// cppcheck-suppress syntaxError - trailing comma is the only way to remove the compiler warning.
TYPED_TEST_SUITE(PointCloudMsgWrapperTest, AllPointTypes, );

using sensor_msgs::PointCloud2View;
using sensor_msgs::PointCloud2Modifier;

/// @test Test that for any of the different types of points we can read and write them into msg.
TYPED_TEST(PointCloudMsgWrapperTest, check_reading_and_writing_generic_points)
{
  using PointT = typename TestFixture::PointT;
  sensor_msgs::msg::PointCloud2 msg;

  // Cannot initialize a wrapper without resetting an empty message.
  EXPECT_THROW(PointCloud2View<PointT>{msg}, std::runtime_error);
  EXPECT_THROW(PointCloud2Modifier<PointT>{msg}, std::runtime_error);

  PointCloud2Modifier<PointT> cloud_wrapper{msg, "some_frame_id"};

  const auto point = create_point<PointT>();

  ASSERT_FALSE(msg.fields.empty());
  ASSERT_FALSE(msg.fields.front().name.empty());
  ASSERT_NO_THROW(cloud_wrapper.push_back(point));

  EXPECT_THROW(PointCloud2View<PointNotPresentInAllPointTypes>{msg}, std::runtime_error);
  EXPECT_THROW(PointCloud2Modifier<PointNotPresentInAllPointTypes>{msg}, std::runtime_error);

  const auto initialized_wrapper = PointCloud2Modifier<PointT>{msg};
  ASSERT_EQ(initialized_wrapper.size(), 1U);
  EXPECT_EQ(initialized_wrapper.at(0U), point);
  EXPECT_EQ(initialized_wrapper[0U], point);
  EXPECT_NO_THROW(cloud_wrapper.push_back(initialized_wrapper[0U]));
  EXPECT_EQ(initialized_wrapper.size(), 2U);

  const auto & const_cloud = msg;
  const PointCloud2View<PointT> const_wrapper{const_cloud};
  ASSERT_EQ(const_wrapper.size(), 2U);
  EXPECT_EQ(const_wrapper.at(0U), point);
  EXPECT_EQ(const_wrapper[0U], point);
  EXPECT_EQ(const_wrapper.at(1U), point);
  EXPECT_EQ(const_wrapper[1U], point);

  // Test that we can iterate over the message.
  std::array<PointT, 2U> points;
  std::size_t index{};
  for (const auto & point : cloud_wrapper) {
    points[index] = point;
    index++;
  }
  ASSERT_EQ(points.size(), cloud_wrapper.size());
  for (auto i = 0U; i < points.size(); ++i) {
    EXPECT_EQ(points[i], cloud_wrapper[i]);
  }
  EXPECT_EQ(points.front(), cloud_wrapper.front());
  EXPECT_EQ(points.back(), cloud_wrapper.back());

  cloud_wrapper.resize(3U);
  EXPECT_EQ(cloud_wrapper.size(), 3U);
  // Check that the untouched points stayed the same.
  for (auto i = 0U; i < points.size(); ++i) {
    EXPECT_EQ(points[i], cloud_wrapper[i]);
  }

  // Change values.
  for (auto & point : cloud_wrapper) {
    point = PointT{};
  }
  for (const auto & point : cloud_wrapper) {
    EXPECT_EQ(PointT{}, point);
  }

  cloud_wrapper.clear();
  EXPECT_TRUE(cloud_wrapper.empty());

  // Cannot reinitialize an already initialized message.
  EXPECT_THROW(PointCloud2Modifier<PointT>(msg, "some_new_frame_id"), std::runtime_error);
}

/// @test Check that using using iterator including std::back_inserter is possible with the wrapper.
TYPED_TEST(PointCloudMsgWrapperTest, iterators) {
  using PointT = typename TestFixture::PointT;
  sensor_msgs::msg::PointCloud2 msg;
  PointCloud2Modifier<PointT> cloud_wrapper{msg, "some_frame_id"};
  std::array<PointT, 2U> points{{create_point<PointT>(), PointT{}}};
  // Fill the message from an array.
  std::transform(
    points.cbegin(), points.cend(), std::back_inserter(cloud_wrapper),
    [](const PointT & point) {return point;});
  ASSERT_EQ(points.size(), cloud_wrapper.size());
  for (auto i = 0U; i < points.size(); ++i) {
    EXPECT_EQ(points[i], cloud_wrapper[i]);
  }
  // Change existing values.
  for (auto & point : cloud_wrapper) {
    point = PointT{};
  }
  const auto & const_cloud = msg;
  const PointCloud2View<PointT> const_wrapper{const_cloud};
  for (const auto & point : const_wrapper) {
    EXPECT_EQ(PointT{}, point);
  }

  // Check the reverse iteration.
  for (auto riter = const_wrapper.rbegin(); riter != const_wrapper.rend(); ++riter) {
    EXPECT_EQ(PointT{}, *riter);
  }
}

/// @test Check initialization with a compicated type with an additional custom field.
TEST(PointCloudMsgWrapperTest, point_with_custom_field) {
  sensor_msgs::msg::PointCloud2 msg;
  // There is no matching field generator for non_standard_test_field present.
  EXPECT_THROW(PointCloud2Modifier<PointWithCustomField>(msg, "some_frame_id"), std::runtime_error);
  EXPECT_THROW(PointCloud2View<PointWithCustomField>{msg}, std::runtime_error);
  using Generators = std::tuple<
    sensor_msgs::field_x_generator,
    sensor_msgs::field_y_generator,
    field_non_standard_test_field_generator>;
  using CustomCloudModifier = PointCloud2Modifier<PointWithCustomField, Generators>;
  // Cannot initialize message without a new frame id provided.
  EXPECT_THROW(CustomCloudModifier{msg}, std::runtime_error);

  CustomCloudModifier cloud_wrapper{msg, "some_frame_id"};
  ASSERT_EQ(msg.fields.size(), 3U);
  // Note that the order of fields is defined by the order of generators in the tuple, NOT by the
  // order of members in the struct. However, the order of fields plays no role in point
  // representation within the message.
  EXPECT_EQ(msg.fields[0].name, "x");
  EXPECT_EQ(msg.fields[1].name, "y");
  EXPECT_EQ(msg.fields[2].name, "non_standard_test_field");
  EXPECT_NO_THROW(cloud_wrapper.push_back({42.0F, 42.0, 23}));

  const CustomCloudModifier initialized_wrapper{msg};
  ASSERT_FALSE(initialized_wrapper.empty());
  ASSERT_EQ(initialized_wrapper.size(), 1U);
  EXPECT_FLOAT_EQ(initialized_wrapper.at(0U).x, 42.0F);
  EXPECT_DOUBLE_EQ(initialized_wrapper.at(0U).non_standard_test_field, 42.0);
  EXPECT_EQ(initialized_wrapper.at(0U).y, 23);
  EXPECT_THROW(initialized_wrapper.at(1U), std::out_of_range);
  EXPECT_FLOAT_EQ(initialized_wrapper[0U].x, 42.0F);
  EXPECT_DOUBLE_EQ(initialized_wrapper[0U].non_standard_test_field, 42.0);
  EXPECT_EQ(initialized_wrapper[0U].y, 23);
  EXPECT_NO_THROW(cloud_wrapper.push_back(initialized_wrapper[0U]));
  ASSERT_EQ(initialized_wrapper.size(), 2U);
}

/// @test Check that a macro we use for readability is not leaking outside of the header file.
TEST(PointCloudMsgWrapperTest, compilation_macro_is_unset) {
  #ifdef COMPILE_IF_MUTABLE
  FAIL() << "Compilation macro should not be available outside of point_cloud_msg_wrapper.hpp file";
  #endif
}


namespace
{

struct PointX
{
  float x;
  friend bool operator==(const PointX & p1, const PointX & p2) noexcept
  {
    return ::nearly_equal(p1.x, p2.x);
  }
};

class ClassPointXY
{
public:
  ClassPointXY() = default;
  explicit ClassPointXY(float x, float y)
  : x_{x}, y_{y} {}

  float & x() {return x_;}
  const float & x() const {return x_;}
  float & y() {return y_;}
  const float & y() const {return y_;}

  friend bool operator==(const ClassPointXY & p1, const ClassPointXY & p2) noexcept
  {
    return ::nearly_equal(p1.x_, p2.x_) &&
           ::nearly_equal(p1.y_, p2.y_);
  }

private:
  float x_;
  float y_;
};

struct PointXYZI
{
  float x;
  float y;
  float z;
  std::int64_t id;
  friend bool operator==(const PointXYZI & p1, const PointXYZI & p2) noexcept
  {
    return ::nearly_equal(p1.x, p2.x) &&
           ::nearly_equal(p1.y, p2.y) &&
           ::nearly_equal(p1.z, p2.z) &&
           (p1.id == p2.id);
  }
};

struct PointXYWithVirtualDestructor
{
  PointXYWithVirtualDestructor() = default;
  PointXYWithVirtualDestructor(float x_, float y_)
  : x{x_}, y{y_} {}

  virtual ~PointXYWithVirtualDestructor() {}

  friend bool operator==(
    const PointXYWithVirtualDestructor & p1,
    const PointXYWithVirtualDestructor & p2) noexcept
  {
    return ::nearly_equal(p1.x, p2.x) &&
           ::nearly_equal(p1.y, p2.y);
  }

  float x{};
  float y{};
};

template<>
PointX create_point() {return PointX{42.0F};}
template<>
PointXYZI create_point() {return PointXYZI{42.0F, 23.0F, 13.0F, 42LL};}
template<>
ClassPointXY create_point() {return ClassPointXY{42.0F, 23.0F};}
template<>
PointXYWithVirtualDestructor create_point() {return PointXYWithVirtualDestructor{42.0F, 23.0F};}
template<>
GeometryPointXYZ create_point()
{
  GeometryPointXYZ point;
  point.set__x(42.0F).set__y(23.0F).set__x(13.0F);
  return point;
}
template<>
CustomAlignedPoint create_point() {return CustomAlignedPoint{42.0F, 23.0F, 4242.0F, 23, 2323.0F};}
}  // namespace
