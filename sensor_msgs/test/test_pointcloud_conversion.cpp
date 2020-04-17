// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define SENSOR_MSGS_SKIP_WARNING

// #warning suppression
// Not working due to preprocessor ignoring pragmas in g++
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53431
// Clang doesn't support suppressing -Wcpp probably like above.
// And I can't find any windows way to suppress it either.

#include "sensor_msgs/point_cloud_conversion.hpp"
#undef SENSOR_MSGS_SKIP_WARNING

#include "sensor_msgs/point_field_conversion.hpp"

TEST(sensor_msgs, PointCloudConversion)
{
  // Build a PointCloud
  sensor_msgs::msg::PointCloud cloud;

  cloud.header.stamp.sec = 100;
  cloud.header.stamp.nanosec = 500;
  cloud.header.frame_id = "cloud_frame";

  sensor_msgs::msg::ChannelFloat32 intensity, range;
  intensity.name = "intensity";
  range.name = "range";

  for (size_t ii = 0; ii < 100; ++ii) {
    auto pt = geometry_msgs::msg::Point32();
    pt.x = static_cast<float>(1 * ii);
    pt.y = static_cast<float>(2 * ii);
    pt.z = static_cast<float>(3 * ii);
    cloud.points.push_back(pt);

    intensity.values.push_back(static_cast<float>(4 * ii));
    range.values.push_back(static_cast<float>(5 * ii));
  }
  cloud.channels.push_back(intensity);
  cloud.channels.push_back(range);

  // Convert to PointCloud2
  sensor_msgs::msg::PointCloud2 cloud2;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  auto ret_cc2 = sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif
  ASSERT_TRUE(ret_cc2);

  EXPECT_EQ(1u, cloud2.height);
  EXPECT_EQ(100u, cloud2.width);

  EXPECT_EQ(cloud2.fields[0].name, "x");
  EXPECT_EQ(cloud2.fields[1].name, "y");
  EXPECT_EQ(cloud2.fields[2].name, "z");
  EXPECT_EQ(cloud2.fields[3].name, "intensity");
  EXPECT_EQ(cloud2.fields[4].name, "range");

  EXPECT_EQ(cloud2.fields.size(), 5u);
  EXPECT_EQ(cloud2.header.frame_id, "cloud_frame");
  EXPECT_EQ(cloud2.header.stamp.sec, 100);
  EXPECT_EQ(cloud2.header.stamp.nanosec, 500u);

  // Convert back to PointCloud
  sensor_msgs::msg::PointCloud cloud3;
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  auto ret_c2c = sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud3);
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif
  ASSERT_TRUE(ret_c2c);
  EXPECT_EQ(cloud3.points.size(), 100u);
  EXPECT_EQ(cloud3.channels.size(), 2u);

  EXPECT_EQ(cloud3.channels[0].name, "intensity");
  EXPECT_EQ(cloud3.channels[0].values.size(), 100u);
  EXPECT_FLOAT_EQ(cloud3.channels[0].values[0], 0.0);
  EXPECT_FLOAT_EQ(cloud3.channels[0].values[10], 40.0);

  EXPECT_EQ(cloud3.channels[1].name, "range");
  EXPECT_EQ(cloud3.channels[1].values.size(), 100u);
  EXPECT_FLOAT_EQ(cloud3.channels[1].values[0], 0.0);
  EXPECT_FLOAT_EQ(cloud3.channels[1].values[10], 50.0);
}
