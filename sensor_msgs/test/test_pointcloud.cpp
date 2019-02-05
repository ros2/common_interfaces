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

#include "sensor_msgs/point_cloud_conversion.hpp"
#include "sensor_msgs/point_field_conversion.hpp"

TEST(PointCloudConversions, CloudToCloud2)
{
  sensor_msgs::msg::PointCloud cloud;

  for (size_t ii = 0; ii < 100; ++ii) {
    auto pt = geometry_msgs::msg::Point32();
    pt.x = ii;
    pt.y = ii;
    pt.z = ii;
    cloud.points.push_back(pt);
  }

  sensor_msgs::msg::PointCloud2 cloud2;

  auto ret = sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
  EXPECT_TRUE(ret);

  EXPECT_EQ(1u, cloud2.height);
  EXPECT_EQ(100u, cloud2.width);

  EXPECT_EQ(cloud2.fields[0].name, "x");
  EXPECT_EQ(cloud2.fields[1].name, "y");
  EXPECT_EQ(cloud2.fields[2].name, "z");
}

