// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <cstdint>
#include <vector>

#include "sensor_msgs/fill_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

TEST(sensor_msgs, FillImage)
{
  const std::vector<uint8_t> pixels = {1, 2, 3, 4, 5, 6};
  sensor_msgs::msg::Image image;
  EXPECT_TRUE(
    sensor_msgs::fillImage(
      image, sensor_msgs::image_encodings::MONO8, 2, 3, 3, pixels.data()));
  EXPECT_EQ(2u, image.height);
  EXPECT_EQ(3u, image.width);
  EXPECT_EQ(3u, image.step);
  ASSERT_EQ(pixels.size(), image.data.size());
  for (size_t i = 0; i < pixels.size(); ++i) {
    EXPECT_EQ(pixels[i], image.data[i]);
  }
}

TEST(sensor_msgs, FillImageEmpty)
{
  sensor_msgs::msg::Image image;
  EXPECT_TRUE(
    sensor_msgs::fillImage(
      image, sensor_msgs::image_encodings::MONO8, 0, 0, 0, nullptr));
  EXPECT_TRUE(image.data.empty());
}
