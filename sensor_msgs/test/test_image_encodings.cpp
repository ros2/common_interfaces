// Copyright (c) 2016, Kentaro Wada
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// this file is originally ported from ROS1:
// https://github.com/ros/common_msgs/blob/53579e6bba3d7f692949227fc4fe06cf47689f3a/sensor_msgs/test/test_image_encodings.cpp

#include <gtest/gtest.h>
#include "sensor_msgs/image_encodings.hpp"

TEST(sensor_msgs, NumChannels)
{
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("mono8"), 1);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("rgb8"), 3);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("8UC"), 1);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("8UC3"), 3);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("8UC10"), 10);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("16UC"), 1);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("16UC3"), 3);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("16UC10"), 10);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("32SC"), 1);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("32SC3"), 3);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("32SC10"), 10);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("64FC"), 1);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("64FC3"), 3);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("64FC10"), 10);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("yuv422"), 2);
  ASSERT_EQ(sensor_msgs::image_encodings::numChannels("yuv422_yuy2"), 2);
}

TEST(sensor_msgs, bitDepth)
{
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("mono8"), 8);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("rgb8"), 8);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("8UC"), 8);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("8UC3"), 8);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("8UC10"), 8);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("16UC"), 16);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("16UC3"), 16);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("16UC10"), 16);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("32SC"), 32);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("32SC3"), 32);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("32SC10"), 32);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("64FC"), 64);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("64FC3"), 64);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("64FC10"), 64);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("yuv422"), 8);
  ASSERT_EQ(sensor_msgs::image_encodings::bitDepth("yuv422_yuy2"), 8);
}
