cmake_minimum_required(VERSION 2.8.3)

project(std_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
# TODO(wjwwood): re-enable all messages when build time is not so important.
#                This will likely be the case when we have a proper build farm
#                and/or we reduce the number of middlewares or languages we
#                build for by default.
  "msg/Bool.msg"
  # "msg/Byte.msg"
  # "msg/ByteMultiArray.msg"
  # "msg/Char.msg"
  # "msg/ColorRGBA.msg"
  "msg/Empty.msg"
  "msg/Float32.msg"
  # "msg/Float32MultiArray.msg"
  "msg/Float64.msg"
  "msg/Float64MultiArray.msg"
  "msg/Header.msg"
  "msg/Int16.msg"
  # "msg/Int16MultiArray.msg"
  "msg/Int32.msg"
  # "msg/Int32MultiArray.msg"
  "msg/Int64.msg"
  # "msg/Int64MultiArray.msg"
  "msg/Int8.msg"
  # "msg/Int8MultiArray.msg"
  "msg/MultiArrayDimension.msg"
  "msg/MultiArrayLayout.msg"
  "msg/String.msg"
  "msg/UInt16.msg"
  # "msg/UInt16MultiArray.msg"
  "msg/UInt32.msg"
  # "msg/UInt32MultiArray.msg"
  "msg/UInt64.msg"
  # "msg/UInt64MultiArray.msg"
  "msg/UInt8.msg"
  # "msg/UInt8MultiArray.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_package()
