cmake_minimum_required(VERSION 2.8.3)

project(introspection_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ROSMeta.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()