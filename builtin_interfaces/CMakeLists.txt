cmake_minimum_required(VERSION 3.5)

project(builtin_interfaces)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Duration.msg"
  "msg/Time.msg"
  ADD_LINTER_TESTS
)

ament_package()
