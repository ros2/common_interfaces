cmake_minimum_required(VERSION 3.5)

project(diagnostic_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/DiagnosticArray.msg"
  "msg/DiagnosticStatus.msg"
  "msg/KeyValue.msg"
)
set(srv_files
  "srv/AddDiagnostics.srv"
  "srv/SelfTest.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
