cmake_minimum_required(VERSION 3.5)

project(std_srvs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/Empty.srv"
  "srv/SetBool.srv"
  "srv/Trigger.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
