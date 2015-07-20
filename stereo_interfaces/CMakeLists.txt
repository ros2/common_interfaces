cmake_minimum_required(VERSION 2.8.3)

project(stereo_interfaces)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(std_interfaces REQUIRED)

set(msg_files
  "msg/DisparityImage.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_interfaces std_interfaces
)

ament_package()
