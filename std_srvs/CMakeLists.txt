cmake_minimum_required(VERSION 2.8.3)

project(std_srvs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/Empty.srv"
  "srv/Trigger.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

ament_package()
