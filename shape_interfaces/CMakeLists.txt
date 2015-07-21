cmake_minimum_required(VERSION 2.8.3)

project(shape_interfaces)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Mesh.msg"
  "msg/MeshTriangle.msg"
  "msg/Plane.msg"
  "msg/SolidPrimitive.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_interfaces
)

ament_package()
