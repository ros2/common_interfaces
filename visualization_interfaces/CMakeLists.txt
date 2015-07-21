cmake_minimum_required(VERSION 2.8.3)

project(visualization_interfaces)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_interfaces REQUIRED)

set(msg_files
  "msg/ImageMarker.msg"
  "msg/InteractiveMarker.msg"
  "msg/InteractiveMarkerControl.msg"
  "msg/InteractiveMarkerFeedback.msg"
  "msg/InteractiveMarkerInit.msg"
  "msg/InteractiveMarkerPose.msg"
  "msg/InteractiveMarkerUpdate.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  "msg/MenuEntry.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_interfaces std_interfaces
)

ament_package()
