cmake_minimum_required(VERSION 3.5)

project(test_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/BoundedArrayNested.msg"
  "msg/BoundedArrayPrimitives.msg"
  "msg/Builtins.msg"
  "msg/DynamicArrayNested.msg"
  "msg/DynamicArrayPrimitives.msg"
  "msg/Empty.msg"
  "msg/FieldsWithSameType.msg"
  "msg/Nested.msg"
  "msg/Primitives.msg"
  "msg/StaticArrayNested.msg"
  "msg/StaticArrayPrimitives.msg"
  "msg/UInt32.msg"
)

set(srv_files
  "srv/Empty.srv"
  "srv/Primitives.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

install(DIRECTORY src/test_msgs
  DESTINATION "${PYTHON_INSTALL_DIR}/")

install(DIRECTORY include/
  DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
