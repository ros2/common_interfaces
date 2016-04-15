cmake_minimum_required(VERSION 2.8.3)

project(geometry_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
# TODO(wjwwood): re-enable all messages when build time is not so important.
#                This will likely be the case when we have a proper build farm
#                and/or we reduce the number of middlewares or languages we
#                build for by default.
  "msg/Point.msg"
  "msg/Point32.msg"
  # "msg/PointStamped.msg"
  # "msg/Polygon.msg"
  # "msg/PolygonStamped.msg"
  "msg/Pose.msg"
  # "msg/Pose2D.msg"
  # "msg/PoseArray.msg"
  "msg/PoseStamped.msg"
  "msg/PoseWithCovariance.msg"
  # "msg/PoseWithCovarianceStamped.msg"
  "msg/Quaternion.msg"
  # "msg/QuaternionStamped.msg"
  "msg/Transform.msg"
  "msg/TransformStamped.msg"
  "msg/Twist.msg"
  # "msg/TwistStamped.msg"
  "msg/TwistWithCovariance.msg"
  # "msg/TwistWithCovarianceStamped.msg"
  "msg/Vector3.msg"
  # "msg/Vector3Stamped.msg"
  # "msg/Wrench.msg"
  # "msg/WrenchStamped.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_package()
