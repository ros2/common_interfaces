cmake_minimum_required(VERSION 2.8.3)

project(sensor_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
# TODO(wjwwood): re-enable all messages when build time is not so important.
#                This will likely be the case when we have a proper build farm
#                and/or we reduce the number of middlewares or languages we
#                build for by default.
  "msg/CameraInfo.msg"
  "msg/ChannelFloat32.msg"
  "msg/CompressedImage.msg"
  # "msg/FluidPressure.msg"
  # "msg/Illuminance.msg"
  "msg/Image.msg"
  "msg/Imu.msg"
  # "msg/JointState.msg"
  # "msg/Joy.msg"
  # "msg/JoyFeedback.msg"
  # "msg/JoyFeedbackArray.msg"
  # "msg/LaserEcho.msg"
  "msg/LaserScan.msg"
  # "msg/MagneticField.msg"
  # "msg/MultiDOFJointState.msg"
  # "msg/MultiEchoLaserScan.msg"
  # "msg/NavSatFix.msg"
  # "msg/NavSatStatus.msg"
  "msg/PointCloud.msg"
  "msg/PointCloud2.msg"
  "msg/PointField.msg"
  # "msg/Range.msg"
  "msg/RegionOfInterest.msg"
  # "msg/RelativeHumidity.msg"
  # "msg/Temperature.msg"
  # "msg/TimeReference.msg"
)
set(srv_files
  "srv/SetCameraInfo.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_package()
