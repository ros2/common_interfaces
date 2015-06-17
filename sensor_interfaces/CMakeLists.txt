cmake_minimum_required(VERSION 2.8.3)

project(sensor_interfaces)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_interfaces REQUIRED)

set(msg_files
  "CameraInfo.msg"
  "ChannelFloat32.msg"
  "CompressedImage.msg"
  "FluidPressure.msg"
  "Illuminance.msg"
  "Image.msg"
  "Imu.msg"
  "JointState.msg"
  "Joy.msg"
  "JoyFeedback.msg"
  "JoyFeedbackArray.msg"
  "LaserEcho.msg"
  "LaserScan.msg"
  "MagneticField.msg"
  "MultiDOFJointState.msg"
  "MultiEchoLaserScan.msg"
  "NavSatFix.msg"
  "NavSatStatus.msg"
  "PointCloud.msg"
  "PointCloud2.msg"
  "PointField.msg"
  "Range.msg"
  "RegionOfInterest.msg"
  "RelativeHumidity.msg"
  "Temperature.msg"
  "TimeReference.msg"
)
set(srv_files
  "SetCameraInfo.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_interfaces std_interfaces
)

ament_package()
