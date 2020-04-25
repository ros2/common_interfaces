# sensor_msgs

This package provides many messages and services relating to sensor devices.

Many of these messages were ported from ROS 1 and a lot of still-relevant documentation can be found through the [ROS 1 sensor_msgs wiki](http://wiki.ros.org/sensor_msgs?distro=noetic).

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## sensor_msgs c++ API
This package provides some common c++ functionality relating to manipulating a couple of particular sensor_msgs messages.

* [fill_image.hpp](include/sensors_msgs/fill_image.hpp): Fill a Image message from type-erased data pointer.
* [image_encodings.hpp](include/sensor_msgs/image_encodings): Definitions and functionality relating to image encodings.
* [point_cloud_conversion.hpp](include/sensor_msgs/point_cloud_conversion.hpp): Functionality for converting between the deprecated PointCloud and PointCloud2 messages.
* [point_cloud2_iterator.hpp](include/sensor_msgs/point_cloud2_iterator.hpp): Tools for modifying and parsing PointCloud2 messages.
* [point_field_conversion.hpp](include/sensor_msgs/point_field_conversion.hpp): A type to enum mapping for the different PointField types, and methods to read and write in a PointCloud2 buffer for the different PointField types.

## Messages (.msg)
* [BatteryState](BatteryState.msg): Describes the power state of the battery.
* [CameraInfo](CameraInfo.msg): Meta information for a camera.
* [ChannelFloat32](ChannelFloat32.msg): Holds optional data associated with each point in a PointCloud message.
* [CompressedImage](CompressedImage.msg): A compressed image.
* [FluidPressure](FluidPressure.msg): Single pressure reading for fluids (air, water, etc) like atmospheric and barometric pressures.
* [Illuminance](Illuminance.msg): Single photometric illuminance measurement.
* [Image](Image.msg): An uncompressed image.
* [Imu](Imu.msg): Holds data from an IMU (Inertial Measurement Unit).
* [JointState](JointState.msg): Holds data to describe the state of a set of torque controlled joints.
* [JoyFeedbackArray](JoyFeedbackArray.msg): An array of JoyFeedback messages.
* [JoyFeedback](JoyFeedback.msg): Describes user feedback in a joystick, like an LED, rumble pad, or buzzer.
* [Joy](Joy.msg): Reports the state of a joystick's axes and buttons.
* [LaserEcho](LaserEcho.msg): A submessage of MultiEchoLaserScan and is not intended to be used separately.
* [LaserScan](LaserScan.msg): Single scan from a planar laser range-finder.
* [MagneticField](MagneticField.msg): Measurement of the Magnetic Field vector at a specific location.
* [MultiDOFJointState](MultiDOFJointState.msg): Representation of state for joints with multiple degrees of freedom, following the structure of JointState.
* [MultiEchoLaserScan](MultiEchoLaserScan.msg): Single scan from a multi-echo planar laser range-finder.
* [NavSatFix](NavSatFix.msg): Navigation Satellite fix for any Global Navigation Satellite System.
* [NavSatStatus](NavSatStatus.msg): Navigation Satellite fix status for any Global Navigation Satellite System.
* [PointCloud2](PointCloud2.msg): Holds a collection of N-dimensional points, which may contain additional information such as normals, intensity, etc.
* [PointCloud](PointCloud.msg): ** ## THIS MESSAGE IS DEPRECATED AS OF FOXY, use PointCloud2 instead**
* [PointField](PointField.msg): Holds the description of one point entry in the PointCloud2 message format.
* [Range](Range.msg): Single range reading from an active ranger that emits energy and reports one range reading that is valid along an arc at the distance measured.
* [RegionOfInterest](RegionOfInterest.msg): Used to specify a region of interest within an image.
* [RelativeHumidity](RelativeHumidity.msg): A single reading from a relative humidity sensor.
* [Temperature](Temperature.msg): A single temperature reading.
* [TimeReference](TimeReference.msg): Measurement from an external time source not actively synchronized with the system clock.

## Services (.srv)
* [SetCameraInfo](SetCameraInfo.srv): Request that a camera stores the given CameraInfo as that camera's calibration information.

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
