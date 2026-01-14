# common_interfaces
Common interfaces is a metapackage (collection of packages) that includes the standard set of messages (.msg) and services (.srv) available on all ROS systems.

# List of Packages

The following packages are contained in the common_interfaces meta package. The links below point to Humble Hawksbill API documentation for each package.

* [diagnostic_msgs](http://docs.ros.org/en/humble/p/diagnostic_msgs/)
* [geometry_msgs](http://docs.ros.org/en/humble/p/geometry_msgs/)
* [nav_msgs](http://docs.ros.org/en/humble/p/nav_msgs/)
* [sensor_msgs](http://docs.ros.org/en/humble/p/sensor_msgs/)
* [sensor_msgs_py](http://docs.ros.org/en/humble/p/sensor_msgs_py/)
* [shape_msgs](http://docs.ros.org/en/humble/p/shape_msgs/)
* [std_msgs](http://docs.ros.org/en/humble/p/std_msgs/)
* [std_srvs](http://docs.ros.org/en/humble/p/std_srvs/)
* [stereo_msgs](http://docs.ros.org/en/humble/p/stereo_msgs/)
* [trajectory_msgs](http://docs.ros.org/en/humble/p/trajectory_msgs/)
* [visualization_msgs](http://docs.ros.org/en/humble/p/visualization_msgs/)

## Purpose

Isolating the messages to communicate between stacks in a shared dependency allows nodes in dependent stacks to communicate without requiring dependencies upon each other.
This repository has been designed to contain the most common messages used between multiple packages to provide a shared dependency which will eliminate a problematic circular dependency.

## Contributing

For how to contribute see [CONTRIBUTING.md](common_interfaces/CONTRIBUTING.md)
