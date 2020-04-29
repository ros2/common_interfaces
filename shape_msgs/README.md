# shape_msgs

This package provides several messages and services for describing 3-dimensional shapes.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [Mesh](msg/Mesh.msg): Holds information describing a mesh for visualization and collision detections.
* [MeshTriangle](msg/MeshTriangle.msg): A single triangle of a mesh.
* [Plane](msg/Plane.msg): Representation of a plane, using the plane equation ax + by + cz + d = 0.
* [SolidPrimitive](msg/SolidPrimitive.msg): Describe a simple shape primitive like a box, a sphere, a cylinder, and a cone.

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
