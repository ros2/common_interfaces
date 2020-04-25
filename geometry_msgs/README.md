# geometry_msgs

This package provides messages for common geometric primitives such as points, vectors, and poses. These primitives are designed to provide a common data type and facilitate interoperability throughout the system.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [Accel](Accel.msg): Expresses acceleration in free space broken into its linear and angular parts.
* [AccelStamped](AccelStamped.msg): An accel with reference coordinate frame and timestamp.
* [AccelWithCovariance](AccelWithCovariance.msg): Acceleration in free space with uncertainty.
* [AccelWithCovarianceStamped](AccelWithCovarianceStamped.msg): An estimated accel with reference coordinate frame and timestamp.
* [Inertia](Inertia.msg): Expresses the inertial properties of a link.
* [InertiaStamped](InertiaStamped.msg): An Inertia with reference coordinate frame and timestamp.
* [Point32](Point32.msg): The position of a 3-dimensional point in free space, with 32-bit fields.
* [Point](Point.msg): The position of a 3-dimensional point in free space.
* [PointStamped](PointStamped.msg): Point with reference coordinate frame and timestamp.
* [Polygon](Polygon.msg): A specification of a polygon where the first and last points are assumed to be connected.
* [PolygonStamped](PolygonStamped.msg): A Polygon with reference coordinate frame and timestamp.
* [Pose2D](Pose2D.msg): **Deprecated as of Foxy and will potentially be removed in any following release.**
* [PoseArray](PoseArray.msg): An array of poses with a header for global reference.
* [Pose](Pose.msg): A representation of pose in free space, composed of position and orientation.
* [PoseStamped](PoseStamped.msg): A Pose with reference coordinate frame and timestamp.
* [PoseWithCovariance](PoseWithCovariance.msg): A pose in free space with uncertainty.
* [PoseWithCovarianceStamped](PoseWithCovarianceStamped.msg): An estimated pose with a reference coordinate frame and timestamp.
* [Quaternion](Quaternion.msg): An orientation in free space in quaternion form.
* [QuaternionStamped](QuaternionStamped.msg): An orientation with reference coordinate frame and timestamp.
* [Transform](Transform.msg): The transform between two coordinate frames in free space.
* [TransformStamped](TransformStamped.msg): A transform from coordinate frame header.frame_id to the coordinate frame child_frame_id.
* [Twist](Twist.msg): Velocity in 3-dimensional free space broken into its linear and angular parts.
* [TwistStamped](TwistStamped.msg): A twist with reference coordinate frame and timestamp.
* [TwistWithCovariance](TwistWithCovariance.msg): Velocity in 3-dimensional free space with uncertainty.
* [TwistWithCovarianceStamped](TwistWithCovarianceStamped.msg): An estimated twist with reference coordinate frame and timestamp.
* [Vector3](Vector3.msg): Represents a vector in 3-dimensional free space.
* [Vector3Stamped](Vector3Stamped.msg): Represents a Vector3 with reference coordinate frame and timestamp.
* [Wrench](Wrench.msg): Represents force in free space, separated into its linear and angular parts.
* [WrenchStamped](WrenchStamped.msg): A wrench with reference coordinate frame and timestamp.


## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
