# trajectory_msgs

This package provides several messages for defining robotic joint trajectories.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [JointTrajectory](msg/JointTrajectory.msg): A coordinated sequence of joint configurations to be reached at prescribed time points.
* [JointTrajectoryPoint](msg/JointTrajectoryPoint.msg): A single configuration for multiple joints in a JointTrajectory.
* [MultiDOFJointTrajectory](msg/MultiDOFJointTrajectory.msg): A representation of a multi-dof joint trajectory (each point is a transformation).
* [MultiDOFJointTrajectoryPoint](msg/MultiDOFJointTrajectoryPoint.msg): A single configuration for multiple joints in a MultiDOFJointTrajectory.

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
