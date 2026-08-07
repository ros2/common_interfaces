# trajectory_msgs

This package provides several messages for defining robotic joint trajectories.

For more information about ROS 2 interfaces, see [docs.ros.org](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html).

## Messages (.msg)
* [BaseTrajectory](msg/BaseTrajectory.msg): A time-parameterized sequence of poses, velocities, and accelerations for a mobile robot base to follow, typically relative to a fixed global frame.
* [BaseTrajectoryPoint](msg/BaseTrajectoryPoint.msg): A single target pose, velocity, and acceleration for the robot base at a specific time from the start of the trajectory.
* [JointTrajectory](msg/JointTrajectory.msg): A coordinated sequence of joint configurations to be reached at prescribed time points.
* [JointTrajectoryPoint](msg/JointTrajectoryPoint.msg): A single configuration for multiple joints in a JointTrajectory.
* [MultiDOFJointTrajectory](msg/MultiDOFJointTrajectory.msg): A representation of a multi-dof joint trajectory (each point is a transformation).
* [MultiDOFJointTrajectoryPoint](msg/MultiDOFJointTrajectoryPoint.msg): A single configuration for multiple joints in a MultiDOFJointTrajectory.

## Quality Declaration
This package claims to be in the **Quality Level 2** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
