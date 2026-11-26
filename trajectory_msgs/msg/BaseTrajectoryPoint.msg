# Each trajectory point specifies the target pose[, velocity[, acceleration]]
# for the robot base at a specific time from the trajectory start.
#
# This message is analogous to JointTrajectoryPoint, but applies to a mobile
# base or any frame tied to base_link.
#
# This message is particularly useful as an input to a trajectory or path-following
# controller, where each point specifies the desired base pose, velocity, and
# acceleration at a given time along the trajectory.
# The pose, velocity, and acceleration are all expressed in the frame specified
# by the parent trajectory's header.frame_id (e.g., "map", "odom", or similar).

# Target pose of the robot base (e.g., base_link) in the planning frame.
geometry_msgs/Pose pose

# Desired velocity of the base at this trajectory point.
# This includes both linear (m/s) and angular (rad/s) components.
# Twist is expressed in the same frame as pose.
geometry_msgs/Twist velocity

# Desired acceleration of the base at this trajectory point.
# This includes both linear (m/s^2) and angular (rad/s^2) components.
# Twist is expressed in the same frame as pose.
geometry_msgs/Twist acceleration

# Desired time from the start of the trajectory to arrive at this point.
builtin_interfaces/Duration time_from_start
