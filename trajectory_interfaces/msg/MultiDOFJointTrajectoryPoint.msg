# Each multi-dof joint can specify a transform (up to 6 DOF).
geometry_interfaces/Transform[] transforms

# There can be a velocity specified for the origin of the joint.
geometry_interfaces/Twist[] velocities

# There can be an acceleration specified for the origin of the joint.
geometry_interfaces/Twist[] accelerations

builtin_interfaces/Duration time_from_start
